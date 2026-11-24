"""Turbofan (separate-exhaust) Brayton cycle model built on the steady-flow energy equation.

For every component:  q - w = delta(h0)  (or delta(h) + delta(KE) in a nozzle),
with w = work OUT of the gas. Irreversibility is carried by isentropic/polytropic
efficiencies, so the first law always closes and the losses show up as extra work (fan,
compressors), lost work (turbines) or reduced kinetic energy (nozzles).

Stations: 0 ambient, 02 inlet exit, 013 fan exit, 025 IPC exit, 03 HPC exit,
04 turbine entry (TET), 05 LPT exit, 9 core nozzle exit.   Pure Python, no dependencies.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from math import exp, log, pi, sqrt

R_AIR, G0 = 287.0, 9.80665
GAMMA_AIR, GAMMA_HOT = 1.40, 1.33
CP_AIR, CP_HOT = 1005.0, 1150.0
R_HOT = CP_HOT * (GAMMA_HOT - 1) / GAMMA_HOT
FUELS_MJ_PER_KG = {"Jet A": 42.8, "Jet A-1": 43.15, "JP-8": 43.2,
                   "Avgas 100LL (100-octane)": 43.5, "Ethanol": 26.8, "Hydrogen": 120.0}
TGT_LIMITS_C = {"take_off": 900.0, "max_continuous": 850.0, "over_temp": 920.0}  # EASA E.111
REF_RPM = {"N1": 2700.0, "N2": 8200.0, "N3": 12600.0}                            # 100% speeds


def isa(alt_m: float, d_isa: float = 0.0):
    """ISA atmosphere (0-20 km) with temperature offset. Returns T [K], p [Pa], rho [kg/m3]."""
    if alt_m <= 11000:
        t = 288.15 - 0.0065 * alt_m
        p = 101325.0 * (t / 288.15) ** 5.25588
    else:
        t = 216.65
        p = 22632.06 * exp(-G0 * (alt_m - 11000) / (R_AIR * t))
    T = t + d_isa
    return T, p, p / (R_AIR * T)


@dataclass(frozen=True)
class Inputs:
    V0: float = 0.0
    T0: float = 288.15
    rho: float = 1.225
    bypass: float = 9.3
    fan_pr: float = 1.5
    n_comp: int = 14           # IPC + HPC (XWB 8 + 6)
    n_turb: int = 9            # HPT + IPT + LPT (XWB 1 + 2 + 6)
    tet: float = 1720.0
    fan_dia: float = 3.0
    fuel: str = "Jet A"
    stage_pr: float = 1.28
    eta_fan: float = 0.92      # polytropic
    eta_comp: float = 0.91     # polytropic
    eta_inlet_pr: float = 0.98
    burner_dp: float = 0.04
    eta_burner: float = 0.99
    eta_mech: float = 0.99
    eta_nozzle: float = 0.98
    fan_face_v: float = 170.0
    hub_ratio: float = 0.30
    # shaft-speed (mean-line) calibration, fitted to XWB take-off speeds
    psi_fan: float = 0.22      # fan loading  dh / U_tip^2
    psi_comp: float = 0.35     # compressor stage loading  dh_stage / U_mean^2
    d_ipc: float = 0.74        # IPC mean diameter at 3.0 m fan, m (scales with fan_dia)
    d_hpc: float = 0.645


@dataclass
class Nozzle:
    V: float; pe: float; Te: float; Ae: float; thrust_p: float; choked: bool


@dataclass
class Result:
    ok: bool = True
    warnings: list = field(default_factory=list)
    T: dict = field(default_factory=dict)
    p: dict = field(default_factory=dict)
    M0: float = 0.0; opr: float = 0.0; f: float = 0.0
    m_total: float = 0.0; m_core: float = 0.0; m_byp: float = 0.0
    tgt: float = 0.0; p_lpt_in: float = 0.0; eta_turb: float = 0.0
    n_ipc: int = 0; n_hpc: int = 0
    core: Nozzle | None = None; fan: Nozzle | None = None
    thrust: float = 0.0; fuel_flow: float = 0.0; tsfc: float = 0.0; spec_thrust: float = 0.0
    eta_th: float = 0.0; eta_p: float = 0.0; eta_o: float = 0.0; eta_ideal: float = 0.0
    rpm: dict = field(default_factory=dict); fan_tip_mach: float = 0.0; fan_tip_speed: float = 0.0


def nozzle(Tt, pt, gamma, cp, m, p0, eta):
    rg = cp * (gamma - 1) / gamma
    if pt <= p0:
        return Nozzle(0.0, p0, Tt, 0.0, 0.0, False)
    crit = ((gamma + 1) / 2) ** (gamma / (gamma - 1))
    choked = pt / p0 >= crit
    if choked:
        Te, pe = Tt * 2 / (gamma + 1), pt / crit
        V = sqrt(gamma * rg * Te) * 0.99
    else:
        Te, pe = Tt * (1 - eta * (1 - (p0 / pt) ** ((gamma - 1) / gamma))), p0
        V = sqrt(2 * cp * (Tt - Te))
    Ae = m / (pe / (rg * Te) * V)
    return Nozzle(V, pe, Te, Ae, (pe - p0) * Ae, choked)


def solve(i: Inputs) -> Result:
    r = Result()
    lhv = FUELS_MJ_PER_KG[i.fuel] * 1e6
    a0 = sqrt(GAMMA_AIR * R_AIR * i.T0)
    r.M0 = i.V0 / a0
    ram = 1 + 0.2 * r.M0 ** 2
    p0 = i.rho * R_AIR * i.T0
    T02, p02 = i.T0 * ram, p0 * ram ** 3.5 * i.eta_inlet_pr
    k = (GAMMA_AIR - 1) / GAMMA_AIR

    T013, p013 = T02 * i.fan_pr ** (k / i.eta_fan), p02 * i.fan_pr
    r.n_ipc = max(1, round(i.n_comp * 8 / 14)) if i.n_comp > 1 else 1
    r.n_hpc = max(0, i.n_comp - r.n_ipc)
    pr_ipc, pr_hpc = i.stage_pr ** r.n_ipc, i.stage_pr ** r.n_hpc
    T025, p025 = T013 * pr_ipc ** (k / i.eta_comp), p013 * pr_ipc
    T03, p03 = T025 * pr_hpc ** (k / i.eta_comp), p025 * pr_hpc
    p04, T04 = p03 * (1 - i.burner_dp), i.tet
    r.T = {"0": i.T0, "02": T02, "013": T013, "025": T025, "03": T03, "04": T04}
    r.p = {"0": p0, "02": p02, "013": p013, "025": p025, "03": p03, "04": p04}
    r.opr = p03 / p02

    # shaft speeds from mean-line loading: dh_stage = psi * U^2, N = 60 U / (pi D)
    U_fan = sqrt(CP_AIR * (T013 - T02) / i.psi_fan)
    sc = i.fan_dia / 3.0
    U_ipc = sqrt(CP_AIR * (T025 - T013) / r.n_ipc / i.psi_comp)
    rpm = {"N1": 60 * U_fan / (pi * i.fan_dia), "N2": 60 * U_ipc / (pi * i.d_ipc * sc)}
    if r.n_hpc:
        U_hpc = sqrt(CP_AIR * (T03 - T025) / r.n_hpc / i.psi_comp)
        rpm["N3"] = 60 * U_hpc / (pi * i.d_hpc * sc)
    r.rpm, r.fan_tip_speed = rpm, U_fan
    r.fan_tip_mach = U_fan / sqrt(GAMMA_AIR * R_AIR * T02)

    den = i.eta_burner * lhv - CP_HOT * T04
    r.f = (CP_HOT * T04 - CP_AIR * T03) / den if den > 0 else -1
    if r.f <= 0:
        r.ok = False
        r.warnings.append("TET is below compressor exit temperature: no heat can be added.")
        return r

    area = pi / 4 * i.fan_dia ** 2 * (1 - i.hub_ratio ** 2)
    r.m_total = i.rho * area * i.fan_face_v
    r.m_core = r.m_total / (1 + i.bypass)
    r.m_byp = r.m_total - r.m_core
    hot = (1 + r.f) * CP_HOT * i.eta_mech            # turbine J/kg-core per K

    r.tgt = T04 - CP_AIR * (T03 - T013) / hot        # LPT inlet: after HPT + IPT
    T05 = r.tgt - (1 + i.bypass) * CP_AIR * (T013 - T02) / hot
    r.T["05"] = T05
    if T05 <= 250:
        r.ok = False
        r.warnings.append("Turbines cannot supply the required work; reduce PR/bypass or raise TET.")
        return r
    r.eta_turb = 0.93 - 0.2 / i.n_turb
    e = GAMMA_HOT / ((GAMMA_HOT - 1) * r.eta_turb)
    r.p_lpt_in = p04 * (r.tgt / T04) ** e
    r.p["05"] = p04 * (T05 / T04) ** e

    r.core = nozzle(T05, r.p["05"], GAMMA_HOT, CP_HOT, r.m_core * (1 + r.f), p0, i.eta_nozzle)
    r.fan = nozzle(T013, p013, GAMMA_AIR, CP_AIR, r.m_byp, p0, i.eta_nozzle)
    r.T["9"], r.p["9"] = r.core.Te, r.core.pe

    r.thrust = (r.m_core * ((1 + r.f) * r.core.V - i.V0) + r.core.thrust_p
                + r.m_byp * (r.fan.V - i.V0) + r.fan.thrust_p)
    r.fuel_flow = r.f * r.m_core
    r.spec_thrust = r.thrust / r.m_total
    waste = 0.5 * (r.m_core * (1 + r.f) * (r.core.V - i.V0) ** 2 + r.m_byp * (r.fan.V - i.V0) ** 2)
    if r.thrust > 0 and r.fuel_flow > 0:
        useful = r.thrust * i.V0
        r.tsfc = r.fuel_flow / r.thrust * 1e6
        r.eta_th = (useful + waste) / (r.fuel_flow * lhv)
        r.eta_p = useful / (useful + waste)
        r.eta_o = r.eta_th * r.eta_p
    r.eta_ideal = 1 - r.opr ** -k

    tgt_c, L = r.tgt - 273.15, TGT_LIMITS_C
    if tgt_c > L["over_temp"]:
        r.warnings.append(f"TGT {tgt_c:.0f} C exceeds the {L['over_temp']:.0f} C over-temperature limit.")
    elif tgt_c > L["take_off"]:
        r.warnings.append(f"TGT {tgt_c:.0f} C exceeds the {L['take_off']:.0f} C take-off limit.")
    elif tgt_c > L["max_continuous"]:
        r.warnings.append(f"TGT {tgt_c:.0f} C exceeds the {L['max_continuous']:.0f} C max-continuous limit.")
    if r.thrust <= 0:
        r.warnings.append("Net thrust is zero or negative.")
    if r.fan_tip_mach > 1.6:
        r.warnings.append(f"Fan tip relative speed is high (M~{r.fan_tip_mach:.2f}).")
    return r


def energy_table(i: Inputs, r: Result):
    """Steady-flow first law per component, kJ per kg of the stream through it
    (fan: per kg total flow; others: per kg core air). q - w = dh + dKE, w = work out."""
    k, kh = (GAMMA_AIR - 1) / GAMMA_AIR, (GAMMA_HOT - 1) / GAMMA_HOT
    T0 = i.T0
    rows = []

    def comp(name, Ta, pa, Tb, pb, mult=1.0):
        dh = CP_AIR * (Tb - Ta) * mult
        ideal = CP_AIR * Ta * ((pb / pa) ** k - 1) * mult
        sg = (CP_AIR * log(Tb / Ta) - R_AIR * log(pb / pa)) * mult
        rows.append(dict(Component=name, q=0.0, w=-dh, dh=dh, dKE=0.0,
                         eta_s=ideal / dh, lost=T0 * sg))

    comp("Fan (all flow)", r.T["02"], r.p["02"], r.T["013"], r.p["013"], 1 + i.bypass)
    comp("IPC", r.T["013"], r.p["013"], r.T["025"], r.p["025"])
    comp("HPC", r.T["025"], r.p["025"], r.T["03"], r.p["03"])
    q = r.f * i.eta_burner * FUELS_MJ_PER_KG[i.fuel] * 1e6
    rows.append(dict(Component="Combustor", q=q, w=0.0,
                     dh=(1 + r.f) * CP_HOT * r.T["04"] - CP_AIR * r.T["03"], dKE=0.0,
                     eta_s=i.eta_burner, lost=T0 * R_AIR * log(1 / (1 - i.burner_dp))))

    def turb(name, Ta, pa, Tb, pb):
        dh = -(1 + r.f) * CP_HOT * (Ta - Tb)
        ideal = (1 + r.f) * CP_HOT * Ta * (1 - (pb / pa) ** kh)
        sg = (1 + r.f) * (CP_HOT * log(Tb / Ta) - R_HOT * log(pb / pa))
        rows.append(dict(Component=name, q=0.0, w=-dh, dh=dh, dKE=0.0,
                         eta_s=-dh / ideal, lost=T0 * sg))

    turb("HPT + IPT", r.T["04"], r.p["04"], r.tgt, r.p_lpt_in)
    turb("LPT", r.tgt, r.p_lpt_in, r.T["05"], r.p["05"])
    c = r.core
    ke_c = (1 + r.f) * c.V ** 2 / 2
    ideal = (1 + r.f) * CP_HOT * r.T["05"] * (1 - (r.p["0"] / r.p["05"]) ** kh) if r.p["05"] > r.p["0"] else 0
    rows.append(dict(Component="Core nozzle", q=0.0, w=0.0,
                     dh=(1 + r.f) * CP_HOT * (c.Te - r.T["05"]), dKE=ke_c,
                     eta_s=ke_c / ideal if ideal else 0.0, lost=ideal - ke_c))
    n = r.fan
    ke_f = (1 + i.bypass) * n.V ** 2 / 2 * (i.bypass / (1 + i.bypass))
    ideal = i.bypass * CP_AIR * r.T["013"] * (1 - (r.p["0"] / r.p["013"]) ** k) if r.p["013"] > r.p["0"] else 0
    rows.append(dict(Component="Fan nozzle (per kg core air)", q=0.0, w=0.0,
                     dh=i.bypass * CP_AIR * (n.Te - r.T["013"]), dKE=i.bypass * n.V ** 2 / 2,
                     eta_s=i.bypass * n.V ** 2 / 2 / ideal if ideal else 0.0,
                     lost=ideal - i.bypass * n.V ** 2 / 2))
    for x in rows:
        x["residual"] = x["q"] - x["w"] - x["dh"] - x["dKE"]
        for key in ("q", "w", "dh", "dKE", "lost", "residual"):
            x[key] /= 1e3
    return rows


def cycle_points(r: Result, n: int = 24):
    """Real cycle as (T, p) points, plus labelled state markers."""
    order = ["0", "02", "013", "025", "03", "04", "05", "9", "0"]
    pts = []
    for a, b in zip(order, order[1:]):
        T1, p1, T2, p2 = r.T[a], r.p[a], r.T[b], r.p[b]
        pts += [(T1 * (T2 / T1) ** t / 1, p1 * (p2 / p1) ** t) for t in (j / n for j in range(n + 1))]
    return pts, {s: (r.T[s], r.p[s]) for s in order[:-1]}


def ideal_points(i: Inputs, r: Result, n: int = 24):
    """Ideal air-standard Brayton cycle between the same inlet pressure, OPR and TET."""
    k = (GAMMA_AIR - 1) / GAMMA_AIR
    T1, p1 = r.T["02"], r.p["02"]
    T2, p2 = T1 * r.opr ** k, p1 * r.opr
    T3, T4 = r.T["04"], r.T["04"] * r.opr ** -k
    st = [(T1, p1), (T2, p2), (T3, p2), (T4, p1), (T1, p1)]
    pts = []
    for (Ta, pa), (Tb, pb) in zip(st, st[1:]):
        pts += [(Ta * (Tb / Ta) ** t, pa * (pb / pa) ** t) for t in (j / n for j in range(n + 1))]
    return pts, {"1": st[0], "2": st[1], "3": st[2], "4": st[3]}


def s_of(T, p):
    return CP_AIR * log(T / 288.0) - R_AIR * log(p / 101325.0)


def v_of(T, p):
    return R_AIR * T / p


PRESETS = {  # name -> (Inputs, altitude m, ISA offset K)
    "Trent XWB-84 take-off (SL, ISA)": (Inputs(), 0, 0),
    "XWB cruise (FL350, M0.85)": (Inputs(V0=252, tet=1580), 10668, 0),
    "Hot day take-off (ISA+40)": (Inputs(), 0, 40),
    "Smaller 5:1 engine, 7.5 km": (Inputs(V0=200, bypass=5, fan_pr=1.6, n_comp=10, n_turb=6,
                                          tet=1450, fan_dia=1.8), 7500, 0),
}

if __name__ == "__main__":
    for name, (inp, alt, d) in PRESETS.items():
        T, p, rho = isa(alt, d)
        from dataclasses import replace
        inp = replace(inp, T0=T, rho=rho)
        r = solve(inp)
        print(f"{name}: F={r.thrust/1e3:.1f} kN TSFC={r.tsfc:.2f} TGT={r.tgt-273.15:.0f}C "
              f"N1={r.rpm['N1']:.0f} N2={r.rpm['N2']:.0f} N3={r.rpm['N3']:.0f} tipM={r.fan_tip_mach:.2f} {r.warnings}")
    inp = replace(PRESETS["Trent XWB-84 take-off (SL, ISA)"][0], T0=288.15, rho=1.225)
    for row in energy_table(inp, solve(inp)):
        print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()})
