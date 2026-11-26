"""Transient (unsteady control-volume) model for the turbofan, built on engine.py.

First law for each control volume (CV):
    dE_cv/dt = W_in - Q_out + sum(m_i (h + V^2/2)) - sum(m_e (h + V^2/2))
  * Spool CVs (LP, IP, HP):  E = 1/2 J w^2,  W_in = turbine shaft power,  W_out = compressor power
        -> J w dw/dt = P_turbine - P_compressor
  * Combustor CV:  E = C T4 (lumped thermal capacity of gas + liner)
        -> C dT4/dt = m_c cp_a T03 + m_f eta_b LHV - (m_c + m_f) cp_h T04
Compressor temperature rise from mean-line loading (dh = psi U^2), so pressure ratio follows speed.
Assumptions: constant bypass ratio, core flow proportional to N1, turbine expansion ratios fixed
at the design point (choked turbine), quasi-steady nozzles, constant flight condition.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import pi
from engine import (CP_AIR, CP_HOT, FUELS_MJ_PER_KG, GAMMA_AIR, GAMMA_HOT, Inputs, Result, nozzle)


@dataclass(frozen=True)
class TParams:
    k0: float = 0.20        # initial fuel flow, fraction of design fuel flow
    k1: float = 1.00        # final fuel flow, fraction of design
    t_step: float = 2.0     # s, when the throttle move starts
    t_ramp: float = 1.0     # s, throttle ramp duration
    t_end: float = 25.0     # s
    J1: float = 1500.0      # LP spool inertia (fan + LPT), kg m^2
    J2: float = 45.0        # IP spool
    J3: float = 15.0        # HP spool
    C_comb: float = 20e3    # combustor effective thermal capacity, J/K
    dt: float = 0.01


def simulate(i: Inputs, r: Result, tp: TParams):
    k = (GAMMA_AIR - 1) / GAMMA_AIR
    kh = (GAMMA_HOT - 1) / GAMMA_HOT
    lhv = FUELS_MJ_PER_KG[i.fuel] * 1e6
    sc = i.fan_dia / 3.0
    rad = 2 * pi / 60
    w0 = (r.rpm["N1"] * rad, r.rpm["N2"] * rad, r.rpm["N3"] * rad)
    Ra = (i.fan_dia / 2, i.d_ipc * sc / 2, i.d_hpc * sc / 2)           # radii for U = w R
    T02, p02, p0 = r.T["02"], r.p["02"], r.p["0"]
    pi_ct = r.p["04"] / r.p_lpt_in
    pi_lpt = r.p_lpt_in / r.p["05"]
    phi = (r.T["03"] - r.T["025"]) / (r.T["03"] - r.T["013"])           # HP share of core turbine work
    em, B = i.eta_mech, i.bypass
    mf0 = r.fuel_flow

    def rates(w1, w2, w3, T4, kf):
        dh_f = i.psi_fan * (w1 * Ra[0]) ** 2
        T013 = T02 + dh_f / CP_AIR
        T025 = T013 + r.n_ipc * i.psi_comp * (w2 * Ra[1]) ** 2 / CP_AIR
        T03 = T025 + r.n_hpc * i.psi_comp * (w3 * Ra[2]) ** 2 / CP_AIR
        p013 = p02 * (T013 / T02) ** (i.eta_fan / k)
        p025 = p013 * (T025 / T013) ** (i.eta_comp / k)
        p03 = p025 * (T03 / T025) ** (i.eta_comp / k)
        p04 = p03 * (1 - i.burner_dp)
        mt = r.m_total * w1 / w0[0]
        mc = mt / (1 + B)
        mf = kf * mf0
        mh = mc + mf
        Tlp = T4 * pi_ct ** (-kh * r.eta_turb)
        T05 = Tlp * pi_lpt ** (-kh * r.eta_turb)
        Pcore = mh * CP_HOT * (T4 - Tlp) * em
        P_hpt, P_ipt = phi * Pcore, (1 - phi) * Pcore
        P_lpt = mh * CP_HOT * (Tlp - T05) * em
        P_fan = mt * CP_AIR * (T013 - T02)
        P_ipc = mc * CP_AIR * (T025 - T013)
        P_hpc = mc * CP_AIR * (T03 - T025)
        net = (P_lpt - P_fan, P_ipt - P_ipc, P_hpt - P_hpc)
        dT4 = (mc * CP_AIR * T03 + mf * i.eta_burner * lhv - mh * CP_HOT * T4) / tp.C_comb
        d = (net[0] / (tp.J1 * w1), net[1] / (tp.J2 * w2), net[2] / (tp.J3 * w3), dT4)
        return d, (net, T013, p013, T03, p04, T05, Tlp, mc, mf, mt)

    def thrust(o):
        net, T013, p013, T03, p04, T05, Tlp, mc, mf, mt = o
        nh = nozzle(T05, p04 / pi_ct / pi_lpt, GAMMA_HOT, CP_HOT, mc + mf, p0, i.eta_nozzle)
        nc = nozzle(T013, p013, GAMMA_AIR, CP_AIR, mt - mc, p0, i.eta_nozzle)
        return (mc + mf) * nh.V - mc * i.V0 + nh.thrust_p + (mt - mc) * (nc.V - i.V0) + nc.thrust_p

    def kf_at(t):
        if t <= tp.t_step: return tp.k0
        if t >= tp.t_step + tp.t_ramp: return tp.k1
        return tp.k0 + (tp.k1 - tp.k0) * (t - tp.t_step) / tp.t_ramp

    s = [*w0, r.T["04"]]
    dtp = 0.02
    for _ in range(int(120 / dtp)):                       # pre-roll to equilibrium at k0
        d, _o = rates(*s, tp.k0)
        s = [x + dtp * dx for x, dx in zip(s, d)]
    out = {n: [] for n in ("t", "N1", "N2", "N3", "T4", "TGT", "F", "mf", "dE_LP", "dE_IP", "dE_HP", "dE_comb", "T013", "T03", "T05", "mt")}
    n_steps, rec = int(tp.t_end / tp.dt), max(1, int(0.05 / tp.dt))
    for n in range(n_steps + 1):
        t = n * tp.dt
        d, o = rates(*s, kf_at(t))
        if n % rec == 0:
            out["t"].append(t)
            for key, x, ref in zip(("N1", "N2", "N3"), s[:3], w0):
                out[key].append(x / rad)
            out["T4"].append(s[3]); out["TGT"].append(o[6] - 273.15)
            out["T013"].append(o[1]); out["T03"].append(o[3]); out["T05"].append(o[5]); out["mt"].append(o[9])
            out["F"].append(thrust(o) / 1e3); out["mf"].append(o[8])
            out["dE_LP"].append(o[0][0] / 1e6); out["dE_IP"].append(o[0][1] / 1e6)
            out["dE_HP"].append(o[0][2] / 1e6); out["dE_comb"].append(tp.C_comb * d[3] / 1e6)
        s = [x + tp.dt * dx for x, dx in zip(s, d)]
    return out


if __name__ == "__main__":
    from dataclasses import replace
    from engine import solve
    inp = replace(Inputs(), T0=288.15, rho=1.225)
    r = solve(inp)
    o = simulate(inp, r, TParams())
    for idx in (0, 20, 40, 60, 100, 160, 240, 360, 500):
        if idx < len(o["t"]):
            print(f"t={o['t'][idx]:5.1f} N1={o['N1'][idx]:6.0f} N2={o['N2'][idx]:6.0f} N3={o['N3'][idx]:6.0f} "
                  f"T4={o['T4'][idx]:5.0f} TGT={o['TGT'][idx]:5.0f}C F={o['F'][idx]:6.1f}kN mf={o['mf'][idx]:5.2f}")
    print("steady check: design N1,N2,N3 =", [round(r.rpm[k]) for k in ("N1", "N2", "N3")], "F=", round(r.thrust / 1e3, 1))
    o2 = simulate(inp, r, TParams(k0=1.0, k1=1.0, t_end=3))
    print("design hold: F", round(o2["F"][0], 1), "->", round(o2["F"][-1], 1), " N1", round(o2["N1"][0]), "->", round(o2["N1"][-1]))
