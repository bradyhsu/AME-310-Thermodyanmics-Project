"""Flight-cycle game: packs the engine design from engine.py into data for the browser
simulator (web/sim.js) and assembles the game page (web/game.html) for st.components.html."""
from __future__ import annotations
import json
from dataclasses import replace
from math import log, pi
from pathlib import Path

from animation import cutaway_data
from engine import FUELS_MJ_PER_KG, REF_RPM, Inputs, solve

WEB = Path(__file__).with_name("web")
GAME_HEIGHT = 1120


def sim_params(i: Inputs):
    """Design-point data for the transient model, taken at sea-level static ISA so the thrust
    lever's 100% equals the sidebar's fuel-air ratio at take-off. Returns (params, result)."""
    ref = replace(i, V0=0.0, T0=288.15, rho=1.225)
    r = solve(ref)
    if not r.ok or "N3" not in r.rpm:
        return None, r
    rad, sc = 2 * pi / 60, i.fan_dia / 3.0
    p0 = r.p["0"]
    mc = r.m_total / (1 + i.bypass)
    pit, pict = r.p["04"] / r.p["05"], r.p["04"] / r.p_lpt_in
    stations = {k: [r.T[k], r.p[k]] for k in ("0", "02", "013", "025", "03", "04", "05")}
    stations["45"] = [r.tgt, r.p_lpt_in]
    stations["9"] = [r.core.Te, r.core.pe]
    P = dict(
        psi_fan=i.psi_fan, psi_comp=i.psi_comp, n_ipc=r.n_ipc, n_hpc=r.n_hpc,
        R=[i.fan_dia / 2, i.d_ipc * sc / 2, i.d_hpc * sc / 2],
        eta_fan=i.eta_fan, eta_comp=i.eta_comp, eta_in=i.eta_inlet_pr, dp_b=i.burner_dp,
        eta_b=i.eta_burner, eta_m=i.eta_mech, eta_n=i.eta_nozzle, eta_t=r.eta_turb,
        lhv=FUELS_MJ_PER_KG[i.fuel] * 1e6,
        w=[r.rpm[k] * rad for k in ("N1", "N2", "N3")], wref=[REF_RPM[k] * rad for k in ("N1", "N2", "N3")],
        mt=r.m_total, mc=mc, mb=r.m_total - mc, mf=r.fuel_flow, f=r.f,
        mfc=r.fuel_flow / (r.p["02"] / 101325.0),               # corrected design fuel flow (theta = 1)
        T02=r.T["02"], p02=r.p["02"], p03=r.p["03"], T4=r.T["04"],
        pr04=r.p["04"] / p0, npr=r.p["05"] / p0, alpha=log(pict) / log(pit),
        phi=(r.T["03"] - r.T["025"]) / (r.T["03"] - r.T["013"]),
        F=r.thrust, tgt=r.tgt, stations=stations,
    )
    return P, r


def game_html(i: Inputs, mission: dict) -> str | None:
    P, r = sim_params(i)
    if P is None:
        return None
    data = dict(P=P, mission=mission, geo=cutaway_data(i, r))
    html = (WEB / "game.html").read_text(encoding="utf-8")
    for key, name in (("/*__SIM__*/", "sim.js"), ("/*__CUTAWAY__*/", "cutaway.js")):
        html = html.replace(key, (WEB / name).read_text(encoding="utf-8"))
    return html.replace("/*__DATA__*/null", json.dumps(data))
