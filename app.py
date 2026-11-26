"""Run:  pip install streamlit matplotlib   then   streamlit run app.py"""
from dataclasses import replace
import matplotlib.pyplot as plt
import streamlit as st
st.set_page_config(page_title="Turbofan Brayton Cycle", layout="wide")
try:
    from animation import HEIGHT, engine_html
    from engine import (FUELS_MJ_PER_KG, PRESETS, REF_RPM, TGT_LIMITS_C, Inputs, cycle_points, energy_table,
                        ideal_points, isa, s_of, solve, tgt_suggestions, v_of)
    from game import GAME_HEIGHT, game_html
except ImportError as e:
    st.error(f"A project file is missing or out of date: {e}. Keep app.py, engine.py, animation.py, game.py "
             "and the web/ folder together.")
    st.stop()

st.title("Turbofan Brayton Cycle Simulator")
st.caption("Separate-exhaust high-bypass turbofan, steady-flow first law per component. "
           "Defaults based on the Trent XWB (EASA TCDS E.111).")

KEYS = ["V0", "bypass", "fan_pr", "n_comp", "n_turb", "fan_dia", "fuel"]


def load_preset():
    inp, alt, d = PRESETS[st.session_state["preset"]]
    for k in KEYS:
        st.session_state[k] = getattr(inp, k)
    st.session_state["far_pct"] = round(inp.far * 100, 2)
    st.session_state["alt"], st.session_state["d_isa"] = float(alt), float(d)
    st.session_state["T0m"], st.session_state["rhom"] = inp.T0, inp.rho


if "preset" not in st.session_state:
    st.session_state["preset"] = next(iter(PRESETS))
    load_preset()

sb = st.sidebar
sb.selectbox("Preset", list(PRESETS), key="preset", on_change=load_preset)
sb.subheader("Atmosphere")
mode = sb.radio("Source", ["ISA altitude", "Manual T and density"], horizontal=True)
if mode == "ISA altitude":
    alt = sb.slider("Altitude (m)", 0.0, 20000.0, step=100.0, key="alt")
    d_isa = sb.slider("ISA temperature offset (K)", -40.0, 40.0, step=1.0, key="d_isa")
    T0, p0, rho = isa(alt, d_isa)
    sb.caption(f"T = {T0:.1f} K, p = {p0/1000:.2f} kPa, rho = {rho:.3f} kg/m3")
else:
    T0 = sb.slider("Ambient temperature (K)", 200.0, 340.0, step=1.0, key="T0m")
    rho = sb.slider("Air density (kg/m3)", 0.1, 1.4, step=0.005, key="rhom")
sb.subheader("Flight and engine")
V0 = sb.slider("Freestream velocity (m/s)", 0.0, 300.0, step=1.0, key="V0")
bypass = sb.slider("Bypass ratio", 1.0, 14.0, step=0.1, key="bypass")
fan_pr = sb.slider("Fan pressure ratio", 1.2, 1.8, step=0.01, key="fan_pr")
n_comp = sb.slider("Compressor stages (IPC + HPC)", 6, 20, step=1, key="n_comp")
n_turb = sb.slider("Turbine stages (HPT + IPT + LPT)", 3, 12, step=1, key="n_turb")
far_pct = sb.slider("Fuel–air ratio (% fuel per kg core air)", 0.5, 4.0, step=0.01, key="far_pct")
fan_dia = sb.slider("Fan diameter (m)", 1.0, 4.0, step=0.01, key="fan_dia")
fuel = sb.selectbox("Fuel", list(FUELS_MJ_PER_KG), key="fuel")

inp = replace(Inputs(), V0=V0, T0=T0, rho=rho, bypass=bypass, fan_pr=fan_pr, n_comp=n_comp,
              n_turb=n_turb, far=far_pct / 100, fan_dia=fan_dia, fuel=fuel)
r = solve(inp)
sb.metric("Turbine entry temperature (result)", f"{r.T['04']:.0f} K", f"{r.T['04'] - 273.15:.0f} C", delta_color="off")
sb.caption("TET is not an input. It comes from the combustor energy balance: compressor exit temperature (inlet air + compression) "
           "plus the heat released by the fuel.")
for w in r.warnings:
    st.warning(w)
if not r.ok:
    st.stop()

with st.expander("How to meet the TGT limit", expanded=(r.tgt - 273.15) > 900):
    lim_name = st.radio("Limit to meet", ["Take-off (5 min)", "Maximum continuous", "Over-temperature (20 s)"], horizontal=True)
    lim = {"Take-off (5 min)": TGT_LIMITS_C["take_off"], "Maximum continuous": TGT_LIMITS_C["max_continuous"],
           "Over-temperature (20 s)": TGT_LIMITS_C["over_temp"]}[lim_name]
    base, opts = tgt_suggestions(inp, lim)
    tgt_now = r.tgt - 273.15
    if tgt_now <= lim:
        st.success(f"TGT is {tgt_now:.0f} C, {lim - tgt_now:.0f} C below the {lim:.0f} C limit. No change needed.")
    else:
        st.error(f"TGT is {tgt_now:.0f} C, {tgt_now - lim:.0f} C over the {lim:.0f} C limit. Any one of these changes, "
                 "applied to your current settings, brings it back inside the limit:")
        if opts:
            st.dataframe([{"Change": o_["lever"], "Setting": o_["change"], "New TET (K)": round(o_["tet_k"]), "New TGT (C)": round(o_["tgt_c"]),
                           "Thrust (kN)": round(o_["thrust_kn"], 1), "Thrust change (%)": round(o_["d_thrust_pct"], 1),
                           "TSFC (g/kN.s)": round(o_["tsfc"], 2), "OPR": round(o_["opr"], 1)} for o_ in opts],
                          width="stretch", hide_index=True)
        else:
            st.info("None of the single changes within the slider ranges are enough. Try combining less fuel with a lower flight speed or altitude, or a cooler day.")
        st.caption("Why these work: TGT is the temperature entering the LP turbine, so it equals TET minus the temperature the HP and IP turbines "
                   "give up to drive the core compressors. Burning less fuel lowers TET and so TGT directly. More core compression (stages or fan pressure ratio) "
                   "makes the HP/IP turbines extract more work. Bypass ratio, fan diameter and turbine stage count do not change it. "
                   "Other practical levers: reduce thrust setting, take off at lower ambient temperature, or derate.")

t0, t1, t2, t3, t4 = st.tabs(["Engine animation", "Cycle diagrams", "First law by component", "Performance and shaft speeds", "Transient"])
with t0:
    st.iframe(engine_html(inp, r), height=HEIGHT + 10)
    st.caption("Live cutaway of the steady operating point. Rotor speeds (slow motion), gas colour and the readouts follow your sliders. "
               "Blue = cold, red = hot. Run the Transient tab to watch a throttle move.")

with t1:
    real, marks = cycle_points(r)
    ideal, imarks = ideal_points(inp, r)
    c1, c2 = st.columns(2)
    for col, title, fx, fy, xl, yl, logy in [
        (c1, "T-s diagram", s_of, lambda T, p: T, "Entropy s (J/kg.K, relative)", "Temperature (K)", False),
        (c2, "p-v diagram", lambda T, p: v_of(T, p), lambda T, p: p / 1e3, "Specific volume v (m3/kg)", "Pressure (kPa)", True)]:
        fig, ax = plt.subplots(figsize=(6, 4.4))
        ax.plot([fx(*q) for q in ideal], [fy(*q) for q in ideal], "--", lw=1.4, label="Ideal Brayton")
        ax.plot([fx(*q) for q in real], [fy(*q) for q in real], lw=2.2, label="Real engine")
        for n, q in marks.items():
            ax.plot(fx(*q), fy(*q), "ko", ms=4); ax.annotate(n, (fx(*q), fy(*q)), xytext=(4, 4), textcoords="offset points")
        if logy:
            ax.set_xscale("log"); ax.set_yscale("log")
        ax.set_xlabel(xl); ax.set_ylabel(yl); ax.set_title(title); ax.grid(alpha=.3, which="both"); ax.legend()
        col.pyplot(fig)
        plt.close(fig)
    st.caption("States: 0 ambient, 02 inlet exit, 013 fan, 025 IPC exit, 03 HPC exit, 04 TET, 05 LPT exit, 9 core nozzle exit. "
               "Plotted with air constants (R = 287 J/kg.K, cp = 1005 J/kg.K); v = RT/p. "
               "The ideal cycle uses the same inlet total conditions, OPR and TET with no losses.")

with t2:
    st.markdown("Steady-flow energy equation for each component: **q − w = Δh + ΔKE** (w = work out of the gas). "
                "Efficiencies enter as extra shaft work in compressors, lost work in turbines and reduced "
                "kinetic energy in nozzles. Values in kJ per kg of the stream through the component.")
    rows = energy_table(inp, r)
    st.dataframe([{"Component": x["Component"], "q": x["q"], "w": x["w"], "Δh": x["dh"], "ΔKE": x["dKE"],
                   "Isentropic eff.": x["eta_s"], "Lost work T0·s_gen": x["lost"], "q−w−Δh−ΔKE": x["residual"]}
                  for x in rows], width="stretch", hide_index=True)
    st.caption("Lost work = T0 · entropy generated (compressors, turbines, burner pressure drop) or the kinetic energy "
               "short of ideal expansion (nozzles). The residual column checks that the first law closes.")

with t3:
    c = st.columns(4)
    c[0].metric("Net thrust", f"{r.thrust/1e3:.1f} kN")
    c[1].metric("TSFC", f"{r.tsfc:.2f} g/kN.s")
    c[2].metric("TGT (LPT inlet)", f"{r.tgt-273.15:.0f} C")
    c[3].metric("Overall PR", f"{r.opr:.1f}")
    c[0].metric("Thermal eff.", f"{r.eta_th*100:.1f} %")
    c[1].metric("Propulsive eff.", f"{r.eta_p*100:.1f} %")
    c[2].metric("Overall eff.", f"{r.eta_o*100:.1f} %")
    c[3].metric("Ideal Brayton eff.", f"{r.eta_ideal*100:.1f} %")
    c[0].metric("TET (result)", f"{r.T['04']:.0f} K")
    c[1].metric("HPC exit T3", f"{r.T['03']:.0f} K")
    c[2].metric("Fuel–air ratio", f"{r.f*100:.2f} %")
    c[3].metric("Inlet stagnation T02", f"{r.T['02']:.0f} K")
    c[0].metric("Fuel flow", f"{r.fuel_flow:.2f} kg/s")
    c[1].metric("Air mass flow", f"{r.m_total:.0f} kg/s")
    c[2].metric("Core jet speed", f"{r.core.V:.0f} m/s")
    c[3].metric("Fan jet speed", f"{r.fan.V:.0f} m/s")
    st.subheader("Shaft speeds")
    s = st.columns(4)
    for col, (n, lab) in zip(s, [("N1", "N1 (LP / fan)"), ("N2", "N2 (IP)"), ("N3", "N3 (HP)")]):
        if n in r.rpm:
            col.metric(lab, f"{r.rpm[n]:,.0f} rpm", f"{r.rpm[n]/REF_RPM[n]*100:.1f} % of XWB 100%", delta_color="off")
    s[3].metric("Fan tip speed", f"{r.fan_tip_speed:.0f} m/s", f"relative M ~ {r.fan_tip_mach:.2f}", delta_color="off")
    st.caption("Speeds come from a mean-line estimate (stage work = loading × U²), with loading and mean diameters "
               "calibrated so the XWB-84 take-off case reproduces its certified speeds (N1 99.1%, N2 100%, N3 97.4%). "
               "Reference 100% speeds: 2700 / 8200 / 12600 rpm. Treat other engine sizes as estimates.")

with t4:
    st.markdown("**Flight-cycle challenge.** Fly one full engine cycle on a twin-engine wide-body powered by the engine "
                "in the sidebar: start-up, taxi, take-off, climb, cruise, descent, landing and shutdown. You control the "
                "engines (starter, fuel switch, thrust lever, reversers). The autopilot flies the path. The charts plot "
                "live as the mission plays out.")
    g = st.columns(3)
    g_isa = g[0].slider("Day temperature (ISA offset, K)", -20, 25, 0, 5, key="g_isa",
                        help="Hot days raise TGT for the same lever position, so you may have to derate.")
    g_tow = g[1].slider("Take-off mass (t)", 180, 280, 220, 10, key="g_tow",
                        help="Heavier means a longer take-off roll, a slower climb and a lower ceiling.")
    g_fl = g[2].select_slider("Cruise level", [290, 310, 330, 350, 370, 390, 410], 350, key="g_fl",
                              format_func=lambda v: f"FL{v}")
    html = game_html(inp, dict(dISA=float(g_isa), tow=g_tow * 1000.0, cruiseFt=g_fl * 100.0))
    if html is None:
        st.error("This engine design does not solve at sea-level take-off conditions, so it cannot be flown. "
                 "Adjust the sidebar (it needs at least two HP compressor stages).")
    else:
        st.iframe(html, height="content")
    st.caption("Engine design (bypass ratio, fan pressure ratio, stages, fuel-air ratio, fan diameter, fuel) comes from the "
               "sidebar. The thrust lever's 100% is your sidebar fuel-air ratio at sea-level take-off. The sidebar's "
               "atmosphere and flight speed are not used here; the mission has its own. Changing any setting restarts "
               "the mission.")
    with st.expander("The physics behind the game"):
        st.markdown(r"""
**Unsteady first law on four control volumes** (one engine; the aircraft has two identical engines):

* **Each spool (LP, IP, HP):** $\frac{d}{dt}\left(\tfrac12 J\omega^2\right) = \dot W_{turbine} - \dot W_{compressor} - \dot W_{friction}\;(+\dot W_{starter}$ on HP$)$
* **Combustor:** $C\,\frac{dT_4}{dt} = \dot m_c c_{p,a} T_{03} + \dot m_{f,burnt}\,\eta_b\,LHV - (\dot m_c + \dot m_{f,burnt})\,c_{p,g} T_4 - hA\,(T_4 - T_0)$

The *Stored-energy rate* chart plots each left-hand side live. When a spool's dE/dt is positive, its turbine is
outrunning its compressor and the spool accelerates.

**Component models.** Compressor temperature rise comes from mean-line loading ($\Delta h = \psi U^2$), so pressure
ratio follows spool speed. Mass flow follows inlet density times blade speed. Turbine expansion follows the pressure ratio
actually available, and nozzles are quasi-steady. At the sea-level static design point the model sits on the
steady solution shown in the other tabs.

**FADEC (fuel control).** The lever schedules *corrected* fuel flow $\dot m_f/(\delta\sqrt\theta)$. The FADEC
limits the fuel-air ratio during acceleration and deceleration, so overshoots stay small and the flame doesn't blow out.
Before light-off it meters a fixed light-off flow. Fuel opened too early pools in the combustor and flashes off at
ignition, which causes a **hot start**.

**Aircraft.** A point mass with an energy balance $(T - D)\,V = W\,\dot h + m V \dot V$. In climb and descent the
autopilot pitches for speed, so your thrust sets the climb or descent rate. In cruise it holds altitude, so thrust minus drag
sets acceleration. On approach it flies a 3° glideslope and you hold the approach speed.

**Ballpark constants (not certified data):** spool inertias 400 / 25 / 10 kg m², combustor capacity 5 kJ/K, start TGT limit
750 °C (game value), wide-body drag polar $C_D = 0.015 + 0.038\,C_L^2$, wing area 443 m². The take-off (900 °C, 5 min),
max-continuous (850 °C) and over-temperature (920 °C) TGT limits follow EASA TCDS E.111.
""")
