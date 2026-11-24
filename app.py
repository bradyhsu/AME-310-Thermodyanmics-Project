"""Run:  pip install streamlit matplotlib   then   streamlit run app.py"""
from dataclasses import replace
import matplotlib.pyplot as plt
import streamlit as st
from engine import (FUELS_MJ_PER_KG, PRESETS, REF_RPM, Inputs, cycle_points, energy_table,
                    ideal_points, isa, s_of, solve, v_of)

st.set_page_config(page_title="Turbofan Brayton Cycle", layout="wide")
st.title("Turbofan Brayton Cycle Simulator")
st.caption("Separate-exhaust high-bypass turbofan, steady-flow first law per component. "
           "Defaults based on the Trent XWB (EASA TCDS E.111).")

KEYS = ["V0", "bypass", "fan_pr", "n_comp", "n_turb", "tet", "fan_dia", "fuel"]


def load_preset():
    inp, alt, d = PRESETS[st.session_state["preset"]]
    for k in KEYS:
        st.session_state[k] = getattr(inp, k)
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
tet = sb.slider("Turbine entry temperature (K)", 1200.0, 2000.0, step=5.0, key="tet")
fan_dia = sb.slider("Fan diameter (m)", 1.0, 4.0, step=0.01, key="fan_dia")
fuel = sb.selectbox("Fuel", list(FUELS_MJ_PER_KG), key="fuel")

inp = replace(Inputs(), V0=V0, T0=T0, rho=rho, bypass=bypass, fan_pr=fan_pr, n_comp=n_comp,
              n_turb=n_turb, tet=tet, fan_dia=fan_dia, fuel=fuel)
r = solve(inp)
for w in r.warnings:
    st.warning(w)
if not r.ok:
    st.stop()

t1, t2, t3 = st.tabs(["Cycle diagrams", "First law by component", "Performance and shaft speeds"])

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
                  for x in rows], use_container_width=True, hide_index=True)
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
