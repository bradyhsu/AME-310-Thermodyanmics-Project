"""Animated turbofan cutaway (HTML canvas) of the steady operating point. Use with
streamlit.components.v1.html(engine_html(...), height=HEIGHT). The drawing code lives in
web/cutaway.js and is shared with the flight-cycle game."""
import json
from pathlib import Path

from engine import REF_RPM, Inputs, Result

HEIGHT = 470
WEB = Path(__file__).with_name("web")


def cutaway_data(i: Inputs, r: Result) -> dict:
    n_ipt = min(2, max(1, i.n_turb - 2))
    return dict(n_ipc=r.n_ipc, n_hpc=r.n_hpc, n_ipt=n_ipt, n_lpt=max(1, i.n_turb - 1 - n_ipt), bypass=i.bypass,
                fan_dia=i.fan_dia, pr_c=i.stage_pr ** i.n_comp, n_comp=i.n_comp, n_turb=i.n_turb)


def steady_frame(r: Result) -> dict:
    return dict(n1=r.rpm["N1"], n2=r.rpm["N2"], n3=r.rpm.get("N3", 0.0), T013=r.T["013"], T025=r.T["025"],
                T03=r.T["03"], T04=r.T["04"], tgt=r.tgt, T05=r.T["05"], flow=1.0, F=r.thrust / 1e3)


def engine_html(i: Inputs, r: Result) -> str:
    cur = steady_frame(r)
    lines = [f"N1 {cur['n1'] / REF_RPM['N1'] * 100:.1f}%   N2 {cur['n2'] / REF_RPM['N2'] * 100:.1f}%   "
             f"N3 {cur['n3'] / REF_RPM['N3'] * 100:.1f}%",
             f"TET {cur['T04']:.0f} K (result)   TGT {cur['tgt'] - 273.15:.0f} C   Thrust {cur['F']:.1f} kN",
             f"Fan diameter {i.fan_dia:.2f} m   Bypass {i.bypass:.1f}:1   {i.n_comp} compressor + {i.n_turb} turbine stages"]
    data = dict(geo=cutaway_data(i, r), cur=cur, lines=lines)
    return (HTML.replace("/*__CUTAWAY__*/", (WEB / "cutaway.js").read_text(encoding="utf-8"))
                .replace("/*__DATA__*/null", json.dumps(data)))


HTML = """<!DOCTYPE html><html><body style="margin:0;background:transparent;font-family:system-ui,sans-serif">
<canvas id="c" style="width:100%;display:block"></canvas>
<script>/*__CUTAWAY__*/</script>
<script>
const D = /*__DATA__*/null, cw = createCutaway(document.getElementById('c'), D.geo);
let last = performance.now();
function loop(now) { cw.draw(D.cur, (now - last) / 1000, D.lines); last = now; requestAnimationFrame(loop); }
requestAnimationFrame(loop);
</script></body></html>"""
