"""Headless check of the flight-cycle game: a scripted 'bot pilot' flies the whole mission in
web/sim.js inside headless Chrome/Edge and prints a phase-by-phase report.

    python tools/bot_test.py [--isa 0] [--tow 250] [--fl 350] [--probe]
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from engine import Inputs  # noqa: E402
from game import WEB, sim_params  # noqa: E402

BROWSERS = [r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            "google-chrome", "chromium", "msedge"]

BOT_ACT = r"""
// Scripted pilot: one control update per call. st holds per-phase memory.
function botAct(m, st, dt, probe) {
  probe = probe || {};
  const KT = Sim.KT, id = m.phase.id, o = m.out, ac = m.ac, a = m.air, c = m.ctl, tgt = o.Tlp - 273.15, lim = m.tgtLimit();
  if (st.pi !== m.pi) { st.pi = m.pi; st.vPrev = undefined; }
  c.brakes = false;
  switch (id) {
    case 'start':
      c.lever = 0;
      if (!c.starter && !c.fuelOn && o.n[2] < 0.1) m.toggleStarter();
      if (!c.fuelOn && o.n[2] >= (probe.fuelAt || 0.22)) m.toggleFuel();
      break;
    case 'taxi': case 'taxiin': {
      const goal = id === 'taxi' ? m.o.holdDist : m.acc.x0 + m.o.gateDist, d = goal - ac.x;
      const vt = Math.max(0, Math.min(8, 0.25 * d));
      c.lever = Math.max(0, Math.min(0.4, c.lever + 0.01 * (vt - ac.V)));
      if (ac.V > vt + 0.5 || d < 2) { c.brakes = true; c.lever = 0; }
      break; }
    case 'takeoff': c.lever = Math.min(1, c.lever + 0.02); if (tgt > lim - 8) c.lever -= 0.03; break;
    case 'climb': c.lever = Math.max(0.3, Math.min(probe.climb || 0.92, c.lever + 0.005 * (lim - 20 - tgt) / 10)); break;
    case 'cruise': c.lever = Math.max(0, Math.min(1, c.lever + 12 * (0.85 - a.mach) * dt)); break;
    case 'descent': c.lever = 0; break;
    case 'approach': { const e = (a.Vapp - ac.V) / KT, dv = (ac.V - (st.vPrev === undefined ? ac.V : st.vPrev)) / dt / KT; st.vPrev = ac.V;
      c.lever = Math.max(0, Math.min(1, c.lever + dt * (0.01 * e - 0.08 * dv))); break; }
    case 'rollout': c.lever = 0; m.setRev(ac.V > 32); c.brakes = true; break;
    case 'shutdown': c.lever = 0; if (c.fuelOn && (m.acc.cool || 0) >= m.o.cooldown + 1) m.toggleFuel(); break;
  }
}
"""

BOT = BOT_ACT + r"""
const KT = Sim.KT, FT = Sim.FT, out = [];
const log = s => out.push(s);
function fly(P, opt, probe) {
  const m = new Sim.Mission(P, opt), c = m.ctl, dt = 1 / 30, bst = {};
  log(`ground idle: N1 ${(m.idle.n[0]*P.w[0]/P.wref[0]*100).toFixed(1)}%  N3 ${(m.idle.n[2]*P.w[2]/P.wref[2]*100).toFixed(1)}%  ` +
      `TGT ${(m.idle.Tlp-273.15).toFixed(0)} C  F ${(m.idle.F/1e3).toFixed(1)} kN  ff ${m.idle.mf.toFixed(3)} kg/s  ` +
      `f ${m.idle.f.toFixed(4)}  mc ${m.idle.mc.toFixed(1)} kg/s`);
  let lastEv = 0, real = 0;
  const phaseStats = {};
  for (let k = 0; k < 30 * 60 * 30 && m.status === 'play'; k++) {
    const id = m.phase.id, tgt = m.out.Tlp - 273.15, ac = m.ac;
    const st = phaseStats[id] = phaseStats[id] || { tgtMax: -1e9, real: 0 };
    st.tgtMax = Math.max(st.tgtMax, tgt); st.real += dt;
    botAct(m, bst, dt, probe);
    m.step(dt); real += dt;
    if (probe.trace && k % Math.round(probe.trace * 30) === 0 && (!probe.only || probe.only.includes(m.phase.id))) {
      const o2 = m.out;
      log(`  t=${m.t.toFixed(1)} ${m.phase.id} lev=${(c.lever*100).toFixed(0)} N1=${(o2.n[0]*100).toFixed(1)} N2=${(o2.n[1]*100).toFixed(1)} N3=${(o2.n[2]*100).toFixed(1)} ` +
          `T4=${o2.T04.toFixed(0)} TGT=${(o2.Tlp-273.15).toFixed(0)} F=${(o2.F/1e3).toFixed(1)} mc=${o2.mc.toFixed(1)} mf=${o2.mf.toFixed(3)} lit=${m.e.lit} ` +
          `V=${(ac.V/KT).toFixed(0)}kt M=${m.air.mach.toFixed(3)} h=${(ac.h/FT).toFixed(0)}ft vs=${(ac.vs/FT*60).toFixed(0)}fpm x=${ac.x.toFixed(0)} D/eng=${(m.air.D/2e3).toFixed(1)}`);
    }
    for (; lastEv < m.events.length; lastEv++) { const e = m.events[lastEv]; if (e.kind !== 'info' || probe.verbose) log(`[${e.t.toFixed(1)}s ${e.kind}] ${e.msg}`); }
  }
  for (const [id, s] of Object.entries(phaseStats)) log(`phase ${id.padEnd(9)} real ${s.real.toFixed(0).padStart(4)} s  max TGT ${s.tgtMax.toFixed(0)} C`);
  log(`status ${m.status} ${m.failMsg}`);
  log(`score ${m.score} / ${Sim.MAX_SCORE}  grade ${m.letter}  fuel ${m.fuel.toFixed(0)} kg  sim time ${(m.t/60).toFixed(1)} min  real ${(real/60).toFixed(1)} min`);
  log(`penalties ${JSON.stringify(m.pens)}`);
}
"""


def run(params, mission, probe, body=None):
    js = (WEB / "sim.js").read_text(encoding="utf-8")
    page = f"""<!DOCTYPE html><html><body><pre id="out"></pre><script>{js}</script><script>{BOT}
const P = {json.dumps(params)};
try {{ {body or f"fly(P, {json.dumps(mission)}, {json.dumps(probe)});"} }}
catch (e) {{ out.push('ERROR ' + e.stack); }}
document.getElementById('out').textContent = out.join('\\n');</script></body></html>"""
    browser = next((b for b in BROWSERS if Path(b).exists() or shutil.which(b)), None)
    if browser is None:
        sys.exit("No Chrome/Edge found for headless run.")
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "bot.html"
        f.write_text(page, encoding="utf-8")
        res = subprocess.run([browser, "--headless=new", "--disable-gpu", f"--user-data-dir={tmp}/prof",
                              "--virtual-time-budget=600000", "--dump-dom", f.as_uri()],
                             capture_output=True, text=True, timeout=600, encoding="utf-8")
    dom = res.stdout
    s, e = dom.find('<pre id="out">'), dom.find("</pre>")
    if s < 0:
        sys.exit("Browser produced no output:\n" + res.stderr[-2000:])
    import html
    return html.unescape(dom[s + len('<pre id="out">'):e])


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--isa", type=float, default=0)
    ap.add_argument("--tow", type=float, default=220)
    ap.add_argument("--fl", type=int, default=350)
    ap.add_argument("--trace", type=float, default=0, help="print state every N real seconds")
    ap.add_argument("--fuel-at", type=float, default=0.22, help="N3 fraction at which the bot opens the fuel")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--only", help="comma-separated phase ids to trace")
    ap.add_argument("--js", help="file with custom JS to run instead of the bot (has Sim, P, log)")
    a = ap.parse_args()
    P, _ = sim_params(replace(Inputs()))
    print(run(P, dict(dISA=a.isa, tow=a.tow * 1000, cruiseFt=a.fl * 100),
              dict(trace=a.trace, fuelAt=a.fuel_at, verbose=a.verbose, only=a.only.split(",") if a.only else None),
              Path(a.js).read_text() if a.js else None))
