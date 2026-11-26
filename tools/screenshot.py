"""Render the game page headless, let the bot pilot fly it to a given phase, and save a PNG.

    python tools/screenshot.py brief|start|taxi|takeoff|climb|cruise|descent|approach|rollout|done [--after 30] [--out shot.png]
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from bot_test import BOT_ACT, BROWSERS  # noqa: E402
from engine import Inputs  # noqa: E402
from game import GAME_HEIGHT, game_html  # noqa: E402

DRIVER = """<script>""" + BOT_ACT + """
addEventListener('load', () => {
  const target = '%s', after = %f;
  if (target === 'brief') return;
  startMission(); const st = {};
  for (let k = 0; k < 30 * 60 * 40 && M.status === 'play'; k++) {
    if (M.phase.id === target && M.pt >= after) break;
    botAct(M, st, 1 / 30); M.step(1 / 30);
  }
  if (target !== 'done') paused = true;
});
</script></body>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target")
    ap.add_argument("--after", type=float, default=20.0, help="sim seconds into the phase")
    ap.add_argument("--out", default=None)
    ap.add_argument("--width", type=int, default=1360)
    a = ap.parse_args()
    html = game_html(Inputs(), dict(dISA=0.0, tow=220e3, cruiseFt=35000.0))
    html = html.replace("</body>", DRIVER % (a.target, a.after))
    browser = next(b for b in BROWSERS if Path(b).exists())
    out = Path(a.out or ROOT / "tools" / f"shot_{a.target}.png").resolve()
    with tempfile.TemporaryDirectory() as tmp:
        page = Path(tmp) / "game.html"
        page.write_text(html, encoding="utf-8")
        subprocess.run([browser, "--headless=new", "--disable-gpu", "--hide-scrollbars", f"--user-data-dir={tmp}/prof",
                        f"--window-size={a.width},{GAME_HEIGHT}", "--virtual-time-budget=4000",
                        f"--screenshot={out}", page.as_uri()], capture_output=True, timeout=300)
    print(out)


if __name__ == "__main__":
    main()
