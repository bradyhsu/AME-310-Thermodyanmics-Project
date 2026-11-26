"""End-to-end smoke test of the running Streamlit app through Chrome DevTools (no extra packages:
uses the `websockets` client, which ships with Streamlit).

    streamlit run app.py --server.headless true --server.port 8599   (in another terminal)
    python tools/app_smoke.py [--url http://localhost:8599] [--out tools/app_game.png]

Opens the Transient tab, starts the mission with real key presses (Enter, S, F), waits, and
saves a screenshot. Reports whether the game reacted to the keys."""
import argparse
import asyncio
import base64
import json
import subprocess
import tempfile
import urllib.request
from pathlib import Path

from websockets.asyncio.client import connect

CHROME = r"C:\Program Files\Google\Chrome\Application\chrome.exe"


class CDP:
    def __init__(self, ws):
        self.ws, self.n = ws, 0

    async def call(self, method, **params):
        self.n += 1
        mid = self.n
        await self.ws.send(json.dumps({"id": mid, "method": method, "params": params}))
        while True:
            msg = json.loads(await self.ws.recv())
            if msg.get("id") == mid:
                if "error" in msg:
                    raise RuntimeError(msg["error"])
                return msg.get("result", {})

    async def js(self, expr):
        r = await self.call("Runtime.evaluate", expression=expr, returnByValue=True, awaitPromise=True)
        if "exceptionDetails" in r:
            raise RuntimeError(r["exceptionDetails"].get("exception", {}).get("description", r["exceptionDetails"]))
        return r.get("result", {}).get("value")

    async def key(self, key, code, vk, text=None, hold=0.05):
        base = dict(key=key, code=code, windowsVirtualKeyCode=vk)
        await self.call("Input.dispatchKeyEvent", type="keyDown", text=text or "", **base)
        await asyncio.sleep(hold)
        await self.call("Input.dispatchKeyEvent", type="keyUp", **base)


GAME = "document.querySelector('iframe[srcdoc*=\"Flight-cycle challenge\"]')"


async def main(url, out):
    tmp = tempfile.mkdtemp()
    proc = subprocess.Popen([CHROME, "--headless=new", "--disable-gpu", "--remote-debugging-port=9333",
                             f"--user-data-dir={tmp}", "--window-size=1500,1400", "about:blank"])
    try:
        for _ in range(50):
            try:
                tabs = json.load(urllib.request.urlopen("http://127.0.0.1:9333/json"))
                break
            except OSError:
                await asyncio.sleep(0.2)
        ws_url = next(t["webSocketDebuggerUrl"] for t in tabs if t["type"] == "page")
        cdp = CDP(await connect(ws_url, max_size=64 * 1024 * 1024))
        await cdp.call("Page.enable")
        await cdp.call("Page.navigate", url=url)
        for _ in range(60):
            await asyncio.sleep(0.5)
            if await cdp.js("[...document.querySelectorAll('[role=tab]')].some(b => b.innerText.includes('Transient'))") and                     await cdp.js("!!document.querySelector('iframe')"):
                break
        tab = await cdp.js("(() => { const b = [...document.querySelectorAll('[role=tab]')].find(b => b.innerText.includes('Transient')); "
                           "b.scrollIntoView(); const r = b.getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; })()")
        for t in ("mousePressed", "mouseReleased"):
            await cdp.call("Input.dispatchMouseEvent", type=t, x=tab[0], y=tab[1], button="left", clickCount=1)
        for _ in range(40):
            await asyncio.sleep(0.5)
            if await cdp.js(f"!!{GAME} && {GAME}.getBoundingClientRect().height > 100 && {GAME}.contentWindow.eval('typeof M') === 'object'"):
                break
        else:
            print("game iframe not found")
            return
        await cdp.js(f"{GAME}.scrollIntoView()")
        await asyncio.sleep(0.5)
        # click "Start mission" like a user would (this also focuses the game), then real key presses
        go = await cdp.js(f"(() => {{ const r = {GAME}.getBoundingClientRect(), b = {GAME}.contentDocument.getElementById('go').getBoundingClientRect(); "
                          f"return [r.x + b.x + b.width / 2, r.y + b.y + b.height / 2]; }})()")
        for t in ("mousePressed", "mouseReleased"):
            await cdp.call("Input.dispatchMouseEvent", type=t, x=go[0], y=go[1], button="left", clickCount=1)
        await asyncio.sleep(0.5)
        g = f"{GAME}.contentWindow"
        print("after click: shown =", await cdp.js(f"{g}.eval('shown')"), "| iframe height", await cdp.js(f"{GAME}.getBoundingClientRect().height"),
              "content", await cdp.js(f"{GAME}.contentDocument.documentElement.scrollHeight"),
              "| focus in game:", await cdp.js(f"document.activeElement === {GAME}"), await cdp.js(f"{GAME}.contentDocument.activeElement.id"))
        await cdp.key("s", "KeyS", 83, "s")
        for _ in range(60):
            await asyncio.sleep(0.5)
            if await cdp.js(f"{g}.eval('M').out.n[2] > 0.2"):
                break
        await cdp.key("f", "KeyF", 70, "f")
        await asyncio.sleep(12)
        state = await cdp.js(f"(() => {{ const M = {g}.eval('M'); return {{ shown: {g}.eval('shown'), phase: M.phase.id, starter: M.ctl.starter, "
                             f"fuel: M.ctl.fuelOn, lit: M.e.lit, n3: +(M.out.n[2] * 100).toFixed(1), tgt: +(M.out.Tlp - 273.15).toFixed(0), "
                             f"events: M.events.map(e => e.msg).slice(-6), samples: M.log.length }}; }})()")
        print(json.dumps(state, indent=1, ensure_ascii=False))
        shot = await cdp.call("Page.captureScreenshot", format="png", captureBeyondViewport=False)
        Path(out).write_bytes(base64.b64decode(shot["data"]))
        print("saved", out)
    finally:
        proc.terminate()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8599")
    ap.add_argument("--out", default=str(Path(__file__).with_name("app_game.png")))
    a = ap.parse_args()
    asyncio.run(main(a.url, a.out))
