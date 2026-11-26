# AME-310-Thermodyanmics-Project
Turbofan simulator/analysis tool for Thermodyanamics I project

## Run

```
pip install -r requirements.txt
python -m streamlit run app.py
```

## Tabs

- **Engine animation / Cycle diagrams / First law by component / Performance**: steady-flow analysis of the
  engine set in the sidebar (`engine.py`).
- **Transient: flight-cycle challenge**: a game that flies one full engine cycle (start-up, taxi, take-off,
  climb, cruise, descent, landing, shutdown) on a twin-engine wide-body. You run the engines (starter, fuel
  switch, thrust lever, reversers) while the autopilot flies the path. Every chart plots live as the mission
  plays. Scoring rewards clean starts, staying inside TGT limits, holding speeds and smooth landings.

## Files

| File | What it does |
|---|---|
| `engine.py` | Steady Brayton-cycle model (design point, first-law table, cycle diagrams) |
| `animation.py` | Engine cutaway for the Engine animation tab |
| `game.py` | Packs the engine design for the browser simulator and builds the game page |
| `web/sim.js` | Transient engine + aircraft model and mission rules (unsteady first law per control volume) |
| `web/game.html` | Game UI: cockpit, live charts, briefing/debrief |
| `web/cutaway.js` | Animated cutaway drawing, shared by both tabs |
| `transient.py` | Earlier fixed-flight-condition throttle-step model (no longer used by the app) |
| `tools/bot_test.py` | Headless check: a scripted pilot flies the whole mission and prints a report (needs Chrome or Edge) |
| `tools/screenshot.py` | Renders the game headless at a chosen phase and saves a PNG |
| `tools/app_smoke.py` | Drives the running Streamlit app through Chrome DevTools and checks the game responds to keys |
