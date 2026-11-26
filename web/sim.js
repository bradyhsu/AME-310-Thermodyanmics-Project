/* Mission simulator core for the turbofan flight-cycle game. No DOM: it runs in the browser
   (web/game.html) and headless (tools/bot_test.py).

   ENGINE - three-spool turbofan, unsteady first law on four control volumes (CVs):
     each spool:  d/dt(1/2 J w^2) = P_turbine - P_compressor - P_friction (+ P_starter on the HP spool)
     combustor:   C dT4/dt = m_c cp_a T03 + m_f,burnt eta_b LHV - (m_c + m_f,burnt) cp_g T4 - hA (T4 - T0)
   Compressor temperature rise from mean-line loading (dh = psi U^2), so pressure ratio follows spool speed.
   Mass flow = inlet density x blade speed (fixed flow coefficient). Turbine expansion follows the pressure
   ratio actually available (p04 / nozzle pressure), so a stopped engine produces no turbine work.
   At the sea-level static design point the model sits on the steady engine.py solution.

   AIRCRAFT - point mass, two engines. Energy equation (T - D) V = W dh/dt + m V dV/dt.
   The autopilot flies the vertical path (speed-on-pitch, altitude hold or glideslope);
   the player flies the thrust lever, so thrust decides climb rate or speed. */
(function (root) {
'use strict';
const CP_A = 1005, CP_H = 1150, G_A = 1.4, G_H = 1.33, R_A = 287, G0 = 9.80665;
const KA = (G_A - 1) / G_A, KH = (G_H - 1) / G_H;
const KT = 0.514444, FT = 0.3048, TAN3 = Math.tan(3 * Math.PI / 180);
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));

// ---------- tunable engine constants (ballpark values, see app "physics" notes) ----------
const EC = {
  J: [400, 25, 10],         // spool inertias kg m^2 (LP, IP, HP)
  C: 5e3,                   // combustor effective thermal capacity J/K
  hA: 200,                  // combustor casing heat loss W/K (the Q_out term)
  fric: [25, 8, 25],        // bearing/accessory drag torque N m
  wMin: 25,                 // rad/s, floor used when turning power into torque
  tauFuel: 0.3,             // s, fuel metering valve lag
  // FADEC fuel limits as fuel-air ratio on the core flow
  mfLight: 0.05,            // kg/s (corrected) light-off fuel flow before the combustor lights
  fStart: 0.018, fAccSlope: 0.018, fAccMargin: 0.005,  // accel limit: fStart up to idle, then the running line + margin
  fDec: 0.007,              // deceleration floor while lit: stops lean blow-out when the lever is chopped
  idleFrac: 0.08,           // idle corrected fuel flow, fraction of design
  flowExp: 3.0, flowLin: 0.1,   // core flow ~ lin*n + (1-lin)*n^exp: steep at idle, nonzero while motoring
  coreMix: 0.2,             // core flow speed n = mix*N2 + (1-mix)*N3
  stT: 350, stFree: 0.55, stCut: 0.38,   // air starter: stall torque N m, free-run N3, auto cut-out N3
  fLBO: 0.0035, fLight: 0.004, fRich: 0.15, fStoich: 0.068, lightN3: 0.12,
  tauPool: 1.5,             // s, unburnt fuel pooled before light-off flashes off over this time
  revEff: 0.45,             // fraction of fan jet momentum turned forward by the reverser
};
const AC = {                // twin-engine wide-body (A350-class)
  S: 443, mu: 0.015, muB: 0.30, CLg: 0.4, nEng: 2,
  cfg: { clean: { CD0: 0.015, K: 0.038, CLmax: 1.5 },
         to:    { CD0: 0.032, K: 0.048, CLmax: 2.2 },
         land:  { CD0: 0.075, K: 0.050, CLmax: 2.6 } },
};

function isa(h, dT) {
  h = Math.max(0, h);
  let t, p;
  if (h <= 11000) { t = 288.15 - 0.0065 * h; p = 101325 * Math.pow(t / 288.15, 5.25588); }
  else { t = 216.65; p = 22632.06 * Math.exp(-G0 * (h - 11000) / (R_A * t)); }
  const T = t + (dT || 0);
  return { T, p, rho: p / (R_A * T), a: Math.sqrt(G_A * R_A * T) };
}

function nozzle(Tt, pt, g, cp, m, p0, eta) {
  if (pt <= p0 || m <= 0) return { V: 0, Fp: 0, Te: Tt, pe: p0 };
  const rg = cp * (g - 1) / g, crit = Math.pow((g + 1) / 2, g / (g - 1));
  if (pt / p0 >= crit) {
    const Te = 2 * Tt / (g + 1), pe = pt / crit, V = Math.sqrt(g * rg * Te) * 0.99;
    const Ae = m / (pe / (rg * Te) * V);
    return { V, Fp: (pe - p0) * Ae, Te, pe };
  }
  const Te = Tt * (1 - eta * (1 - Math.pow(p0 / pt, (g - 1) / g)));
  return { V: Math.sqrt(2 * cp * Math.max(0, Tt - Te)), Fp: 0, Te, pe: p0 };
}

// ---------- engine ----------
// P: design data from game.py. s: {w:[w1,w2,w3] rad/s, T4 K, mf kg/s, lit}. amb from isa().
function engineEval(P, s, amb, V0, ctl) {
  const ram = 1 + 0.2 * Math.pow(V0 / amb.a, 2);
  const T02 = amb.T * ram, p02 = amb.p * Math.pow(ram, 3.5) * P.eta_in;
  const w = s.w, n = [w[0] / P.w[0], w[1] / P.w[1], w[2] / P.w[2]];
  const T013 = T02 + P.psi_fan * Math.pow(w[0] * P.R[0], 2) / CP_A;
  const T025 = T013 + P.n_ipc * P.psi_comp * Math.pow(w[1] * P.R[1], 2) / CP_A;
  const T03 = T025 + P.n_hpc * P.psi_comp * Math.pow(w[2] * P.R[2], 2) / CP_A;
  const p013 = p02 * Math.pow(T013 / T02, P.eta_fan / KA);
  const p025 = p013 * Math.pow(T025 / T013, P.eta_comp / KA);
  const p03 = p025 * Math.pow(T03 / T025, P.eta_comp / KA);
  const p04 = p03 * (1 - P.dp_b);
  const sig = (p02 / T02) / (P.p02 / P.T02);                       // inlet density ratio
  const nc = Math.max(0, EC.coreMix * n[1] + (1 - EC.coreMix) * n[2]);
  const mb = P.mb * sig * n[0], mc = P.mc * sig * (EC.flowLin * nc + (1 - EC.flowLin) * Math.pow(nc, EC.flowExp)), mt = mb + mc;

  // fuel control: lever sets CORRECTED fuel flow, m_f / (delta sqrt(theta))
  // FADEC: the lever schedules corrected fuel flow m_f / (delta sqrt(theta)); acceleration and
  // deceleration limits hold the fuel-air ratio inside [fDec, fAcc(N3)]. Before light-off it meters a
  // fixed light-off flow, so fuel opened too early pools in the combustor (the hot-start trap).
  const corr = (p02 / 101325) * Math.sqrt(T02 / 288.15);
  const sched = corr * P.mfc * (EC.idleFrac + (1 - EC.idleFrac) * clamp(ctl.lever, 0, 1));
  const fA = Math.max(EC.fStart, P.fIdle + EC.fAccSlope * (n[2] - P.n3Idle) + EC.fAccMargin);
  const cmd = !ctl.fuelOn ? 0 : !s.lit ? EC.mfLight * corr : clamp(sched, EC.fDec * mc, fA * mc);
  const f = mc > 1e-6 ? s.mf / mc : (s.mf > 0 ? 99 : 0);
  const poolBurn = s.lit ? (s.pool || 0) / EC.tauPool : 0;              // pooled fuel from a late light-off
  const burn = s.lit ? Math.min(s.mf + poolBurn, EC.fStoich * mc) : 0;
  const fb = mc > 1e-6 ? burn / mc : 0;
  const cpg = CP_A + (CP_H - CP_A) * clamp(fb / P.f, 0, 1.5);        // gas cp rises with burnt fuel
  const mh = mc + burn;

  // turbines: expansion follows available pressure ratio, split HP+IP / LP as at design
  const pr04 = p04 / amb.p;
  const npr = 1 + (P.npr - 1) * clamp((pr04 - 1) / (P.pr04 - 1), 0, 2);
  const pit = Math.max(1, pr04 / npr), pict = Math.pow(pit, P.alpha), pilpt = pit / pict;
  const ex = KH * P.eta_t;
  const Tlp = s.T4 * Math.pow(pict, -ex), T05 = Tlp * Math.pow(pilpt, -ex), p05 = p04 / pit;
  const Pcore = mh * cpg * (s.T4 - Tlp) * P.eta_m, Plpt = mh * cpg * (Tlp - T05) * P.eta_m;
  const Phpt = P.phi * Pcore, Pipt = (1 - P.phi) * Pcore;
  const Pfan = mt * CP_A * (T013 - T02), Pipc = mc * CP_A * (T025 - T013), Phpc = mc * CP_A * (T03 - T025);
  const Pfr = [0, 1, 2].map(k => EC.fric[k] * w[k] * Math.min(1, w[k] / 5));
  const tqSt = ctl.starter ? EC.stT * Math.max(0, 1 - n[2] / EC.stFree) : 0;
  const net = [Plpt - Pfan - Pfr[0], Pipt - Pipc - Pfr[1], Phpt - Phpc - Pfr[2]];
  const dw = net.map((Pn, k) => Pn / (EC.J[k] * Math.max(w[k], EC.wMin)) + (k === 2 ? tqSt / EC.J[2] : 0));
  const dT4 = (mc * CP_A * T03 + burn * P.eta_b * P.lhv - mh * cpg * s.T4 - EC.hA * (s.T4 - amb.T)) / EC.C;

  const nh = nozzle(T05, p05, G_H, cpg, mh, amb.p, P.eta_n);
  const nb = nozzle(T013, p013, G_A, CP_A, mb, amb.p, P.eta_n);
  const Fc = mh * nh.V - mc * V0 + nh.Fp;
  const Fb = ctl.rev ? -EC.revEff * mb * nb.V - mb * V0 : mb * (nb.V - V0) + nb.Fp;
  return {
    n, T0: amb.T, p0: amb.p, T02, p02, T013, p013, T025, p025, T03, p03, T04: s.T4, p04, Tlp, plp: p04 / pict,
    T05, p05, T9: nh.Te, p9: nh.pe, mt, mc, mb, mf: s.mf, burn, f, cmd, F: Fc + Fb,
    Pst: tqSt * w[2], dE: [net[0], net[1], net[2] + tqSt * w[2], EC.C * dT4], dw, dT4, poolBurn,
  };
}

function engineStep(P, s, amb, V0, ctl, dt) {
  const o = engineEval(P, s, amb, V0, ctl);
  for (let k = 0; k < 3; k++) s.w[k] = Math.max(0, s.w[k] + dt * o.dw[k]);
  s.T4 += dt * o.dT4;
  s.mf += (o.cmd - s.mf) * Math.min(1, dt / EC.tauFuel);
  s.pool = Math.max(0, (s.pool || 0) + dt * (s.lit ? -o.poolBurn : (ctl.fuelOn ? s.mf : 0)));
  if (!ctl.fuelOn && !s.lit) s.pool = Math.max(0, s.pool - dt * s.pool / 5);   // drains overboard
  o.flameout = false;
  if (s.lit && (!ctl.fuelOn || o.f < EC.fLBO || o.mc < 0.3)) { s.lit = false; o.flameout = ctl.fuelOn; }
  else if (!s.lit && ctl.fuelOn && o.n[2] >= EC.lightN3 && o.mc >= 0.3 && o.f >= EC.fLight && o.f <= EC.fRich) { s.lit = true; o.lightoff = true; }
  return o;
}

// Ground-idle reference (ISA sea level) that anchors the FADEC acceleration schedule.
function prepare(P) {
  if (P.n3Idle) return P;
  P.n3Idle = 0.44; P.fIdle = 0.0155;
  const o = settle(P, isa(0, 0), 0, 0, 90).o;
  P.n3Idle = o.n[2]; P.fIdle = o.f;
  return P;
}

function settle(P, amb, V0, lever, secs) {
  const s = { w: P.w.slice(), T4: P.T4, mf: P.mf, lit: true, pool: 0 };
  const ctl = { lever, fuelOn: true, starter: false, rev: false };
  for (let k = 0; k < secs / 0.01; k++) engineStep(P, s, amb, V0, ctl, 0.01);
  return { s, o: engineEval(P, s, amb, V0, ctl) };
}

// ---------- mission ----------
const PHASES = [
  { id: 'start',    name: 'Engine start',         short: 'Start',    warp: 2,   cfg: 'to' },
  { id: 'taxi',     name: 'Taxi out',             short: 'Taxi',     warp: 1,   cfg: 'to' },
  { id: 'takeoff',  name: 'Take-off',             short: 'Take-off', warp: 1,   cfg: 'to' },
  { id: 'climb',    name: 'Climb',                short: 'Climb',    warp: 20,  cfg: 'clean' },
  { id: 'cruise',   name: 'Cruise',               short: 'Cruise',   warp: 30,  cfg: 'clean' },
  { id: 'descent',  name: 'Descent',              short: 'Descent',  warp: 20,  cfg: 'clean' },
  { id: 'approach', name: 'Approach & touchdown', short: 'Approach', warp: 1.5, cfg: 'land' },
  { id: 'rollout',  name: 'Landing roll',         short: 'Rollout',  warp: 1,   cfg: 'land' },
  { id: 'taxiin',   name: 'Taxi in',              short: 'Taxi in',  warp: 1,   cfg: 'clean' },
  { id: 'shutdown', name: 'Shutdown',             short: 'Shutdown', warp: 4,   cfg: 'clean' },
];
const LIM = { start: 750, takeoff: 900, mct: 850, over: 920 };   // start limit is a game value
const MAX_SCORE = 1100;     // 11 graded items x 100

class Mission {
  constructor(P, opt) {
    this.P = prepare(P);
    this.o = Object.assign({ dISA: 0, tow: 220e3, cruiseFt: 35000, runway: 3500, landRunway: 3000,
                             holdDist: 300, gateDist: 300, cruiseTime: 900, cooldown: 60 }, opt || {});
    const a = isa(0, this.o.dISA);
    this.idle = settle(P, a, 0, 0, 90).o;                 // ground-idle reference for this day
    this.e = { w: [0, 0, 0], T4: a.T, mf: 0, lit: false, pool: 0 };
    this.ac = { x: 0, h: 0, V: 0, vs: 0, m: this.o.tow, ground: true };
    this.ctl = { lever: 0, fuelOn: false, starter: false, brakes: false, rev: false, ff: false };
    this.t = 0; this.pi = 0; this.pt = 0; this.status = 'play'; this.failMsg = '';
    this.grades = []; this.pens = {}; this.events = []; this.log = []; this.marks = [{ t: 0, pi: 0 }];
    this.fuel = 0; this.acc = {}; this.flags = {}; this.warn = [];
    this.hc = this.o.cruiseFt * FT;
    this.out = engineEval(P, this.e, a, 0, this.ctl);
    this.air = { D: 0, Vs: 1, VR: 0, Vapp: 0, ias: 0, mach: 0 };
    this.enter(0);
    this.sample();
  }
  get phase() { return PHASES[this.pi]; }
  event(kind, msg) { this.events.push({ t: this.t, kind, msg }); }
  penalize(key, pts) { this.pens[key] = (this.pens[key] || 0) + pts; }
  grade(label, q, note) { q = Math.round(clamp(q, 0, 100)); this.grades.push({ label, q, note }); this.event('score', `${label}: +${q}` + (note ? ` (${note})` : '')); }
  fail(msg) { if (this.status !== 'play') return; this.status = 'fail'; this.failMsg = msg; this.event('fail', msg); }
  get penalty() { return Object.values(this.pens).reduce((a, b) => a + b, 0); }
  get score() { return Math.round(this.grades.reduce((a, g) => a + g.q, 0) - this.penalty); }
  get letter() {
    if (this.status === 'fail') return 'F';
    const s = this.score / MAX_SCORE;
    return s >= 0.9 ? 'S' : s >= 0.8 ? 'A' : s >= 0.65 ? 'B' : s >= 0.5 ? 'C' : 'D';
  }
  get warp() {
    let w = this.phase.warp;
    if (this.phase.id === 'shutdown' && !this.ctl.fuelOn) w = 8;
    return w * (this.ctl.ff ? 4 : 1);
  }
  enter(i) {
    this.pi = i; this.pt = 0; this.acc = { exceed: 0 };
    if (i > 0) { this.marks.push({ t: this.t, pi: i }); this.event('phase', PHASES[i].name); }
    const id = PHASES[i].id;
    if (id === 'takeoff') { this.ac.x = 0; this.ac.V = 0; this.flags.toT = this.t;
      this.event('info', `Lined up on a ${this.o.runway} m runway. Take-off rating: TGT limit 900 °C for 5 min.`); }
    if (id === 'approach') { this.flags.xthr = this.ac.x + this.ac.h / TAN3; this.acc.err2 = 0; this.acc.time = 0; }
    if (id === 'taxiin') this.acc.x0 = this.ac.x;
  }

  // limits & helpers
  tgtLimit() {
    const id = this.phase.id;
    if (id === 'start') return LIM.start;
    if (this.flags.toT !== undefined && this.t - this.flags.toT <= 300) return LIM.takeoff;   // 5-min take-off rating
    return LIM.mct;
  }
  toggleStarter() {
    if (this.status !== 'play') return;
    if (!this.ctl.starter && this.e.w[2] / this.P.w[2] >= EC.stCut) { this.event('info', 'Starter is locked out above 38% N3.'); return; }
    this.ctl.starter = !this.ctl.starter;
    this.event('info', this.ctl.starter ? 'Starter engaged.' : 'Starter off.');
  }
  toggleFuel() {
    if (this.status !== 'play') return;
    this.ctl.fuelOn = !this.ctl.fuelOn;
    this.event('info', this.ctl.fuelOn ? 'Fuel control switch RUN (igniters on).' : 'Fuel control switch CUTOFF.');
    if (this.ctl.fuelOn && this.phase.id === 'start' && this.ctl.lever > 0.05) this.event('warn', 'Thrust lever is not at idle for the start.');
  }
  setRev(on) {
    if (on && !(this.ac.ground && this.phase.id === 'rollout')) { if (on) this.event('info', 'Reverse thrust is only available on the landing roll.'); return; }
    if (this.ctl.rev !== on) { this.ctl.rev = on; this.event('info', on ? 'Reversers deployed.' : 'Reversers stowed.'); }
  }

  step(dtReal) {
    if (this.status !== 'play') return;
    const simDt = Math.min(0.1, Math.max(0, dtReal)) * this.warp;
    const nsub = Math.max(1, Math.ceil(simDt / 0.01)), h = simDt / nsub;
    for (let k = 0; k < nsub && this.status === 'play'; k++) this.sub(h);
    this.sample();
  }

  sub(h) {
    const P = this.P, ac = this.ac, ctl = this.ctl, id = this.phase.id;
    const amb = isa(ac.h, this.o.dISA);
    if (ctl.starter && this.e.w[2] / P.w[2] >= EC.stCut) { ctl.starter = false; this.event('info', 'Starter cut-out at 38% N3.'); }
    const o = engineStep(P, this.e, amb, ac.V, ctl, h);
    this.out = o;
    if (o.lightoff) this.event('info', 'Light-off: combustor lit.');
    if (o.flameout && !ac.ground) { this.event('warn', 'FLAMEOUT. Igniters are on: add fuel to relight.'); this.penalize('Flameout', 100); }
    this.fuel += AC.nEng * this.e.mf * h; ac.m -= AC.nEng * this.e.mf * h;
    this.t += h; this.pt += h;

    // ---------- aircraft ----------
    const cf = AC.cfg[this.phase.cfg], W = ac.m * G0, q = 0.5 * amb.rho * ac.V * ac.V;
    const T = AC.nEng * o.F;
    const Vs = Math.sqrt(2 * W / (amb.rho * AC.S * cf.CLmax));
    const air = this.air, rhoSL = isa(0, this.o.dISA).rho;          // reference speeds for this weight
    const VsTO = Math.sqrt(2 * W / (rhoSL * AC.S * AC.cfg.to.CLmax));
    air.VR = 1.1 * VsTO; air.V2 = 1.2 * VsTO;
    air.Vapp = 1.23 * Math.sqrt(2 * W / (rhoSL * AC.S * AC.cfg.land.CLmax)) + 5 * KT;
    air.Vs = Vs; air.ias = ac.V * Math.sqrt(amb.rho / 1.225) / KT; air.mach = ac.V / amb.a;
    if (ac.ground) {
      const parked = id === 'start' || id === 'shutdown';
      const L = q * AC.S * AC.CLg, N = Math.max(0, W - L);
      const brake = parked || ctl.brakes;
      const Fres = q * AC.S * (cf.CD0 + cf.K * AC.CLg * AC.CLg) + (AC.mu + (brake ? AC.muB : 0)) * N;
      const holding = parked || (ac.V < 0.05 && Math.abs(T) <= Fres);
      air.D = holding ? Math.min(Fres, Math.abs(T)) : Fres;      // static friction only balances the applied thrust
      if (holding) ac.V = 0;
      else ac.V = Math.max(0, ac.V + h * (T - Fres) / ac.m);
      ac.x += ac.V * h; ac.vs = 0;
      if (id === 'takeoff' && ac.V >= air.VR) { ac.ground = false; this.flags.liftoff = this.t; this.event('info', 'Rotate. Positive climb.'); }
    } else {
      const CL = W / Math.max(1, q * AC.S), D = q * AC.S * (cf.CD0 + cf.K * CL * CL);
      air.D = D;
      const Ps = (T - D) * ac.V / W;                     // specific excess power, m/s
      let vsCmd, dVdt;
      const speedOnPitch = (Vt, lo, hi) => {
        const want = clamp(0.25 * (Vt - ac.V), -0.6, 0.6);
        return clamp(Ps - ac.V / G0 * want, lo, hi);
      };
      if (id === 'takeoff' || id === 'climb') {
        const Vt = id === 'takeoff' ? air.V2 + 10 * KT : this.climbSpeed(amb);
        vsCmd = speedOnPitch(Vt, -3, Math.min(18, 0.12 * (this.hc - ac.h) + 0.5));
      } else if (id === 'cruise') {
        vsCmd = clamp(0.1 * (this.hc - ac.h), -6, 6);
      } else if (id === 'descent') {
        vsCmd = speedOnPitch(this.descentSpeed(amb, ac.h), Math.max(-20, 0.12 * (457 - ac.h) - 0.5), 0);
      } else {                                             // approach: 3 deg glideslope + flare
        const hg = TAN3 * (this.flags.xthr - ac.x);
        vsCmd = -ac.V * TAN3 + 0.25 * (hg - ac.h);
        if (ac.h < 15) vsCmd = Math.max(vsCmd, -(0.6 + 0.12 * ac.h));
      }
      ac.vs += clamp(vsCmd - ac.vs, -2 * h, 2 * h);      // vertical-speed changes limited to 0.2 g
      dVdt = G0 / ac.V * (Ps - ac.vs);
      ac.V = Math.max(1, ac.V + dVdt * h);
      ac.h += ac.vs * h; ac.x += ac.V * h;
      if (ac.V < Vs) return this.fail(`Stall: airspeed ${(air.ias).toFixed(0)} kt is below the stall speed. Not enough thrust for the flight path.`);
      if (ac.h <= 0) {
        if (id !== 'approach') return this.fail('The aircraft hit the ground. Not enough thrust to hold the flight path.');
        ac.h = 0; ac.ground = true;
        const sink = -ac.vs; ac.vs = 0;
        this.acc.sink = sink; this.acc.tdIas = air.ias;
        this.event('info', `Touchdown: sink rate ${(sink / FT * 60).toFixed(0)} fpm at ${air.ias.toFixed(0)} kt.`);
        if (sink > 4.5) return this.fail(`Hard landing: sink rate ${(sink / FT * 60).toFixed(0)} fpm. Landing gear failed.`);
        const rms = Math.sqrt(this.acc.err2 / Math.max(1, this.acc.time));
        this.grade('Approach speed', 100 * (1 - rms / (10 * KT)), `rms error ${(rms / KT).toFixed(1)} kt`);
        this.grade('Touchdown', 100 * (3 - sink) / 2, `${(sink / FT * 60).toFixed(0)} fpm`);
        return this.enter(7);
      }
    }
    this.checks(o, amb, h);
    if (this.status === 'play') this.phaseLogic(o, h);
  }

  climbSpeed(amb) {   // 250 kt IAS below FL100, 300 kt above, capped at M0.82
    const ias = (this.ac.h < 10000 * FT ? 250 : 300) * KT;
    return Math.min(ias / Math.sqrt(amb.rho / 1.225), 0.82 * amb.a);
  }
  descentSpeed(amb, hgt) {
    const ias = (hgt < 4000 * FT ? 180 : hgt < 11000 * FT ? 245 : 290) * KT;
    return Math.min(ias / Math.sqrt(amb.rho / 1.225), 0.82 * amb.a);
  }

  checks(o, amb, h) {
    const ac = this.ac, air = this.air, id = this.phase.id;
    const tgt = o.Tlp - 273.15, lim = this.tgtLimit();
    if (tgt > lim) {
      if (id === 'start') return this.fail(`Hot start: TGT reached ${tgt.toFixed(0)} °C, above the ${LIM.start} °C start limit. ` +
        'Fuel went in with too little airflow, and fuel that pooled before light-off flashed off at once. Wait for N3 ≥ 15% before fuel RUN and keep the lever at idle.');
      this.acc.exceed += (tgt - lim) * h;
      this.penalize('TGT above limit', 0.5 * (tgt - lim) * h);
    }
    if (tgt > LIM.over && !this.flags.over) { this.flags.over = true; this.penalize('Over-temperature events', 100); this.event('warn', `OVER-TEMPERATURE: TGT ${tgt.toFixed(0)} °C > ${LIM.over} °C.`); }
    if (tgt < LIM.takeoff - 20) this.flags.over = false;
    const nPct = [o.n[0] * this.P.w[0] / this.P.wref[0], o.n[2] * this.P.w[2] / this.P.wref[2]];
    if (Math.max(...nPct) > 1.02) this.penalize('Shaft overspeed', 10 * h);
    if (ac.ground && (id === 'taxi' || id === 'taxiin') && ac.V > 30 * KT) this.penalize('Taxi above 30 kt', 5 * h);
    if (!ac.ground && ac.h < 10000 * FT && air.ias > 260) this.penalize('Above 250 kt below 10,000 ft', 5 * h);
    if (!ac.ground && air.mach > 0.89) this.penalize('Mach overspeed (Mmo 0.89)', 10 * h);
    if (id === 'rollout' && this.ctl.rev && ac.V < 60 * KT && ac.V > 1) this.penalize('Reversers deployed below 60 kt', 3 * h);
    if (id === 'start' && this.ctl.fuelOn && !this.e.lit) {
      this.acc.wet = (this.acc.wet || 0) + h;
      if (this.acc.wet > 10) return this.fail('Wet start: fuel flowed for 10 s without light-off. Introduce fuel only once N3 is above 12% with the starter running.');
    }
    const warn = [];
    if (!ac.ground && ac.V < 1.1 * air.Vs) warn.push('STALL WARNING');
    if (tgt > lim) warn.push('TGT LIMIT');
    if (Math.max(...nPct) > 1.02) warn.push('OVERSPEED');
    if (!ac.ground && air.mach > 0.89) warn.push('MACH');
    if (this.ctl.fuelOn && !this.e.lit && !ac.ground) warn.push('FLAMEOUT');
    this.warn = warn;
  }

  phaseLogic(o, h) {
    const ac = this.ac, ctl = this.ctl, id = this.phase.id, acc = this.acc;
    const tgt = o.Tlp - 273.15;
    switch (id) {
      case 'start': {
        acc.peak = Math.max(acc.peak || -1e9, tgt);
        const n3 = o.n[2], idle3 = this.idle.n[2];
        const stable = Math.abs(o.dw[2] / this.P.w[2]) < 0.002;
        acc.stable = (this.e.lit && !ctl.starter && n3 > 0.97 * idle3 && stable) ? (acc.stable || 0) + h : 0;
        if (acc.stable > 2) {
          this.grade('Engine start', LIM.start - acc.peak, `peak TGT ${acc.peak.toFixed(0)} °C`);
          this.enter(1);
        }
        break;
      }
      case 'taxi': {
        const d = ac.x;
        if (d > this.o.holdDist + 60 && !this.flags.incursion) { this.flags.incursion = true; this.penalize('Runway incursion', 150); this.event('warn', 'RUNWAY INCURSION: you passed the holding point.'); }
        if (ac.V < 0.3 && d >= this.o.holdDist - 25) {
          this.grade('Taxi stop at holding point', 100 - 2 * Math.abs(d - this.o.holdDist), `${(d - this.o.holdDist).toFixed(0)} m from the line`);
          this.enter(2);
        }
        break;
      }
      case 'takeoff':
        if (ac.ground && ac.x > this.o.runway) return this.fail('Runway overrun: not enough thrust to reach rotation speed before the end of the runway.');
        if (ac.h >= 1500 * FT) { this.grade('Take-off', 100 - acc.exceed, acc.exceed > 0 ? `${acc.exceed.toFixed(0)} °C·s over limit` : 'within TGT limit'); this.enter(3); }
        break;
      case 'climb':
        acc.slow = (ac.h > 20000 * FT && ac.vs < 1.5) ? (acc.slow || 0) + h : 0;
        if (acc.slow > 120) {
          this.hc = Math.floor(ac.h / FT / 1000) * 1000 * FT;
          this.event('warn', `Climb rate below 300 fpm: too heavy for FL${(this.o.cruiseFt / 100).toFixed(0)}. Levelling at FL${(this.hc / FT / 100).toFixed(0)}.`);
        }
        if (ac.h >= this.hc - 10) { this.grade('Climb', 100 - acc.exceed / 2, acc.exceed > 0 ? `${acc.exceed.toFixed(0)} °C·s over limit` : 'within TGT limit'); this.enter(4); }
        break;
      case 'cruise': {
        const amb = isa(ac.h, this.o.dISA);
        acc.dev = (acc.dev || 0) + Math.abs(ac.V / amb.a - 0.85) * h;
        if (this.pt >= this.o.cruiseTime) {
          const avg = acc.dev / this.pt;
          this.grade('Cruise Mach hold', 100 * (1 - avg / 0.02), `mean |M - 0.85| = ${avg.toFixed(3)}`);
          this.enter(5);
        }
        break;
      }
      case 'descent':
        acc.lev = (acc.lev || 0) + ctl.lever * h;
        if (ac.h <= 1500 * FT + 20) {
          const avg = acc.lev / this.pt;
          this.grade('Idle descent', 100 * (1 - avg / 0.3), `average lever ${(avg * 100).toFixed(0)}%`);
          this.enter(6);
        }
        break;
      case 'approach':
        if (ac.h < 1000 * FT) { acc.err2 += Math.pow(ac.V - this.air.Vapp, 2) * h; acc.time += h; }   // stabilised by 1,000 ft
        break;
      case 'rollout': {
        const d = ac.x - this.flags.xthr;
        if (d > this.o.landRunway) return this.fail('Runway overrun on landing. Use reverse thrust (R) and brakes (B).');
        if (ac.V < 12 && !ctl.rev) {
          this.grade('Landing roll', 100 * (this.o.landRunway - d) / 1000, `${d.toFixed(0)} m past the threshold`);
          this.enter(8);
        }
        break;
      }
      case 'taxiin': {
        const d = ac.x - acc.x0, gate = this.o.gateDist;
        if (d > gate + 40 && !this.flags.stand) { this.flags.stand = true; this.penalize('Overran the gate', 100); this.event('warn', 'You rolled past the stand mark.'); }
        if (ac.V < 0.3 && d >= gate - 30) {
          this.grade('Park at gate', 100 - 4 * Math.abs(d - gate), `${(d - gate).toFixed(0)} m from the mark`);
          this.enter(9);
        }
        break;
      }
      case 'shutdown':
        if (this.e.lit && ctl.lever < 0.05) acc.cool = (acc.cool || 0) + h;
        if (!ctl.fuelOn && acc.cut === undefined) {
          acc.cut = acc.cool || 0;
          if (acc.cut < this.o.cooldown) { this.penalize('Shut down without cool-down', 50); this.event('warn', 'Fuel cut before the cool-down: thermal shock to the hot section.'); }
        }
        if (!ctl.fuelOn && o.n[2] < 0.05 && o.n[1] < 0.05) {
          this.grade('Shutdown', acc.cut >= this.o.cooldown ? 100 : 0, acc.cut >= this.o.cooldown ? 'cooled down first' : 'no cool-down');
          this.status = 'done'; this.event('phase', 'Mission complete');
        }
        break;
    }
  }

  objective() {
    const ac = this.ac, ctl = this.ctl, o = this.out, id = this.phase.id, KTs = v => (v / KT).toFixed(0);
    const n3 = (o.n[2] * 100).toFixed(0);
    switch (id) {
      case 'start':
        if (!ctl.fuelOn && !ctl.starter && o.n[2] < 0.12) return 'Press S to engage the air starter. It spins the HP spool (N3).';
        if (!ctl.fuelOn) return `Starter motoring: N3 ${n3}%. Press F (fuel RUN) once N3 is at least 15%. Too early means a hot start. Keep the lever at idle.`;
        if (!this.e.lit) return 'Fuel on, igniters firing: waiting for light-off.';
        if (ctl.starter) return `Light-off! The combustor is heating the gas. Watch TGT (start limit ${LIM.start} °C). Starter cuts out at 38% N3.`;
        return `Accelerating to ground idle (N3 ~${(this.idle.n[2] * 100).toFixed(0)}%).`;
      case 'taxi': return `Taxi to the runway holding point: ${(this.o.holdDist - ac.x).toFixed(0)} m to go. ` +
        'A touch of thrust gets you rolling, and idle thrust keeps you creeping. Keep under 30 kt, hold B to brake, and stop on the line.';
      case 'takeoff':
        if (ac.ground) return `Set take-off thrust (↑ or Home). Rotation is automatic at VR ${KTs(this.air.VR)} kt. ` +
          `Runway left: ${Math.max(0, this.o.runway - ac.x).toFixed(0)} m. TGT limit 900 °C.`;
        return 'Climbing to 1,500 ft. The autopilot flies the pitch; your thrust decides the climb rate.';
      case 'climb': return `Climb to FL${(this.o.cruiseFt / 100).toFixed(0)}. ` +
        `After 5 min at take-off thrust the limit drops to ${LIM.mct} °C, so throttle back to climb thrust. More thrust means a faster climb.`;
      case 'cruise': return `Hold Mach 0.85 ±0.01 with thrust for ${(this.o.cruiseTime / 60).toFixed(0)} min ` +
        `(${Math.max(0, (this.o.cruiseTime - this.pt) / 60).toFixed(1)} min left). The autopilot holds altitude, so thrust minus drag sets acceleration.`;
      case 'descent': return 'Descend to 1,500 ft. Idle thrust is the most efficient. ' +
        'The autopilot pitches for speed, so any thrust you add just makes the descent shallower. 250 kt limit below 10,000 ft.';
      case 'approach':
        return `Landing configuration. The autopilot flies the 3° glideslope. Hold Vapp ${KTs(this.air.Vapp)} kt with thrust. ` +
          `Height ${(ac.h / FT).toFixed(0)} ft, ${(Math.max(0, this.flags.xthr - ac.x) / 1000).toFixed(1)} km to the threshold.`;
      case 'rollout': return `Idle, hold R for reverse thrust and B for brakes. Stow the reversers (release R) below 60 kt. ` +
        `Runway remaining: ${Math.max(0, this.flags.xthr + this.o.landRunway - ac.x).toFixed(0)} m.`;
      case 'taxiin': return `Taxi to the gate: ${(this.acc.x0 + this.o.gateDist - ac.x).toFixed(0)} m to the stand mark. Stop on it (B to brake).`;
      case 'shutdown':
        if (ctl.fuelOn) {
          const c = this.acc.cool || 0;
          return c < this.o.cooldown ? `Parked. Cool-down at idle: ${(this.o.cooldown - c).toFixed(0)} s left, so the hot section cools evenly. Don't cut the fuel yet.`
                                     : 'Cool-down complete. Press F to cut off the fuel.';
        }
        return `Fuel off. Spooling down: N1 ${(o.n[0] * 100).toFixed(0)}%, N3 ${n3}%.`;
    }
    return '';
  }

  sample() {
    const o = this.out, P = this.P, ac = this.ac;
    this.log.push({
      t: this.t, pi: this.pi,
      n1: o.n[0] * P.w[0] / P.wref[0] * 100, n2: o.n[1] * P.w[1] / P.wref[1] * 100, n3: o.n[2] * P.w[2] / P.wref[2] * 100,
      F: o.F / 1e3, D: this.air.D / AC.nEng / 1e3, tgt: o.Tlp - 273.15, tet: o.T04 - 273.15, lim: this.tgtLimit(),
      lp: o.dE[0] / 1e6, ip: o.dE[1] / 1e6, hp: o.dE[2] / 1e6, cb: o.dE[3] / 1e6,
      alt: ac.h / FT, ias: this.air.ias, mach: this.air.mach, vs: ac.vs / FT * 60,
      lever: this.ctl.lever * 100, ff: o.mf, mt: o.mt, x: ac.x,
    });
  }
}

const API = { isa, nozzle, engineEval, engineStep, settle, prepare, Mission, PHASES, LIM, EC, AC, KT, FT, MAX_SCORE, CP_A, R_A };
if (typeof module !== 'undefined') module.exports = API; else root.Sim = API;
})(typeof window !== 'undefined' ? window : this);
