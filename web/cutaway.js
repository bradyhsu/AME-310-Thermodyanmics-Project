/* Animated turbofan cutaway on a <canvas>, shared by the Engine animation tab and the game.
   const cw = createCutaway(canvas, geo);  cw.draw(cur, dtReal, textLines)
   cur: {n1, n2, n3 (rpm), T013, T025, T03, T04, tgt, T05 (K), flow (fraction of design)} */
function createCutaway(cv, D, opts) {
  opts = opts || {};
  const ctx = cv.getContext('2d'), W = 900, H = 460, CY = 235;
  const dark = opts.dark !== undefined ? opts.dark : matchMedia('(prefers-color-scheme: dark)').matches;
  const FG = dark ? '#e6eaf0' : '#1b2330', LN = dark ? '#8a94a6' : '#5b6475';
  function fit() {
    const w = cv.clientWidth, d = devicePixelRatio || 1;
    if (!w) return;
    cv.width = w * d; cv.height = w * H / W * d; cv.style.height = (w * H / W) + 'px';
    ctx.setTransform(cv.width / W, 0, 0, cv.width / W, 0, 0);
  }
  fit(); addEventListener('resize', fit);
  // ---- geometry from the engine parameters ----
  const Rf = Math.min(150, 38 * D.fan_dia), Rd = Rf + 2, Rn = Rf + 14, Rs = .24 * Rf;
  const cs = Math.max(.2 * Rf, Math.min(.85 * Rf, .52 * Rf * Math.sqrt(10.3 / (1 + D.bypass))));   // core inlet radius shrinks as bypass grows
  const rEx = cs * Math.pow(D.pr_c, -.22), rIpc = cs * Math.pow(rEx / cs, D.n_ipc / (D.n_ipc + D.n_hpc));  // compressor narrows with pressure ratio
  const rL = Math.min(.9 * Rf, Math.max(rEx * 2.1, cs * .95) * (1 + .015 * D.n_lpt));
  let a0 = 150; const S = {}, seg = w => { const s = [a0, a0 + w]; a0 += w; return s; };
  S.ipc = seg(D.n_ipc * 12 + 10); S.hpc = seg(D.n_hpc * 11 + 10); S.comb = seg(60); S.hpt = seg(30); S.ipt = seg(D.n_ipt * 28); S.lpt = seg(D.n_lpt * 20 + 6); S.noz = seg(80);
  const sx = Math.min(1, 630 / (a0 - 150)), X = v => 150 + (v - 150) * sx, X0 = 150, XE = X(a0), XB = XE - 60;
  const RC = [[X0, cs], [X(S.ipc[1]), rIpc], [X(S.hpc[1]), rEx], [X(S.comb[1]), rEx * 1.35], [X(S.hpt[1]), rEx * 1.5], [X(S.ipt[1]), rEx * 1.7], [X(S.lpt[1]), rL], [XE, rL * .7]];
  function interp(P, x) { for (let k = 0; k < P.length - 1; k++) if (x <= P[k + 1][0]) { const u = (x - P[k][0]) / (P[k + 1][0] - P[k][0] || 1); return P[k][1] + (P[k + 1][1] - P[k][1]) * u; } return P[P.length - 1][1]; }
  const rc = x => interp(RC, x), rh = x => .38 * rc(x);
  const ST = [[0, [59, 130, 246]], [.3, [56, 189, 248]], [.55, [251, 191, 36]], [.8, [249, 115, 22]], [1, [239, 68, 68]]];
  function col(T, a) {
    const t = Math.max(0, Math.min(1, (T - 250) / 1750)); let k = 0; while (k < ST.length - 2 && t > ST[k + 1][0]) k++;
    const [t0, c0] = ST[k], [t1, c1] = ST[k + 1], u = (t - t0) / (t1 - t0); return `rgba(${c0.map((v, j) => Math.round(v + (c1[j] - v) * u)).join(',')},${a})`;
  }
  const ph = [0, 0, 0];
  function TX(c) { return [[X0, c.T013], [X(S.ipc[1]), c.T025], [X(S.hpc[1]), c.T03], [X(S.comb[0]) + 5, c.T03], [X(S.comb[1]), c.T04], [X(S.ipt[1]), c.tgt], [X(S.lpt[1]), c.T05], [XE, c.T05]]; }
  const P = []; for (let k = 0; k < 110; k++) P.push({ core: true, x: X0 + Math.random() * (XE - X0 + 80), r: Math.random(), s: Math.random() < .5 ? -1 : 1, v: .8 + Math.random() * .4 });
  for (let k = 0; k < 110; k++) P.push({ core: false, x: 110 + Math.random() * 770, r: Math.random(), s: Math.random() < .5 ? -1 : 1, v: .8 + Math.random() * .4 });
  function rowsX(a, b, n) { const o = []; for (let k = 0; k < n; k++) o.push(n == 1 ? (a + b) / 2 : a + (b - a) * k / (n - 1)); return o; }
  const rows = [];   // [x, spool 0=LP 1=IP 2=HP]
  rowsX(X(S.ipc[0] + 10), X(S.ipc[1] - 8), D.n_ipc).forEach(x => rows.push([x, 1]));
  rowsX(X(S.hpc[0] + 8), X(S.hpc[1] - 6), D.n_hpc).forEach(x => rows.push([x, 2]));
  rows.push([X(S.hpt[0] + 14), 2]); rowsX(X(S.ipt[0] + 12), X(S.ipt[1] - 12), D.n_ipt).forEach(x => rows.push([x, 1]));
  rowsX(X(S.lpt[0] + 10), X(S.lpt[1] - 10), D.n_lpt).forEach(x => rows.push([x, 0]));
  const lab = [['Fan', 108], ['IPC', X((S.ipc[0] + S.ipc[1]) / 2)], ['HPC', X((S.hpc[0] + S.hpc[1]) / 2)], ['Comb.', X((S.comb[0] + S.comb[1]) / 2)],
    ['HPT/IPT', X((S.hpt[0] + S.ipt[1]) / 2)], ['LPT', X((S.lpt[0] + S.lpt[1]) / 2)], ['Core nozzle', X((S.noz[0] + S.noz[1]) / 2)]];

  function draw(cur, dt, lines) {
    if (cv.width === 0 || Math.abs(cv.width - cv.clientWidth * (devicePixelRatio || 1)) > 2) fit();
    dt = Math.max(0, Math.min(.05, dt));
    const SLOW = .03, rp = [cur.n1, cur.n2, cur.n3]; for (let k = 0; k < 3; k++) ph[k] += rp[k] / 60 * 2 * Math.PI * SLOW * dt * 60;
    ctx.clearRect(0, 0, W, H); ctx.lineWidth = 1.2; ctx.strokeStyle = LN;
    ctx.fillStyle = col(cur.T013, .22); ctx.fillRect(110, CY - Rd, XB - 110, 2 * Rd);                                   // bypass duct
    const g = ctx.createLinearGradient(X0, 0, XE, 0), T = TX(cur); T.forEach(p => g.addColorStop(Math.max(0, Math.min(1, (p[0] - X0) / (XE - X0))), col(p[1], .55)));
    ctx.fillStyle = g; ctx.beginPath(); for (let x = X0; x <= XE; x += 8) ctx.lineTo(x, CY - rc(x)); for (let x = XE; x >= X0; x -= 8) ctx.lineTo(x, CY + rc(x)); ctx.fill();   // core gas
    const rE = rc(XE), pg = ctx.createLinearGradient(XE, 0, 890, 0); pg.addColorStop(0, col(cur.T05, .5 * Math.min(1, cur.flow * 3))); pg.addColorStop(1, col(cur.T05, 0));
    ctx.fillStyle = pg; ctx.fillRect(XE, CY - rE, 890 - XE, 2 * rE);
    const bg = ctx.createLinearGradient(XB, 0, 890, 0); bg.addColorStop(0, col(cur.T013, .3 * Math.min(1, cur.flow * 3))); bg.addColorStop(1, col(cur.T013, 0)); ctx.fillStyle = bg;
    const bh = Math.max(4, Rd * .85 - rE - 6); ctx.fillRect(XB, CY - Rd * .85, 890 - XB, bh); ctx.fillRect(XB, CY + rE + 6, 890 - XB, bh);
    ctx.fillStyle = dark ? '#2a3340' : '#cfd6e0';                                                                // hub + spinner
    ctx.beginPath(); for (let x = X0; x <= XE; x += 8) ctx.lineTo(x, CY - rh(x)); for (let x = XE; x >= X0; x -= 8) ctx.lineTo(x, CY + rh(x)); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(60, CY); ctx.quadraticCurveTo(84, CY - Rs, 108, CY - Rs); ctx.lineTo(X0, CY - rh(X0)); ctx.lineTo(X0, CY + rh(X0)); ctx.lineTo(108, CY + Rs); ctx.quadraticCurveTo(84, CY + Rs, 60, CY); ctx.fill(); ctx.stroke();
    for (const s of [-1, 1]) {
      ctx.beginPath(); for (let x = X0; x <= XE; x += 8) ctx.lineTo(x, CY + s * (rc(x) + 4)); ctx.stroke();                 // core cowl, nacelle
      ctx.beginPath(); ctx.moveTo(60, CY + s * Rn); ctx.quadraticCurveTo(46, CY + s * (Rn + Rd) / 2, 62, CY + s * (Rd - 8)); ctx.lineTo(110, CY + s * Rd); ctx.lineTo(XB, CY + s * Rd); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(60, CY + s * Rn); ctx.lineTo(XB - 20, CY + s * Rn); ctx.lineTo(XB + 10, CY + s * Rd); ctx.stroke();
    }
    ctx.strokeStyle = col(cur.T04, .9); ctx.lineWidth = 2; for (let k = 0; k < 5; k++) { const x = X(S.comb[0] + 8 + k * 11); for (const s of [-1, 1]) { ctx.beginPath(); ctx.moveTo(x, CY + s * (rh(x) + 2)); ctx.lineTo(x, CY + s * (rc(x) - 3)); ctx.stroke(); } }
    ctx.lineWidth = 1.2;
    for (const [x, sp] of rows) {
      const a = ph[sp]; ctx.strokeStyle = LN;
      for (const s of [-1, 1]) { ctx.beginPath(); ctx.moveTo(x + 9 * sx, CY + s * rh(x)); ctx.lineTo(x + 12 * sx, CY + s * rc(x)); ctx.stroke(); }
      for (let b = 0; b < 12; b++) {
        const th = a + b * Math.PI / 6, sn = Math.sin(th), al = .25 + .75 * Math.max(0, Math.cos(th));
        ctx.strokeStyle = dark ? `rgba(230,235,245,${al})` : `rgba(40,50,70,${al})`; ctx.beginPath(); ctx.moveTo(x, CY + rh(x) * sn); ctx.lineTo(x, CY + rc(x) * sn); ctx.stroke();
      }
    }
    for (let b = 0; b < 16; b++) {                                                                                     // fan
      const th = ph[0] + b * Math.PI / 8, sn = Math.sin(th), al = .3 + .7 * Math.max(0, Math.cos(th));
      ctx.strokeStyle = dark ? `rgba(230,235,245,${al})` : `rgba(40,50,70,${al})`; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(108, CY + Rs * sn); ctx.lineTo(108, CY + Rf * sn); ctx.stroke();
    }
    ctx.lineWidth = 1.2;
    const f = cur.flow;
    for (const p of P) {
      if (p.core) {
        p.x += dt * 60 * p.v * f * (1 + (cs - rc(Math.min(p.x, XE))) * .03 / Math.max(.3, cs / 62)) * 2.2; if (p.x > XE + 90) { p.x = X0; p.r = Math.random(); }
        const r = p.x < XE ? rh(p.x) + p.r * (rc(p.x) - rh(p.x)) : p.r * rE * .9, y = CY + p.s * r;
        ctx.fillStyle = col(interp(T, Math.min(p.x, XE)), .95 * Math.min(1, f * 4)); ctx.beginPath(); ctx.arc(p.x, y, 1.9, 0, 7); ctx.fill();
      } else {
        p.x += dt * 60 * p.v * f * 1.5 * (1 + .4 * Math.min(1, Math.max(0, cur.T013 - 250) / 150)); if (p.x > 880) { p.x = 110; p.r = Math.random(); }
        const lo = Math.max(rc(Math.min(p.x, XE)) + 8, Rs + 4), hi = Math.max(Rd - 4, lo + 2), y = CY + p.s * (lo + p.r * (hi - lo));
        ctx.fillStyle = col(cur.T013, .8 * Math.min(1, f * 4)); ctx.beginPath(); ctx.arc(p.x, y, 1.7, 0, 7); ctx.fill();
      }
    }
    ctx.fillStyle = FG; ctx.font = '13px system-ui';
    (lines || []).forEach((s, k) => ctx.fillText(s, 14, 20 + 18 * k));
    ctx.font = '11px system-ui'; ctx.textAlign = 'center'; lab.forEach(([n, x]) => ctx.fillText(n, x, CY + Rn + 18)); ctx.textAlign = 'left';
    const lg = ctx.createLinearGradient(14, 0, 214, 0); ST.forEach(s => lg.addColorStop(s[0], `rgb(${s[1].join(',')})`)); ctx.fillStyle = lg; ctx.fillRect(14, H - 14, 200, 8);
    ctx.fillStyle = FG; ctx.fillText('250 K', 14, H - 18); ctx.fillText('2000 K', 178, H - 18); ctx.fillText('Gas temperature (blade rotation shown in slow motion)', 226, H - 6);
  }
  return { draw };
}
