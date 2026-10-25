/* A recorded run, played back: every frame on the bus, every state and
   command keel published, every flag it raised, on one clock.

   The stage is the robot and keel's nodes, with what moves between them at
   the moment shown; the transport below plays, steps and zooms through the
   run. Tap anything to inspect it. Data: runs/<name>.js and
   runs/<name>-wire.js, written by keel-book. */
(() => {
  'use strict';
  const NAME = document.body.dataset.run;
  const run = (window.KEEL_RUNS || {})[NAME], wire = (window.KEEL_WIRE || {})[NAME];
  if (!run || !wire) return;
  const D = window.Damiao;
  const SVG = 'http://www.w3.org/2000/svg';
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  /* ─────────────── helpers ─────────────── */
  function svg(tag, attrs, parent) {
    const e = document.createElementNS(SVG, tag);
    for (const [k, v] of Object.entries(attrs || {})) if (v != null) e.setAttribute(k, v);
    if (parent) parent.append(e);
    return e;
  }
  function stext(parent, x, y, s, cls, anchor) {
    const t = svg('text', { x, y, class: cls || 't', 'text-anchor': anchor || null }, parent);
    t.textContent = s;
    return t;
  }
  function html(tag, attrs, parent, content) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs || {})) if (v != null) e.setAttribute(k, v);
    if (content != null) e.textContent = content;
    if (parent) parent.append(e);
    return e;
  }
  const $ = (id) => document.getElementById(id);
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const hex = (n, w) => '0x' + n.toString(16).toUpperCase().padStart(w || 2, '0');
  const signed = (x, d) => (x >= 0 ? '+' : '−') + Math.abs(x).toFixed(d);
  const dur = (x) => {
    const a = Math.abs(x);
    if (a >= 1e6) return (x / 1e6).toFixed(2) + ' s';
    if (a >= 1000) return (x / 1000).toFixed(a >= 1e4 ? 1 : 2) + ' ms';
    return x.toFixed(a < 10 ? 1 : 0) + ' µs';
  };
  const clock = (t) => {
    const s = t / 1e6;
    const [a, b] = s.toFixed(6).split('.');
    return `${a}.${b.slice(0, 3)} ${b.slice(3)} s`;
  };
  /* The last index with xs[i] <= t, or -1. */
  function before(xs, t) {
    let lo = 0, hi = xs.length - 1, r = -1;
    while (lo <= hi) { const m = (lo + hi) >> 1; if (xs[m] <= t) { r = m; lo = m + 1; } else hi = m - 1; }
    return r;
  }
  function along(pts, f) {
    let total = 0;
    const segs = [];
    for (let i = 1; i < pts.length; i++) { const l = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]); segs.push(l); total += l; }
    let d = clamp(f, 0, 1) * total;
    for (let i = 0; i < segs.length; i++) {
      if (d <= segs[i] || i === segs.length - 1) {
        const k = segs[i] ? d / segs[i] : 0;
        return [pts[i][0] + (pts[i + 1][0] - pts[i][0]) * k, pts[i][1] + (pts[i + 1][1] - pts[i][1]) * k];
      }
      d -= segs[i];
    }
    return pts[0];
  }
  const points = (pts) => pts.map((p) => p.join(',')).join(' ');

  /* ─────────────── the run, unpacked (times in µs from the first state) ─────────────── */
  const { motors, joints } = run;
  const J = motors.length;
  const U = wire.unit_us;
  const F = wire.frames, NF = F.id.length;
  const fReady = new Float64Array(NF), fStart = new Float64Array(NF), fEnd = new Float64Array(NF);
  const fAns = new Uint8Array(NF), fJ = new Int8Array(NF);
  {
    let acc = 0;
    for (let i = 0; i < NF; i++) {
      acc += F.ready[i];
      fReady[i] = acc * U;
      fStart[i] = (acc + F.wait[i]) * U;
      fEnd[i] = fStart[i] + F.dur[i] * U;
      fAns[i] = F.id[i] >= 0x10 ? 1 : 0;
      fJ[i] = motors.findIndex((m) => (fAns[i] ? m.answer : m.id) === F.id[i]);
    }
  }
  const payload = (i) => F.payloads[F.data[i]];
  const undelta = (xs) => { let a = 0; return Float64Array.from(xs, (d) => (a += d) * U); };
  const S = undelta(wire.states), C = undelta(wire.commands);
  const T = S[S.length - 1];
  // Per motor: its answers and its commands, in order.
  const answersOf = [...Array(J)].map(() => []), commandsOf = [...Array(J)].map(() => []);
  for (let i = 0; i < NF; i++) if (fJ[i] >= 0) (fAns[i] ? answersOf : commandsOf)[fJ[i]].push(i);
  const endsOf = (list) => Float64Array.from(list, (i) => fEnd[i]);
  const answerEnds = answersOf.map(endsOf), commandEnds = commandsOf.map(endsOf);
  // Per cycle: its command, and when keel-can wrote that command's frames.
  const NS = S.length;
  const cmdOf = new Float64Array(NS).fill(NaN), wFirst = new Float64Array(NS).fill(NaN), wLast = new Float64Array(NS).fill(NaN);
  {
    let c = 0;
    for (let k = 0; k < NS; k++) {
      while (c < C.length && C[c] < S[k]) c++;
      if (c < C.length && (k + 1 >= NS || C[c] < S[k + 1])) cmdOf[k] = C[c];
    }
    for (let k = 0; k < NS; k++) {
      if (isNaN(cmdOf[k])) continue;
      let i = Math.max(0, before(fEnd, cmdOf[k]) - 4);
      for (let n = 0; i < NF && n < 40; i++, n++) {
        if (fAns[i] || fReady[i] < cmdOf[k]) continue;
        if (fReady[i] > cmdOf[k] + 1500) break;
        if (isNaN(wFirst[k]) || fReady[i] < wFirst[k]) wFirst[k] = fReady[i];
        if (isNaN(wLast[k]) || fReady[i] > wLast[k]) wLast[k] = fReady[i];
      }
    }
  }
  // Traced cycles: the hops keel measured, by state index.
  const traced = new Map(run.cycles.map((c) => [c.seq, c]));
  const hopOf = (c, node, input) => c && c.hops.find((h) => h.node === node && h.input.startsWith(input));
  // keel's flags, per joint, as they changed.
  const flagT = [...Array(J)].map(() => []), flagV = [...Array(J)].map(() => []);
  for (const [t, j, f] of wire.flags) { flagT[j].push(t * 1e6); flagV[j].push(f); }
  const flagsAt = (j, t) => { const k = before(flagT[j], t); return k < 0 ? 0 : flagV[j][k]; };
  const logs = run.logs.map((l) => ({ t: l.t * 1e6, text: l.text, node: l.node }));
  // Moments worth a jump: a traced cycle, a joint going quiet, keel noticing.
  const marks = [];
  {
    const first = wire.flags.find((f) => f[2]);
    if (first) {
      const j = first[1], tf = first[0] * 1e6;
      // Back from the flag: the last answer in which the joint still pushed.
      const ends = answersOf[j];
      let k = before(answerEnds[j], tf);
      while (k > 0 && Math.abs(D.answer(payload(ends[k]), motors[j]).t) < 0.5) k--;
      const died = fEnd[ends[Math.min(k + 1, ends.length - 1)]];
      marks.push({ t: died - 300, label: `${short(j)} stops pushing`, cls: 'dock' });
      marks.push({ t: tf - 20, label: `keel flags ${short(j)}`, cls: 'fault' });
    }
  }
  function short(j) { return joints[j] === 'gripper' ? 'G' : 'J' + (j + 1); }
  const firstTraced = run.cycles.find((c) => c.t > 1.5) || run.cycles[0];
  // Every moment something starts or ends, for stepping.
  const events = (() => {
    const xs = [];
    for (let i = 0; i < NF; i++) xs.push(fReady[i], fStart[i], fEnd[i]);
    for (let k = 0; k < NS; k++) { xs.push(S[k]); if (!isNaN(cmdOf[k])) xs.push(cmdOf[k]); }
    for (const c of run.cycles) {
      const s0 = S[c.seq];
      for (const h of c.hops) xs.push(s0 + h.taken, s0 + h.released);
    }
    // One frame often ends as the next starts: keep each moment once.
    const a = Float64Array.from(xs).sort();
    return a.filter((v, i) => i === 0 || v - a[i - 1] > 0.05);
  })();

  /* ─────────────── what is happening at t ─────────────── */
  function at(t) {
    const k = before(S, t);
    const cyc = traced.get(k);
    const s0 = k >= 0 ? S[k] : 0;
    // Frames on the wire and waiting.
    let i = Math.max(0, before(fEnd, t) - 2);
    const onWire = [], waitingCmd = [], waitingAns = [];
    for (let n = 0; i < NF && n < 48; i++, n++) {
      if (fStart[i] > t + 2500) break;
      if (fStart[i] <= t && t < fEnd[i]) onWire.push(i);
      else if (fReady[i] <= t && t < fStart[i]) (fAns[i] ? waitingAns : waitingCmd).push(i);
    }
    // The state and the command of this cycle, in flight or at work.
    const msgs = [];
    let ctrlBusy = false, armBusy = false, recBusy = false;
    if (k >= 0) {
      const ck = cmdOf[k], wf = wFirst[k];
      if (cyc) {
        const ctrl = hopOf(cyc, 'controller', 'state'), arm = hopOf(cyc, 'arm', 'command');
        const recS = hopOf(cyc, 'recorder', 'state'), recC = hopOf(cyc, 'recorder', 'command');
        const fly = (h, kind) => { if (h && s0 + h.published <= t && t < s0 + h.taken) msgs.push({ kind, to: h.node, f: (t - s0 - h.published) / Math.max(0.5, h.taken - h.published), k }); };
        fly(ctrl, 'state'); fly(recS, 'state'); fly(arm, 'command'); fly(recC, 'command');
        const work = (h) => h && s0 + h.taken <= t && t < s0 + h.released + 0.5;
        ctrlBusy = work(ctrl); recBusy = work(recS) || work(recC);
        armBusy = work(arm) || (!isNaN(wf) && t >= s0 + (arm ? arm.released : 0) && t <= wLast[k] + 1);
      } else {
        // Not traced: what's known is when the state and the command were
        // published, and when the command's first frame was written.
        if (!isNaN(ck) && t < ck) msgs.push({ kind: 'state', to: 'controller', f: (t - s0) / Math.max(1, ck - s0), k, untraced: true });
        if (!isNaN(ck) && !isNaN(wf) && ck <= t && t < wf) msgs.push({ kind: 'command', to: 'arm', f: (t - ck) / Math.max(1, wf - ck), k, untraced: true });
        armBusy = !isNaN(wf) && t >= wf && t <= wLast[k] + 1;
      }
      if (t - s0 < 2) armBusy = true;
    }
    return { k, cyc, s0, onWire, waitingCmd, waitingAns, msgs, ctrlBusy, armBusy, recBusy };
  }
  const quiet = (a) => !a.onWire.length && !a.waitingCmd.length && !a.waitingAns.length && !a.msgs.length && !a.ctrlBusy && !a.armBusy && !a.recBusy;

  /* ─────────────── page ─────────────── */
  const stageHost = $('stage'), overlay = $('stage-over'), nowLine = $('now');
  const inspector = $('inspector'), inspBody = $('insp-body');
  const lanes = $('lanes'), overview = $('overview');
  const playBtn = $('play'), backBtn = $('back'), fwdBtn = $('fwd'), timeOut = $('time'), skipBox = $('skip');
  const speedBtns = [...document.querySelectorAll('#speed button')];
  const SPEEDS = speedBtns.map((b) => +b.dataset.us);

  let t = S[firstTraced.seq] + 330, playing = false, speed = 0, sel = null, layoutCache = null, last = 0, toast = null;
  try { const v = +localStorage.getItem('keel-run-speed'); if (v >= 0 && v < SPEEDS.length) speed = v; } catch (e) {}

  /* The stage: the robot, the bus, keel's nodes. */
  function layout(W) {
    // drydock's panel: the arm in 3D, over a row of its eight motors.
    const CHIP_H = 34, GAP = 5;
    const chips = (dock) => {
      const w = (dock.w - 20 - GAP * (J - 1)) / J;
      return motors.map((_, j) => ({ x: dock.x + 10 + j * (w + GAP), y: dock.y + dock.h - CHIP_H - 10, w, h: CHIP_H }));
    };
    if (W >= 760) {
      const H = clamp(Math.round(W * 0.42), 380, 470);
      const x = (f) => Math.round(f * W);
      const dock = { x: 8, y: 16, w: x(0.38), h: H - 32 };
      const tiles = chips(dock);
      const view = { x: dock.x + 4, y: dock.y + 46, w: dock.w - 8, h: dock.h - 46 - CHIP_H - 16 };
      const cy = Math.round(H * 0.42);
      const arm = { x: x(0.5), y: cy - 42, w: x(0.145), h: 84 };
      const controller = { x: x(0.815), y: cy - 42, w: W - 8 - x(0.815), h: 84 };
      const recorder = { x: x(0.66), y: H - 92, w: x(0.14), h: 54 };
      return {
        W, H, dock, tiles, view, arm, controller, recorder, wide: true,
        bus: [[arm.x, cy], [dock.x + dock.w, cy]],
        state: [[arm.x + arm.w, cy - 18], [controller.x, cy - 18]],
        command: [[controller.x, cy + 18], [arm.x + arm.w, cy + 18]],
        recState: [[arm.x + arm.w / 2, arm.y + arm.h], [arm.x + arm.w / 2, recorder.y + recorder.h / 2], [recorder.x, recorder.y + recorder.h / 2]],
        recCommand: [[controller.x + controller.w / 2, controller.y + controller.h], [controller.x + controller.w / 2, recorder.y + recorder.h / 2], [recorder.x + recorder.w, recorder.y + recorder.h / 2]],
      };
    }
    const viewH = Math.round(Math.min(320, W * 0.66));
    const dock = { x: 8, y: 8, w: W - 16, h: 44 + viewH + CHIP_H + 16 };
    const tiles = chips(dock);
    const view = { x: dock.x + 4, y: dock.y + 44, w: dock.w - 8, h: viewH };
    const y0 = dock.y + dock.h + 70;
    const arm = { x: 8, y: y0, w: Math.round(W * 0.36), h: 66 };
    const controller = { x: W - 8 - Math.round(W * 0.36), y: y0, w: Math.round(W * 0.36), h: 66 };
    const recorder = { x: Math.round(W * 0.3), y: y0 + 112, w: Math.round(W * 0.4), h: 46 };
    const bx = arm.x + arm.w / 2;
    return {
      W, H: recorder.y + recorder.h + 10, dock, tiles, view, arm, controller, recorder, wide: false,
      bus: [[bx, arm.y], [bx, dock.y + dock.h]],
      state: [[arm.x + arm.w, y0 + 22], [controller.x, y0 + 22]],
      command: [[controller.x, y0 + 46], [arm.x + arm.w, y0 + 46]],
      recState: [[arm.x + arm.w / 2, arm.y + arm.h], [arm.x + arm.w / 2, recorder.y + recorder.h / 2], [recorder.x, recorder.y + recorder.h / 2]],
      recCommand: [[controller.x + controller.w / 2, controller.y + controller.h], [controller.x + controller.w / 2, recorder.y + recorder.h / 2], [recorder.x + recorder.w, recorder.y + recorder.h / 2]],
    };
  }

  let stage;
  /* The 3D view: a canvas laid over drydock's panel, where the layout says. */
  function placeView(L) {
    const box = $('stage-3d');
    if (!box) return;
    const k = stageHost.clientWidth / L.W;
    Object.assign(box.style, { left: L.view.x * k + 'px', top: L.view.y * k + 'px', width: L.view.w * k + 'px', height: L.view.h * k + 'px' });
  }
  // The arm in 3D, from the robot's model (models/<name>.js), when there's one.
  const model = (window.KEEL_MODELS || {})[document.body.dataset.model];
  const arm3d = model && window.Arm3D && $('arm3d') ? window.Arm3D.create($('arm3d'), model) : null;
  if (!arm3d && $('stage-3d')) $('stage-3d').hidden = true;
  function buildStage() {
    const W = Math.max(300, stageHost.clientWidth);
    const L = layout(W);
    const s = svg('svg', { viewBox: `0 0 ${W} ${L.H}`, width: W, height: L.H, role: 'img', class: 'stage-svg',
      'aria-label': 'OpenArm’s eight motors in drydock, the CAN bus, and keel’s nodes keel-can, the controller and the recorder, with what is moving between them at the moment shown.' });
    const head = svg('marker', { id: 'head', viewBox: '0 0 8 8', refX: 7, refY: 4, markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse' }, svg('defs', null, s));
    svg('path', { d: 'M0,0 L8,4 L0,8 z', class: 'r-head' }, head);
    const pick = (node, what) => {
      node.classList.add('r-pick');
      node.setAttribute('tabindex', '0');
      node.setAttribute('role', 'button');
      node.setAttribute('aria-label',
        what.kind === 'motor' ? `${short(what.j)}, ${joints[what.j]}`
        : what.kind === 'node' ? 'node ' + what.name
        : what.kind === 'message' ? (what.which === 'state' ? 'arm/state' : 'controller/command')
        : what.kind === 'bus' ? 'vcan0, the bus' : 'drydock');
      node.addEventListener('click', (e) => { e.stopPropagation(); select(what); });
      node.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); select(what); } });
    };
    // drydock
    const dock = svg('rect', { x: L.dock.x, y: L.dock.y, width: L.dock.w, height: L.dock.h, rx: 6, class: 'r-dock' }, s);
    pick(dock, { kind: 'dock' });
    stext(s, L.dock.x + 14, L.dock.y + 24, 'drydock', 't r-name');
    stext(s, L.dock.x + 14, L.dock.y + 40, 'OpenArm as its motors report it · drag to turn', 't r-sub');
    const tiles = L.tiles.map((b, j) => {
      const g = svg('g', { class: 'r-tile' }, s);
      const box = svg('rect', { x: b.x, y: b.y, width: b.w, height: b.h, rx: 6, class: 'r-tile-box' }, g);
      stext(g, b.x + b.w / 2, b.y + 15, short(j), 't r-tile-name', 'middle');
      // Torque: how hard it pushes, of its peak.
      const barY = b.y + b.h - 10, barX = b.x + 5, barW = b.w - 10;
      svg('rect', { x: barX, y: barY, width: barW, height: 4, rx: 6, class: 'r-bar-bg' }, g);
      const bar = svg('rect', { x: barX + barW / 2, y: barY, width: 0, height: 4, rx: 6, class: 'r-bar' }, g);
      pick(box, { kind: 'motor', j });
      return { g, box, bar, barX, barW };
    });
    placeView(L);
    // the bus
    const bus = svg('polyline', { points: points(L.bus), class: 'r-bus' }, s);
    const busHit = svg('polyline', { points: points(L.bus), class: 'r-hit' }, s);
    pick(busHit, { kind: 'bus' });
    const bm = [(L.bus[0][0] + L.bus[1][0]) / 2, (L.bus[0][1] + L.bus[1][1]) / 2];
    stext(s, L.wide ? bm[0] : bm[0] + 10, L.wide ? bm[1] + 26 : bm[1] + 4, 'vcan0 · CAN FD', 't r-sub', L.wide ? 'middle' : 'start');
    // keel's nodes
    const node = (b, name, id, what) => {
      const r = svg('rect', { x: b.x, y: b.y, width: b.w, height: b.h, rx: 6, class: 'r-node' }, s);
      stext(s, b.x + 12, b.y + 24, name, 't r-name');
      stext(s, b.x + 12, b.y + 40, id, 't r-sub');
      pick(r, what);
      return r;
    };
    const armBox = node(L.arm, 'keel-can', 'node arm', { kind: 'node', name: 'arm' });
    const ctrlBox = node(L.controller, 'openarm-wave', 'node controller', { kind: 'node', name: 'controller' });
    const recBox = node(L.recorder, 'keel-recorder', 'node recorder', { kind: 'node', name: 'recorder' });
    const edge = (pts, cls, label, lx, ly, what) => {
      svg('polyline', { points: points(pts), class: cls, 'marker-end': 'url(#head)' }, s);
      const hit = svg('polyline', { points: points(pts), class: 'r-hit' }, s);
      if (label) stext(s, lx, ly, label, 't r-edge-t', 'middle');
      pick(hit, what);
    };
    const mid = (p) => [(p[0][0] + p[1][0]) / 2, (p[0][1] + p[1][1]) / 2];
    const sm = mid(L.state), cm = mid(L.command);
    edge(L.state, 'r-edge', L.wide ? 'arm/state' : 'state', sm[0], sm[1] - 8, { kind: 'message', which: 'state' });
    edge(L.command, 'r-edge', L.wide ? 'controller/command' : 'command', cm[0], cm[1] + 17, { kind: 'message', which: 'command' });
    edge(L.recState, 'r-edge quiet', null, 0, 0, { kind: 'node', name: 'recorder' });
    edge(L.recCommand, 'r-edge quiet', null, 0, 0, { kind: 'node', name: 'recorder' });
    const live = svg('g', { class: 'r-live' }, s);
    stageHost.replaceChildren(s);
    stage = { L, s, tiles, bus, armBox, ctrlBox, recBox, live };
  }

  function drawStage(a) {
    const { L, tiles, armBox, ctrlBox, recBox, live } = stage;
    live.replaceChildren();
    // Motors: where each says it is (its last answer), where it was told
    // (its last command), how hard it pushes, and what keel flags.
    const reported = new Map(), told = new Map(), flaggedIds = [];
    tiles.forEach((tl, j) => {
      const ai = before(answerEnds[j], t), ci = before(commandEnds[j], t);
      const m = motors[j];
      const ans = ai >= 0 ? D.answer(payload(answersOf[j][ai]), m) : null;
      const cmd = ci >= 0 ? D.command(payload(commandsOf[j][ci]), m) : null;
      if (ans) reported.set(m.id, ans.p);
      if (cmd) told.set(m.id, cmd.p);
      if (flagsAt(j, t)) flaggedIds.push(m.id);
      const f = ans ? clamp(ans.t / m.t, -1, 1) : 0;
      const half = tl.barW / 2;
      tl.bar.setAttribute('x', f < 0 ? tl.barX + half + f * half : tl.barX + half);
      tl.bar.setAttribute('width', Math.abs(f) * half);
      const flagged = flagsAt(j, t);
      tl.g.classList.toggle('bad', !!flagged);
      // Lit from its command landing until its answer leaves.
      let lit = false;
      if (ci >= 0) {
        const cEnd = commandEnds[j][ci];
        const nextA = answersOf[j][ai + 1];
        lit = t >= cEnd && (nextA === undefined || t < fEnd[nextA]) && t - cEnd < 1500;
      }
      tl.g.classList.toggle('lit', lit);
      tl.g.classList.toggle('chosen', !!sel && sel.kind === 'motor' && sel.j === j);
    });
    if (arm3d) arm3d.set(reported, told.size ? told : null, flaggedIds);
    // Frames: across the bus, and queued at either end.
    const horizontal = L.bus[0][1] === L.bus[1][1];
    const queue = (list, end, dir, cls) => list.forEach((i, n) => {
      const [x, y] = end;
      const qx = horizontal ? x + dir * (10 + n * 9) : x + 14 + n * 9;
      const qy = horizontal ? y - 16 : y + dir * 10;
      const r = svg('rect', { x: qx - 3.5, y: qy - 3.5, width: 7, height: 7, rx: 6, class: 'r-queue ' + cls }, live);
      r.addEventListener('click', (e) => { e.stopPropagation(); select({ kind: 'frame', i }); });
    });
    queue(a.waitingCmd, L.bus[0], -1, 'cmd');
    queue(a.waitingAns, L.bus[1], 1, 'ans');
    for (const i of a.onWire) {
      const k = (t - fStart[i]) / (fEnd[i] - fStart[i]);
      const [x, y] = along(fAns[i] ? [L.bus[1], L.bus[0]] : L.bus, 0.1 + k * 0.8);
      const g = svg('g', { class: 'r-packet ' + (fAns[i] ? 'ans' : 'cmd') + (sel && sel.kind === 'frame' && sel.i === i ? ' chosen' : '') }, live);
      svg('rect', { x: x - 20, y: y - 10, width: 40, height: 20, rx: 6 }, g);
      stext(g, x, y + 4, hex(F.id[i]), 't', 'middle');
      g.addEventListener('click', (e) => { e.stopPropagation(); select({ kind: 'frame', i }); });
    }
    // Messages: in shared memory; what travels is the word that one is there.
    for (const m of a.msgs) {
      const path = m.kind === 'state' ? (m.to === 'controller' ? L.state : L.recState) : (m.to === 'arm' ? L.command : L.recCommand);
      const [x, y] = along(path, m.f);
      const g = svg('g', { class: 'r-msg ' + m.kind + (m.to === 'recorder' ? ' small' : '') }, live);
      if (m.to === 'recorder') svg('circle', { cx: x, cy: y, r: 5 }, g);
      else {
        const w = m.kind === 'state' ? 46 : 64;
        svg('rect', { x: x - w / 2, y: y - 9, width: w, height: 18, rx: 9 }, g);
        stext(g, x, y + 4, m.kind, 't', 'middle');
      }
      g.addEventListener('click', (e) => { e.stopPropagation(); select({ kind: 'message', which: m.kind, k: m.k }); });
    }
    armBox.classList.toggle('active', a.armBusy);
    ctrlBox.classList.toggle('active', a.ctrlBusy);
    recBox.classList.toggle('active', a.recBusy);
    for (const [b, name] of [[armBox, 'arm'], [ctrlBox, 'controller'], [recBox, 'recorder']]) b.classList.toggle('chosen', !!sel && sel.kind === 'node' && sel.name === name);
  }

  /* A log line, said near keel-can as playback passes it. */
  let lastLog = -1;
  function drawToast() {
    const k = before(logs.map((l) => l.t), t);
    if (k === lastLog) return;
    lastLog = k;
    overlay.replaceChildren();
    if (k < 0 || t - logs[k].t > 1.5e6) return;
    const L = stage.L;
    const b = html('div', { class: 'toast' + (/gives|not following|frozen|behind/.test(logs[k].text) ? ' bad' : '') }, overlay);
    html('span', { class: 'mono' }, b, `[arm] ${(logs[k].t / 1e6).toFixed(1)}s`);
    html('span', null, b, logs[k].text);
    const sx = stageHost.clientWidth / L.W;
    b.style.left = Math.max(8, (L.wide ? L.arm.x - 20 : 8) * sx) + 'px';
    b.style.top = (L.wide ? L.arm.y - 72 : L.arm.y - 62) * sx + 'px';
  }

  /* ─────────────── the transport: lanes, overview ─────────────── */
  const colors = {};
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of ['accent', 'accent-soft', 'dock', 'dock-soft', 'wait', 'fault', 'ink', 'muted', 'rule', 'grid', 'paper', 'panel']) colors[k] = cs.getPropertyValue('--' + k).trim();
  }
  function canvasFor(c) {
    const dpr = window.devicePixelRatio || 1;
    const w = c.clientWidth, h = c.clientHeight;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
    const g = c.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    return { g, w, h };
  }
  const LANE_LEFT = () => (lanes.clientWidth < 560 ? 76 : 84);
  let laneHits = [];
  function windowSpan() { return SPEEDS[speed] * 6; }   // six seconds of screen
  function drawLanes() {
    const { g, w, h } = canvasFor(lanes);
    const left = LANE_LEFT(), right = 8;
    const span = windowSpan();
    const t0 = t - span * 0.3, t1 = t0 + span;
    const x = (v) => left + ((v - t0) / span) * (w - left - right);
    const names = ['keel-can', 'controller', 'vcan0'];
    const laneH = (h - 18) / names.length;
    g.font = '11px ' + getComputedStyle(document.body).getPropertyValue('--mono');
    g.textBaseline = 'middle';
    names.forEach((n, i) => {
      g.fillStyle = colors.paper;
      g.fillRect(left, i * laneH + 2, w - left - right, laneH - 4);
      g.fillStyle = colors.muted;
      g.textAlign = 'right';
      g.fillText(n, left - 8, i * laneH + laneH / 2);
    });
    // Grid and labels: a round step.
    const step = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1e3, 2e3, 5e3, 1e4, 2e4, 5e4, 1e5, 2e5, 5e5, 1e6, 2e6].find((s) => span / s <= (w < 560 ? 5 : 9));
    g.textAlign = 'center';
    for (let v = Math.ceil(t0 / step) * step; v <= t1; v += step) {
      const xx = x(v);
      if (xx < left) continue;
      g.fillStyle = colors.grid;
      g.fillRect(xx, 2, 1, names.length * laneH - 4);
      g.fillStyle = colors.muted;
      g.fillText(step >= 1e6 ? (v / 1e6).toFixed(0) + ' s' : step >= 1e3 ? (v / 1e6).toFixed(step >= 1e5 ? 1 : 3) + ' s' : (v / 1e6).toFixed(4) + ' s', xx, h - 8);
    }
    g.save();
    g.beginPath(); g.rect(left, 0, w - left - right, h); g.clip();
    laneHits = [];
    const y = (i) => i * laneH + 2, lh = laneH - 4;
    const detailed = span <= 60000;
    // keel-can: a tick at each state it publishes; its writing of the command.
    const k0 = Math.max(0, before(S, t0)), k1 = before(S, t1);
    const every = Math.max(1, Math.ceil((k1 - k0) / ((w - left) / 3)));
    for (let k = k0; k <= k1; k += every) {
      g.fillStyle = colors.ink;
      g.fillRect(x(S[k]) - 0.5, y(0) + 3, 1.5, lh - 6);
      if (detailed && !isNaN(wFirst[k])) {
        g.fillStyle = colors.accent;
        g.fillRect(x(wFirst[k]), y(0) + lh * 0.3, Math.max(1.5, x(wLast[k]) - x(wFirst[k])), lh * 0.4);
      }
      // The controller: from the state to its command, or as traced.
      const cyc = traced.get(k);
      if (detailed && !isNaN(cmdOf[k])) {
        const ctrl = hopOf(cyc, 'controller', 'state');
        if (ctrl) {
          g.fillStyle = colors.muted;
          g.fillRect(x(S[k]), y(1) + lh / 2 - 0.5, x(S[k] + ctrl.taken) - x(S[k]), 1);
          g.fillStyle = colors.accent;
          g.fillRect(x(S[k] + ctrl.taken), y(1) + 4, Math.max(2, x(S[k] + ctrl.released) - x(S[k] + ctrl.taken)), lh - 8);
        } else {
          g.fillStyle = colors['accent-soft'];
          g.fillRect(x(S[k]), y(1) + 4, Math.max(2, x(cmdOf[k]) - x(S[k])), lh - 8);
        }
        laneHits.push({ x0: x(S[k]), x1: Math.max(x(S[k]) + 6, x(cmdOf[k])), lane: 1, what: { kind: 'message', which: 'state', k } });
      }
    }
    // The bus: each frame, or how busy it was.
    if (detailed) {
      let i = Math.max(0, before(fEnd, t0));
      for (; i < NF && fStart[i] <= t1 + 3000; i++) {
        if (fJ[i] < 0) continue;
        const xs = x(fStart[i]), xe = x(fEnd[i]);
        if (fStart[i] - fReady[i] > 0.5) {
          g.fillStyle = colors.wait;
          g.fillRect(x(fReady[i]), y(2) + 2 + (fAns[i] ? 3 : 0), Math.max(1, xs - x(fReady[i])), 1.5);
        }
        g.fillStyle = fAns[i] ? colors.dock : colors.accent;
        g.fillRect(xs, y(2) + 8, Math.max(1, xe - xs - (xe - xs > 4 ? 1 : 0)), lh - 12);
        if (sel && sel.kind === 'frame' && sel.i === i) { g.strokeStyle = colors.ink; g.lineWidth = 2; g.strokeRect(xs, y(2) + 7, xe - xs, lh - 10); }
        if (xe - xs > 30) { g.fillStyle = colors.paper; g.textAlign = 'center'; g.fillText(hex(F.id[i]), (xs + xe) / 2, y(2) + 8 + (lh - 12) / 2); }
        laneHits.push({ x0: xs, x1: Math.max(xe, xs + 3), lane: 2, what: { kind: 'frame', i } });
      }
    } else {
      // Busy share per pixel column.
      const cols = Math.ceil(w - left - right);
      const busy = new Float32Array(cols);
      const per = span / cols;
      let i = Math.max(0, before(fEnd, t0));
      for (; i < NF && fStart[i] <= t1; i++) {
        const c0 = Math.floor((fStart[i] - t0) / per), c1 = Math.floor((fEnd[i] - t0) / per);
        for (let c = Math.max(0, c0); c <= Math.min(cols - 1, c1); c++) busy[c] += (Math.min(fEnd[i], t0 + (c + 1) * per) - Math.max(fStart[i], t0 + c * per)) / per;
      }
      g.fillStyle = colors.accent;
      for (let c = 0; c < cols; c++) {
        const b = clamp(busy[c], 0, 1);
        g.globalAlpha = 0.25 + 0.75 * b;
        const bh = (lh - 8) * b;
        g.fillRect(left + c, y(2) + 4 + (lh - 8 - bh), 1, bh);
      }
      g.globalAlpha = 1;
      g.fillStyle = colors.muted;
      g.textAlign = 'left';
      g.fillText('how busy the bus is', left + 6, y(2) + 9);
      // Flags, as keel raised them, on the controller's lane.
      for (let j = 0; j < J; j++) for (let k = 0; k < flagT[j].length; k++) {
        if (!flagV[j][k]) continue;
        const a = flagT[j][k], b = k + 1 < flagT[j].length ? flagT[j][k + 1] : T;
        g.fillStyle = colors.fault;
        g.fillRect(x(a), y(1) + 6, Math.max(2, x(b) - x(a)), lh - 12);
      }
    }
    g.restore();
    // The playhead.
    g.fillStyle = colors.ink;
    g.fillRect(x(t) - 0.75, 0, 1.5, names.length * laneH);
  }

  function drawOverview() {
    const { g, w, h } = canvasFor(overview);
    const left = LANE_LEFT(), right = 8;
    const x = (v) => left + (v / T) * (w - left - right);
    g.fillStyle = colors.paper;
    g.fillRect(left, 6, w - left - right, h - 22);
    g.font = '11px ' + getComputedStyle(document.body).getPropertyValue('--mono');
    g.textBaseline = 'middle';
    g.fillStyle = colors.muted;
    g.textAlign = 'right';
    g.fillText('the run', left - 8, 6 + (h - 22) / 2);
    // Flags: any joint, red.
    for (let j = 0; j < J; j++) for (let k = 0; k < flagT[j].length; k++) {
      if (!flagV[j][k]) continue;
      const a = flagT[j][k], b = k + 1 < flagT[j].length ? flagT[j][k + 1] : T;
      g.fillStyle = colors.fault;
      g.fillRect(x(a), 10, Math.max(2, x(b) - x(a)), h - 30);
    }
    // Log lines and marks.
    g.fillStyle = colors.ink;
    for (const l of logs) { const lx = x(l.t); g.beginPath(); g.moveTo(lx, h - 16); g.lineTo(lx - 4, h - 10); g.lineTo(lx + 4, h - 10); g.fill(); }
    // The window the lanes show.
    const span = windowSpan();
    const wx0 = x(t - span * 0.3), wx1 = x(t + span * 0.7);
    if (wx1 - wx0 > 3) { g.strokeStyle = colors.accent; g.lineWidth = 1; g.strokeRect(wx0, 7, wx1 - wx0, h - 24); }
    g.fillStyle = colors.accent;
    g.fillRect(x(t) - 1, 2, 2, h - 14);
    g.textAlign = 'center';
    g.fillStyle = colors.muted;
    for (let s = 0; s <= T / 1e6; s += T > 20e6 ? 5 : 2) g.fillText(s + ' s', x(s * 1e6), h - 4);
  }

  /* ─────────────── the line under the stage ─────────────── */
  function narrate(a) {
    const big = SPEEDS[speed] >= 1e5;
    const k = a.k;
    if (big) {
      const bad = [];
      for (let j = 0; j < J; j++) { const f = flagsAt(j, t); if (f) bad.push(`${short(j)} ${D.flagNames(f).join(', ')}`); }
      return `Cycle ${k + 1} of ${NS}, 500 a second. ` + (bad.length ? `keel flags ${bad.join('; ')}.` : 'Every joint follows its commands.');
    }
    if (a.onWire.length) {
      const i = a.onWire[0], j = fJ[i];
      return fAns[i]
        ? `${hex(F.id[i])} crossing the bus: ${joints[j]} answering, ${dur(fEnd[i] - fStart[i])} on the wire` + (fStart[i] - fReady[i] > 1 ? `, after ${dur(fStart[i] - fReady[i])} waiting for it.` : '.')
        : `${hex(F.id[i])} crossing the bus: keel-can’s command to ${joints[j]}, ${dur(fEnd[i] - fStart[i])} on the wire` + (fStart[i] - fReady[i] > 1 ? `, after ${dur(fStart[i] - fReady[i])} waiting for it.` : '.');
    }
    const m = a.msgs.find((x) => x.to !== 'recorder');
    if (m) return m.kind === 'state'
      ? `The state of cycle ${k + 1} on its way to the controller: published ${dur(t - a.s0)} ago.` + (m.untraced ? ' (This cycle wasn’t traced: drawn from when the state and the command were published.)' : '')
      : `The controller’s command, on its way back to keel-can.` + (m.untraced ? ' (Not traced: drawn from when it was published and its first frame written.)' : '');
    if (a.ctrlBusy) return 'The controller works out eight commands.';
    if (a.armBusy) return t - a.s0 < 3 ? `Tick: keel-can publishes cycle ${k + 1}’s state.` : 'keel-can writes the command’s eight frames to the bus.';
    if (a.recBusy) return 'The recorder writes it down.';
    const next = k + 1 < NS ? S[k + 1] - t : 0;
    return `Quiet. The motors move; MuJoCo steps every millisecond. Next tick in ${dur(next)}.`;
  }

  /* ─────────────── the inspector ─────────────── */
  function select(what) {
    sel = what;
    inspector.hidden = !what;
    render();
  }
  function rows(list, parent) {
    const dl = html('dl', { class: 'kv' }, parent);
    for (const [k, v, cls] of list) { html('dt', null, dl, k); html('dd', { class: cls || null }, dl, v); }
  }
  function bits(h, fields, parent, ans) {
    const box = html('div', { class: 'bits' + (ans ? ' ans' : '') }, parent);
    fields.forEach(([name, at, n, val], f) => {
      const cell = html('div', { class: 'field f' + (f % 2) }, box);
      html('code', null, cell, h.slice(at, at + n).toUpperCase());
      html('span', { class: 'fname' }, cell, name);
      html('span', { class: 'fval' }, cell, val);
      cell.style.flexGrow = n;
    });
  }
  function inspect(a) {
    if (!sel) return;
    inspBody.replaceChildren();
    const title = (s, sub) => { html('h3', null, inspBody, s); if (sub) html('p', { class: 'sub' }, inspBody, sub); };
    if (sel.kind === 'frame') {
      const i = sel.i, j = fJ[i], m = motors[j], h = payload(i);
      const v = fAns[i] ? D.answer(h, m) : D.command(h, m);
      title(`${hex(F.id[i])} · ${fAns[i] ? 'answer from' : 'command to'} ${joints[j]}`,
        fAns[i] ? 'What the motor says: where it is, how fast it turns, the torque it gives.' : 'An MIT command: the motor pushes kp × (target − position) + kd × (velocity − its velocity) + torque.');
      bits(h, v.fields, inspBody, fAns[i]);
      rows([
        ['ready', `${clock(fReady[i])}`],
        ['waited for the bus', dur(fStart[i] - fReady[i])],
        ['on the wire', dur(fEnd[i] - fStart[i]) + ' (CAN FD, 8 bytes)'],
        ['arrived', clock(fEnd[i])],
      ], inspBody);
      const go = html('button', { type: 'button', class: 'btn small' }, inspBody, 'Go to its start');
      go.addEventListener('click', () => { setPlaying(false); seek(fStart[i] - 1); });
    } else if (sel.kind === 'motor') {
      const j = sel.j, m = motors[j];
      const ai = before(answerEnds[j], t), ci = before(commandEnds[j], t);
      const ans = ai >= 0 ? D.answer(payload(answersOf[j][ai]), m) : null;
      const cmd = ci >= 0 ? D.command(payload(commandsOf[j][ci]), m) : null;
      const f = flagsAt(j, t);
      title(`${short(j)} · ${joints[j]}`, `Motor ${hex(m.id)}, answering on ${hex(m.answer)}: ±${m.p} rad, ±${m.v} rad/s, ±${m.t} N m.`);
      const list = [];
      if (ans) list.push(['position', signed(ans.p, 3) + ' rad'], ['velocity', signed(ans.v, 2) + ' rad/s'], ['torque', signed(ans.t, 2) + ' N m'], ['drive', D.STATUS[ans.status] || String(ans.status)]);
      if (cmd) {
        list.push(['told', `${signed(cmd.p, 3)} rad, kp ${cmd.kp.toFixed(0)}, kd ${cmd.kd.toFixed(2)}`]);
        if (ans) list.push(['asks for', signed(cmd.kp * (cmd.p - ans.p) + cmd.kd * (cmd.v - ans.v) + cmd.t, 2) + ' N m']);
      }
      list.push(['keel says', f ? D.flagNames(f).join(', ') : 'follows', f ? 'bad' : null]);
      rows(list, inspBody);
      html('p', { class: 'sub' }, inspBody, 'The dial turns by the angle the motor reports; the dashed needle is its last target. The bar is its torque, of its peak.');
    } else if (sel.kind === 'node') {
      const about = {
        arm: ['keel-can · node arm', 'Damiao’s bus master, 500 times a second: reads the answers, publishes them as the state, waits for the command (300 µs at most), writes it as eight frames. It watches its joints too: the flags are its.'],
        controller: ['openarm-wave · node controller', 'Holds the home pose, swings the first joint, opens and closes the gripper: eight MIT commands for every state.'],
        recorder: ['keel-recorder · node recorder', 'Writes every state and command to a file. This page is drawn from what it wrote, with keel’s traces and drydock’s bus trace.'],
      }[sel.name];
      title(about[0], about[1]);
      if (a.cyc) {
        const list = [];
        for (const h of a.cyc.hops.filter((x) => x.node === sel.name)) list.push([h.input, `taken ${dur(h.taken)} after the tick, worked ${dur(h.released - h.taken)}`]);
        if (list.length) { html('p', { class: 'sub' }, inspBody, `Cycle ${a.k + 1} was traced:`); rows(list, inspBody); }
      } else html('p', { class: 'sub' }, inspBody, 'keel traces one cycle in fifty; this one wasn’t. Step to a traced one with the marks in the controller’s lane, or pick another moment.');
      if (sel.name === 'arm') {
        const said = logs.filter((l) => l.t <= t);
        if (said.length) { html('p', { class: 'sub' }, inspBody, 'Its log, so far:'); const ol = html('ol', { class: 'said' }, inspBody); for (const l of said) html('li', null, ol, `${(l.t / 1e6).toFixed(1)}s  ${l.text}`); }
      }
    } else if (sel.kind === 'message') {
      const k = sel.k != null ? sel.k : a.k;
      const cyc = traced.get(k);
      if (sel.which === 'state') {
        title(`arm/state · cycle ${k + 1}`, 'Eight joints × 32 bytes: position, velocity and torque (f64), keel’s flags and the drive’s status (u32). Published once, in shared memory; each reader is told it’s there.');
      } else {
        title(`controller/command · cycle ${k + 1}`, 'Eight joints × 40 bytes: an MIT command each (target, velocity, kp, kd, torque). keel-can waits for it right after publishing the state.');
      }
      const list = [['state published', clock(S[k])]];
      if (!isNaN(cmdOf[k])) list.push(['command published', `${dur(cmdOf[k] - S[k])} after`]);
      if (!isNaN(wFirst[k])) list.push(['first frame written', `${dur(wFirst[k] - S[k])} after`]);
      if (cyc) for (const h of cyc.hops) list.push([`${h.node} took ${h.input.split(' ')[0]}`, `${dur(h.taken)} after the tick`]);
      else list.push(['traced', 'no: keel samples one cycle in fifty']);
      rows(list, inspBody);
    } else if (sel.kind === 'bus') {
      let busy = 0;
      const from = t - 100000;
      for (let i = Math.max(0, before(fEnd, from)); i < NF && fStart[i] < t; i++) busy += Math.min(fEnd[i], t) - Math.max(fStart[i], from);
      title('vcan0 · CAN FD', 'One frame at a time; when two want it, the lower id wins. Arbitration at 1 Mbit/s, data at 5 Mbit/s, about 52 µs a frame.');
      rows([['busy', `${Math.round((busy / 100000) * 100)}% of the last 100 ms`], ['a cycle', '8 commands and 8 answers, 2 ms']], inspBody);
    } else if (sel.kind === 'dock') {
      title('drydock', 'The robot: OpenArm’s model in MuJoCo, behind a SocketCAN bus, with Damiao’s motors answering as the real ones do, frame for frame. It ran with --break openarm_joint4:dead@6.');
    }
  }

  /* ─────────────── playing ─────────────── */
  function render() {
    const a = at(t);
    drawStage(a);
    drawToast();
    drawLanes();
    drawOverview();
    nowLine.textContent = narrate(a);
    timeOut.textContent = clock(t);
    if (sel) inspect(a);
    return a;
  }
  function seek(v) { t = clamp(v, 0, T); render(); }
  let renderedAt = 0;
  function frame(now) {
    if (!playing) return;
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    const a = at(t);
    const fast = skipBox.checked && SPEEDS[speed] < 1e5 && quiet(a) ? 12 : 1;
    t += dt * SPEEDS[speed] * fast;
    $('ff').hidden = fast === 1;
    if (t >= T) { t = T; setPlaying(false); }
    // The inspector rebuilds a few times a second, not every frame.
    const keep = sel;
    if (sel && now - renderedAt < 120) sel = null;
    else renderedAt = now;
    render();
    sel = keep;
    requestAnimationFrame(frame);
  }
  function setPlaying(on) {
    playing = on;
    playBtn.textContent = on ? 'Pause' : 'Play';
    playBtn.setAttribute('aria-pressed', String(on));
    $('ff').hidden = true;
    if (on) { if (t >= T) t = 0; last = performance.now(); requestAnimationFrame(frame); }
  }
  function stepBy(dir) {
    setPlaying(false);
    const big = SPEEDS[speed] >= 1e5;
    if (big) {   // a cycle at a time
      const k = before(S, t);
      seek(dir > 0 ? S[Math.min(NS - 1, k + 1)] : S[Math.max(0, (S[k] < t - 0.5 ? k : k - 1))]);
      return;
    }
    const k = before(events, t + 1e-3);
    seek(dir > 0 ? events[Math.min(events.length - 1, k + 1)] + 1e-3 : events[Math.max(0, events[k] > t - 0.01 ? k - 1 : k)] + 1e-3);
  }
  function setSpeed(i) {
    speed = i;
    speedBtns.forEach((b, k) => b.setAttribute('aria-pressed', String(k === i)));
    try { localStorage.setItem('keel-run-speed', String(i)); } catch (e) {}
    render();
  }

  playBtn.addEventListener('click', () => setPlaying(!playing));
  backBtn.addEventListener('click', () => stepBy(-1));
  fwdBtn.addEventListener('click', () => stepBy(1));
  speedBtns.forEach((b, i) => b.addEventListener('click', () => setSpeed(i)));
  $('insp-close').addEventListener('click', () => select(null));
  stageHost.addEventListener('click', () => select(null));
  document.addEventListener('keydown', (e) => {
    if (e.target.closest('input, textarea')) return;
    if (e.key === ' ' && !e.target.closest('button, [role=button]')) { e.preventDefault(); setPlaying(!playing); }
    else if (e.key === 'ArrowRight') { e.preventDefault(); stepBy(1); }
    else if (e.key === 'ArrowLeft') { e.preventDefault(); stepBy(-1); }
    else if (e.key === 'Escape') select(null);
    else if (/^[1-5]$/.test(e.key)) setSpeed(+e.key - 1);
  });
  // Lanes: tap a frame or a cycle to inspect it; drag to move through time.
  {
    let drag = null;
    lanes.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, t, moved: false }; lanes.setPointerCapture(e.pointerId); });
    lanes.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x;
      if (Math.abs(dx) > 3) drag.moved = true;
      if (drag.moved) { setPlaying(false); seek(drag.t - (dx / (lanes.clientWidth - LANE_LEFT() - 8)) * windowSpan()); }
    });
    lanes.addEventListener('pointerup', (e) => {
      if (drag && !drag.moved) {
        const r = lanes.getBoundingClientRect();
        const px = e.clientX - r.left, lane = Math.floor((e.clientY - r.top) / ((lanes.clientHeight - 18) / 3));
        const hit = laneHits.find((hh) => hh.lane === lane && px >= hh.x0 - 2 && px <= hh.x1 + 2);
        if (hit) select(hit.what);
      }
      drag = null;
    });
  }
  // Overview: tap or drag to go anywhere in the run.
  {
    let down = false;
    const go = (e) => {
      const r = overview.getBoundingClientRect(), left = LANE_LEFT();
      seek(clamp((e.clientX - r.left - left) / (r.width - left - 8), 0, 1) * T);
    };
    overview.addEventListener('pointerdown', (e) => { down = true; overview.setPointerCapture(e.pointerId); setPlaying(false); go(e); });
    overview.addEventListener('pointermove', (e) => { if (down) go(e); });
    overview.addEventListener('pointerup', () => { down = false; });
  }
  // Jumps: the moments worth seeing.
  const jumps = $('jumps');
  const jump = (label, to, cls) => {
    const b = html('button', { type: 'button', class: 'chip ' + (cls || '') }, jumps, label);
    b.addEventListener('click', () => { setPlaying(false); seek(to); });
  };
  jump('a traced cycle', S[firstTraced.seq] + 330);
  for (const m of marks) jump(m.label, m.t, m.cls);

  readColors();
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { readColors(); render(); });
  new MutationObserver(() => { readColors(); render(); }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  buildStage();
  setSpeed(speed);
  let width = stageHost.clientWidth;
  new ResizeObserver(() => {
    if (stageHost.clientWidth === width) { render(); return; }
    width = stageHost.clientWidth;
    buildStage(); lastLog = -2; render();
  }).observe(stageHost);
  // On phones the inspector is a sheet that sits just above the transport.
  const transport = lanes.parentElement;
  new ResizeObserver(() => document.documentElement.style.setProperty('--transport-h', transport.offsetHeight + 'px')).observe(transport);
  if (reduced) $('skip').checked = true;
  document.body.classList.add('ready');
})();
