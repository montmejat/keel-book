/* The book's figures: a recorded run of keel driving drydock, replayed.

   A figure is <div class="viz" data-viz="cycle|replay" data-run="name">,
   drawn from runs/<name>.js (written by keel-book from a real run). Every
   number on them was measured; nothing is made up for the picture.

   - cycle:  one 2 ms cycle, hop by hop: the state's way to the controller,
             the command's way back, the frames across the bus.
   - replay: the whole run, joint by joint, with what the arm's node said. */
(() => {
  'use strict';
  const SVG = 'http://www.w3.org/2000/svg';
  const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;

  function svg(tag, attrs, parent) {
    const e = document.createElementNS(SVG, tag);
    for (const [k, v] of Object.entries(attrs || {})) if (v != null) e.setAttribute(k, v);
    if (parent) parent.append(e);
    return e;
  }
  function text(parent, x, y, s, cls, anchor) {
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
  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
  const us = (x) => (Math.abs(x) >= 1000 ? (x / 1000).toFixed(2) + ' ms' : x.toFixed(Math.abs(x) < 10 ? 1 : 0) + ' µs');
  const signed = (x, d) => (x >= 0 ? '+' : '−') + Math.abs(x).toFixed(d);
  const hex = (n, w) => '0x' + n.toString(16).toUpperCase().padStart(w || 2, '0');

  /* Damiao's frames and keel's flags: damiao.js. */
  const { STATUS, flagNames } = window.Damiao;
  const decodeCommand = window.Damiao.command, decodeAnswer = window.Damiao.answer;

  /* ── One tooltip for every figure ── */
  const tip = html('div', { class: 'viz-tip', role: 'tooltip', hidden: '' }, document.body);
  function showTip(x, y, rows) {
    tip.replaceChildren();
    for (const [k, v, cls] of rows) {
      const row = html('div', { class: cls ? 'row ' + cls : 'row' }, tip);
      if (v === undefined) { row.classList.add('head'); row.textContent = k; continue; }
      html('span', null, row, k);
      html('b', null, row, v);
    }
    tip.hidden = false;
    const r = tip.getBoundingClientRect();
    const left = clamp(x + 14, 8, window.innerWidth - r.width - 8);
    const top = y + 16 + r.height > window.innerHeight - 8 ? y - r.height - 12 : y + 16;
    tip.style.left = left + 'px';
    tip.style.top = Math.max(8, top) + 'px';
  }
  const hideTip = () => { tip.hidden = true; };
  function hover(node, rows) {
    node.classList.add('v-hit');
    node.setAttribute('tabindex', '0');
    node.addEventListener('pointerenter', (e) => showTip(e.clientX, e.clientY, rows()));
    node.addEventListener('pointermove', (e) => showTip(e.clientX, e.clientY, rows()));
    node.addEventListener('pointerleave', hideTip);
    node.addEventListener('focus', () => { const r = node.getBoundingClientRect(); showTip(r.left, r.bottom, rows()); });
    node.addEventListener('blur', hideTip);
  }
  document.addEventListener('scroll', hideTip, { passive: true });

  function button(parent, label, title, cls) {
    const b = html('button', { type: 'button', class: 'btn ' + (cls || ''), title, 'aria-label': title || label }, parent, label);
    return b;
  }
  function onResize(el, fn) {
    let w = el.clientWidth, timer = 0;
    new ResizeObserver(() => {
      if (el.clientWidth === w) return;
      w = el.clientWidth;
      clearTimeout(timer);
      timer = setTimeout(fn, 60);
    }).observe(el);
  }
  /* Along a polyline, at 0..1 of its length. */
  function along(pts, f) {
    const segs = [];
    let total = 0;
    for (let i = 1; i < pts.length; i++) {
      const l = Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
      segs.push(l); total += l;
    }
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

  /* When the run first flags a joint, and which. */
  function firstFault(run) {
    const S = run.series;
    for (let i = 0; i < S.t.length; i++) {
      const j = S.flags[i].findIndex((f) => f);
      if (j >= 0) return { t: S.t[i], joint: j };
    }
    return null;
  }

  /* ═════════════════════════ One cycle, hop by hop ═════════════════════════ */

  function cycleFigure(fig, run) {
    const { motors, joints, cycles } = run;
    const fault = firstFault(run);
    const motorOf = (id, answer) => motors.findIndex((m) => (answer ? m.answer : m.id) === id);
    const short = (j) => (joints[j] === 'gripper' ? 'G' : 'J' + (j + 1));

    // Typical transits, over every sampled cycle, for the arrows' tooltips.
    const transit = {};
    for (const c of cycles) for (const h of c.hops) {
      (transit[h.node + '|' + h.input] = transit[h.node + '|' + h.input] || []).push(h.taken - h.published);
    }
    const stats = (key) => {
      const xs = (transit[key] || []).slice().sort((a, b) => a - b);
      return xs.length ? { p50: xs[Math.floor(xs.length / 2)], max: xs[xs.length - 1], n: xs.length } : null;
    };

    // Open on a healthy cycle, a second and a half before any fault; the
    // picker goes to the broken ones.
    let ci = fault ? Math.max(0, cycles.findIndex((c) => c.t > fault.t - 1.5)) : Math.floor(cycles.length / 2);
    let step = 0, cursor = 0, playing = false, raf = 0;
    let domain = [0, 2000], domainFrom = null, domainTo = null, domainStart = 0;
    let model, map, line;

    // Controls
    fig.replaceChildren();
    const bar = html('div', { class: 'viz-bar' }, fig);
    const playBtn = button(bar, 'Play', 'Play this cycle', 'viz-play');
    const prevBtn = button(bar, '←', 'Previous step');
    const nextBtn = button(bar, '→', 'Next step');
    const stepOut = html('span', { class: 'viz-step mono' }, bar);
    const pickWrap = html('label', { class: 'viz-pick' }, bar);
    html('span', { class: 'mono muted' }, pickWrap, 'cycle');
    const pick = html('input', { type: 'range', min: 0, max: cycles.length - 1, step: 1, id: (fig.id || 'cycle') + '-pick',
      'aria-label': 'Which sampled cycle' }, pickWrap);
    const pickOut = html('span', { class: 'mono viz-pick-out' }, pickWrap);
    const mapHost = html('div', { class: 'viz-map' }, fig);
    const say = html('div', { class: 'viz-say', 'aria-live': 'polite' }, fig);
    const lineHost = html('div', { class: 'viz-line' }, fig);
    const values = html('details', { class: 'viz-values' }, fig);
    html('summary', null, values, 'What this cycle’s state and command said');
    const valuesBody = html('div', { class: 'scroll' }, values);
    html('p', { class: 'note' }, fig,
      'Measured: keel’s sampled traces and drydock’s bus trace, on the same clock, from one OpenArm run. ' +
      'Hover a frame, a box or an arrow for its numbers. keel samples a trace every 100 ms; the picker goes through them.');

    function build() {
      const c = cycles[ci];
      const hop = (node, input) => c.hops.find((h) => h.node === node && h.input.startsWith(input));
      const ctrl = hop('controller', 'state'), armIn = hop('arm', 'command');
      const recS = hop('recorder', 'state'), recC = hop('recorder', 'command');
      const frames = c.frames.map(([id, ans, ready, start, end, data]) => {
        const j = motorOf(id, ans);
        return { id, ans, ready, start, end, data, j };
      }).filter((f) => f.j >= 0);
      const prev = frames.filter((f) => f.ready < 0), cur = frames.filter((f) => f.ready >= 0 && f.ready < 2000);
      const pc = prev.filter((f) => !f.ans), pa = prev.filter((f) => f.ans);
      const cc = cur.filter((f) => !f.ans), ca = cur.filter((f) => f.ans);
      const last = (xs, k) => (xs.length ? xs[xs.length - 1][k] : 0);
      const busy = cur.reduce((s, f) => s + (f.end - f.start), 0) / 2000;
      const waited = Math.max(0, ...cur.map((f) => f.start - f.ready));
      const firstReady = pc.length ? pc[0].ready : -1900;
      const lastAnswer = last(ca, 'end');
      const sent = last(cc, 'ready');
      const steps = [
        { title: 'The round before',
          text: `Before this tick, keel-can sent eight commands, and each motor answered: where it is, how fast it turns, the torque it gives. ` +
                `The last answer landed ${us(-last(pa, 'end'))} before the tick. Those answers are this cycle’s state.`,
          from: firstReady, to: 0, domain: [firstReady - 40, 80] },
        { title: 'Tick: the state goes out',
          text: `At the tick keel-can publishes the eight answers as one state message, 8 joints × 32 bytes, in shared memory: no one copies it. ` +
                `The recorder had it ${us(recS ? recS.taken : 0)} later; the controller, woken up, ${us(ctrl.taken)} later.`,
          from: 0, to: ctrl.taken, domain: [-12, Math.max(armIn.released, sent) + 16] },
        { title: 'The controller decides',
          text: `openarm-wave reads the state and works out eight MIT commands in ${us(ctrl.released - ctrl.taken)}: ` +
                `for each joint a target, a velocity, the gains kp and kd, and a torque to add.`,
          from: ctrl.taken, to: armIn.published, domain: [-12, Math.max(armIn.released, sent) + 16] },
        { title: 'The command comes back',
          text: `keel-can has waited for this answer to its state since the tick (it would give up after 300 µs). ` +
                `It has it ${us(armIn.taken - armIn.published)} after it was sent, ${us(armIn.taken)} after the tick.`,
          from: armIn.published, to: armIn.released, domain: [-12, Math.max(armIn.released, sent) + 16] },
        { title: 'Eight frames go out',
          text: `keel-can packs each command into 8 bytes and writes them to vcan0. On a bus one frame crosses at a time, ` +
                `each in about 52 µs at CAN FD’s 1 and 5 Mbit/s: the eighth waited ${us(cc.length ? cc[cc.length - 1].start - cc[cc.length - 1].ready : 0)} for its turn.`,
          from: armIn.released, to: last(cc, 'end'), domain: [-12, lastAnswer + 40] },
        { title: 'The motors answer',
          text: `Each motor answers as its command lands, but its answer waits: on CAN the lowest id wins the bus, and every command ` +
                `(0x01–0x08) outranks every answer (0x11–0x18). The last answer lands ${us(lastAnswer)} after the tick. Waiting is amber.`,
          from: last(cc, 'end'), to: lastAnswer, domain: [-12, lastAnswer + 40] },
        { title: 'Quiet until the next tick',
          text: `The bus was busy ${Math.round(busy * 100)}% of this cycle; the longest wait for it was ${us(waited)}. ` +
                `Meanwhile MuJoCo steps every millisecond, each motor pushing on its joint. At 2 ms keel-can ticks again, and these answers are the next state.`,
          from: lastAnswer, to: 2000, domain: [-30, 2030] },
      ];
      model = { c, ctrl, armIn, recS, recC, frames, prev, cur, pc, pa, cc, ca, steps, sent };
      pickOut.textContent = `${c.t.toFixed(2)} s · ` + (fault && c.t >= fault.t ? `${short(fault.joint)} flagged` : 'all joints ok');
      buildValues();
    }

    /* The map: drydock's motors, the bus, keel's three nodes. */
    function layout(W) {
      if (W >= 640) {
        const x = (f) => Math.round(f * W);
        const dock = { x: x(0.01), y: 18, w: x(0.24), h: 184 };
        const chipW = (dock.w - 30) / 2;
        const chips = motors.map((_, j) => ({ x: dock.x + 10 + (j % 2) * (chipW + 10), y: 62 + Math.floor(j / 2) * 32, w: chipW, h: 24 }));
        const arm = { x: x(0.42), y: 78, w: x(0.18), h: 62 };
        const controller = { x: x(0.79), y: 78, w: x(0.2), h: 62 };
        const recorder = { x: x(0.615), y: 170, w: x(0.155), h: 40 };
        return {
          H: 222, dock, chips, arm, controller, recorder,
          bus: [[arm.x, 110], [dock.x + dock.w, 110]],
          state: [[arm.x + arm.w, 98], [controller.x, 98]],
          command: [[controller.x, 122], [arm.x + arm.w, 122]],
          recState: [[arm.x + arm.w * 0.5, arm.y + arm.h], [arm.x + arm.w * 0.5, 190], [recorder.x, 190]],
          recCommand: [[controller.x + controller.w * 0.5, controller.y + controller.h], [controller.x + controller.w * 0.5, 190], [recorder.x + recorder.w, 190]],
          stateLabel: 'arm/state', commandLabel: 'controller/command',
        };
      }
      const dock = { x: 4, y: 8, w: W - 8, h: 84 };
      const chipW = (dock.w - 20 - 7 * 5) / 8;
      const chips = motors.map((_, j) => ({ x: dock.x + 10 + j * (chipW + 5), y: 52, w: chipW, h: 26 }));
      const arm = { x: 4, y: 146, w: Math.round(W * 0.36), h: 54 };
      const controller = { x: Math.round(W * 0.64) - 4, y: 146, w: Math.round(W * 0.36), h: 54 };
      const recorder = { x: Math.round(W * 0.3), y: 236, w: Math.round(W * 0.4), h: 38 };
      const bx = arm.x + arm.w * 0.5;
      return {
        H: 284, dock, chips, arm, controller, recorder,
        bus: [[bx, arm.y], [bx, dock.y + dock.h]],
        state: [[arm.x + arm.w, 162], [controller.x, 162]],
        command: [[controller.x, 186], [arm.x + arm.w, 186]],
        recState: [[arm.x + arm.w * 0.5, arm.y + arm.h], [arm.x + arm.w * 0.5, 255], [recorder.x, 255]],
        recCommand: [[controller.x + controller.w * 0.5, controller.y + controller.h], [controller.x + controller.w * 0.5, 255], [recorder.x + recorder.w, 255]],
        stateLabel: 'state', commandLabel: 'command',
      };
    }

    function buildMap() {
      const W = mapHost.clientWidth || 800;
      const L = layout(W);
      const s = svg('svg', { viewBox: `0 0 ${W} ${L.H}`, width: W, height: L.H, role: 'img',
        'aria-label': 'drydock’s eight motors on vcan0, keel-can, the controller and the recorder, with what is moving at this moment.' });
      const defs = svg('defs', null, s);
      const mk = svg('marker', { id: (fig.id || 'c') + '-head', viewBox: '0 0 8 8', refX: 7, refY: 4, markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse' }, defs);
      svg('path', { d: 'M0,0 L8,4 L0,8 z', class: 'v-head' }, mk);
      const head = `url(#${(fig.id || 'c')}-head)`;

      const box = (b, cls, name, sub) => {
        const g = svg('g', null, s);
        svg('rect', { x: b.x, y: b.y, width: b.w, height: b.h, rx: 6, class: cls }, g);
        text(g, b.x + 10, b.y + 20, name, 't v-name');
        if (sub) text(g, b.x + 10, b.y + 36, sub, 't muted v-sub');
        return g;
      };
      const dockG = box(L.dock, 'v-dock', 'drydock', W >= 640 ? 'Damiao motors · MuJoCo' : 'eight Damiao motors · MuJoCo');
      hover(dockG.querySelector('rect'), () => [['drydock'], ['what it is', 'the robot: OpenArm’s model in MuJoCo'], ['its motors', '8 Damiao drives, MIT mode'], ['steps', 'every 1 ms, in real time']]);
      const chips = L.chips.map((b, j) => {
        const g = svg('g', null, s);
        const r = svg('rect', { x: b.x, y: b.y, width: b.w, height: b.h, rx: 6, class: 'v-chip' }, g);
        text(g, b.x + b.w / 2, b.y + b.h / 2 + 4, W >= 640 ? `${short(j)} · ${hex(motors[j].id)}` : short(j), 't v-chip-t', 'middle');
        hover(r, () => {
          const st = model.c.state[j];
          return [[joints[j]], ['motor', `${hex(motors[j].id)}, answers on ${hex(motors[j].answer)}`],
            ['range', `±${motors[j].p} rad, ±${motors[j].v} rad/s, ±${motors[j].t} N m`],
            ['position', signed(st[0], 3) + ' rad'], ['torque', signed(st[2], 2) + ' N m'],
            ['status', STATUS[st[4]] || hex(st[4], 1)]];
        });
        return r;
      });
      const nodeBox = (b, name, sub, about) => {
        const g = box(b, 'v-box', name, sub);
        hover(g.querySelector('rect'), about);
        return g.querySelector('rect');
      };
      const armBox = nodeBox(L.arm, 'keel-can', 'node arm', () => [['keel-can, node arm'], ['does', 'Damiao’s bus master, 500 Hz'],
        ['each tick', 'reads the answers, publishes the state, waits for the command, writes it'], ['this cycle', `command in hand ${us(model.armIn.taken)} after the tick`]]);
      const ctrlBox = nodeBox(L.controller, 'openarm-wave', 'node controller', () => [['openarm-wave, node controller'],
        ['does', 'the home pose, the first joint swinging, the gripper'], ['this cycle', `woke ${us(model.ctrl.taken)} after the tick, worked ${us(model.ctrl.released - model.ctrl.taken)}`]]);
      const recBox = nodeBox(L.recorder, 'keel-recorder', null, () => [['keel-recorder, node recorder'], ['does', 'writes every state and command to a file'],
        ['why here', 'this figure is drawn from what it wrote']]);

      const edge = (pts, cls, label, at, key, about) => {
        const g = svg('g', null, s);
        svg('polyline', { points: points(pts), class: cls, 'marker-end': cls.includes('bus') ? null : head }, g);
        const hit = svg('polyline', { points: points(pts), class: 'v-edge-hit' }, g);
        if (label) {
          const [lx, ly] = at;
          text(g, lx, ly, label, 't muted v-edge-t', 'middle');
        }
        hover(hit, () => {
          const st = key && stats(key);
          const rows = [[label || 'vcan0']].concat(about());
          if (st) rows.push(['typical', `${us(st.p50)} (median of ${st.n})`], ['slowest', us(st.max)]);
          return rows;
        });
        return g;
      };
      const mid = (pts) => [(pts[0][0] + pts[pts.length - 1][0]) / 2, (pts[0][1] + pts[pts.length - 1][1]) / 2];
      const [sx, sy] = mid(L.state), [cx, cy] = mid(L.command);
      edge(L.bus, 'v-bus', null, null, null, () => [['what', 'CAN FD, 1 Mbit/s arbitration, 5 Mbit/s data'], ['carries', 'one frame at a time; lowest id first']]);
      const [bx, by] = mid(L.bus);
      text(s, W >= 640 ? bx : bx + 8, W >= 640 ? by + 22 : by + 4, 'vcan0 · CAN FD', 't muted', W >= 640 ? 'middle' : 'start');
      edge(L.state, 'v-edge', L.stateLabel, [sx, sy - 7], 'controller|state ← arm/state',
        () => [['carries', '8 joints × 32 bytes: position, velocity, torque, status'], ['transit', 'published → the controller has it']]);
      edge(L.command, 'v-edge', L.commandLabel, [cx, cy + 15], 'arm|command ← controller/command',
        () => [['carries', '8 joints × 40 bytes: an MIT command each'], ['transit', 'published → keel-can has it']]);
      edge(L.recState, 'v-edge v-edge-quiet', null, null, 'recorder|state ← arm/state', () => [['recorder ← arm/state']]);
      edge(L.recCommand, 'v-edge v-edge-quiet', null, null, 'recorder|command ← controller/command', () => [['recorder ← controller/command']]);

      const live = svg('g', { class: 'v-live' }, s);
      mapHost.replaceChildren(s);
      map = { L, live, chips, armBox, ctrlBox, recBox, W };
    }

    function drawMap(t) {
      const { L, live, chips, armBox, ctrlBox, recBox } = map;
      const m = model;
      live.replaceChildren();
      // Frames on the bus, and those waiting for it.
      const waitingCmd = m.frames.filter((f) => !f.ans && f.ready <= t && t < f.start).length;
      const waitingAns = m.frames.filter((f) => f.ans && f.ready <= t && t < f.start).length;
      const queue = (n, [x, y], dir, cls) => {
        for (let k = 0; k < n; k++) {
          const horizontal = L.bus[0][1] === L.bus[1][1];
          const qx = horizontal ? x + dir * (6 + k * 7) : x + 8 + k * 7;
          const qy = horizontal ? y - 18 : y + dir * 8;
          svg('rect', { x: qx - 3, y: qy - 3, width: 6, height: 6, rx: 6, class: 'v-queue ' + cls }, live);
        }
      };
      queue(waitingCmd, L.bus[0], -1, 'cmd');
      queue(waitingAns, L.bus[1], 1, 'ans');
      for (const f of m.frames) {
        if (!(f.start <= t && t < f.end)) continue;
        const k = (t - f.start) / (f.end - f.start);
        // Inset from the boxes, so a packet leaving one doesn't cover it.
        const [x, y] = along(f.ans ? [L.bus[1], L.bus[0]] : L.bus, 0.12 + k * 0.76);
        svg('rect', { x: x - 15, y: y - 8, width: 30, height: 16, rx: 6, class: 'v-packet ' + (f.ans ? 'ans' : 'cmd') }, live);
        text(live, x, y + 4, hex(f.id), 't v-packet-t', 'middle');
      }
      // A motor glows from its command landing to its answer leaving.
      chips.forEach((r, j) => {
        const cmd = m.frames.filter((f) => !f.ans && f.j === j && f.end <= t).pop();
        const ans = cmd && m.frames.find((f) => f.ans && f.j === j && f.ready >= cmd.end - 1);
        r.classList.toggle('lit', !!(cmd && ans && t < ans.end));
      });
      // Messages in transit, nodes at work.
      const hops = m.c.hops;
      const path = (h) => (h.node === 'controller' ? L.state : h.node === 'arm' ? L.command : h.input.startsWith('state') ? L.recState : L.recCommand);
      for (const h of hops) {
        if (h.published <= t && t < h.taken) {
          const [x, y] = along(path(h), (t - h.published) / Math.max(1, h.taken - h.published));
          svg('circle', { cx: x, cy: y, r: 6, class: 'v-dot' }, live);
        }
      }
      const busy = (h) => h && h.taken <= t && t < h.released + 0.5;
      ctrlBox.classList.toggle('active', busy(m.ctrl));
      recBox.classList.toggle('active', busy(m.recS) || busy(m.recC));
      armBox.classList.toggle('active', busy(m.armIn) || (t >= -6 && t < 1) || (t >= m.armIn.released && t < m.sent + 1) ||
        m.pc.some((f) => Math.abs(f.ready - t) < 2));
    }

    /* The timeline: four lanes, to scale, zooming with the step. */
    function buildLine() {
      const W = lineHost.clientWidth || 800;
      const left = W >= 640 ? 96 : 70, right = 10, top = 8, laneH = 30, gap = 8;
      const lanes = ['arm', 'controller', 'recorder', 'vcan0'];
      const H = top + lanes.length * (laneH + gap) + 26;
      const s = svg('svg', { viewBox: `0 0 ${W} ${H}`, width: W, height: H, role: 'img',
        'aria-label': 'The cycle to scale: what each node did and each frame on the bus, against microseconds from the tick.' });
      const clipId = (fig.id || 'c') + '-clip';
      const cp = svg('clipPath', { id: clipId }, svg('defs', null, s));
      svg('rect', { x: left, y: 0, width: W - left - right, height: H }, cp);
      const laneY = (i) => top + i * (laneH + gap);
      lanes.forEach((name, i) => {
        svg('rect', { x: left, y: laneY(i), width: W - left - right, height: laneH, class: 'lane' }, s);
        const label = { arm: 'keel-can', controller: 'controller', recorder: 'recorder', vcan0: 'vcan0' }[name];
        text(s, left - 8, laneY(i) + laneH / 2 + 4, label, 't', 'end');
      });
      const axis = svg('g', { class: 'v-axis' }, s);
      const body = svg('g', { 'clip-path': `url(#${clipId})` }, s);
      const items = [];
      const m = model;
      // kind: bar | wait | mark;  t0, t1 in µs; lane; label; cls; rows()
      const add = (lane, t0, t1, kind, cls, label, rows) => {
        const y = laneY(lane);
        let node;
        const wy = cls === 'bus' ? y + 3 : y + laneH / 2;  // a frame's wait runs above the frames ahead of it
        if (kind === 'wait') node = svg('line', { y1: wy, y2: wy, class: 'v-waitline ' + (cls || '') }, body);
        else if (kind === 'mark') node = svg('line', { y1: y + 3, y2: y + laneH - 3, class: 'v-mark ' + (cls || '') }, body);
        else node = svg('rect', { y: y + 5, height: laneH - 10, rx: 1.5, class: 'v-bar ' + (cls || '') }, body);
        const lab = label ? text(body, 0, y + laneH / 2 + 4, label, 't v-bar-t', 'middle') : null;
        if (rows) hover(node, rows);
        items.push({ node, lab, t0, t1, kind });
      };
      const hopRows = (h, what) => () => [[what], ['published', us(h.published) + ' after the tick'],
        ['taken', `${us(h.taken)} (${us(h.taken - h.published)} on the way)`], ['released', `${us(h.released)} (${us(h.released - h.taken)} at work)`]];
      // keel-can
      add(0, 0, 0, 'mark', 'tick', null, () => [['tick'], ['keel-can', 'publishes the state']]);
      add(0, -2000, -2000, 'mark', 'tick');
      add(0, 2000, 2000, 'mark', 'tick next', null, () => [['next tick'], ['at', '2 ms: 500 Hz']]);
      add(0, 0, m.armIn.taken, 'wait', '', null, () => [['keel-can waits for the command'], ['for', us(m.armIn.taken)]]);
      add(0, m.armIn.taken, m.armIn.released, 'bar', 'k', null, hopRows(m.armIn, 'keel-can takes the command'));
      if (m.cc.length) add(0, m.armIn.released, m.sent, 'bar', 'k soft', null, () => [['keel-can writes 8 frames'], ['took', us(m.sent - m.armIn.released)]]);
      if (m.pc.length) add(0, m.pc[0].ready - 2, m.pc[m.pc.length - 1].ready, 'bar', 'k soft', null, () => [['the round before'], ['keel-can', 'wrote its 8 frames']]);
      // controller
      add(1, 0, m.ctrl.taken, 'wait', 'transit', null, hopRows(m.ctrl, 'the state, on its way to the controller'));
      add(1, m.ctrl.taken, m.ctrl.released, 'bar', 'k', 'works', hopRows(m.ctrl, 'the controller works'));
      add(1, m.armIn.published, m.armIn.taken, 'wait', 'transit', null, hopRows(m.armIn, 'the command, on its way to keel-can'));
      // recorder
      if (m.recS) { add(2, 0, m.recS.taken, 'wait', 'transit'); add(2, m.recS.taken, m.recS.released, 'bar', 'k', null, hopRows(m.recS, 'the recorder writes the state')); }
      if (m.recC) { add(2, m.recC.published, m.recC.taken, 'wait', 'transit'); add(2, m.recC.taken, m.recC.released, 'bar', 'k', null, hopRows(m.recC, 'the recorder writes the command')); }
      // the bus
      for (const f of m.frames) {
        if (f.start > f.ready + 0.5) add(3, f.ready, f.start, 'wait', 'bus', null, frameRows(f));
        add(3, f.start, f.end, 'bar', f.ans ? 'd' : 'k', hex(f.id), frameRows(f));
      }
      const cursorLine = svg('line', { y1: 2, y2: top + lanes.length * (laneH + gap) - gap + 4, class: 'v-cursor' }, s);
      const cursorT = text(s, 0, H - 4, '', 't v-cursor-t', 'middle');
      lineHost.replaceChildren(s);
      line = { s, W, left, right, H, items, axis, cursorLine, cursorT, top, laneH, gap, lanes };
    }

    function frameRows(f) {
      return () => {
        const m = motors[f.j];
        const rows = [[`${hex(f.id)} · ${f.ans ? 'answer from' : 'command to'} ${joints[f.j]}`]];
        rows.push(['bytes', f.data.match(/../g).join(' ')]);
        if (f.ans) {
          const a = decodeAnswer(f.data, m);
          rows.push(['status', STATUS[a.status] || hex(a.status, 1)], ['position', signed(a.p, 3) + ' rad'],
            ['velocity', signed(a.v, 2) + ' rad/s'], ['torque', signed(a.t, 2) + ' N m'], ['temperatures', `${a.mos} °C drive, ${a.rotor} °C coil`]);
        } else {
          const c = decodeCommand(f.data, m);
          rows.push(['target', signed(c.p, 3) + ' rad'], ['velocity', signed(c.v, 2) + ' rad/s'],
            ['kp, kd', `${c.kp.toFixed(1)}, ${c.kd.toFixed(2)}`], ['torque added', signed(c.t, 2) + ' N m']);
        }
        rows.push(['ready', us(f.ready)], ['waited', us(f.start - f.ready)], ['on the wire', us(f.end - f.start)]);
        return rows;
      };
    }

    function drawLine(t) {
      const { W, left, right, items, axis, cursorLine, cursorT } = line;
      const [d0, d1] = domain;
      const x = (v) => left + ((v - d0) / (d1 - d0)) * (W - left - right);
      for (const it of items) {
        const x0 = x(it.t0), x1 = x(it.t1);
        if (it.kind === 'bar') {
          it.node.setAttribute('x', x0);
          it.node.setAttribute('width', Math.max(1, x1 - x0));
        } else {
          it.node.setAttribute('x1', x0);
          it.node.setAttribute('x2', it.kind === 'mark' ? x0 : x1);
        }
        if (it.lab) {
          it.lab.setAttribute('x', (x0 + x1) / 2);
          it.lab.style.display = x1 - x0 > it.lab.textContent.length * 7.2 + 6 ? '' : 'none';
        }
      }
      // Axis: a round step for the span shown.
      axis.replaceChildren();
      const span = d1 - d0;
      const stepUs = [1, 2, 5, 10, 20, 25, 50, 100, 200, 250, 500, 1000].find((s) => span / s <= (W < 640 ? 5 : 9)) || 1000;
      const y0 = line.top + line.lanes.length * (line.laneH + line.gap) - line.gap;
      for (let v = Math.ceil(d0 / stepUs) * stepUs; v <= d1; v += stepUs) {
        const xx = x(v);
        if (xx < left - 1) continue;
        svg('line', { x1: xx, x2: xx, y1: 2, y2: y0, class: 'grid' }, axis);
        text(axis, xx, y0 + 14, v === 0 ? 'tick' : (v / 1000).toFixed(span < 300 ? 2 : span < 2500 ? 1 : 0) + ' ms', 't muted v-axis-t', 'middle');
      }
      axis.parentNode.insertBefore(axis, axis.parentNode.querySelector('g[clip-path]'));
      const cx = x(t);
      const inside = cx >= left && cx <= W - right;
      cursorLine.style.display = inside ? '' : 'none';
      cursorLine.setAttribute('x1', cx); cursorLine.setAttribute('x2', cx);
      cursorT.style.display = 'none';
    }

    function buildValues() {
      const { c } = model;
      const t = html('table', { class: 'data viz-table' });
      const head = html('tr', null, html('thead', null, t));
      for (const h of ['joint', 'position', 'velocity', 'torque', 'status', 'target', 'kp · kd', 'torque asked']) html('th', null, head, h);
      const tb = html('tbody', null, t);
      c.state.forEach((st, j) => {
        const cm = c.command[j];
        const asked = cm[2] * (cm[0] - st[0]) + cm[3] * (cm[1] - st[1]) + cm[4];
        const tr = html('tr', { class: st[3] ? 'flagged' : null }, tb);
        html('td', { class: 'mono' }, tr, joints[j]);
        html('td', { class: 'num' }, tr, signed(st[0], 3) + ' rad');
        html('td', { class: 'num' }, tr, signed(st[1], 2) + ' rad/s');
        html('td', { class: 'num' }, tr, signed(st[2], 2) + ' N m');
        html('td', { class: 'mono' }, tr, st[3] ? flagNames(st[3]).join(', ') : 'ok');
        html('td', { class: 'num' }, tr, signed(cm[0], 3) + ' rad');
        html('td', { class: 'num' }, tr, `${cm[2]} · ${cm[3]}`);
        html('td', { class: 'num' }, tr, signed(asked, 2) + ' N m');
      });
      valuesBody.replaceChildren(t);
    }

    function render() {
      const st = model.steps[step];
      stepOut.textContent = `${step + 1} / ${model.steps.length}`;
      say.replaceChildren();
      html('strong', null, say, st.title + '. ');
      say.append(st.text);
      prevBtn.disabled = step === 0;
      nextBtn.disabled = step === model.steps.length - 1;
      drawMap(cursor);
      drawLine(cursor);
    }

    function goTo(n, animate) {
      step = clamp(n, 0, model.steps.length - 1);
      const st = model.steps[step];
      domainFrom = domain.slice();
      domainTo = st.domain;
      domainStart = performance.now();
      cursor = animate ? st.from : st.to;
      if (reduced || !animate) { domain = domainTo.slice(); domainFrom = null; }
      render();
    }

    const STEP_MS = 3200, ZOOM_MS = 450;
    let stepStart = 0;
    function frame(now) {
      raf = 0;
      if (domainFrom) {
        const k = clamp((now - domainStart) / ZOOM_MS, 0, 1), e = k < 0.5 ? 2 * k * k : 1 - Math.pow(-2 * k + 2, 2) / 2;
        domain = [domainFrom[0] + (domainTo[0] - domainFrom[0]) * e, domainFrom[1] + (domainTo[1] - domainFrom[1]) * e];
        if (k >= 1) domainFrom = null;
      }
      if (playing) {
        const st = model.steps[step];
        const k = clamp((now - stepStart) / STEP_MS, 0, 1);
        cursor = st.from + (st.to - st.from) * k;
        if (k >= 1) {
          if (step < model.steps.length - 1) { goTo(step + 1, true); stepStart = now; }
          else setPlaying(false);
        }
      }
      render();
      if (playing || domainFrom) raf = requestAnimationFrame(frame);
    }
    const kick = () => { if (!raf) raf = requestAnimationFrame(frame); };
    function setPlaying(on) {
      playing = on;
      playBtn.textContent = on ? 'Pause' : 'Play';
      playBtn.setAttribute('aria-label', on ? 'Pause' : 'Play this cycle');
      if (on) {
        if (step === model.steps.length - 1 && cursor >= model.steps[step].to) goTo(0, true);
        else { const st = model.steps[step]; if (cursor >= st.to) cursor = st.from; goTo(step, true); }
        stepStart = performance.now() - ((cursor - model.steps[step].from) / Math.max(1, model.steps[step].to - model.steps[step].from)) * STEP_MS;
        kick();
      }
    }
    playBtn.addEventListener('click', () => setPlaying(!playing));
    prevBtn.addEventListener('click', () => { setPlaying(false); goTo(step - 1, true); cursor = model.steps[step].to; kick(); });
    nextBtn.addEventListener('click', () => { setPlaying(false); goTo(step + 1, true); cursor = model.steps[step].to; kick(); });
    pick.addEventListener('input', () => {
      ci = +pick.value; build(); buildMap(); buildLine(); goTo(step, false);
    });
    fig.addEventListener('keydown', (e) => {
      if (e.target === pick) return;
      if (e.key === 'ArrowRight') { nextBtn.click(); e.preventDefault(); }
      if (e.key === 'ArrowLeft') { prevBtn.click(); e.preventDefault(); }
    });

    pick.value = ci;
    build();
    buildMap();
    buildLine();
    goTo(4, false);   // at rest: the frames crossing, the busiest picture
    onResize(fig, () => { buildMap(); buildLine(); render(); });
  }

  /* ═════════════════════════ The whole run, replayed ═════════════════════════ */

  function replayFigure(fig, run) {
    const S = run.series, n = S.t.length, T = S.t[n - 1];
    const joints = run.joints;
    const fault = firstFault(run);
    let j = fault ? fault.joint : 0;
    let i = indexAt(fault ? fault.t + 1.2 : T / 2);
    let playing = false, raf = 0, last = 0, chart;

    function indexAt(t) {
      let lo = 0, hi = n - 1;
      while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (S.t[mid] <= t) lo = mid; else hi = mid - 1; }
      return lo;
    }

    fig.replaceChildren();
    const bar = html('div', { class: 'viz-bar' }, fig);
    const playBtn = button(bar, 'Play', 'Play the run', 'viz-play');
    const scrub = html('input', { type: 'range', min: 0, max: n - 1, step: 1, class: 'viz-scrub', id: (fig.id || 'replay') + '-t', 'aria-label': 'Time in the run' }, bar);
    const tOut = html('span', { class: 'mono viz-time' }, bar);
    const dials = html('div', { class: 'viz-dials', role: 'group', 'aria-label': 'Joints: pick one to chart' }, fig);
    const chartHost = html('div', { class: 'viz-chart' }, fig);
    const logs = html('ol', { class: 'viz-log' }, fig);
    html('p', { class: 'note' }, fig,
      'Measured: every fifth state keel-can published (100 Hz of its 500), the command in force when it did, and its log. ' +
      'Each dial turns by the joint’s angle; its outline is where it was told to be. Pick a joint to chart it; tap the chart or a log line to go there.');

    // Dials
    const dialEls = joints.map((name, k) => {
      const b = html('button', { type: 'button', class: 'viz-dial', 'aria-pressed': 'false' }, dials);
      const s = svg('svg', { viewBox: '-40 -40 80 80', width: 64, height: 64, 'aria-hidden': 'true' }, b);
      svg('circle', { r: 34, class: 'v-dial-face' }, s);
      for (let a = 0; a < 12; a++) {
        const r0 = a % 3 ? 30 : 27, ang = (a * Math.PI) / 6;
        svg('line', { x1: Math.sin(ang) * r0, y1: -Math.cos(ang) * r0, x2: Math.sin(ang) * 34, y2: -Math.cos(ang) * 34, class: 'v-dial-tick' }, s);
      }
      const ghost = svg('line', { x1: 0, y1: 0, x2: 0, y2: -30, class: 'v-dial-target' }, s);
      const needle = svg('line', { x1: 0, y1: 0, x2: 0, y2: -30, class: 'v-dial-needle' }, s);
      svg('circle', { r: 3.5, class: 'v-dial-hub' }, s);
      html('span', { class: 'viz-dial-name mono' }, b, name === 'gripper' ? 'gripper' : 'J' + (k + 1));
      const val = html('span', { class: 'viz-dial-val mono' }, b);
      const flag = html('span', { class: 'viz-dial-flag mono' }, b);
      b.addEventListener('click', () => { j = k; buildChart(); draw(); });
      b.title = name;
      return { b, ghost, needle, val, flag };
    });

    // The log
    const logItems = run.logs.map((l) => {
      const li = html('li', null, logs);
      const a = html('button', { type: 'button' }, li);
      html('span', { class: 'mono' }, a, l.t.toFixed(1) + ' s');
      html('span', null, a, l.text);
      a.addEventListener('click', () => { setPlaying(false); i = indexAt(l.t + 0.02); draw(); });
      return { li, t: l.t };
    });
    logs.setAttribute('aria-label', 'What the arm’s node said');

    function buildChart() {
      const W = chartHost.clientWidth || 800;
      const left = 52, right = 14, top = 26, ph = W < 640 ? 100 : 120, gap = 46;
      const bandY = top + 2 * ph + gap + 18, H = bandY + 50;
      const x = (t) => left + (t / T) * (W - left - right);
      const s = svg('svg', { viewBox: `0 0 ${W} ${H}`, width: W, height: H, role: 'img',
        'aria-label': `${joints[j]} over the run: its position against its target, and the torque it gave against the torque asked.` });
      const col = (key) => S[key].map((r) => (r ? r[j] : null));
      const pos = col('position'), tgt = col('target'), eff = col('effort'), ask = col('asked'), flags = col('flags');
      const ext = (a, b) => {
        const xs = a.concat(b).filter((v) => v != null);
        let lo = Math.min(...xs), hi = Math.max(...xs);
        if (hi - lo < 0.2) { lo -= 0.1; hi += 0.1; }
        const pad = (hi - lo) * 0.08;
        return [lo - pad, hi + pad];
      };
      const panel = (y0, values, title, unit) => {
        const [lo, hi] = ext(values[0], values[1]);
        const y = (v) => y0 + ph - ((v - lo) / (hi - lo)) * ph;
        const g = svg('g', null, s);
        text(g, left, y0 - 9, title, 't v-panel-t');
        // ticks: 4 round values
        const step = [0.05, 0.1, 0.2, 0.25, 0.5, 1, 2, 5, 10, 20, 25, 50, 100].find((st) => (hi - lo) / st <= 4) || 100;
        for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
          svg('line', { x1: left, x2: W - right, y1: y(v), y2: y(v), class: 'grid' }, g);
          text(g, left - 6, y(v) + 4, (Math.abs(v) < 1e-9 ? 0 : +v.toFixed(2)) + '', 't muted v-axis-t', 'end');
        }
        if (lo < 0 && hi > 0) svg('line', { x1: left, x2: W - right, y1: y(0), y2: y(0), class: 'v-zero' }, g);
        const line = (vals, cls) => {
          let d = '', pen = false;
          vals.forEach((v, k) => {
            if (v == null) { pen = false; return; }
            d += (pen ? 'L' : 'M') + x(S.t[k]).toFixed(1) + ',' + y(v).toFixed(1);
            pen = true;
          });
          svg('path', { d, class: cls }, g);
        };
        line(values[1], 'v-ln-told');
        line(values[0], 'v-ln-meas');
        text(g, W - right, y0 - 9, unit, 't muted', 'end');
        return { y, lo, hi };
      };
      const pA = panel(top, [pos, tgt], 'Position', 'rad');
      const pB = panel(top + ph + gap, [eff, ask], 'Torque', 'N m');
      // Legend, once, above: what each line is.
      const lg = svg('g', null, s);
      const lx = Math.max(left + 90, W - right - 250);
      svg('line', { x1: lx, x2: lx + 22, y1: top - 13, y2: top - 13, class: 'v-ln-meas' }, lg);
      text(lg, lx + 28, top - 9, 'measured', 't');
      svg('line', { x1: lx + 104, x2: lx + 126, y1: top - 13, y2: top - 13, class: 'v-ln-told' }, lg);
      text(lg, lx + 132, top - 9, 'commanded', 't');
      // Status band: when the loop flagged this joint.
      text(s, left - 6, bandY + 13, 'status', 't muted v-axis-t', 'end');
      svg('rect', { x: left, y: bandY, width: W - left - right, height: 18, class: 'lane' }, s);
      let k0 = null;
      const seg = (a, b, f) => {
        const r = svg('rect', { x: x(S.t[a]), y: bandY + 2, width: Math.max(2, x(S.t[b]) - x(S.t[a])), height: 14, rx: 6, class: 'v-flag' }, s);
        const names = flagNames(f).join(', ');
        const w = x(S.t[b]) - x(S.t[a]);
        if (w > names.length * 7 + 8) text(s, x(S.t[a]) + 6, bandY + 13, names, 't v-flag-t');
        hover(r, () => [[joints[j]], ['flagged', names], ['from', S.t[a].toFixed(2) + ' s'], ['to', S.t[b].toFixed(2) + ' s']]);
      };
      for (let k = 0; k <= n; k++) {
        const f = k < n ? flags[k] : 0;
        if (k0 !== null && (f !== flags[k0] || k === n)) { seg(k0, k - 1, flags[k0]); k0 = null; }
        if (k0 === null && f) k0 = k;
      }
      // Log lines, as marks on the time axis.
      for (const l of run.logs) {
        const lx2 = x(l.t);
        const mark = svg('path', { d: `M${lx2},${bandY + 22} l5,8 l-10,0 z`, class: 'v-logmark' }, s);
        hover(mark, () => [[l.t.toFixed(2) + ' s · arm'], ['said', l.text]]);
      }
      for (let t = 0; t <= T; t += T > 20 ? 5 : 2) text(s, x(t), H - 2, t + ' s', 't muted v-axis-t', 'middle');
      const head = svg('line', { y1: top - 4, y2: bandY + 18, class: 'v-cursor' }, s);
      const hoverLine = svg('line', { y1: top - 4, y2: bandY + 18, class: 'v-hover', style: 'display:none' }, s);
      const dotA = svg('circle', { r: 4, class: 'v-head-dot' }, s), dotB = svg('circle', { r: 4, class: 'v-head-dot' }, s);
      // Hover: values under the pointer; tap or drag: go there.
      const overlay = svg('rect', { x: left, y: top - 4, width: W - left - right, height: bandY + 18 - top + 4, class: 'v-overlay' }, s);
      const tAt = (e) => {
        const r = s.getBoundingClientRect();
        return clamp(((e.clientX - r.left) * (W / r.width) - left) / (W - left - right), 0, 1) * T;
      };
      let dragging = false;
      overlay.addEventListener('pointerdown', (e) => { dragging = true; overlay.setPointerCapture(e.pointerId); setPlaying(false); i = indexAt(tAt(e)); draw(); });
      overlay.addEventListener('pointerup', () => { dragging = false; });
      overlay.addEventListener('pointermove', (e) => {
        const k = indexAt(tAt(e));
        if (dragging) { i = k; draw(); }
        hoverLine.style.display = '';
        hoverLine.setAttribute('x1', x(S.t[k])); hoverLine.setAttribute('x2', x(S.t[k]));
        const rows = [[`${S.t[k].toFixed(2)} s · ${joints[j]}`], ['position', signed(pos[k], 3) + ' rad']];
        if (tgt[k] != null) rows.push(['target', signed(tgt[k], 3) + ' rad']);
        rows.push(['torque given', signed(eff[k], 2) + ' N m']);
        if (ask[k] != null) rows.push(['torque asked', signed(ask[k], 2) + ' N m']);
        rows.push(['status', flags[k] ? flagNames(flags[k]).join(', ') : 'ok', flags[k] ? 'bad' : null]);
        showTip(e.clientX, e.clientY, rows);
      });
      overlay.addEventListener('pointerleave', () => { hoverLine.style.display = 'none'; hideTip(); });
      chartHost.replaceChildren(s);
      chart = { x, head, dotA, dotB, pA, pB, pos, eff };
    }

    function draw() {
      i = clamp(i, 0, n - 1);
      scrub.value = i;
      const t = S.t[i];
      const w = Math.floor(t / run.load.window_s);
      const load = run.load.busy[Math.min(w, run.load.busy.length - 1)];
      tOut.textContent = `${t.toFixed(2)} s · bus ${Math.round(load * 100)}% busy`;
      dialEls.forEach((d, k) => {
        const p = S.position[i][k], tg = S.target[i] ? S.target[i][k] : null, f = S.flags[i][k];
        d.needle.setAttribute('transform', `rotate(${(p * 180) / Math.PI})`);
        d.ghost.style.display = tg == null ? 'none' : '';
        if (tg != null) d.ghost.setAttribute('transform', `rotate(${(tg * 180) / Math.PI})`);
        d.val.textContent = signed(p, 2);
        d.flag.textContent = f ? flagNames(f)[0] : `${signed(S.effort[i][k], 1)} N m`;
        d.b.classList.toggle('bad', !!f);
        d.b.setAttribute('aria-pressed', String(k === j));
        d.b.setAttribute('aria-label', `${joints[k]}: ${p.toFixed(2)} rad${tg != null ? `, told ${tg.toFixed(2)}` : ''}, ${f ? flagNames(f).join(', ') : 'ok'}`);
      });
      const hx = chart.x(t);
      chart.head.setAttribute('x1', hx); chart.head.setAttribute('x2', hx);
      chart.dotA.setAttribute('cx', hx); chart.dotA.setAttribute('cy', chart.pA.y(chart.pos[i]));
      chart.dotB.setAttribute('cx', hx); chart.dotB.setAttribute('cy', chart.pB.y(chart.eff[i]));
      let current = -1;
      logItems.forEach((l, k) => { if (l.t <= t + 1e-6) current = k; });
      logItems.forEach((l, k) => { l.li.classList.toggle('now', k === current); l.li.classList.toggle('later', l.t > t + 1e-6); });
    }

    function frame(now) {
      raf = 0;
      if (!playing) return;
      const dt = (now - last) / 1000;
      last = now;
      const next = indexAt(S.t[i] + dt);
      i = next === i && i < n - 1 && dt > 0.012 ? i + 1 : next;
      if (i >= n - 1) setPlaying(false);
      draw();
      if (playing) raf = requestAnimationFrame(frame);
    }
    function setPlaying(on) {
      playing = on;
      playBtn.textContent = on ? 'Pause' : 'Play';
      if (on) {
        if (i >= n - 1) i = 0;
        last = performance.now();
        if (!raf) raf = requestAnimationFrame(frame);
      }
    }
    playBtn.addEventListener('click', () => setPlaying(!playing));
    scrub.addEventListener('input', () => { setPlaying(false); i = +scrub.value; draw(); });

    buildChart();
    draw();
    onResize(fig, () => { buildChart(); draw(); });
  }

  const FIGURES = { cycle: cycleFigure, replay: replayFigure };
  function start() {
    for (const fig of document.querySelectorAll('.viz[data-viz]')) {
      const run = (window.KEEL_RUNS || {})[fig.dataset.run];
      if (!run || !FIGURES[fig.dataset.viz]) continue;
      try { FIGURES[fig.dataset.viz](fig, run); } catch (e) { console.error('viz', fig.dataset.viz, e); }
    }
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start);
  else start();
})();
