/* A robot in 3D: the body keel-book wrote from drydock's robot file and its
   MuJoCo model (models/<name>.js), posed by the angles its motors report,
   with a ghost where they were told to be. Plain WebGL, no library, so the
   book still opens from disk.

     const arm = Arm3D.create(canvas, KEEL_MODELS.openarm);
     arm.set(reported, told, flagged);   // Maps by motor id: rad, rad, bool

   Drag to turn it, wheel or pinch to come closer. */
(() => {
  'use strict';

  /* ── 4×4 matrices, column-major, as WebGL takes them ── */
  const ident = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  function mul(a, b) {
    const o = new Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
    return o;
  }
  // MuJoCo's quaternions are w x y z.
  function place(q, p) {
    let [w, x, y, z] = q;
    const n = Math.hypot(w, x, y, z) || 1;
    w /= n; x /= n; y /= n; z /= n;
    return [
      1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 0,
      2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 0,
      2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y), 0,
      p[0], p[1], p[2], 1,
    ];
  }
  function turn(axis, a) {
    const n = Math.hypot(...axis) || 1, s = Math.sin(a / 2);
    return place([Math.cos(a / 2), (axis[0] / n) * s, (axis[1] / n) * s, (axis[2] / n) * s], [0, 0, 0]);
  }
  const slide = (axis, d) => { const m = ident(); m[12] = axis[0] * d; m[13] = axis[1] * d; m[14] = axis[2] * d; return m; };
  function perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
    return [f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0];
  }
  function lookAt(e, t, u) {
    const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    const norm = (a) => { const n = Math.hypot(...a) || 1; return a.map((v) => v / n); };
    const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const z = norm(sub(e, t)), x = norm(cross(u, z)), y = cross(z, x);
    return [x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, e), -dot(y, e), -dot(z, e), 1];
  }

  /* ── the model's meshes: grid cells to metres, one normal per face ── */
  function buffer(s) {
    const bin = atob(s), u = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
    return u.buffer;
  }
  function unpack(m) {
    const v = new Uint16Array(buffer(m.v));
    const f = m.f32 ? new Uint32Array(buffer(m.f)) : new Uint16Array(buffer(m.f));
    const pos = new Float32Array(f.length * 3), nor = new Float32Array(f.length * 3);
    const at = (i, c) => m.min[c] + v[i * 3 + c] * m.cell;
    for (let t = 0; t < f.length; t += 3) {
      const p = [0, 1, 2].map((k) => [0, 1, 2].map((c) => at(f[t + k], c)));
      const a = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
      const b = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
      let n = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
      const l = Math.hypot(...n) || 1;
      n = n.map((x) => x / l);
      for (let k = 0; k < 3; k++) for (let c = 0; c < 3; c++) { pos[(t + k) * 3 + c] = p[k][c]; nor[(t + k) * 3 + c] = n[c]; }
    }
    return { pos, nor, count: f.length };
  }

  /* ── colors from the page's tokens, as GL wants them ── */
  const probe = document.createElement('canvas').getContext('2d');
  function rgb(css) {
    probe.fillStyle = '#000';
    probe.fillStyle = css;
    const s = probe.fillStyle;
    if (s[0] === '#') return [1, 3, 5].map((i) => parseInt(s.slice(i, i + 2), 16) / 255);
    const m = s.match(/[\d.]+/g) || [0, 0, 0];
    return m.slice(0, 3).map((x) => +x / 255);
  }
  const mix = (a, b, k) => a.map((x, i) => x * (1 - k) + b[i] * k);

  const VS = `
    attribute vec3 aPos; attribute vec3 aNor;
    uniform mat4 uVP; uniform mat4 uM;
    varying vec3 vN;
    void main() { vN = mat3(uM) * aNor; gl_Position = uVP * uM * vec4(aPos, 1.0); }`;
  const FS = `
    precision mediump float;
    uniform vec4 uColor; uniform vec3 uLight; uniform float uFlat;
    varying vec3 vN;
    void main() {
      vec3 n = normalize(vN);
      float d = abs(dot(n, uLight));
      float fill = 0.5 + 0.5 * n.z;
      vec3 c = uColor.rgb * mix(0.32 + 0.5 * d + 0.18 * fill, 1.0, uFlat);
      gl_FragColor = vec4(c, uColor.a);
    }`;

  function create(canvas, model) {
    const gl = canvas.getContext('webgl', { antialias: true, alpha: true, premultipliedAlpha: false });
    if (!gl) { canvas.replaceWith(Object.assign(document.createElement('p'), { className: 'note', textContent: 'This browser has no WebGL: the arm can’t be drawn.' })); return null; }
    const shader = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); return s; };
    const prog = gl.createProgram();
    gl.attachShader(prog, shader(gl.VERTEX_SHADER, VS));
    gl.attachShader(prog, shader(gl.FRAGMENT_SHADER, FS));
    gl.linkProgram(prog);
    gl.useProgram(prog);
    const loc = {
      pos: gl.getAttribLocation(prog, 'aPos'), nor: gl.getAttribLocation(prog, 'aNor'),
      vp: gl.getUniformLocation(prog, 'uVP'), m: gl.getUniformLocation(prog, 'uM'),
      color: gl.getUniformLocation(prog, 'uColor'), light: gl.getUniformLocation(prog, 'uLight'), flat: gl.getUniformLocation(prog, 'uFlat'),
    };
    const upload = (pos, nor) => {
      const p = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, p); gl.bufferData(gl.ARRAY_BUFFER, pos, gl.STATIC_DRAW);
      const n = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, n); gl.bufferData(gl.ARRAY_BUFFER, nor, gl.STATIC_DRAW);
      return { p, n };
    };
    const meshes = {};
    for (const [name, m] of Object.entries(model.meshes)) {
      const u = unpack(m);
      meshes[name] = { ...upload(u.pos, u.nor), count: u.count };
    }
    // The floor: a metre square, lines every 10 cm.
    const lines = [];
    for (let k = -5; k <= 5; k++) lines.push(k / 10, -0.5, 0, k / 10, 0.5, 0, -0.5, k / 10, 0, 0.5, k / 10, 0);
    const floorPos = new Float32Array(lines);
    const floor = { ...upload(floorPos, new Float32Array(floorPos.length).fill(0).map((_, i) => (i % 3 === 2 ? 1 : 0))), count: floorPos.length / 3 };

    // Which joint each motor turns, and through what.
    const motorOf = new Map(model.motors.map((m) => [m.id, m]));
    const jointBody = new Map(model.bodies.map((b, i) => [b.joint && b.joint.name, i]));
    function angles(byMotor) {
      const q = {};
      for (const [id, v] of byMotor) { const m = motorOf.get(id); if (m && v != null) q[m.joint] = m.offset + m.ratio * v; }
      for (const [joint, leader] of Object.entries(model.follows)) if (q[leader] != null) q[joint] = q[leader];
      return q;
    }
    function pose(q) {
      const world = [];
      model.bodies.forEach((b, i) => {
        let T = place(b.quat, b.pos);
        if (b.parent != null) T = mul(world[b.parent], T);
        if (b.joint) {
          const v = q[b.joint.name] || 0;
          T = mul(T, b.joint.type === 'slide' ? slide(b.joint.axis, v) : turn(b.joint.axis, v));
        }
        world[i] = T;
      });
      return world;
    }

    let reported = new Map(), told = null, flagged = new Set();
    let az = 2.35, el = 0.3, dist = 1.15;
    const target = [0, 0, 0.34];
    let colors;
    function readColors() {
      const cs = getComputedStyle(document.documentElement);
      const c = (k) => rgb(cs.getPropertyValue('--' + k).trim());
      const ink = c('ink'), paper = c('paper');
      colors = { link: mix(ink, paper, 0.28), base: mix(ink, paper, 0.45), fault: c('fault'), ghost: c('accent'), grid: c('rule') };
    }
    readColors();

    let queued = false;
    function draw() {
      queued = false;
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth, h = canvas.clientHeight;
      if (!w || !h) return;
      if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST);
      const eye = [target[0] + dist * Math.cos(el) * Math.cos(az), target[1] + dist * Math.cos(el) * Math.sin(az), target[2] + dist * Math.sin(el)];
      const vp = mul(perspective(0.62, w / h, 0.05, 20), lookAt(eye, target, [0, 0, 1]));
      gl.uniformMatrix4fv(loc.vp, false, vp);
      const l = [eye[0] - target[0] + 0.3, eye[1] - target[1] - 0.4, eye[2] - target[2] + 0.8], ln = Math.hypot(...l);
      gl.uniform3f(loc.light, l[0] / ln, l[1] / ln, l[2] / ln);
      const bind = (m) => {
        gl.bindBuffer(gl.ARRAY_BUFFER, m.p); gl.enableVertexAttribArray(loc.pos); gl.vertexAttribPointer(loc.pos, 3, gl.FLOAT, false, 0, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, m.n); gl.enableVertexAttribArray(loc.nor); gl.vertexAttribPointer(loc.nor, 3, gl.FLOAT, false, 0, 0);
      };
      // Floor
      gl.uniform1f(loc.flat, 1);
      gl.uniformMatrix4fv(loc.m, false, ident());
      gl.uniform4f(loc.color, ...colors.grid, 1);
      bind(floor);
      gl.drawArrays(gl.LINES, 0, floor.count);
      gl.uniform1f(loc.flat, 0);
      const bodies = (world, color) => {
        model.bodies.forEach((b, i) => {
          for (const g of b.geoms) {
            const m = meshes[g.mesh];
            gl.uniformMatrix4fv(loc.m, false, mul(world[i], place(g.quat, g.pos)));
            gl.uniform4f(loc.color, ...color(b, i));
            bind(m);
            gl.drawArrays(gl.TRIANGLES, 0, m.count);
          }
        });
      };
      // The arm, as its motors say it is: the base lighter, a flagged
      // joint's body red.
      bodies(pose(angles(reported)), (b, i) => {
        if (b.joint && flagged.has(b.joint.name)) return [...colors.fault, 1];
        return [...(i === 0 ? colors.base : colors.link), 1];
      });
      // Where it was told to be, see-through, over it.
      if (told) {
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        gl.depthMask(false);
        bodies(pose(angles(told)), () => [...colors.ghost, 0.2]);
        gl.depthMask(true);
        gl.disable(gl.BLEND);
      }
    }
    const redraw = () => { if (!queued) { queued = true; requestAnimationFrame(draw); } };

    // Turning and zooming.
    const pointers = new Map();
    let pinch = 0;
    canvas.addEventListener('pointerdown', (e) => { canvas.setPointerCapture(e.pointerId); pointers.set(e.pointerId, [e.clientX, e.clientY]); });
    canvas.addEventListener('pointermove', (e) => {
      if (!pointers.has(e.pointerId)) return;
      const [px, py] = pointers.get(e.pointerId);
      pointers.set(e.pointerId, [e.clientX, e.clientY]);
      if (pointers.size === 1) {
        az -= (e.clientX - px) * 0.01;
        el = Math.max(-0.2, Math.min(1.45, el + (e.clientY - py) * 0.01));
      } else if (pointers.size === 2) {
        const [a, b] = [...pointers.values()];
        const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch) dist = Math.max(0.5, Math.min(4, dist * (pinch / d)));
        pinch = d;
      }
      redraw();
    });
    const up = (e) => { pointers.delete(e.pointerId); pinch = 0; };
    canvas.addEventListener('pointerup', up);
    canvas.addEventListener('pointercancel', up);
    canvas.addEventListener('wheel', (e) => { e.preventDefault(); dist = Math.max(0.5, Math.min(4, dist * Math.exp(e.deltaY * 0.001))); redraw(); }, { passive: false });
    new ResizeObserver(redraw).observe(canvas);
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { readColors(); redraw(); });
    new MutationObserver(() => { readColors(); redraw(); }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

    const same = (a, b) => {
      if (a === b) return true;
      if (!a || !b || a.size !== b.size) return false;
      for (const [k, v] of a) if (Math.abs((b.get(k) ?? NaN) - v) > 1e-4) return false;
      return true;
    };
    return {
      /* Maps by motor id: the angles the motors report, the ones they were
         told (or null), and the ids of the motors keel flags. */
      set(rep, tol, flaggedIds) {
        const f = new Set([...flaggedIds].map((id) => motorOf.get(id) && motorOf.get(id).joint).filter(Boolean));
        const fs = [...f].sort().join();
        if (same(rep, reported) && same(tol, told) && fs === [...flagged].sort().join()) return;
        reported = rep; told = tol; flagged = f;
        redraw();
      },
      redraw,
      jointBody,
    };
  }

  window.Arm3D = { create };
})();
