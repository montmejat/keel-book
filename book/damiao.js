/* Damiao's MIT-mode frames, decoded as keel-can and drydock do, and keel's
   joint flags: shared by the book's figures (viz.js) and the run's page
   (run.js). */
(() => {
  'use strict';
  const KP_MAX = 500, KD_MAX = 5;
  const STATUS = { 0: 'disabled', 1: 'enabled', 8: 'overvoltage', 9: 'undervoltage', 10: 'overcurrent',
    11: 'drive overheated', 12: 'coil overheated', 13: 'communication lost', 14: 'overload' };
  const FLAGS = [[1, 'drive error'], [2, 'disabled'], [4, 'underpowered'], [8, 'behind'], [16, 'frozen']];
  const flagNames = (f) => FLAGS.filter(([b]) => f & b).map(([, n]) => n);
  const bytes = (h) => h.match(/../g).map((b) => parseInt(b, 16));
  const uf = (x, lo, hi, bits) => (x * (hi - lo)) / ((1 << bits) - 1) + lo;

  // Each field: its name, where it sits in the 16 hex digits, and its value.
  function command(h, m) {
    const d = bytes(h);
    const v = {
      p: uf((d[0] << 8) | d[1], -m.p, m.p, 16),
      v: uf((d[2] << 4) | (d[3] >> 4), -m.v, m.v, 12),
      kp: uf(((d[3] & 15) << 8) | d[4], 0, KP_MAX, 12),
      kd: uf((d[5] << 4) | (d[6] >> 4), 0, KD_MAX, 12),
      t: uf(((d[6] & 15) << 8) | d[7], -m.t, m.t, 12),
    };
    v.fields = [
      ['target', 0, 4, v.p.toFixed(3) + ' rad'], ['velocity', 4, 3, v.v.toFixed(2) + ' rad/s'],
      ['kp', 7, 3, v.kp.toFixed(1)], ['kd', 10, 3, v.kd.toFixed(2)], ['torque', 13, 3, v.t.toFixed(2) + ' N m'],
    ];
    return v;
  }
  function answer(h, m) {
    const d = bytes(h);
    const v = {
      status: d[0] >> 4,
      p: uf((d[1] << 8) | d[2], -m.p, m.p, 16),
      v: uf((d[3] << 4) | (d[4] >> 4), -m.v, m.v, 12),
      t: uf(((d[4] & 15) << 8) | d[5], -m.t, m.t, 12),
      mos: d[6], rotor: d[7],
    };
    v.fields = [
      ['status', 0, 1, STATUS[v.status] || String(v.status)], ['id', 1, 1, String(d[0] & 15)],
      ['position', 2, 4, v.p.toFixed(3) + ' rad'], ['velocity', 6, 3, v.v.toFixed(2) + ' rad/s'],
      ['torque', 9, 3, v.t.toFixed(2) + ' N m'], ['drive °C', 12, 2, String(v.mos)], ['coil °C', 14, 2, String(v.rotor)],
    ];
    return v;
  }
  window.Damiao = { command, answer, STATUS, FLAGS, flagNames, KP_MAX, KD_MAX };
})();
