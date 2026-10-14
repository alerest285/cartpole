// Run with node --test tests/test_timeline.cjs; no frontend dependencies.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const Timeline = vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../web/timeline.js'), 'utf8') + '\nSampleTimeline');
const row = (t, x = t / 100, a = 0, v = 0) => [t, x, a, 0, 0, v, v, 0, 0, 0, 3];
const near = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) < tolerance, `${a} != ${b}`);

test('angle seam takes the short path; voltage retains the measured step', () => {
  const s = new Timeline();
  s.push([row(0, 0, Math.PI - 0.02, -5), row(0.01, 0.002, -Math.PI + 0.02, 5)], 10);
  const mid = s.at(0.005);
  near(Math.abs(mid.a), Math.PI); near(mid.x, 0.001);
  assert.equal(mid.v, -5); assert.equal(s.at(0.01).v, 5);
  assert.equal(s.broken(1), false);
});

test('simulator reset is a jump, not a fictitious swing', () => {
  const s = new Timeline();
  s.push([row(0, 0.02, 0.01), row(0.01, 0, -Math.PI)], 10);
  near(s.at(0.009).a, 0.01); near(s.at(0.009).x, 0.02);
  near(s.at(0.01).a, -Math.PI); assert.equal(s.broken(1), true);
});

test('jittered packets produce monotonic frame-rate motion and never extrapolate', () => {
  const s = new Timeline(); let next = 0, last = null, changed = 0;
  // ~30 Hz batches, with 8 ms jitter; display runs at 60 Hz.
  for (let frame = 0; frame < 180; frame++) {
    const now = frame * 1000 / 60;
    if (frame % 2 === 0) {
      const edge = Math.max(0, Math.floor((now - (frame % 6 === 0 ? 8 : 0)) / 10));
      const rows = []; while (next <= edge) rows.push(row(next++ / 100));
      s.push(rows, now);
    }
    const pose = s.sample(now);
    assert.ok(pose.t <= s.row(s.count - 1)[0]);
    if (last !== null) { assert.ok(pose.t >= last); if (pose.t > last) changed++; }
    if (frame > 20) assert.ok(now / 1000 - pose.t >= 0.06 && now / 1000 - pose.t < 0.1);
    last = pose.t;
  }
  assert.ok(changed > 170, `${changed} frames moved`);
  const stopped = s.sample(10000); const still = s.sample(20000);
  near(stopped.t, still.t); near(stopped.x, still.x);
});

test('resume after suspension catches up to live samples without replay', () => {
  const s = new Timeline();
  s.push([row(0), row(0.01)], 10); s.sample(10);
  s.push(Array.from({ length: 20 }, (_, i) => row(10 + i / 100)), 10190);
  near(s.sample(10190).t, 10.12);
});

test('new server clock clears old history and follows the new epoch', () => {
  const s = new Timeline();
  s.push([row(100), row(100.01)], 10); s.sample(10);
  s.push([row(0), row(0.01)], 20);
  assert.equal(s.count, 2); near(s.sample(20).t, 0);
});

test('long sessions retain ordered, bounded history and correct interpolation', () => {
  const s = new Timeline(1200);
  s.push(Array.from({ length: 5000 }, (_, i) => row(i / 100)), 49990);
  assert.equal(s.count, 1200); near(s.row(0)[0], 38);
  near(s.at(40.005).x, 0.40005); near(s.sample(49990).t, 49.92);
});
