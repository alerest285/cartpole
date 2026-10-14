// Presentation only: timestamped telemetry, never fed back to the controller.
"use strict";

class SampleTimeline {
  constructor(capacity = 1200, delay = 0.07) {
    this.capacity = capacity; this.delay = delay;
    this.rows = new Array(capacity);
    this.breaks = new Uint8Array(capacity);
    this.clear();
  }

  clear() {
    this.head = 0; this.count = 0;
    this.offset = null; this.time = null; this.arrival = null;
  }

  row(i) { return this.rows[(this.head - this.count + i + this.capacity) % this.capacity]; }
  broken(i) { return !!this.breaks[(this.head - this.count + i + this.capacity) % this.capacity]; }
  static angleDelta(a, b) { return Math.atan2(Math.sin(b - a), Math.cos(b - a)); }

  push(rows, now = performance.now()) {
    if (!rows.length) return;
    // A suspended tab or interrupted stream should resume at the live edge.
    if (this.arrival !== null && now - this.arrival > 250) this.offset = this.time = null;
    for (const r of rows) {
      let prev = this.count ? this.row(this.count - 1) : null;
      if (prev && r[0] < prev[0]) { this.clear(); prev = null; }
      if (prev && r[0] === prev[0]) continue;
      // Resets are discontinuities, not a fast swing through imaginary states.
      const jump = prev && (r[0] - prev[0] > 0.1 || Math.abs(r[1] - prev[1]) > 0.1
        || Math.abs(SampleTimeline.angleDelta(prev[2], r[2])) > Math.PI / 2);
      this.rows[this.head] = r;
      this.breaks[this.head] = jump ? 1 : 0;
      this.head = (this.head + 1) % this.capacity;
      this.count = Math.min(this.capacity, this.count + 1);
    }
    const offset = this.row(this.count - 1)[0] - now / 1000;
    // Filter packet-arrival jitter; the display clock advances every frame.
    this.offset = this.offset === null ? offset : this.offset + 0.05 * (offset - this.offset);
    this.arrival = now;
  }

  at(t) {
    if (!this.count) return null;
    let lo = 0, hi = this.count - 1;
    while (lo < hi) {
      const mid = Math.ceil((lo + hi) / 2);
      if (this.row(mid)[0] <= t) lo = mid; else hi = mid - 1;
    }
    const a = this.row(lo), b = this.row(Math.min(lo + 1, this.count - 1));
    const f = a === b || this.broken(lo + 1) ? 0 : Math.max(0, Math.min(1, (t - a[0]) / (b[0] - a[0])));
    const angle = a[2] + f * SampleTimeline.angleDelta(a[2], b[2]);
    return { t, x: a[1] + f * (b[1] - a[1]), a: Math.atan2(Math.sin(angle), Math.cos(angle)),
      v: a[6], vd: a[5], ph: a[10] }; // voltage is held, never cosmetically filtered
  }

  sample(now = performance.now()) {
    if (!this.count) return null;
    const oldest = this.row(0)[0], latest = this.row(this.count - 1)[0];
    // No extrapolation: when samples stop, both the rig and plots freeze.
    this.time = Math.min(latest, Math.max(oldest, this.time ?? oldest, now / 1000 + this.offset - this.delay));
    return this.at(this.time);
  }
}
