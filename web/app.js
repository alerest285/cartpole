// Cart-pole console. One WebSocket; the server owns all state.
"use strict";

const $ = (id) => document.getElementById(id);
const timeline = new SampleTimeline();
let samplesAfter = -Infinity;
let ws = null, S = { schemas: [], params: {}, status: null, policies: null, runs: [], fit: null, cols: [], infos: {} };
let learnCurve = [];
const palette = new Map();
const css = (n) => {
  if (!palette.has(n)) palette.set(n, getComputedStyle(document.documentElement).getPropertyValue(n).trim());
  return palette.get(n);
};
matchMedia("(prefers-color-scheme: light)").addEventListener("change", () => { palette.clear(); drawLearn(); });

// ------------------------------------------------------------- websocket
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => onMsg(JSON.parse(e.data));
  ws.onclose = () => { $("backend").textContent = "disconnected, retrying"; window.Dashboard?.disconnected(); setTimeout(connect, 1000); };
}
function send(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }
setInterval(() => send({ type: "beat" }), 100);   // dead-man: the server disarms if this stops

function onMsg(m) {
  switch (m.type) {
    case "hello":
      timeline.clear();
      samplesAfter = m.status.t; // discard pre-reset/backfilled samples after a new hello
      Object.assign(S, m);
      buildModes(); renderStatus(m.status); renderPolicies(); renderRuns(); renderFit(); buildSafety(m.status);
      if (m.job && m.job.log) m.job.log.forEach(jobEvent);
      break;
    case "hello_partial":
      if (m.policies) S.policies = m.policies;
      if (m.runs) S.runs = m.runs;
      if ("fit" in m) S.fit = m.fit;
      if (m.schemas) { S.schemas = m.schemas; buildModes(true); }
      if (m.status) renderStatus(m.status);
      renderPolicies(); renderRuns(); renderFit();
      break;
    case "samples": ingest(m.rows); break;
    case "status":
      renderStatus(m.status);
      if (m.info) renderInfo(m.info);
      (m.msgs || []).forEach((x) => logLine("msgs", new Date(x.t * 1000).toLocaleTimeString() + "  " + x.msg));
      break;
    case "info": S.params[m.key] = m.params; if (m.key === S.status?.mode) renderInfo(m.info); break;
    case "job": m.events.forEach(jobEvent); break;
    case "runs": S.runs = m.runs; renderRuns(); break;
    case "error": logLine("msgs", "error: " + m.msg); flashFault(m.msg); break;
  }
  window.Dashboard?.sync(m);
}

function ingest(rows) {
  timeline.push(rows.filter((r) => r[0] >= samplesAfter));
}

// ---------------------------------------------------------------- status
const PHASE = ["", "waiting: lift the pole by hand", "swinging", "balancing", "resetting", "exploring", "exciting"];
function renderStatus(st) {
  if (!st) return;
  S.status = st;
  const be = st.backend || {};
  $("backend").textContent = be.name + (be.sim ? "" : "  LIVE HARDWARE");
  $("backend").classList.toggle("hw", !be.sim);
  $("sim-tools").hidden = !be.sim;
  $("demo").hidden = be.name !== "simulator";
  $("demo").textContent = st.armed ? "Restart demo" : "Run demo";
  $("sim-hint").hidden = be.name !== "simulator" || st.armed;
  $("arm").hidden = be.sim && st.mode === "idle";
  $("arm").textContent = be.sim ? "Run selected" : "Arm";
  $("armstate").textContent = st.armed ? (be.sim ? "running" : "ARMED") : (be.sim ? "ready" : "disarmed");
  $("armstate").classList.toggle("armed", st.armed);
  $("arm").disabled = st.armed;
  const f = $("fault");
  if (st.fault) { f.hidden = false; f.textContent = st.fault; } else f.hidden = true;
  if (!st.armed) $("r-v").textContent = "0.00";
  if ($("mode").value !== st.mode && document.activeElement !== $("mode")) { $("mode").value = st.mode; buildParams(); }
  $("mode-tag").textContent = labelOf(st.mode);
  $("phase").textContent = st.armed ? (st.ctrl.info.phase || "") : "";
  const t = st.timing || {};
  $("timing").textContent = `period mean ${fmt(t.period_ms_mean, 3)} ms  max ${fmt(t.period_ms_max, 2)} ms\ncompute max ${fmt(t.compute_ms_max, 3)} ms   overruns ${t.overruns ?? 0}`;
  $("hw").textContent = Object.entries(be).map(([k, v]) => `${k}: ${Array.isArray(v) ? v.join("; ") || "none" : v}`).join("\n")
    + `\ncounts: cart ${st.counts[0]}  pole ${st.counts[1]}`;
  if (document.activeElement !== $("estimator")) $("estimator").value = st.estimator;
  $("estimator").disabled = st.armed;
  $("derivative-w").disabled = st.armed || st.estimator !== "derivative";
  if (document.activeElement !== $("derivative-w")) $("derivative-w").value = st.estimator_w ?? 60;
}
const fmt = (v, d) => (v == null ? "—" : Number(v).toFixed(d));
const labelOf = (k) => (S.schemas.find((s) => s.key === k) || {}).label || k;
function flashFault(msg) { const f = $("fault"); f.hidden = false; f.textContent = msg; }

// --------------------------------------------------------- control panel
function buildModes(keep) {
  const sel = $("mode"); const cur = keep ? sel.value : S.status?.mode;
  sel.innerHTML = "";
  for (const s of S.schemas) { const o = document.createElement("option"); o.value = s.key; o.textContent = s.label; sel.appendChild(o); }
  sel.value = cur || "idle";
  buildParams();
}
$("mode").onchange = () => { send({ type: "mode", key: $("mode").value }); buildParams(); };

function buildParams() {
  const key = $("mode").value;
  const sc = S.schemas.find((s) => s.key === key); if (!sc) return;
  $("blurb").textContent = sc.blurb;
  const vals = S.params[key] || {};
  const form = $("params"); form.innerHTML = "";
  const choiceVals = new Set(sc.params.filter((q) => q.kind === "choice").flatMap((q) => q.choices));
  const selected = new Set(sc.params.filter((q) => q.kind === "choice").map((q) => String(vals[q.name] ?? q.default)));
  let lastGroup = null;
  for (const q of sc.params) {
    if (q.group && choiceVals.has(q.group) && !selected.has(q.group)) continue;
    if (q.group && q.group !== lastGroup) { const d = document.createElement("div"); d.className = "sub"; d.textContent = q.group; form.appendChild(d); }
    lastGroup = q.group;
    const lab = document.createElement("label"); lab.textContent = q.label; form.appendChild(lab);
    let inp;
    const v = vals[q.name] ?? q.default;
    if (q.kind === "choice") {
      inp = document.createElement("select");
      for (const c of q.choices) { const o = document.createElement("option"); o.value = c; o.textContent = c; inp.appendChild(o); }
      inp.value = v;
    } else if (q.kind === "bool") {
      inp = document.createElement("input"); inp.type = "checkbox"; inp.checked = !!v;
    } else {
      inp = document.createElement("input"); inp.type = "number"; inp.step = q.step; inp.min = q.lo; inp.max = q.hi; inp.value = v;
    }
    inp.dataset.name = q.name; inp.dataset.kind = q.kind;
    inp.onchange = () => {
      const val = q.kind === "bool" ? inp.checked : q.kind === "choice" ? inp.value : Number(inp.value);
      S.params[key] = { ...(S.params[key] || {}), [q.name]: val };
      send({ type: "params", key, values: { [q.name]: val } });
      if (q.kind === "choice") buildParams();
    };
    form.appendChild(inp);
    if (q.help) { const h = document.createElement("div"); h.className = "help"; h.textContent = q.help; form.appendChild(h); }
  }
  $("manual-group").hidden = !["manual", "manual_swing"].includes(key);
  renderInfo((S.status && S.status.mode === key) ? S.status.ctrl.info : S.infos[key] || {});
}
function renderInfo(info) {
  const lines = [];
  for (const [k, v] of Object.entries(info || {})) {
    if (k === "phase") continue;
    lines.push(`${k}: ${Array.isArray(v) ? v.join("  ") : v}`);
  }
  $("info").textContent = lines.join("\n");
}

$("arm").onclick = () => send({ type: "arm" });
$("demo").onclick = () => {
  if (S.status?.backend.name !== "simulator") return;
  document.querySelector('[data-tab="control"]').click();
  timeline.clear();
  $("xref").value = 0; $("xref-v").textContent = "0.0 cm";
  send({ type: "demo" });
};
$("stop").onclick = () => send({ type: "stop" });
document.addEventListener("keydown", (e) => {
  if (e.code === "Space" && !(e.target instanceof HTMLInputElement && e.target.type === "text")) { e.preventDefault(); send({ type: "stop" }); }
  if (["manual", "manual_swing"].includes(S.status?.mode) && (e.code === "ArrowLeft" || e.code === "ArrowRight")) {
    e.preventDefault(); const m = e.code === "ArrowLeft" ? -1 : 1; $("manual").value = m; sendManual();
  }
});
document.addEventListener("keyup", (e) => {
  if (e.code === "ArrowLeft" || e.code === "ArrowRight") { $("manual").value = 0; sendManual(); }
});
$("xref").oninput = () => { const v = Number($("xref").value); $("xref-v").textContent = v.toFixed(1) + " cm"; send({ type: "ref", x_ref: v / 100 }); };
function sendManual() { const v = Number($("manual").value); $("man-v").textContent = v.toFixed(2); send({ type: "ref", manual: v }); }
$("manual").oninput = sendManual;
$("manual").onchange = () => { $("manual").value = 0; sendManual(); };
let pointerLast = null, pointerMoved = 0;
const clearManual = () => { pointerLast = null; $("manual").value = 0; sendManual(); };
$("manual-pad").onpointerdown = (e) => { e.currentTarget.setPointerCapture(e.pointerId); pointerLast = { x: e.clientX, t: performance.now() }; };
$("manual-pad").onpointermove = (e) => {
  if (!pointerLast) return;
  const now = performance.now(), dt = Math.max(8, now - pointerLast.t);
  $("manual").value = Math.max(-1, Math.min(1, (e.clientX - pointerLast.x) / dt * 2));
  sendManual(); pointerLast = { x: e.clientX, t: now }; pointerMoved = now;
};
$("manual-pad").onpointerup = $("manual-pad").onpointercancel = clearManual;
window.addEventListener("blur", clearManual);
setInterval(() => {
  if (pointerLast && performance.now() - pointerMoved > 120 && Number($("manual").value) !== 0) {
    $("manual").value = 0; sendManual();
  }
}, 60);
$("poke-l").onclick = () => send({ type: "poke", dv: 1.5 });
$("poke-r").onclick = () => send({ type: "poke", dv: -1.5 });
$("reset-hang").onclick = () => send({ type: "reset_sim" });
$("reset-up").onclick = () => send({ type: "reset_sim", upright: true });
$("zero").onclick = () => send({ type: "zero" });
$("clear").onclick = () => send({ type: "clear_fault" });
$("estimator").onchange = () => send({ type: "estimator", kind: $("estimator").value });
$("derivative-w").onchange = () => send({ type: "estimator", kind: $("estimator").value, w: Number($("derivative-w").value) });

// ----------------------------------------------------------------- safety
const SAFETY = [
  ["v_cap", "Voltage cap [V]", 0.5, 10, 0.5], ["cbf", "Track barrier (CBF)", "bool"],
  ["x_soft", "Barrier at [m]", 0.05, 0.18, 0.005], ["wn", "Barrier stiffness [rad/s]", 2, 40, 1],
  ["x_hard", "Hard stop at [m]", 0.05, 0.195, 0.005],
];
function buildSafety(st) {
  const f = $("safety"); f.innerHTML = "";
  for (const [k, label, lo, hi, step] of SAFETY) {
    const l = document.createElement("label"); l.textContent = label; f.appendChild(l);
    const i = document.createElement("input");
    if (lo === "bool") { i.type = "checkbox"; i.checked = !!st.safety[k]; }
    else { i.type = "number"; i.min = lo; i.max = hi; i.step = step; i.value = st.safety[k]; }
    i.onchange = () => send({ type: "safety", values: { [k]: i.type === "checkbox" ? i.checked : Number(i.value) } });
    f.appendChild(i);
  }
}

// ------------------------------------------------------------------ learn
function renderPolicies() {
  const reg = S.policies || { versions: {} };
  const tb = $("pol-table").querySelector("tbody"); tb.innerHTML = "";
  const vers = Object.keys(reg.versions || {}).sort().reverse();
  for (const v of vers) {
    const e = reg.versions[v]; const hw = e.hardware;
    const tr = document.createElement("tr"); if (v === reg.current) tr.className = "cur";
    tr.innerHTML = `<td>${v}</td><td>${pct(e.eval?.success)}</td><td>${fmt(e.eval?.t_up_mean, 1)}</td>`
      + `<td>${hw ? pct(hw.success) + " /" + hw.runs : "—"}</td><td class="dim">${(e.note || "").slice(0, 28)}</td>`;
    tb.appendChild(tr);
  }
  if (!vers.length) tb.innerHTML = `<tr><td colspan="5" class="dim">no policy yet: Train new</td></tr>`;
  const cur = reg.current && reg.versions[reg.current];
  if (cur && cur.curve && !learnCurve.length) learnCurve = cur.curve;
  drawLearn();
}
const pct = (x) => (x == null ? "—" : Math.round(x * 100) + "%");
function jobEvent(ev) {
  if (ev.phase === "train" && ev.iter != null) {
    learnCurve.push(ev);
    if (ev.iter % 10 === 0) logLine("joblog", `iter ${ev.iter}/${ev.iters ?? "?"}  return ${fmt(ev.ep_return, 1)}  upright ${pct(ev.upright_end)}  fail ${pct(ev.fail_rate)}`);
    drawLearn();
  } else if (ev.phase === "log") logLine("joblog", ev.msg);
  else if (ev.phase === "finished") logLine("joblog", `job ${ev.kind} finished (exit ${ev.rc})` + (ev.stderr ? "\n" + ev.stderr : ""));
  else if (ev.phase) logLine("joblog", ev.phase + (ev.version ? " " + ev.version : ""));
}
$("improve").onclick = () => { learnCurve = []; send({ type: "improve", iters: Number($("imp-iters").value), include_sim: $("imp-sim").checked }); };
$("sysid").onclick = () => send({ type: "sysid", include_sim: $("imp-sim").checked });
$("cancel").onclick = () => send({ type: "cancel_job" });
$("train").onclick = () => { learnCurve = []; send({ type: "train", iters: Number($("tr-iters").value), rand_scale: Number($("tr-rand").value) }); };
$("train-warm").onclick = () => { learnCurve = []; send({ type: "train", iters: Number($("tr-iters").value), rand_scale: Number($("tr-rand").value), warm: "current" }); };

// ------------------------------------------------------------------- runs
function renderRuns() {
  const tb = $("runs-table").querySelector("tbody"); tb.innerHTML = "";
  for (const r of S.runs.slice(0, 80)) {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${r.id.slice(4, 15)}${r.sim ? "" : " <b>hw</b>"}</td><td>${r.mode}${r.ppo_episode ? "*" : ""}</td><td>${fmt(r.duration_s, 1)}</td>`
      + `<td class="dim">${(r.end || "").slice(0, 22)}</td><td><a href="/api/runs/${r.id}.csv">csv</a></td>`;
    tb.appendChild(tr);
  }
  if (!S.runs.length) tb.innerHTML = `<tr><td colspan="5" class="dim">nothing recorded yet: arm any mode</td></tr>`;
}
function renderFit() {
  const p = S.status?.plant; const f = S.fit?.current;
  let s = p ? `B_f ${fmt(p.B_f, 3)} N s/m   k_amp ${fmt(p.k_amp, 3)}\nB_p ${fmt(p.B_p, 4)}   F_coul ${fmt(p.F_coul, 3)} N   delay ${p.delay_steps}\nm_c ${fmt(p.m_c, 3)} kg   I_factor ${fmt(p.I_factor, 3)}` : "";
  if (f) s += `\n\nlast sysid ${f.time}\n${f.n_samples} estimation samples from ${f.runs.length} runs\nR2 cart ${f.r2_cart}, held-out ${f.validation_r2_cart ?? "not recorded"}, pole ${f.r2_pole}` + (f.problems?.length ? "\n" + f.problems.join("\n") : "");
  else s += "\n\nno sysid fit yet: record Identify runs, then Sysid";
  $("fit").textContent = s;
}
$("use-fit").onclick = () => send({ type: "reload_plant", fitted: true });
$("use-nominal").onclick = () => send({ type: "reload_plant", fitted: false });

// -------------------------------------------------------------------- tabs
document.querySelectorAll(".tabs button").forEach((b) => b.onclick = () => {
  window.Dashboard?.showConsole();
  document.querySelectorAll(".tabs button").forEach((x) => x.classList.toggle("on", x === b));
  document.querySelectorAll(".tab").forEach((t) => t.hidden = t.id !== "tab-" + b.dataset.tab);
  if (b.dataset.tab === "learn") drawLearn();
});
function logLine(id, s) { const el = $(id); el.textContent = (el.textContent + s + "\n").split("\n").slice(-200).join("\n"); el.scrollTop = el.scrollHeight; }

fetch("course.json").then((r) => { if (!r.ok) throw new Error("Course guide unavailable"); return r.json(); }).then((guide) => {
  $("course-note").textContent = guide.note;
  for (const step of guide.steps) {
    const section = document.createElement("section"); section.className = "course-step";
    const title = document.createElement("h3"); title.textContent = step.title; section.appendChild(title);
    const detail = document.createElement("p"); detail.textContent = step.detail; section.appendChild(detail);
    for (const command of step.commands) { const code = document.createElement("code"); code.textContent = command; section.appendChild(code); }
    const pending = document.createElement("p"); pending.className = "dim"; pending.textContent = "At the rig: " + step.pending; section.appendChild(pending);
    $("course-steps").appendChild(section);
  }
}).catch((e) => { $("course-note").textContent = e.message; });

// ---------------------------------------------------------------- drawing
const surfaces = new WeakMap();
const resizeCanvas = new ResizeObserver((entries) => {
  for (const { target, contentRect } of entries) {
    const s = surfaces.get(target);
    if (s) { s.w = contentRect.width; s.h = contentRect.height; }
    if (target.id === "ch-learn") drawLearn();
  }
});
function fit(canvas) {
  let s = surfaces.get(canvas);
  if (!s) {
    s = { g: canvas.getContext("2d"), w: canvas.clientWidth, h: canvas.clientHeight };
    surfaces.set(canvas, s); resizeCanvas.observe(canvas);
  }
  const d = devicePixelRatio || 1, w = Math.round(s.w * d), h = Math.round(s.h * d);
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  s.g.setTransform(d, 0, 0, d, 0, 0);
  return [s.g, s.w, s.h];
}
function drawScene(pose) {
  const [g, W, H] = fit($("scene"));
  g.clearRect(0, 0, W, H);
  const st = S.status; if (!st || !pose) return;
  const scale = Math.min((W - 64) / 0.62, (H - 68) / 0.76); // room for controls and full swing
  const cx = W / 2, cy = H * 0.5 + 10;
  const X = (m) => cx + m * scale;
  const ink = css("--ink"), dim = css("--dim"), line = css("--line"), acc = css("--accent"), stop = css("--stop");
  // track, soft limit, watchdog
  g.strokeStyle = line; g.lineWidth = 2; g.beginPath(); g.moveTo(X(-0.23), cy + 14); g.lineTo(X(0.23), cy + 14); g.stroke();
  const sf = st.safety || {};
  g.setLineDash([4, 4]); g.strokeStyle = dim; g.lineWidth = 1;
  for (const s of [-1, 1]) { g.beginPath(); g.moveTo(X(s * sf.x_soft), cy - 4); g.lineTo(X(s * sf.x_soft), cy + 24); g.stroke(); }
  g.setLineDash([]); g.strokeStyle = stop;
  for (const s of [-1, 1]) { g.beginPath(); g.moveTo(X(s * 0.2), cy - 8); g.lineTo(X(s * 0.2), cy + 28); g.stroke(); }
  g.fillStyle = dim; g.font = "11px " + css("--mono");
  g.fillText("-20 cm", X(-0.2) - 18, cy + 42); g.fillText("+20 cm", X(0.2) - 18, cy + 42);
  // setpoint
  const xr = st.ref?.x_ref || 0; g.fillStyle = acc; g.beginPath(); g.moveTo(X(xr), cy + 16); g.lineTo(X(xr) - 5, cy + 26); g.lineTo(X(xr) + 5, cy + 26); g.fill();
  // cart + pole
  const { x, a } = pose, L = 0.3556;
  const cw = 0.08 * scale, ch = 0.035 * scale;
  g.fillStyle = st.armed ? acc : dim; g.fillRect(X(x) - cw / 2, cy - ch / 2, cw, ch);
  const px = X(x), py = cy - ch / 2;
  const tx = px - Math.sin(a) * L * scale, ty = py - Math.cos(a) * L * scale;
  g.strokeStyle = ink; g.lineWidth = 6; g.lineCap = "round"; g.beginPath(); g.moveTo(px, py); g.lineTo(tx, ty); g.stroke();
  g.fillStyle = ink; g.beginPath(); g.arc(px, py, 5, 0, 7); g.fill();
  // voltage arrow
  const v = st.armed ? pose.v : 0;
  if (Math.abs(v) > 0.05) {
    g.strokeStyle = acc; g.lineWidth = 3; const y = cy; const x0 = v > 0 ? X(x) + cw / 2 + 4 : X(x) - cw / 2 - 4; const x1 = x0 + Math.sign(v) * Math.min(80, Math.abs(v) * 8);
    g.beginPath(); g.moveTo(x0, y); g.lineTo(x1, y); g.lineTo(x1 - Math.sign(v) * 6, y - 5); g.moveTo(x1, y); g.lineTo(x1 - Math.sign(v) * 6, y + 5); g.stroke();
  }
}
function balanceBands(end) {
  const bands = []; let start = null, prev = null;
  for (let i = 0; i < timeline.count; i++) {
    const r = timeline.row(i);
    if (r[0] > end) break;
    if (start !== null && (r[10] !== 3 || timeline.broken(i))) {
      bands.push([start, timeline.broken(i) ? prev[0] : r[0]]); start = null;
    }
    if (r[10] === 3 && start === null) start = r[0];
    prev = r;
  }
  if (start !== null) bands.push([start, end]);
  return bands;
}
const COLUMNS = { x: 1, a: 2, vd: 5, v: 6 };
function strip(id, keys, colors, lo, hi, label, unit, scaleF, pose, bands) {
  const [g, W, H] = fit($(id)); g.clearRect(0, 0, W, H);
  const pad = 30, w = W - pad - 8, h = H - 16;
  const Y = (v) => 8 + h - ((v * scaleF - lo) / (hi - lo)) * h;
  const end = pose?.t ?? 0, X = (t) => pad + (1 + (t - end) / 10) * w;
  g.save(); g.beginPath(); g.rect(pad, 8, w, h); g.clip();
  g.fillStyle = css("--accent") + "22";
  for (const [a, b] of bands) g.fillRect(X(a), 8, X(b) - X(a), h);
  g.strokeStyle = css("--line"); g.lineWidth = 1; g.beginPath(); g.moveTo(pad, Y(0)); g.lineTo(pad + w, Y(0)); g.stroke();
  keys.forEach((k, j) => {
    const col = COLUMNS[k], step = k === "v" || k === "vd";
    g.strokeStyle = colors[j]; g.lineWidth = j === 0 ? 1.6 : 1; g.beginPath();
    let prev = null;
    for (let i = 0; i < timeline.count; i++) {
      const r = timeline.row(i); if (r[0] > end) break;
      const px = X(r[0]), py = Y(r[col]);
      if (!prev || timeline.broken(i) || (k === "a" && Math.abs(r[col] - prev[col]) > Math.PI)) g.moveTo(px, py);
      else { if (step) g.lineTo(px, Y(prev[col])); g.lineTo(px, py); }
      prev = r;
    }
    // The live endpoint follows the same presentation clock as the pendulum.
    if (prev && pose && !(k === "a" && Math.abs(pose[k] - prev[col]) > Math.PI)) g.lineTo(X(end), Y(pose[k]));
    g.stroke();
  });
  g.restore();
  g.fillStyle = css("--dim"); g.font = "11px " + css("--mono");
  g.fillText(label, pad + 4, 20); g.fillText(String(hi) + unit, 2, 14); g.fillText(String(lo), 2, H - 6);
}
function drawLearn() {
  const c = $("ch-learn"); if (c.offsetParent === null) return;
  const [g, W, H] = fit(c); g.clearRect(0, 0, W, H);
  const pad = 34, w = W - pad - 10, h = H - 30;
  g.fillStyle = css("--dim"); g.font = "11px " + css("--mono");
  const reg = S.policies || { versions: {} }; const vers = Object.keys(reg.versions || {}).sort();
  if (learnCurve.length > 1) {
    const ys = learnCurve.map((r) => r.ep_return).filter((v) => v != null);
    if (ys.length > 1) {
      const lo = Math.min(...ys), hi = Math.max(...ys);
      g.fillText(`training return  ${fmt(lo, 0)} … ${fmt(hi, 0)}`, pad, 14);
      g.strokeStyle = css("--accent"); g.lineWidth = 1.5; g.beginPath();
      learnCurve.forEach((r, i) => { if (r.ep_return == null) return; const px = pad + (i / (learnCurve.length - 1)) * w, py = 20 + h - ((r.ep_return - lo) / (hi - lo || 1)) * h; i ? g.lineTo(px, py) : g.moveTo(px, py); });
      g.stroke(); return;
    }
  }
  if (vers.length) {
    g.fillText("per version: sim success (bar), rig success (dot)", pad, 14);
    const bw = Math.min(30, w / vers.length - 4);
    vers.forEach((v, i) => {
      const e = reg.versions[v], x = pad + i * (bw + 4);
      const sh = (e.eval?.success || 0) * h; g.fillStyle = css("--accent") + "88"; g.fillRect(x, 20 + h - sh, bw, sh);
      if (e.hardware?.success != null) { g.fillStyle = css("--ink"); g.beginPath(); g.arc(x + bw / 2, 20 + h - e.hardware.success * h, 4, 0, 7); g.fill(); }
      g.fillStyle = css("--dim"); g.fillText(v.slice(1), x, H - 2);
    });
  } else g.fillText("no policies yet", pad, 14);
}
let visualPose = null, lastReadout = -Infinity;
function frame(now) {
  const st = S.status;
  visualPose = timeline.sample(now) || (st ? { t: st.t, x: st.s[0], a: st.s[1], v: st.v } : null);
  drawScene(visualPose);
  if (visualPose && now - lastReadout >= 100) {
    $("r-x").textContent = (visualPose.x * 100).toFixed(2);
    $("r-a").textContent = (visualPose.a * 180 / Math.PI).toFixed(1);
    $("r-v").textContent = (st?.armed ? visualPose.v : 0).toFixed(2);
    lastReadout = now;
  }
  const bands = balanceBands(visualPose?.t ?? 0);
  strip("ch-x", ["x"], [css("--plot-x")], -20, 20, "cart x [cm] · last 10 s", "", 100, visualPose, bands);
  strip("ch-a", ["a"], [css("--plot-a")], -180, 180, "pole α [deg]  (0 = upright)", "", 180 / Math.PI, visualPose, bands);
  strip("ch-v", ["v", "vd"], [css("--plot-v"), css("--dim")], -10, 10, "voltage [V]  (grey: requested)", "", 1, visualPose, bands);
  requestAnimationFrame(frame);
}
if (location.hash) { const b = document.querySelector(`.tabs button[data-tab="${location.hash.slice(1)}"]`); if (b) b.click(); }
connect();
requestAnimationFrame(frame);
