"use strict";

(() => {
  let data = null, busy = false, connected = !!S.status, job = { running: false, kind: null };
  let lastRuns = "", scope = "", frictionReport = null;
  const scene = document.querySelector(".scene-wrap");
  const make = (tag, text, cls) => { const e = document.createElement(tag); if (text != null) e.textContent = text; if (cls) e.className = cls; return e; };
  const n = (value, digits = 2) => value == null ? "—" : Number(value).toFixed(digits);
  function notice(message, error = false) { $("dashboard-notice").textContent = message; $("dashboard-notice").classList.toggle("error", error); }
  function showConsole(tab) {
    $("dashboard").hidden = true; $("console-view").hidden = false;
    $("console-scene").append(scene);
    $("show-dashboard").classList.remove("on"); $("show-console").classList.add("on");
    if (tab) document.querySelector(`[data-tab="${tab}"]`)?.click();
  }
  function showDashboard() {
    $("console-view").hidden = true; $("dashboard").hidden = false;
    $("dashboard-scene").append(scene);
    $("show-dashboard").classList.add("on"); $("show-console").classList.remove("on");
    refresh();
  }
  function controls() {
    const unavailable = !connected || busy || job.running || S.status?.armed;
    document.querySelectorAll("[data-idle]").forEach(b => { b.disabled = !!unavailable; b.title = unavailable ? "Stop the controller and wait for any job to finish" : ""; });
    $("import-recording").disabled ||= !!$("friction-run").value;
    $("dashboard-cancel").disabled = !job.running;
    $("dashboard-job-state").textContent = job.running ? `${job.kind} · running` : "idle";
    $("dashboard-job-state").classList.toggle("armed", !!job.running);
    // Offline computations reserve the machine while a job is running.
    if (job.running) { $("arm").disabled = true; $("demo").disabled = true; }
    else { $("demo").disabled = !connected; $("arm").disabled = !connected || !!S.status?.armed; }
  }
  function selectController(key, values) {
    if (!connected || S.status?.armed || job.running) return notice("Stop the controller before choosing a commissioning method.", true);
    send({ type: "mode", key });
    if (values) send({ type: "params", key, values });
    showConsole("control");
    notice("Controller selected. Review settings in the control room before arming.");
  }
  const sequence = [
    ["Verify signs & timing", "Centre, hang, zero; complete the hardware probe.", "Setup", () => showConsole("setup")],
    ["Record the plant", "30–40 s of varied motion in both directions.", "Identify", () => selectController("excite")],
    ["Fit & validate", "Check friction and confirm on a separate recording.", "Analyze", () => { showDashboard(); $("friction-form").scrollIntoView({ behavior: "smooth", block: "center" }); }],
    ["Establish balance", "LQR at 4 V; quiet hold, then a small disturbance.", "LQR", () => selectController("statefb", { method: "lqr" })],
    ["Attempt swing-up", "Use the validated model; keep track limits active.", "Energy", () => selectController("energy")],
  ];
  function checklist() {
    const next = S.status?.backend.sim ? "simulation" : "physical rig";
    if (scope === next) return;
    scope = next;
    $("check-scope").textContent = `${scope} · notes saved in this browser`;
    let stored = [];
    try { stored = JSON.parse(localStorage.getItem("cartpole-checklist-" + scope) || "[]"); } catch (_) { /* storage may be disabled */ }
    if (!Array.isArray(stored)) stored = [];
    $("commissioning-steps").replaceChildren();
    const update = () => { $("check-count").textContent = `${document.querySelectorAll(".check-step input:checked").length} / 5 noted`; };
    sequence.forEach(([title, detail, action, run], i) => {
      const row = make("div", null, "check-step"), input = make("input"); input.type = "checkbox"; input.checked = stored[i] === true; input.id = `check-${i}`;
      const label = make("label"); label.htmlFor = input.id; label.append(make("strong", title), make("span", detail));
      const button = make("button", action + " →", "text-button"); button.onclick = run;
      if ([1,3,4].includes(i)) button.dataset.idle = "";
      input.onchange = () => { update(); try { localStorage.setItem("cartpole-checklist-" + scope, JSON.stringify([...document.querySelectorAll(".check-step input")].map(x => x.checked))); } catch (_) { /* keep in-memory state */ } };
      row.append(input, label, button); $("commissioning-steps").append(row);
    }); update();
  }
  function model() {
    const p = S.status?.plant; if (!p) return;
    const rows = [["Viscous friction B_f", `${n(p.B_f, 3)} N s/m`], ["Coulomb friction", `${n(p.F_coul, 3)} N`], ["Motor gain", n(p.k_amp, 3)], ["Cart mass", `${n(p.m_c, 3)} kg`], ["Actuation delay", `${p.delay_steps} samples`], ["Voltage cap", `${n(S.status.safety.v_cap, 1)} V`]];
    $("dashboard-plant").replaceChildren(...rows.flatMap(([a,b]) => [make("dt", a), make("dd", b)]));
    const f = S.fit?.current;
    const matches = f?.params && Object.entries(f.params).every(([key,value]) => Math.abs(Number(p[key]) - Number(value)) < 1e-8);
    $("model-label").textContent = matches ? "fitted model active" : "current model";
    $("dashboard-fit").textContent = f ? `Latest joint fit: held-out R² ${n(f.validation_r2_cart, 4)} · ${f.runs?.length ?? 0} recordings. ${matches ? "Matches the active parameters." : "Review and apply in Runs."}` : "No joint fit saved. Current friction values are priors until measured.";
  }
  function runs() {
    const signature = JSON.stringify((S.runs || []).map(r => r.id)); if (signature === lastRuns) return; lastRuns = signature;
    const select = $("friction-run"), chosen = select.value;
    select.replaceChildren(new Option("Upload CSV instead", ""));
    for (const r of S.runs || []) select.add(new Option(`${r.sim ? "SIM" : "RIG"} · ${r.mode} · ${n(r.duration_s,1)} s · ${r.id}`, r.id));
    if ([...select.options].some(o => o.value === chosen)) select.value = chosen;
    uploadSelection();
  }
  function uploadSelection() { $("upload-fields").hidden = !!$("friction-run").value; controls(); }
  function metric(label, value, detail) { const card = make("div", null, "metric"); card.append(make("span", label, "metric-label"), make("strong", value, "metric-value"), make("span", detail, "small-note")); return card; }
  function evidenceLink(key, label) { const a = make("a", label); a.href = "/api/evidence/" + key; a.target = "_blank"; a.rel = "noopener"; return a; }
  function reports() {
    if (!data) return;
    const r = data.reports, bench = r.bench || [], rehearsal = r.rehearsal, search = r.search;
    const passed = bench.reduce((s,b) => s + b.passed,0), total = bench.reduce((s,b) => s + b.of,0);
    $("dashboard-metrics").replaceChildren(
      metric("Standard benchmark", total ? `${passed} / ${total}` : "—", "simulation cases passed"),
      metric("Identification rehearsal", n(rehearsal?.fit?.validation_r2_cart,4), "held-out cart R² · simulated data"),
      metric("Swing-up after fitting", rehearsal ? `${n(rehearsal.after_refit.t_upright)} s` : "—", "mismatched simulated plant"),
      metric("Physical commissioning", "Pending", "signs, friction, balance & swing-up")
    );
    const tbody = $("dashboard-bench").querySelector("tbody"); tbody.replaceChildren();
    for (const b of bench) { const row = make("tr"); for (const value of [b.controller,`${b.passed}/${b.of}`, b.t_up_mean == null ? "upright start" : `${n(b.t_up_mean)} s`,`${n(b.x_max_cm,1)} cm`]) row.append(make("td",value)); tbody.append(row); }
    if (!bench.length) { const row = make("tr"), cell = make("td","No benchmark report yet."); cell.colSpan=4; row.append(cell); tbody.append(row); }
    const target = $("dashboard-evidence"); target.replaceChildren();
    if (search) { const e = make("div",null,"evidence-item"); e.append(make("strong",`${search.passed}/${search.total} on fresh combined-error cases`),make("span","The broader energy search still fails cases. Its candidate was not installed. "),evidenceLink("search","Inspect report ↗")); target.append(e); }
    if (rehearsal) { const e = make("div",null,"evidence-item"); e.append(make("strong","Identification recovered a failed swing-up"),make("span",`40 s recording → model fit → balance in ${n(rehearsal.after_refit.t_upright)} s. Simulation only. `),evidenceLink("rehearsal","Inspect rehearsal ↗")); target.append(e); }
    if (r.tuning) { const e = make("div",null,"evidence-item"); e.append(evidenceLink("tuning","Latest local uncertainty sweep ↗")); target.append(e); }
    if (r.playground) { const ilc = r.playground.ilc || [], e=make("div",null,"evidence-item"); e.append(make("strong", "Offline playground"),make("span", `ILC tracking: ${n(ilc[0]?.rms_cm,3)} → ${n(Math.min(...ilc.map(x=>x.rms_cm)),3)} cm RMS. Show / MPC results are simulation only. `),evidenceLink("playground","Results ↗")); target.append(e); }
  }
  const courseActions = [["5 V / 10 Hz method",()=>selectController("sine_test")],["Analyze friction",()=>$("friction-form").scrollIntoView({behavior:"smooth",block:"center"})],["PID controls",()=>selectController("pid")],["LQR / pole placement",()=>selectController("statefb")],["Swing-up controls",()=>selectController("energy")],["PPO & learning",()=>showConsole("learn")]];
  function course() {
    const target=$("dashboard-course"); target.replaceChildren();
    (data.course?.steps || []).forEach((step,i)=>{
      const card=make("section",null,"dash-card course-card"); card.append(make("div",`EXERCISE 0${i+1}`,"step-number"),make("h3",step.title.split(" · ").slice(1).join(" · ")),make("p",step.detail,"small-note"));
      const button=make("button",courseActions[i][0]+" →","btn small"); button.onclick=courseActions[i][1]; card.append(button);
      if ([0,2,3,4].includes(i)) button.dataset.idle = "";
      const details=make("details"); details.append(make("summary","Commands & rig tasks"));
      for(const cmd of step.commands) details.append(make("div",cmd,"command"));
      details.append(make("p",step.pending,"rig-pending")); card.append(details); target.append(card);
    });
  }
  function materials() {
    $("dashboard-materials").replaceChildren();
    for (const item of data.materials) {
      const link=make(item.format==="md"?"button":"a",null,"dash-card material-link"); link.append(make("strong",item.title),make("span", item.available ? item.format+" ↗" : "not built"));
      if (item.format==="md") link.onclick=()=>openDocument(item.key);
      else { link.href=item.url; link.target="_blank"; link.rel="noopener"; }
      if (!item.available) { link.removeAttribute("href"); link.disabled=true; }
      $("dashboard-materials").append(link);
    }
  }
  async function refresh() {
    try { const response=await fetch("/api/dashboard"); if(!response.ok) throw Error("Dashboard data unavailable"); data=await response.json(); reports(); course(); materials(); if(data.reports.friction) showFriction(data.reports.friction); }
    catch(error) { notice(error.message,true); }
  }
  function showFriction(result) {
    frictionReport=result; const target=$("friction-result"); target.replaceChildren();
    target.append(make("span",result.accepted?"Accepted for review":"Rejected — inspect data",`pill ${result.accepted?"accepted":"rejected"}`));
    target.append(make("p",`${result.source?.sim === false ? "RIG" : "SIM"} · ${result.source?.name || "recording"} · ${n(result.duration_s,1)} s · ${result.n_train} estimation / ${result.n_validation} validation samples`,"small-note"));
    const table=make("table",null,"tbl"), header=make("tr"); for(const text of ["Model","B_f [N s/m]","F_c [N]","Held-out R²"]) header.append(make("th",text)); table.append(header);
    for(const [key,m] of Object.entries(result.models)) { const row=make("tr"); for(const value of [key==="viscous"?"Viscous":"+ Coulomb",n(m.B_f,3),n(m.F_coul,3),n(m.validation_r2,4)]) row.append(make("td",value)); table.append(row); }
    target.append(table,make("p",`Selected: ${result.selected}. ${result.note}`,"small-note"));
    for(const problem of result.models[result.selected].problems) target.append(make("p",problem,"small-note"));
    const button=make("button","Download report","btn small"); button.type="button"; button.onclick=()=>{ const url=URL.createObjectURL(new Blob([JSON.stringify(frictionReport,null,2)],{type:"application/json"}));const a=make("a");a.href=url;a.download="friction-report.json";a.click();setTimeout(()=>URL.revokeObjectURL(url),1000); };target.append(button);
  }
  async function payload() {
    const rid=$("friction-run").value;
    if(rid) return {run_id:rid,delay_steps:Number($("friction-delay").value)};
    const file=$("friction-file").files[0];if(!file) throw Error("Choose a CSV recording first.");
    if(file.size>7*1024*1024) throw Error("Use a CSV smaller than 7 MB.");
    return {csv:await file.text(),filename:file.name,hardware:$("upload-hardware").checked,delay_steps:Number($("friction-delay").value)};
  }
  async function analyze(importing=false) {
    busy=true;controls();
    try { const body=await payload(); const response=await fetch(importing?"/api/import":"/api/friction",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)}); const result=await response.json();if(!response.ok)throw Error(result.error||"Request failed");
      if(importing) { notice(`Imported ${result.sim?"simulated":"physical-rig"} recording ${result.id}. It is available in Runs and joint identification.`);send({type:"refresh"}); }
      else { showFriction(result);notice(result.accepted?"Friction report ready for review. Active model unchanged.":"Friction fit rejected. Review the reported issues before collecting another record.",!result.accepted); }
    } catch(error) { notice(error.message,true);$("friction-result").replaceChildren(make("p",error.message)); }
    finally {busy=false;controls();}
  }
  async function openDocument(key) {
    const item=data?.materials.find(x=>x.key===key);$("document-title").textContent=item?.title||"Lab guide";$("document-content").textContent="Loading…";$("document-dialog").showModal();
    try {const response=await fetch("/materials/"+key);if(!response.ok)throw Error("Document unavailable");const source=await response.text();const target=$("document-content");target.replaceChildren();let block=null;
      for(const line of source.split("\n")) {if(line.startsWith("```")){if(block)block=null;else{block=make("pre");target.append(block);}continue;}if(block){block.textContent+=line+"\n";continue;}if(!line.trim())continue;const heading=line.match(/^(#{1,4})\s+(.*)/);if(heading)target.append(make("h"+Math.min(heading[1].length+1,4),heading[2]));else target.append(make("p",line,line.startsWith("|")?"md-table":""));}
    }catch(error){$("document-content").textContent=error.message;}
  }
  function sync(message) {
    if(message.type==="hello" || message.type==="status" || message.type==="hello_partial") { connected=true; checklist();model(); }
    if(message.type==="hello" && message.job) {job=message.job;if(job.log?.length)for(const e of job.log)jobLog(e);}
    if(["hello","hello_partial","runs"].includes(message.type))runs();
    if(message.type==="job") { job={running:message.running,kind:message.kind}; message.events.forEach(jobLog);if(!job.running)refresh(); }
    if(message.type==="error") {notice(message.msg,true);send({type:"refresh"});}
    controls();
  }
  function jobLog(event) {const text=event.phase==="finished" ? `${event.kind}: ${event.rc===0?"completed":"failed (exit "+event.rc+")"}\n${event.stdout||""}${event.stderr||""}` : event.msg || [event.phase,event.kind,event.iter!=null?`iteration ${event.iter}`:""].filter(Boolean).join(" · ");logLine("dashboard-joblog",text);}
  $("show-dashboard").onclick=showDashboard;$("show-console").onclick=()=>showConsole("control");
  $("friction-form").onsubmit=e=>{e.preventDefault();analyze();};$("import-recording").onclick=()=>analyze(true);$("friction-run").onchange=uploadSelection;
  $("refresh-dashboard").onclick=()=>{refresh();send({type:"refresh"});};$("dashboard-cancel").onclick=()=>send({type:"cancel_job"});
  $("close-document").onclick=()=>$("document-dialog").close();
  document.querySelectorAll("[data-console]").forEach(b=>b.onclick=()=>showConsole(b.dataset.console));
  document.querySelectorAll("[data-doc]").forEach(b=>b.onclick=()=>openDocument(b.dataset.doc));
  document.querySelectorAll("[data-job]").forEach(b=>b.onclick=()=>{job={running:true,kind:b.dataset.job};controls();logLine("dashboard-joblog",`Starting ${job.kind}…`);send({type:"lab_job",task:job.kind});});
  window.Dashboard={sync,showConsole,showDashboard,refresh,disconnected(){connected=false;controls();notice("Controller disconnected. Reconnecting; displayed measurements are frozen.",true);}};
  showDashboard();sync({type:"hello",job:S.job||{running:false}});
  notice("Simulation and software evidence are ready. Physical commissioning and MATLAB / QUARC execution remain pending.");
  if(["control","learn","runs","setup","course"].includes(location.hash.slice(1)))showConsole(location.hash.slice(1));
})();
