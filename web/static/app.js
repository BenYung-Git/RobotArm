/* RobotArm virtual demo — Phase 1C frontend logic.
 *
 * Polling:
 *   GET /api/v1/status  every 400 ms  → cabinet + arm snapshot
 *   No WebSocket, no Socket.IO, no Server-Sent Events.
 *
 * Actions (all POST, all cabinet-mediated):
 *   /api/v1/cabinet/estop
 *   /api/v1/cabinet/reset-estop
 *   /api/v1/cabinet/protective-stop
 *   /api/v1/cabinet/reset-protective-stop
 *   /api/v1/cabinet/fault
 *   /api/v1/cabinet/reset-fault
 *   /api/v1/virtual-io/inputs/{signal}
 *   /api/v1/jobs/demo-sequence/start
 *   /api/load_pose   (legacy; cabinet-mediated)
 *   /api/move_home   (legacy; cabinet-mediated)
 *   /api/set_targets (legacy; cabinet-mediated)
 *
 * All requests go to the same origin (Flask serves the page too), so
 * no absolute URLs / no CORS handling needed. The Cloudflare tunnel
 * just proxies HTTP traffic.
 *
 * DISABLED states (gating buttons):
 *   motion buttons  ← disabled when cabinet_state ∈ {ESTOP, PROTECTIVE_STOP, FAULT, RUNNING}
 *   job start       ← disabled when cabinet_state ≠ READY
 *   reset buttons   ← disabled when not in the matching fault state
 */
(function () {
  "use strict";

  // ---- State --------------------------------------------------------------
  let spec = null;
  let STRINGS = null;
  let FALLBACK = null;
  let lastStatus = null;
  let lastJointRad = [0, 0, 0, 0, 0, 0];
  let lastTargetsRad = [0, 0, 0, 0, 0, 0];
  let lastTcp = [0, 0, 0];
  let userDragging = false;

  // Polling
  const POLL_MS = 400;  // 2.5 Hz — within brief's 250–500 ms range
  let pollTimer = null;

  // ---- Helpers -----------------------------------------------------------
  function el(tag, attrs, children) {
    const e = document.createElement(tag);
    if (attrs) {
      for (const [k, v] of Object.entries(attrs)) {
        if (k === "class") e.className = v;
        else if (k === "dataset") Object.assign(e.dataset, v);
        else if (k.startsWith("on") && typeof v === "function") {
          e.addEventListener(k.slice(2).toLowerCase(), v);
        } else if (v === true) e.setAttribute(k, "");
        else if (v === false || v == null) {/* skip */}
        else e.setAttribute(k, v);
      }
    }
    if (children != null) {
      for (const c of [].concat(children)) {
        if (c == null) continue;
        e.appendChild(typeof c === "string" ? document.createTextNode(c) : c);
      }
    }
    return e;
  }

  async function api(path, opts) {
    opts = opts || {};
    if (opts.body && typeof opts.body !== "string") {
      opts.body = JSON.stringify(opts.body);
      opts.headers = Object.assign({"Content-Type": "application/json"}, opts.headers || {});
    }
    const r = await fetch(path, opts);
    let json = null;
    try { json = await r.json(); } catch (_) {}
    if (!r.ok || (json && json.ok === false)) {
      const msg = (json && json.error) || `${path} → HTTP ${r.status}`;
      throw new Error(msg);
    }
    return json;
  }

  function t(key) {
    if (STRINGS && Object.prototype.hasOwnProperty.call(STRINGS, key)) return STRINGS[key];
    if (FALLBACK && Object.prototype.hasOwnProperty.call(FALLBACK, key)) return FALLBACK[key];
    return key;
  }

  function applyStaticI18n() {
    const nodes = document.querySelectorAll("[data-i18n]");
    for (const node of nodes) {
      const key = node.getAttribute("data-i18n");
      const text = t(key);
      if (text == null) continue;
      if (text.indexOf("<") !== -1) node.innerHTML = text;
      else node.textContent = text;
    }
    document.documentElement.setAttribute("lang",
      (STRINGS && STRINGS.__lang) || "en");
  }

  async function loadTranslations() {
    const params = new URLSearchParams(window.location.search);
    const q = params.get("lang");
    let url = "/api/translations";
    if (q) url += "?lang=" + encodeURIComponent(q);
    try {
      const j = await api(url);
      STRINGS = j.strings || {};
      STRINGS.__lang = j.lang || "en";
      FALLBACK = j.fallback || {};
    } catch (e) {
      STRINGS = {};
      FALLBACK = {};
      console.warn("translations load failed:", e);
    }
    applyStaticI18n();
  }

  // ---- Spec / sliders -----------------------------------------------------
  async function loadSpec() {
    try {
      const j = await api("/api/spec");
      spec = (j && j.spec) ? j.spec : j;
      renderSliders();
    } catch (e) {
      const host = document.getElementById("slider-list");
      host.textContent = "";
      host.appendChild(
        el("p", { class: "hint error" }, t("loading_failed") + e.message)
      );
    }
  }

  function renderSliders() {
    const host = document.getElementById("slider-list");
    host.innerHTML = "";
    for (let i = 0; i < spec.dof; i++) {
      const name = spec.joint_names[i];
      const lo = spec.lower_limits_deg[i];
      const hi = spec.upper_limits_deg[i];
      const localized = t(`joint_${i}`);
      const displayName = (localized && localized !== `joint_${i}`)
        ? localized
        : name;
      const row = el("div", { class: "slider-row" });
      const label = el("label", { for: `slider-${i}` }, `[${i}] ${displayName}`);
      const input = el("input", {
        type: "range",
        id: `slider-${i}`,
        min: lo, max: hi, step: 0.5, value: 0,
        "data-index": i,
        title: name,
      });
      input.addEventListener("pointerdown", () => { userDragging = true; });
      input.addEventListener("pointerup", () => { userDragging = false; });
      input.addEventListener("input", onSliderInput);
      input.addEventListener("change", onSliderChange);
      const val = el("span", { class: "val", id: `val-${i}` }, "0.00°");
      row.appendChild(label);
      row.appendChild(input);
      row.appendChild(val);
      host.appendChild(row);
    }
  }

  function syncSlidersFromState() {
    if (!spec || userDragging) return;
    for (let i = 0; i < spec.dof; i++) {
      const input = document.getElementById(`slider-${i}`);
      const v = lastTargetsRad[i] * 57.29577951308232;
      if (input && Math.abs(parseFloat(input.value) - v) > 0.1) {
        input.value = v;
      }
      const valSpan = document.getElementById(`val-${i}`);
      if (valSpan) valSpan.textContent = `${v.toFixed(2)}°`;
    }
  }

  let lastSent = 0;
  function onSliderInput(ev) {
    const i = +ev.target.dataset.index;
    const valSpan = document.getElementById(`val-${i}`);
    if (valSpan) valSpan.textContent = `${parseFloat(ev.target.value).toFixed(2)}°`;
    const now = Date.now();
    if (now - lastSent < 200) return;
    lastSent = now;
    sendTargetsFromSliders();
  }
  function onSliderChange(ev) { sendTargetsFromSliders(); }

  function sendTargetsFromSliders() {
    if (!spec) return;
    const joints = [];
    for (let i = 0; i < spec.dof; i++) {
      joints.push(parseFloat(document.getElementById(`slider-${i}`).value));
    }
    // Send to cabinet (legacy endpoint) — same backend, cabinet mediates.
    api("/api/set_targets", { method: "POST", body: { joints } })
      .then((j) => {
        if (j && j.state) {
          lastJointRad = j.state.joints_rad || lastJointRad;
          lastTargetsRad = j.state.targets_rad || lastTargetsRad;
          lastTcp = j.state.tcp_position || lastTcp;
          updateCanvas();
        }
      })
      .catch((e) => { console.warn("set_targets:", e.message); });
  }

  // ---- Status polling ----------------------------------------------------
  // Brief §7.2 (modified): no WebSocket. Polling /api/v1/status at POLL_MS.
  async function poll() {
    try {
      const j = await api("/api/v1/status");
      applyStatus(j.status);
    } catch (e) {
      console.warn("poll:", e.message);
    }
  }

  function applyStatus(s) {
    if (!s) return;
    lastStatus = s;
    lastJointRad = (s.joint_positions_deg || []).map(d => d / 57.29577951308232);
    lastTargetsRad = (s.target_positions_deg || []).map(d => d / 57.29577951308232);
    if (Array.isArray(s.tcp_estimate_m)) lastTcp = s.tcp_estimate_m;

    // Cabinet identity strip
    setText("id-mode", s.mode || "");
    setText("id-cabinet-type", s.cabinet_type || "");
    setText("id-real-ctrl", s.real_robot_control || "");
    setText("id-phys-cabinet", s.controller_connection || "");

    // Cabinet state badge (color-coded)
    const cabState = s.cabinet_state || "—";
    const cabStateEl = document.getElementById("id-cabinet-state");
    if (cabStateEl) {
      cabStateEl.textContent = cabState;
      cabStateEl.className = "id-value state-badge state-" + cabState;
    }

    // Cabinet Status panel
    setText("cab-motion", s.motion_state || "");
    setText("cab-safety", s.safety_state || "");
    if (s.fault) {
      setText("cab-fault", `${s.fault.code || ""} — ${s.fault.message || ""}`);
    } else {
      setText("cab-fault", t("fault_none"));
    }
    setText("cab-updated", s.updated_at ? new Date(s.updated_at * 1000).toLocaleTimeString() : "—");

    // Job
    // Render priority (per 2026-09-16 bug-fix spec):
    //   1. active_job_id set       -> live progress for that running job
    //   2. last_job_result present -> terminal result of the most recent job
    //   3. neither                 -> show "—"
    if (s.active_job_id) {
      setText("job-id", s.active_job_id);
      setText("job-progress",
        `${s.current_step_index || 1} / ${s.total_steps || "?"}`);
      setText("job-step",
        s.current_step_id
          ? `${s.current_step_id} (${s.current_step_name || ""})`
          : (s.current_step_name || "—"));
      setText("job-elapsed", "");
    } else if (s.last_job_result) {
      // Terminal result rendering — kept until a new job starts.
      const r = s.last_job_result;
      const isZh = (STRINGS && STRINGS.__lang === "zh");
      const stepLabel = isZh && r.current_step_zh
        ? r.current_step_zh
        : (r.current_step || "—");
      setText("job-id", r.job_id || "—");
      setText("job-progress",
        `${r.current_step_index || "?"} / ${r.total_steps || "?"}`);
      setText("job-step", stepLabel);
      // Elapsed is rounded to 1 decimal for readability in the UI.
      const elapsed = (typeof r.elapsed_s === "number")
        ? `${r.elapsed_s.toFixed(2)} s`
        : "—";
      setText("job-elapsed", elapsed);
    } else {
      setText("job-id", "—");
      setText("job-progress", "—");
      setText("job-step", "—");
      setText("job-elapsed", "—");
    }

    // Virtual I/O
    const vi = s.virtual_inputs || {};
    setCheck("io-part-present", vi.part_present);
    setCheck("io-fixture-clamped", vi.fixture_clamped);
    setCheck("io-welder-ready", vi.welder_ready);
    setCheck("io-safety-gate-closed", vi.safety_gate_closed);

    const vo = s.virtual_outputs || {};
    setLight("io-stack-green", vo.stack_light_green);
    setLight("io-stack-red", vo.stack_light_red);
    setLight("io-cycle-running", vo.cycle_running);

    // Status Light (Phase 1C task §3) — UI ONLY renders backend payload.
    // No client-side derivation of color / label / blink.
    applyStatusLight(s.status_light);

    // Live status
    setText("s-mode", s.mode || "");
    setText("s-joints", formatDeg(s.joint_positions_deg));
    setText("s-targets", formatDeg(s.target_positions_deg));
    setText("s-tcp",
      s.tcp_estimate_m
        ? `(${s.tcp_estimate_m[0].toFixed(3)}, ${s.tcp_estimate_m[1].toFixed(3)}, ${s.tcp_estimate_m[2].toFixed(3)})`
        : "—");

    // E-stop banner
    const isEstopped = (s.cabinet_state === "ESTOP"
      || s.cabinet_state === "PROTECTIVE_STOP"
      || s.cabinet_state === "FAULT");
    document.getElementById("estop-banner")
      .classList.toggle("hidden", !isEstopped);

    // Sync sliders to targets
    syncSlidersFromState();
    updateCanvas();

    // Disable/enable buttons per cabinet state
    updateButtonStates(s);
  }

  function updateButtonStates(s) {
    const cab = s.cabinet_state;
    const motionBlocked = (cab === "ESTOP" || cab === "PROTECTIVE_STOP" || cab === "FAULT");
    const jobActive = (cab === "RUNNING");

    setDisabled("btn-home", motionBlocked || jobActive);
    setDisabled("btn-pose-a", motionBlocked || jobActive);
    setDisabled("btn-pose-b", motionBlocked || jobActive);

    setDisabled("btn-job-start", cab !== "READY");
    setDisabled("btn-estop", motionBlocked);
    setDisabled("btn-reset-estop", cab !== "ESTOP");
    setDisabled("btn-prot-stop", motionBlocked);
    setDisabled("btn-reset-prot-stop", cab !== "PROTECTIVE_STOP");
    setDisabled("btn-inject-fault", cab === "OFFLINE");
    setDisabled("btn-reset-fault", cab !== "FAULT");

    // Sliders also blocked when motion blocked
    if (spec) {
      for (let i = 0; i < spec.dof; i++) {
        const sl = document.getElementById(`slider-${i}`);
        if (sl) sl.disabled = motionBlocked || jobActive;
      }
    }
  }

  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = (value == null) ? "" : value;
  }
  function setCheck(id, value) {
    const el = document.getElementById(id);
    if (el) el.checked = !!value;
  }
  function setLight(id, value) {
    const el = document.getElementById(id);
    if (!el) return;
    el.classList.toggle("on", !!value);
    el.classList.toggle("off", !value);
    el.textContent = value ? "●" : "○";
  }
  function setDisabled(id, disabled) {
    const el = document.getElementById(id);
    if (el) el.disabled = !!disabled;
  }
  function formatDeg(arr) {
    if (!Array.isArray(arr)) return "—";
    return arr.map(d => `${d >= 0 ? "+" : ""}${d.toFixed(2)}°`).join("  ");
  }

  // ---- Status Light rendering (Phase 1C task §3) -----------------------
  // UI is a strict renderer: every field comes from the backend payload.
  // No client-side color / label / blink derivation.
  const STATUS_LIGHT_VALID_COLORS = new Set(["gray", "green", "yellow", "red"]);
  function applyStatusLight(light) {
    if (!light || typeof light !== "object") return;
    const color = STATUS_LIGHT_VALID_COLORS.has(light.color) ? light.color : "gray";
    const blink = !!light.is_blinking;

    // Bulb: re-class with status-light-<color> + optional blinking
    const bulb = document.getElementById("status-light-bulb");
    if (bulb) {
      bulb.className = "status-light-bulb status-light-" + color
        + (blink ? " status-light-blinking" : "");
    }

    // Field cells: pure render of backend payload (i18n-aware blink label).
    setText("sl-color", color);
    setText("sl-blink", blink ? t("status_light_yes") : t("status_light_no"));
    setText("sl-label-zh", light.label_zh || "—");
    setText("sl-label-en", light.label_en || "—");
    setText("sl-reason-zh", light.reason_zh || "—");
    setText("sl-reason-en", light.reason_en || "—");
  }

  // ---- 2D canvas (FK side-view) -----------------------------------------
  const LINK_LENGTHS = {
    base_height: 0.089, upper_arm: 0.425, forearm: 0.392,
    wrist1: 0.109, wrist2: 0.094, tcp_offset: 0.082,
  };
  const ORIGIN_SVG = { x: 300, y: 360 };
  const PX_PER_M = 220;

  function fk2d(joints_rad) {
    const j = joints_rad || [0, 0, 0, 0, 0, 0];
    const L = LINK_LENGTHS;
    const base = { x: 0, y: 0 };
    const shoulder = { x: 0, y: L.base_height };
    const a1 = Math.PI / 2 - j[1];
    const elbow = {
      x: shoulder.x + L.upper_arm * Math.cos(a1),
      y: shoulder.y + L.upper_arm * Math.sin(a1),
    };
    const a2 = a1 - j[2];
    const w1 = {
      x: elbow.x + L.forearm * Math.cos(a2),
      y: elbow.y + L.forearm * Math.sin(a2),
    };
    const a3 = a2 - j[3] * 0.5;
    const w2 = { x: w1.x + L.wrist1 * Math.cos(a3), y: w1.y + L.wrist1 * Math.sin(a3) };
    const a4 = a3 - j[4] * 0.5;
    const w3 = { x: w2.x + L.wrist2 * Math.cos(a4), y: w2.y + L.wrist2 * Math.sin(a4) };
    const a5 = a4 - j[5] * 0.5;
    const tcp = { x: w3.x + L.tcp_offset * Math.cos(a5), y: w3.y + L.tcp_offset * Math.sin(a5) };
    return [base, shoulder, elbow, w1, w2, w3, tcp];
  }
  function worldToSvg(p) {
    return { x: ORIGIN_SVG.x + p.x * PX_PER_M, y: ORIGIN_SVG.y - p.y * PX_PER_M };
  }
  function updateCanvas() {
    const linksG = document.getElementById("arm-links");
    const jointsG = document.getElementById("arm-joints");
    const tcpEl = document.getElementById("arm-tcp");
    if (!linksG || !jointsG || !tcpEl) return;
    const pts = fk2d(lastJointRad).map(worldToSvg);
    while (linksG.firstChild) linksG.removeChild(linksG.firstChild);
    while (jointsG.firstChild) jointsG.removeChild(jointsG.firstChild);
    const NS = "http://www.w3.org/2000/svg";
    const segs = [[pts[0],pts[1]],[pts[1],pts[2]],[pts[2],pts[3]],
                  [pts[3],pts[4]],[pts[4],pts[5]],[pts[5],pts[6]]];
    for (const [a, b] of segs) {
      const line = document.createElementNS(NS, "line");
      line.setAttribute("x1", a.x.toFixed(2));
      line.setAttribute("y1", a.y.toFixed(2));
      line.setAttribute("x2", b.x.toFixed(2));
      line.setAttribute("y2", b.y.toFixed(2));
      linksG.appendChild(line);
    }
    for (let i = 1; i <= 5; i++) {
      const c = document.createElementNS(NS, "circle");
      c.setAttribute("cx", pts[i].x.toFixed(2));
      c.setAttribute("cy", pts[i].y.toFixed(2));
      c.setAttribute("r", "4");
      jointsG.appendChild(c);
    }
    tcpEl.setAttribute("cx", pts[6].x.toFixed(2));
    tcpEl.setAttribute("cy", pts[6].y.toFixed(2));
  }
  function drawGrid() {
    const g = document.getElementById("canvas-grid");
    if (!g) return;
    const NS = "http://www.w3.org/2000/svg";
    for (let y = 0; y <= 400; y += 50) {
      const ln = document.createElementNS(NS, "line");
      ln.setAttribute("x1", "0"); ln.setAttribute("y1", String(y));
      ln.setAttribute("x2", "600"); ln.setAttribute("y2", String(y));
      g.appendChild(ln);
    }
    for (let x = 0; x <= 600; x += 50) {
      const ln = document.createElementNS(NS, "line");
      ln.setAttribute("x1", String(x)); ln.setAttribute("y1", "0");
      ln.setAttribute("x2", String(x)); ln.setAttribute("y2", "400");
      g.appendChild(ln);
    }
  }

  // ---- Wire buttons ------------------------------------------------------
  function wireButton(id, handler) {
    const btn = document.getElementById(id);
    if (!btn) return;
    btn.addEventListener("click", () => {
      // Don't double-fire while disabled.
      if (btn.disabled) return;
      handler();
    });
  }

  function wireAllButtons() {
    // Motion (cabinet-mediated via legacy endpoints)
    wireButton("btn-home", () => {
      api("/api/move_home", { method: "POST" }).catch(e => console.warn("home:", e.message));
    });
    wireButton("btn-pose-a", () => {
      api("/api/load_pose", { method: "POST", body: { file: "pose_a.json" } })
        .catch(e => console.warn("pose_a:", e.message));
    });
    wireButton("btn-pose-b", () => {
      api("/api/load_pose", { method: "POST", body: { file: "pose_b.json" } })
        .catch(e => console.warn("pose_b:", e.message));
    });

    // Safety triggers
    wireButton("btn-estop", () => {
      api("/api/v1/cabinet/estop", { method: "POST" })
        .catch(e => console.warn("estop:", e.message));
    });
    wireButton("btn-reset-estop", () => {
      api("/api/v1/cabinet/reset-estop", { method: "POST" })
        .catch(e => console.warn("reset-estop:", e.message));
    });
    wireButton("btn-prot-stop", () => {
      api("/api/v1/cabinet/protective-stop", { method: "POST" })
        .catch(e => console.warn("prot-stop:", e.message));
    });
    wireButton("btn-reset-prot-stop", () => {
      api("/api/v1/cabinet/reset-protective-stop", { method: "POST" })
        .catch(e => console.warn("reset-prot-stop:", e.message));
    });
    wireButton("btn-inject-fault", () => {
      api("/api/v1/cabinet/fault", {
        method: "POST",
        body: { code: "VIRTUAL_FAULT_USER", message: "injected via UI" },
      }).catch(e => console.warn("fault:", e.message));
    });
    wireButton("btn-reset-fault", () => {
      api("/api/v1/cabinet/reset-fault", { method: "POST" })
        .catch(e => console.warn("reset-fault:", e.message));
    });

    // Job
    wireButton("btn-job-start", () => {
      api("/api/v1/jobs/demo-sequence/start", { method: "POST" })
        .catch(e => console.warn("job-start:", e.message));
    });

    // Virtual I/O toggles — push to backend immediately on change
    function wireInput(signal, checkboxId) {
      const cb = document.getElementById(checkboxId);
      if (!cb) return;
      cb.addEventListener("change", () => {
        api(`/api/v1/virtual-io/inputs/${encodeURIComponent(signal)}`, {
          method: "POST",
          body: { value: cb.checked },
        }).catch(e => {
          console.warn(`set_io ${signal}:`, e.message);
          // Revert the UI on failure
          cb.checked = !cb.checked;
        });
      });
    }
    wireInput("part_present", "io-part-present");
    wireInput("fixture_clamped", "io-fixture-clamped");
    wireInput("welder_ready", "io-welder-ready");
    wireInput("safety_gate_closed", "io-safety-gate-closed");
  }

  function loadSpecAndStartPolling() {
    loadSpec().then(() => {
      wireAllButtons();
      poll();
      if (pollTimer) clearInterval(pollTimer);
      pollTimer = setInterval(poll, POLL_MS);
    });
  }

  // ---- Boot --------------------------------------------------------------
  loadTranslations().then(loadSpecAndStartPolling);
  drawGrid();
  wireStatusModal();

  // Expose a tiny API for tests / debugging.
  window.__robotarm = { poll, applyStatus, lastStatus: () => lastStatus,
                        openStatusModal, buildStatusSnapshot };

  // ---- Status copy modal -------------------------------------------------
  //
  // 2026-09-16 addition (Ben): provide a plain-text snapshot of the live
  // status so it can be selected + copied via Ctrl+A / Ctrl+C, without
  // fighting the 400ms polling-driven DOM updates that wipe text selection
  // on the live status table.
  //
  // Pure frontend. Backend / tests / README / app.py untouched.
  function buildStatusSnapshot(s) {
    // Defensive: if no status yet, return a placeholder.
    if (!s) return "(no status yet — wait for first poll)";
    const sl = s.status_light || {};
    const vi = s.virtual_inputs || {};
    const vo = s.virtual_outputs || {};
    const lj = s.last_job_result || null;
    const lines = [];
    lines.push(`Cabinet State: ${s.cabinet_state || "—"}`);
    const slDesc = `${sl.color || "—"}${sl.is_blinking ? " (blinking)" : ""} - ${sl.label_en || "—"}${sl.reason_en ? " (" + sl.reason_en + ")" : ""}`;
    lines.push(`Status Light: ${slDesc}`);
    if (sl.label_zh || sl.reason_zh) {
      lines.push(`Status Light (zh): ${sl.label_zh || "—"}${sl.reason_zh ? " (" + sl.reason_zh + ")" : ""}`);
    }
    lines.push(`Safety State: ${s.safety_state || "—"}`);
    lines.push(`Motion State: ${s.motion_state || "—"}`);
    lines.push(`Fault: ${s.fault ? ((s.fault.code || "") + " — " + (s.fault.message || "")) : "(none)"}`);
    lines.push("Virtual Inputs:");
    lines.push(`  - part_present: ${vi.part_present}`);
    lines.push(`  - fixture_clamped: ${vi.fixture_clamped}`);
    lines.push(`  - welder_ready: ${vi.welder_ready}`);
    lines.push(`  - safety_gate_closed: ${vi.safety_gate_closed}`);
    lines.push(`Cycle Running: ${vo.cycle_running}`);
    lines.push(`Stack Light Green: ${vo.stack_light_green}`);
    lines.push(`Stack Light Red: ${vo.stack_light_red}`);
    lines.push("Active Job:");
    if (s.active_job_id) {
      lines.push(`  - job_id: ${s.active_job_id}`);
      lines.push(`  - progress: ${s.current_step_index || "?"} / ${s.total_steps || "?"}`);
      lines.push(`  - current_step_id: ${s.current_step_id || "—"}`);
      lines.push(`  - current_step_name: ${s.current_step_name || "—"}`);
    } else {
      lines.push("  (none)");
    }
    lines.push("Last Job Result:");
    if (lj) {
      lines.push(`  - job_id: ${lj.job_id}`);
      lines.push(`  - progress: ${lj.current_step_index || "?"} / ${lj.total_steps || "?"}`);
      lines.push(`  - current_step: ${lj.current_step || "—"}${lj.current_step_zh ? " / " + lj.current_step_zh : ""}`);
      lines.push(`  - result: ${lj.result || "—"}`);
      lines.push(`  - success: ${lj.success}`);
      lines.push(`  - elapsed: ${typeof lj.elapsed_s === "number" ? lj.elapsed_s.toFixed(3) + "s" : "—"}`);
      lines.push(`  - finished_at: ${typeof lj.finished_at === "number" ? new Date(lj.finished_at * 1000).toISOString() : "—"}`);
    } else {
      lines.push("  (none)");
    }
    return lines.join("\n");
  }

  function openStatusModal() {
    const backdrop = document.getElementById("status-modal-backdrop");
    const body = document.getElementById("status-modal-body");
    if (!backdrop || !body) return;
    body.textContent = buildStatusSnapshot(lastStatus);
    backdrop.classList.remove("hidden");
    backdrop.setAttribute("aria-hidden", "false");
    // Move focus into the body so Ctrl+A selects the snapshot text.
    body.focus();
  }

  function closeStatusModal() {
    const backdrop = document.getElementById("status-modal-backdrop");
    if (!backdrop) return;
    backdrop.classList.add("hidden");
    backdrop.setAttribute("aria-hidden", "true");
    // Return focus to the trigger button.
    const trigger = document.getElementById("btn-status-copy");
    if (trigger) trigger.focus();
  }

  function wireStatusModal() {
    const trigger = document.getElementById("btn-status-copy");
    const backdrop = document.getElementById("status-modal-backdrop");
    const closeBtn = document.getElementById("status-modal-close");
    const closeBtn2 = document.getElementById("status-modal-close-btn");
    if (trigger) {
      trigger.addEventListener("click", openStatusModal);
    }
    if (closeBtn) {
      closeBtn.addEventListener("click", closeStatusModal);
    }
    if (closeBtn2) {
      closeBtn2.addEventListener("click", closeStatusModal);
    }
    if (backdrop) {
      // Click on the backdrop itself (NOT inside the modal) closes.
      backdrop.addEventListener("click", (e) => {
        if (e.target === backdrop) closeStatusModal();
      });
    }
    // Esc closes the modal.
    document.addEventListener("keydown", (e) => {
      if (e.key === "Escape" && backdrop && !backdrop.classList.contains("hidden")) {
        closeStatusModal();
      }
    });
  }
})();
