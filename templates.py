#!/usr/bin/env python3
# coding=utf-8


def build_index_html():
    return """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>SLAM Live Map</title>
  <style>
    html, body {
      margin: 0;
      padding: 0;
      background:
        radial-gradient(circle at top left, rgba(40, 84, 84, 0.24), transparent 28%),
        linear-gradient(180deg, #151718 0%, #232526 100%);
      color: #eef3f2;
      font-family: "Segoe UI", "Trebuchet MS", Arial, sans-serif;
      width: 100%;
      height: 100%;
      overflow: hidden;
    }

    .wrap {
      position: relative;
      width: 100%;
      height: 100%;
      background:
        radial-gradient(circle at top left, rgba(50, 126, 110, 0.18), transparent 32%),
        linear-gradient(180deg, #1b1d1e 0%, #292b2c 100%);
      overflow: hidden;
    }

    .viewport {
      position: absolute;
      inset: 0;
      overflow: hidden;
      cursor: grab;
      touch-action: none;
    }

    .viewport.dragging {
      cursor: grabbing;
    }

    .scene {
      position: absolute;
      left: 0;
      top: 0;
      transform-origin: 0 0;
      will-change: transform;
    }

    #mapImg {
      position: absolute;
      left: 0;
      top: 0;
      display: block;
      user-select: none;
      -webkit-user-drag: none;
      image-rendering: pixelated;
      image-rendering: crisp-edges;
      filter: none;
      background: #4a4a4a;
    }

    #overlay {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      pointer-events: none;
    }

    .toolbar {
      position: absolute;
      top: 18px;
      left: 18px;
      z-index: 20;
      display: flex;
      flex-direction: column;
      gap: 12px;
      background: rgba(29, 31, 32, 0.84);
      backdrop-filter: blur(14px);
      -webkit-backdrop-filter: blur(14px);
      padding: 14px;
      border-radius: 16px;
      border: 1px solid rgba(255, 255, 255, 0.08);
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.30);
      min-width: 260px;
      max-width: 330px;
    }

    .toolbar.collapsed {
      min-width: 0;
      width: auto;
      padding: 10px 12px;
    }

    .toolbar-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
    }

    .toolbar-title {
      font-size: 12px;
      font-weight: 700;
      color: #f2f7f5;
      letter-spacing: 0.12em;
    }

    .toolbar-body {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .toolbar.collapsed .toolbar-body {
      display: none;
    }

    .row {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      align-items: center;
    }

    .segmented {
      display: inline-flex;
      gap: 8px;
      padding: 4px;
      border-radius: 14px;
      background: rgba(255, 255, 255, 0.05);
      border: 1px solid rgba(255, 255, 255, 0.05);
      width: fit-content;
    }

    .section-card {
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255, 255, 255, 0.035);
      border: 1px solid rgba(255, 255, 255, 0.05);
    }

    .compact-stack {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .btn {
      padding: 9px 11px;
      font-size: 13px;
      border: none;
      border-radius: 10px;
      cursor: pointer;
      font-weight: 700;
      color: #f7fbfa;
      background: linear-gradient(180deg, #3f4245 0%, #2f3336 100%);
      transition: transform 120ms ease, filter 120ms ease, background 120ms ease;
    }

    .btn:hover {
      filter: brightness(1.08);
    }

    .btn:active {
      transform: translateY(1px);
    }

    .btn-blue { background: linear-gradient(180deg, #40a4ff 0%, #1f7fdb 100%); }
    .btn-danger { background: linear-gradient(180deg, #ff655d 0%, #dd362f 100%); }
    .btn-orange { background: linear-gradient(180deg, #ffb23a 0%, #f08a08 100%); }
    .btn-green { background: linear-gradient(180deg, #46b95f 0%, #2f8b46 100%); }

    .btn-ghost {
      background: rgba(255, 255, 255, 0.08);
      min-width: 34px;
      padding: 6px 10px;
      font-size: 16px;
      line-height: 1;
    }

    .btn-toggle.active {
      background: linear-gradient(180deg, #5caef8 0%, #2581db 100%);
      box-shadow: inset 0 0 0 2px rgba(255,255,255,0.78);
      color: #ffffff;
    }

    .btn-toggle.active.btn-green {
      background: linear-gradient(180deg, #53ca70 0%, #2f8b46 100%);
    }

    .btn-toggle.active.btn-orange {
      background: linear-gradient(180deg, #ffbf57 0%, #f08a08 100%);
    }

    .btn-wide {
      flex: 1 1 0;
      min-width: 0;
    }

    .action-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }

    .action-grid .btn {
      width: 100%;
    }

    .action-grid-top {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }

    .panel {
      position: absolute;
      right: 18px;
      top: 18px;
      z-index: 20;
      background: rgba(24, 26, 27, 0.88);
      backdrop-filter: blur(16px);
      -webkit-backdrop-filter: blur(16px);
      padding: 14px;
      border-radius: 16px;
      border: 1px solid rgba(255,255,255,0.08);
      min-width: 320px;
      max-width: min(420px, calc(100vw - 36px));
      max-height: calc(100vh - 36px);
      overflow: auto;
      font-size: 13px;
      line-height: 1.5;
      box-shadow: 0 18px 42px rgba(0, 0, 0, 0.32);
    }

    .panel h3 {
      margin: 0;
      font-size: 15px;
    }

    .panel-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      margin-bottom: 12px;
    }

    .metric-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 6px;
    }

    .status-chip {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 8px;
      border-radius: 999px;
      background: rgba(255,255,255,0.06);
      font-size: 12px;
      color: #d3dcda;
    }

    .status-chip::before {
      content: "";
      width: 8px;
      height: 8px;
      border-radius: 999px;
      background: #8da39d;
    }

    .status-chip.ok::before { background: #59d377; }
    .status-chip.warn::before { background: #f3c359; }
    .status-chip.bad::before { background: #ff746d; }

    .panel-section {
      margin-top: 12px;
      padding-top: 12px;
      border-top: 1px solid rgba(255,255,255,0.08);
    }

    .ok { color: #6DFF7A; }
    .bad { color: #FF6B6B; }
    .warn { color: #FFD54F; }

    .small {
      color: #bbb;
      font-size: 12px;
    }

    .mono {
      font-family: Consolas, monospace;
      white-space: pre-wrap;
      word-break: break-word;
    }

    label.cb {
      display: inline-flex;
      align-items: center;
      gap: 4px;
      font-size: 12px;
      color: #ddd;
    }

    .display-toggles {
      gap: 10px 12px;
    }

    input[type="file"] {
      display: none;
    }

    @media (max-width: 820px) {
      .toolbar {
        top: 12px;
        left: 12px;
        max-width: calc(100vw - 24px);
      }

      .panel {
        top: 12px;
        right: 12px;
        left: 12px;
        max-width: none;
        min-width: 0;
      }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div id="toolbar" class="toolbar">
      <div class="toolbar-header">
        <div class="toolbar-title">OPTIONS</div>
        <button id="toolbarToggleBtn" class="btn btn-ghost" title="Collapse options">-</button>
      </div>

      <div id="toolbarBody" class="toolbar-body">
        <div class="section-card compact-stack">
          <div class="segmented">
            <button id="viewBtn" class="btn btn-blue btn-toggle active">VIEW</button>
            <button id="navBtn" class="btn btn-toggle">NAV</button>
          </div>
          <div class="row display-toggles">
            <label class="cb"><input type="checkbox" id="showRobot" checked/>Robot</label>
            <label class="cb"><input type="checkbox" id="showPath" checked/>Path</label>
            <label class="cb"><input type="checkbox" id="showScan" checked/>Scan</label>
            <label class="cb"><input type="checkbox" id="showGrid"/>Grid</label>
          </div>
        </div>

        <div id="viewPanel" class="section-card compact-stack">
          <div class="action-grid-top">
            <button id="saveBtn" class="btn btn-blue btn-wide">SAVE MAP</button>
            <button id="loadBtn" class="btn btn-blue btn-wide">LOAD MAP</button>
            <input id="mapFile" type="file" accept=".zip,.bundle.zip"/>
          </div>
          <div class="action-grid">
            <button id="resetViewBtn" class="btn">RESET VIEW</button>
            <button id="rotateLeftBtn" class="btn">ROTATE -</button>
            <button id="rotateRightBtn" class="btn">ROTATE +</button>
            <button id="autoAlignBtn" class="btn">AUTO ALIGN</button>
            <button id="resetAngleBtn" class="btn">ANGLE 0</button>
          </div>
          <div class="small" id="angleInfo">Display angle: 0.0 deg</div>
        </div>

        <div id="navPanel" class="section-card" style="display:none">
          <div class="row">
            <button id="statusToggleBtn" class="btn btn-toggle btn-wide">STATUS</button>
          </div>
          <div class="row">
            <button id="setGoalBtn" class="btn btn-green btn-toggle btn-wide active">SET GOAL</button>
            <button id="setInitPoseBtn" class="btn btn-orange btn-toggle btn-wide">SET INITIAL POSE</button>
          </div>
          <div class="row">
            <button id="clearBtn" class="btn btn-danger btn-wide">STOP & CLEAR PATH</button>
          </div>
          <div class="small" id="navHint">
            SET GOAL: click 1 lan chon diem, click lan 2 chon huong.
          </div>
        </div>
      </div>
    </div>

    <div id="statusPanel" class="panel" style="display:none">
      <div class="panel-header">
        <h3>System Status</h3>
        <button id="statusCloseBtn" class="btn btn-ghost" title="Hide status">x</button>
      </div>
      <div id="statusBox">Waiting...</div>
      <div class="panel-section">
        <div id="goalInfo">Goal: none</div>
        <div id="cursorInfo" class="small">Coordinate: --</div>
      </div>
      <div class="panel-section">
        <div><b>SLAM process</b></div>
        <div id="slamStatusBox" class="small">Checking...</div>
      </div>
    </div>

    <div id="viewport" class="viewport">
      <div id="scene" class="scene">
        <img id="mapImg" src="/map.png" />
      </div>
      <canvas id="overlay"></canvas>
    </div>
  </div>

  <script>
    const viewport = document.getElementById("viewport");
    const scene = document.getElementById("scene");
    const mapImg = document.getElementById("mapImg");
    const overlay = document.getElementById("overlay");
    const ctx = overlay.getContext("2d");

    const toolbar = document.getElementById("toolbar");
    const toolbarBody = document.getElementById("toolbarBody");
    const toolbarToggleBtn = document.getElementById("toolbarToggleBtn");
    const viewBtn = document.getElementById("viewBtn");
    const navBtn = document.getElementById("navBtn");
    const viewPanel = document.getElementById("viewPanel");
    const navPanel = document.getElementById("navPanel");
    const statusPanel = document.getElementById("statusPanel");

    const saveBtn = document.getElementById("saveBtn");
    const loadBtn = document.getElementById("loadBtn");
    const resetViewBtn = document.getElementById("resetViewBtn");
    const rotateLeftBtn = document.getElementById("rotateLeftBtn");
    const rotateRightBtn = document.getElementById("rotateRightBtn");
    const autoAlignBtn = document.getElementById("autoAlignBtn");
    const resetAngleBtn = document.getElementById("resetAngleBtn");
    const statusToggleBtn = document.getElementById("statusToggleBtn");
    const statusCloseBtn = document.getElementById("statusCloseBtn");
    const clearBtn = document.getElementById("clearBtn");
    const setGoalBtn = document.getElementById("setGoalBtn");
    const setInitPoseBtn = document.getElementById("setInitPoseBtn");
    const navHint = document.getElementById("navHint");
    const mapFile = document.getElementById("mapFile");

    const showRobot = document.getElementById("showRobot");
    const showPath = document.getElementById("showPath");
    const showGrid = document.getElementById("showGrid");
    const showScan = document.getElementById("showScan"); 

    const statusBox = document.getElementById("statusBox");
    const goalInfo = document.getElementById("goalInfo");
    const cursorInfo = document.getElementById("cursorInfo");
    const slamStatusBox = document.getElementById("slamStatusBox");
    const angleInfo = document.getElementById("angleInfo");

    let scale = 1.0;
    let tx = 0.0;
    let ty = 0.0;
    let viewRotation = 0.0;

    let dragging = false;
    let dragMoved = false;
    let lastX = 0;
    let lastY = 0;

    let mapVersion = -1;
    let lastState = null;
    let uiMode = "view";

    let pendingGoal = null;
    let pendingInitPose = null;
    let mouseImg = null;
    let navAction = "goal";

    let hasInitialFit = false;
    let toolbarCollapsed = false;
    let statusVisible = false;

    function getRotationKey() {
      return "slam_display_rotation_deg";
    }

    function getToolbarCollapsedKey() {
      return "slam_toolbar_collapsed";
    }

    function getStatusVisibleKey() {
      return "slam_status_visible";
    }

    function loadRotation() {
      try {
        const raw = window.localStorage.getItem(getRotationKey());
        if (raw === null) return 0.0;
        const deg = Number(raw);
        if (!Number.isFinite(deg)) return 0.0;
        return deg * Math.PI / 180.0;
      } catch (_) {
        return 0.0;
      }
    }

    function saveRotation() {
      try {
        window.localStorage.setItem(
          getRotationKey(),
          String(viewRotation * 180.0 / Math.PI)
        );
      } catch (_) {
      }
    }

    function loadToolbarCollapsed() {
      try {
        return window.localStorage.getItem(getToolbarCollapsedKey()) === "1";
      } catch (_) {
        return false;
      }
    }

    function saveToolbarCollapsed() {
      try {
        window.localStorage.setItem(
          getToolbarCollapsedKey(),
          toolbarCollapsed ? "1" : "0"
        );
      } catch (_) {
      }
    }

    function loadStatusVisible() {
      try {
        return window.localStorage.getItem(getStatusVisibleKey()) === "1";
      } catch (_) {
        return false;
      }
    }

    function saveStatusVisible() {
      try {
        window.localStorage.setItem(
          getStatusVisibleKey(),
          statusVisible ? "1" : "0"
        );
      } catch (_) {
      }
    }

    function setToolbarCollapsed(collapsed) {
      toolbarCollapsed = !!collapsed;
      toolbar.classList.toggle("collapsed", toolbarCollapsed);
      toolbarToggleBtn.innerText = toolbarCollapsed ? "+" : "-";
      toolbarToggleBtn.title = toolbarCollapsed ? "Open options" : "Collapse options";
      toolbarBody.setAttribute("aria-hidden", toolbarCollapsed ? "true" : "false");
      saveToolbarCollapsed();
    }

    function syncStatusPanelVisibility() {
      const shouldShow = uiMode === "nav" && statusVisible;
      statusPanel.style.display = shouldShow ? "block" : "none";
      setToggleButton(statusToggleBtn, shouldShow);
    }

    function setStatusVisible(visible) {
      statusVisible = !!visible;
      syncStatusPanelVisibility();
      saveStatusVisible();
    }

    function getRotatedBounds(width, height, angle) {
      const cx = width * 0.5;
      const cy = height * 0.5;
      const c = Math.cos(angle);
      const s = Math.sin(angle);
      const corners = [
        { x: 0, y: 0 },
        { x: width, y: 0 },
        { x: width, y: height },
        { x: 0, y: height }
      ];

      let minX = Infinity;
      let minY = Infinity;
      let maxX = -Infinity;
      let maxY = -Infinity;

      for (const p of corners) {
        const dx = p.x - cx;
        const dy = p.y - cy;
        const rx = dx * c - dy * s + cx;
        const ry = dx * s + dy * c + cy;
        minX = Math.min(minX, rx);
        minY = Math.min(minY, ry);
        maxX = Math.max(maxX, rx);
        maxY = Math.max(maxY, ry);
      }

      return {
        minX: minX,
        minY: minY,
        maxX: maxX,
        maxY: maxY,
        width: maxX - minX,
        height: maxY - minY
      };
    }

    function rotateImagePoint(ix, iy) {
      const size = getMapPixelSize();
      const cx = size.w * 0.5;
      const cy = size.h * 0.5;
      const dx = ix - cx;
      const dy = iy - cy;
      const c = Math.cos(viewRotation);
      const s = Math.sin(viewRotation);
      return {
        x: dx * c - dy * s + cx,
        y: dx * s + dy * c + cy
      };
    }

    function unrotateImagePoint(ix, iy) {
      const size = getMapPixelSize();
      const cx = size.w * 0.5;
      const cy = size.h * 0.5;
      const dx = ix - cx;
      const dy = iy - cy;
      const c = Math.cos(viewRotation);
      const s = Math.sin(viewRotation);
      return {
        x: dx * c + dy * s + cx,
        y: -dx * s + dy * c + cy
      };
    }

    function updateAngleInfo() {
      angleInfo.innerText = `Display angle: ${(viewRotation * 180.0 / Math.PI).toFixed(1)} deg`;
    }

    function api(path, options) {
      return fetch(path, options);
    }

    function setToggleButton(btn, active) {
      btn.classList.toggle("active", !!active);
    }

    function setNavAction(action) {
      navAction = action;
      const isGoal = action === "goal";

      setToggleButton(setGoalBtn, isGoal);
      setToggleButton(setInitPoseBtn, !isGoal);

      pendingGoal = null;
      pendingInitPose = null;
      mouseImg = null;

      navHint.innerText = isGoal
        ? "SET GOAL: click 1 lan chon diem, click lan 2 chon huong."
        : "SET INITIAL POSE: click 1 lan chon vi tri robot, click lan 2 chon huong robot.";

      drawOverlay();
    }

    function switchMode(mode) {
      uiMode = mode;
      const isView = mode === "view";

      viewPanel.style.display = isView ? "block" : "none";
      navPanel.style.display = isView ? "none" : "block";
      syncStatusPanelVisibility();

      setToggleButton(viewBtn, isView);
      setToggleButton(navBtn, !isView);

      pendingGoal = null;
      pendingInitPose = null;
      mouseImg = null;

      if (!isView) {
        setNavAction(navAction || "goal");
      } else {
        drawOverlay();
      }
    }

    function applyTransform() {
      const size = getMapPixelSize();
      const cx = size.w * 0.5;
      const cy = size.h * 0.5;
      const deg = viewRotation * 180.0 / Math.PI;
      scene.style.transform =
        `translate(${tx}px, ${ty}px) scale(${scale}) translate(${cx}px, ${cy}px) rotate(${deg}deg) translate(${-cx}px, ${-cy}px)`;
    }

    function getMapPixelSize() {
      return {
        w: mapImg.naturalWidth || mapImg.width || mapImg.clientWidth || 1,
        h: mapImg.naturalHeight || mapImg.height || mapImg.clientHeight || 1
      };
    }

    function centerMapAtCurrentScale() {
      const vpRect = viewport.getBoundingClientRect();
      const size = getMapPixelSize();
      const bounds = getRotatedBounds(size.w, size.h, viewRotation);
      tx = (vpRect.width - bounds.width * scale) * 0.5 - bounds.minX * scale;
      ty = (vpRect.height - bounds.height * scale) * 0.5 - bounds.minY * scale;
    }

    function fitToScreen() {
      const vpRect = viewport.getBoundingClientRect();
      const size = getMapPixelSize();
      const bounds = getRotatedBounds(size.w, size.h, viewRotation);

      const pad = 0.985;
      const sx = vpRect.width / Math.max(bounds.width, 1e-6);
      const sy = vpRect.height / Math.max(bounds.height, 1e-6);
      scale = Math.max(0.05, Math.min(20.0, Math.min(sx, sy) * pad));

      centerMapAtCurrentScale();
      applyTransform();
      drawOverlay();
    }

    function resetView() {
      fitToScreen();
    }

    function syncOverlaySize() {
      const size = getMapPixelSize();
      const dpr = window.devicePixelRatio || 1;
      const vpRect = viewport.getBoundingClientRect();

      mapImg.style.width = size.w + "px";
      mapImg.style.height = size.h + "px";

      scene.style.width = size.w + "px";
      scene.style.height = size.h + "px";

      overlay.width = Math.round(vpRect.width * dpr);
      overlay.height = Math.round(vpRect.height * dpr);
      overlay.style.width = vpRect.width + "px";
      overlay.style.height = vpRect.height + "px";

      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.scale(dpr, dpr);

      applyTransform();
      drawOverlay();
    }

    function fmtNum(v, d=2) {
      if (v === null || v === undefined || Number.isNaN(v)) return "--";
      return Number(v).toFixed(d);
    }

    function fmtAge(v) {
      if (v === null || v === undefined) return "--";
      return `${v.toFixed(1)} s`;
    }

    function getRenderInfo() {
      return lastState ? lastState.render_info : null;
    }

    function worldToImage(wx, wy) {
      const r = getRenderInfo();
      if (!r) return null;

      const size = getMapPixelSize();
      if (size.w <= 0 || size.h <= 0) return null;

      const widthM = r.width_cells * r.resolution;
      const heightM = r.height_cells * r.resolution;
      if (widthM <= 0 || heightM <= 0) return null;

      const u = (wx - r.origin_x) / widthM;
      const v = (wy - r.origin_y) / heightM;

      return {
        x: u * size.w,
        y: (1.0 - v) * size.h
      };
    }

    function imageToScreen(ix, iy) {
      const p = rotateImagePoint(ix, iy);
      return {
        x: tx + p.x * scale,
        y: ty + p.y * scale
      };
    }

    function worldToScreen(wx, wy) {
      const p = worldToImage(wx, wy);
      if (!p) return null;
      return imageToScreen(p.x, p.y);
    }

    function imageToWorld(ix, iy) {
      const r = getRenderInfo();
      if (!r) return null;

      const size = getMapPixelSize();
      if (size.w <= 0 || size.h <= 0) return null;

      const u = ix / size.w;
      const v = 1.0 - (iy / size.h);

      const widthM = r.width_cells * r.resolution;
      const heightM = r.height_cells * r.resolution;

      return {
        x: r.origin_x + u * widthM,
        y: r.origin_y + v * heightM,
        u: u,
        v: v
      };
    }

    function clientToImageLocal(clientX, clientY) {
      const rect = viewport.getBoundingClientRect();
      const size = getMapPixelSize();

      const sx = clientX - rect.left;
      const sy = clientY - rect.top;

      const x = (sx - tx) / scale;
      const y = (sy - ty) / scale;
      const p = unrotateImagePoint(x, y);

      return {
        x: p.x,
        y: p.y,
        rect: {
          width: size.w,
          height: size.h
        }
      };
    }

    function zoomAt(clientX, clientY, factor) {
      const rect = viewport.getBoundingClientRect();
      const sx = clientX - rect.left;
      const sy = clientY - rect.top;

      const oldScale = scale;
      const newScale = Math.max(0.05, Math.min(20.0, oldScale * factor));
      if (Math.abs(newScale - oldScale) < 1e-9) return;

      const rotatedX = (sx - tx) / oldScale;
      const rotatedY = (sy - ty) / oldScale;

      scale = newScale;
      tx = sx - rotatedX * scale;
      ty = sy - rotatedY * scale;

      applyTransform();
      drawOverlay();
    }

    function setDisplayRotation(angleRad) {
      viewRotation = angleRad;
      updateAngleInfo();
      saveRotation();
      fitToScreen();
    }

    function normalizeAngle(angleRad) {
      let a = angleRad;
      while (a > Math.PI) a -= Math.PI * 2.0;
      while (a < -Math.PI) a += Math.PI * 2.0;
      return a;
    }

    function autoAlignDisplay() {
      if (!lastState || !lastState.pose || !lastState.pose.ok) return;

      const yaw = Number(lastState.pose.theta || 0.0);
      const quarterTurn = Math.PI * 0.5;
      const snapped = Math.round(yaw / quarterTurn) * quarterTurn;
      const correction = normalizeAngle(snapped - yaw);
      setDisplayRotation(correction);
    }

    function drawPath(list, color, width) {
      if (!list || list.length < 2) return;

      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.beginPath();

      let started = false;
      for (const p of list) {
        const q = worldToScreen(p.x, p.y);
        if (!q) continue;

        if (!started) {
          ctx.moveTo(q.x, q.y);
          started = true;
        } else {
          ctx.lineTo(q.x, q.y);
        }
      }
      ctx.stroke();
    }

    function getDisplayPath(list) {
      if (!list || list.length === 0) return list;
      if (!lastState || !lastState.pose || !lastState.pose.ok) return list;

      const rx = Number(lastState.pose.x);
      const ry = Number(lastState.pose.y);

      let bestIdx = 0;
      let bestDist2 = Infinity;

      for (let i = 0; i < list.length; i++) {
        const p = list[i];
        const dx = Number(p.x) - rx;
        const dy = Number(p.y) - ry;
        const d2 = dx * dx + dy * dy;
        if (d2 < bestDist2) {
          bestDist2 = d2;
          bestIdx = i;
        }
      }

      const out = [{ x: rx, y: ry }];
      for (let i = bestIdx; i < list.length; i++) {
        out.push(list[i]);
      }
      return out;
    }

    function drawScanPoints(list) {
      if (!showScan.checked) return;
      if (!list || list.length === 0) return;

      ctx.fillStyle = "rgba(0,255,120,0.85)";

      for (const p of list) {
        const q = worldToScreen(p.x, p.y);
        if (!q) continue;
        ctx.fillRect(q.x - 1.0, q.y - 1.0, 2.0, 2.0);
      }
    }
    function drawRobot(x, y, yaw) {
      if (!showRobot.checked) return;

      const p0 = worldToScreen(x, y);
      const px = worldToScreen(x + 0.14 * Math.cos(yaw), y + 0.14 * Math.sin(yaw));
      const py = worldToScreen(x + 0.14 * Math.cos(yaw + Math.PI / 2.0), y + 0.14 * Math.sin(yaw + Math.PI / 2.0));
      if (!p0) return;

      ctx.strokeStyle = "rgba(0,255,255,0.9)";
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      ctx.arc(p0.x, p0.y, 3.5, 0, Math.PI * 2);
      ctx.stroke();

      if (px) {
        ctx.strokeStyle = "rgba(255,50,50,0.95)";
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(px.x, px.y);
        ctx.stroke();
      }

      if (py) {
        ctx.strokeStyle = "rgba(50,255,80,0.95)";
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(py.x, py.y);
        ctx.stroke();
      }
    }

    function drawGoal(goal) {
      if (!goal || goal.x === null || goal.y === null || goal.yaw === null) return;

      const p = worldToScreen(goal.x, goal.y);
      if (!p) return;

      ctx.strokeStyle = "rgba(255,60,60,0.95)";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 6.0, 0, Math.PI * 2);
      ctx.stroke();

      ctx.strokeStyle = "rgba(255,255,255,0.95)";
      ctx.lineWidth = 1.7;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3.0, 0, Math.PI * 2);
      ctx.stroke();

      ctx.fillStyle = "rgba(255,60,60,1)";
      ctx.beginPath();
      ctx.arc(p.x, p.y, 1.5, 0, Math.PI * 2);
      ctx.fill();

      const p2 = worldToScreen(
        goal.x + 0.15 * Math.cos(goal.yaw),
        goal.y + 0.15 * Math.sin(goal.yaw)
      );

      if (p2) {
        ctx.strokeStyle = "rgba(255,60,60,0.95)";
        ctx.lineWidth = 2.1;
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }
    }

    function drawPendingGoal() {
      if (!pendingGoal) return;

      const p0 = imageToScreen(pendingGoal.imgx, pendingGoal.imgy);

      ctx.strokeStyle = "rgba(0,255,255,0.85)";
      ctx.lineWidth = 1.7;
      ctx.beginPath();
      ctx.arc(p0.x, p0.y, 3.0, 0, Math.PI * 2);
      ctx.stroke();

      if (mouseImg) {
        const p1 = imageToScreen(mouseImg.x, mouseImg.y);
        ctx.strokeStyle = "rgba(255,213,79,0.9)";
        ctx.lineWidth = 2.0;
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(p1.x, p1.y);
        ctx.stroke();
      }
    }

    function drawPendingInitPose() {
      if (!pendingInitPose) return;

      const p0 = imageToScreen(pendingInitPose.imgx, pendingInitPose.imgy);

      ctx.strokeStyle = "rgba(255,165,0,0.95)";
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      ctx.arc(p0.x, p0.y, 5.0, 0, Math.PI * 2);
      ctx.stroke();

      ctx.fillStyle = "rgba(255,165,0,0.9)";
      ctx.beginPath();
      ctx.arc(p0.x, p0.y, 2.0, 0, Math.PI * 2);
      ctx.fill();

      if (mouseImg) {
        const p1 = imageToScreen(mouseImg.x, mouseImg.y);
        ctx.strokeStyle = "rgba(255,200,80,0.95)";
        ctx.lineWidth = 2.2;
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(p1.x, p1.y);
        ctx.stroke();
      }
    }

    function drawGridOverlay() {
      if (!showGrid.checked) return;

      const r = getRenderInfo();
      if (!r) return;

      const widthM = r.width_cells * r.resolution;
      const heightM = r.height_cells * r.resolution;
      const step = 0.5;

      const startX = Math.floor(r.origin_x / step) * step;
      const endX = r.origin_x + widthM;

      for (let x = startX; x <= endX + 1e-6; x += step) {
        const p1 = worldToScreen(x, r.origin_y);
        const p2 = worldToScreen(x, r.origin_y + heightM);
        if (!p1 || !p2) continue;

        const isMajor = Math.abs((x / 1.0) - Math.round(x / 1.0)) < 1e-6;
        ctx.strokeStyle = isMajor ? "rgba(255,255,255,0.16)" : "rgba(255,255,255,0.07)";
        ctx.lineWidth = isMajor ? 0.8 : 0.45;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }

      const startY = Math.floor(r.origin_y / step) * step;
      const endY = r.origin_y + heightM;

      for (let y = startY; y <= endY + 1e-6; y += step) {
        const p1 = worldToScreen(r.origin_x, y);
        const p2 = worldToScreen(r.origin_x + widthM, y);
        if (!p1 || !p2) continue;

        const isMajor = Math.abs((y / 1.0) - Math.round(y / 1.0)) < 1e-6;
        ctx.strokeStyle = isMajor ? "rgba(255,255,255,0.16)" : "rgba(255,255,255,0.07)";
        ctx.lineWidth = isMajor ? 0.8 : 0.45;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }
    }

    function drawOverlay() {
      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.clearRect(0, 0, overlay.width, overlay.height);
      ctx.restore();

      if (!lastState || !lastState.render_info) {
        drawPendingGoal();
        drawPendingInitPose();
        return;
      }

      drawGridOverlay();

      if (lastState.scan && lastState.scan.ok) {
        drawScanPoints(lastState.scan.points);
      }

      if (showPath.checked) {
        drawPath(
          getDisplayPath(lastState.paths.received_plan || lastState.paths.plan),
          "rgba(0,180,255,0.95)",
          3.0
        );

        drawPath(
          getDisplayPath(lastState.paths.local_plan),
          "rgba(255,80,80,0.95)",
          2.0
        );
      }

      if (lastState.pose && lastState.pose.ok) {
        drawRobot(lastState.pose.x, lastState.pose.y, lastState.pose.theta);
      }
      if (uiMode === "nav") {
        drawGoal(lastState.goal);
        drawPendingGoal();
        drawPendingInitPose();
      }
    }

    function updateStatus(state) {
      const slamOk = !!state.status.slam_ok;
      const tfOk = !!state.status.tf_ok;
      const plannerOk = !!state.status.planner_ok;

      const slamClass = slamOk ? "ok" : "bad";
      const tfClass = tfOk ? "ok" : "bad";
      const plannerClass = plannerOk ? "ok" : "warn";

      const pose = state.pose || {};
      const goal = state.goal || {};

      statusBox.innerHTML = `
        <div class="row">
          <span class="status-chip ${slamClass}">SLAM ${slamOk ? "OK" : "NO MAP"}</span>
          <span class="status-chip ${tfClass}">TF ${tfOk ? "OK" : "TIMEOUT"}</span>
          <span class="status-chip ${plannerClass}">PLANNER ${plannerOk ? "OK" : "IDLE/FAIL"}</span>
        </div>
        <div class="metric-grid">
          <div>Robot: <span class="mono" style="font-weight:700;color:#ffffff">x=${fmtNum(pose.x)}, y=${fmtNum(pose.y)}, yaw=${fmtNum((pose.theta || 0) * 180.0 / Math.PI, 1)} deg</span></div>
          <div>Map age: ${fmtAge(state.status.map_age_sec)}</div>
          <div>Pose age: ${fmtAge(state.status.pose_age_sec)}</div>
        </div>
      `;

      if (goal.x === null || goal.y === null || goal.yaw === null) {
        goalInfo.innerText = "Goal: none";
      } else {
        goalInfo.innerText =
          `Goal: x=${goal.x.toFixed(2)}, y=${goal.y.toFixed(2)}, yaw=${(goal.yaw * 180.0 / Math.PI).toFixed(1)} deg`;
      }
    }

    async function fetchSlamStatus() {
      try {
        const res = await api("/slam_status");
        const j = await res.json();

        if (j.running) {
          slamStatusBox.innerHTML =
            `<div class="ok">Running</div><div class="mono">${j.processes.join("\\n")}</div>`;
        } else {
          slamStatusBox.innerHTML =
            `<div class="warn">Not running</div><div class="mono">${(j.processes || []).join("\\n")}</div>`;
        }
      } catch (e) {
        slamStatusBox.innerHTML = `<div class="bad">Status error: ${e}</div>`;
      }
    }

    async function fetchState() {
      try {
        const res = await api("/state");
        const state = await res.json();
        lastState = state;

        if (state.map_version !== mapVersion) {
          mapVersion = state.map_version;
          mapImg.src = "/map.png?v=" + mapVersion + "&t=" + Date.now();
        }

        if (uiMode === "nav") {
          updateStatus(state);
        }

        drawOverlay();
      } catch (_) {
      }
    }

    async function sendGoal(x, y, yaw, u, v) {
      const url =
        `/set_goal_pose?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}` +
        `&u=${encodeURIComponent(u)}&v=${encodeURIComponent(v)}&yaw=${encodeURIComponent(yaw)}`;
      const res = await api(url);
      if (!res.ok) throw new Error("set_goal_pose failed");
    }

    async function sendInitialPose(x, y, yaw, u, v) {
      const url =
        `/set_initial_pose?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}` +
        `&u=${encodeURIComponent(u)}&v=${encodeURIComponent(v)}&yaw=${encodeURIComponent(yaw)}`;
      const res = await api(url);
      if (!res.ok) throw new Error("set_initial_pose failed");
    }

    mapImg.addEventListener("load", () => {
      syncOverlaySize();
      if (!hasInitialFit) {
        fitToScreen();
        hasInitialFit = true;
      } else {
        applyTransform();
        drawOverlay();
      }
    });

    window.addEventListener("resize", () => {
      syncOverlaySize();
    });

    viewport.addEventListener("wheel", (e) => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.1 : 0.9;
      zoomAt(e.clientX, e.clientY, factor);
    }, { passive: false });

    viewport.addEventListener("mousedown", (e) => {
      if (e.button !== 0) return;
      dragging = true;
      dragMoved = false;
      lastX = e.clientX;
      lastY = e.clientY;
      viewport.classList.add("dragging");
    });

    window.addEventListener("mouseup", () => {
      dragging = false;
      viewport.classList.remove("dragging");
    });

    window.addEventListener("mousemove", (e) => {
      if (dragging) {
        const dx = e.clientX - lastX;
        const dy = e.clientY - lastY;

        if (Math.abs(dx) > 2 || Math.abs(dy) > 2) {
          dragMoved = true;
        }

        tx += dx;
        ty += dy;
        lastX = e.clientX;
        lastY = e.clientY;

        applyTransform();
      }

      const p = clientToImageLocal(e.clientX, e.clientY);

      if (
        uiMode === "nav" &&
        p.x >= 0 && p.y >= 0 &&
        p.x <= p.rect.width && p.y <= p.rect.height &&
        lastState && lastState.render_info
      ) {
        const w = imageToWorld(p.x, p.y);
        if (w) {
          cursorInfo.innerText = `Coordinate: x=${w.x.toFixed(2)}, y=${w.y.toFixed(2)}`;
        }
      } else if (uiMode === "nav") {
        cursorInfo.innerText = "Coordinate: --";
      }

      if (pendingGoal || pendingInitPose) {
        mouseImg = { x: p.x, y: p.y };
      }

      if (dragging || pendingGoal || pendingInitPose) {
        drawOverlay();
      }
    });

    mapImg.addEventListener("click", async (e) => {
      if (uiMode !== "nav") return;
      if (dragMoved) return;
      if (!lastState || !lastState.render_info) return;

      const p = clientToImageLocal(e.clientX, e.clientY);

      if (p.x < 0 || p.y < 0 || p.x > p.rect.width || p.y > p.rect.height) {
        return;
      }

      const w = imageToWorld(p.x, p.y);
      if (!w) return;

      try {
        if (navAction === "goal") {
          if (!pendingGoal) {
            pendingGoal = {
              imgx: p.x,
              imgy: p.y,
              x: w.x,
              y: w.y,
              u: w.u,
              v: w.v
            };
            pendingInitPose = null;
            mouseImg = { x: p.x, y: p.y };
            drawOverlay();
            return;
          }

          const dx = p.x - pendingGoal.imgx;
          const dy = p.y - pendingGoal.imgy;
          const yaw = Math.atan2(-dy, dx);

          await sendGoal(pendingGoal.x, pendingGoal.y, yaw, pendingGoal.u, pendingGoal.v);

          pendingGoal = null;
          mouseImg = null;
          drawOverlay();
          return;
        }

        if (navAction === "initpose") {
          if (!pendingInitPose) {
            pendingInitPose = {
              imgx: p.x,
              imgy: p.y,
              x: w.x,
              y: w.y,
              u: w.u,
              v: w.v
            };
            pendingGoal = null;
            mouseImg = { x: p.x, y: p.y };
            drawOverlay();
            return;
          }

          const dx = p.x - pendingInitPose.imgx;
          const dy = p.y - pendingInitPose.imgy;
          const yaw = Math.atan2(-dy, dx);

          await sendInitialPose(pendingInitPose.x, pendingInitPose.y, yaw, pendingInitPose.u, pendingInitPose.v);

          pendingInitPose = null;
          mouseImg = null;
          drawOverlay();
          return;
        }
      } catch (err) {
        alert("Action failed: " + err);
      }
    });

    viewBtn.addEventListener("click", () => switchMode("view"));
    navBtn.addEventListener("click", () => switchMode("nav"));
    setGoalBtn.addEventListener("click", () => setNavAction("goal"));
    setInitPoseBtn.addEventListener("click", () => setNavAction("initpose"));

    resetViewBtn.addEventListener("click", resetView);
    rotateLeftBtn.addEventListener("click", () => setDisplayRotation(viewRotation - Math.PI / 36.0));
    rotateRightBtn.addEventListener("click", () => setDisplayRotation(viewRotation + Math.PI / 36.0));
    autoAlignBtn.addEventListener("click", autoAlignDisplay);
    resetAngleBtn.addEventListener("click", () => setDisplayRotation(0.0));
    toolbarToggleBtn.addEventListener("click", () => setToolbarCollapsed(!toolbarCollapsed));
    statusToggleBtn.addEventListener("click", () => setStatusVisible(!statusVisible));
    statusCloseBtn.addEventListener("click", () => setStatusVisible(false));

    clearBtn.addEventListener("click", async () => {
      try {
        await api("/clear_path");
      } catch (_) {}
    });

    saveBtn.addEventListener("click", async () => {
      const name = prompt("Map name (letters/numbers only):", "room1");
      if (!name) return;

      const safe = name.trim().replace(/[^a-zA-Z0-9_\\-]/g, "_");
      if (!safe) return;

      try {
        const res = await api(`/save_map?name=${encodeURIComponent(safe)}`);
        if (!res.ok) throw new Error("save_map failed");

        setTimeout(() => {
          window.open(`/maps/${encodeURIComponent(safe)}.bundle.zip`, "_blank");
        }, 1000);
      } catch (e) {
        alert("Save map failed: " + e);
      }
    });

    loadBtn.addEventListener("click", () => mapFile.click());

    mapFile.addEventListener("change", async (e) => {
      const file = e.target.files[0];
      if (!file) return;

      const formData = new FormData();
      formData.append("file", file);

      try {
        const res = await api("/upload_map", { method: "POST", body: formData });
        if (!res.ok) throw new Error("Upload failed");
        await fetchState();
        alert("Loaded map bundle: " + file.name);
      } catch (err) {
        alert("Upload error: " + err);
      } finally {
        mapFile.value = "";
      }
    });

    showRobot.addEventListener("change", drawOverlay);
    showPath.addEventListener("change", drawOverlay);
    showScan.addEventListener("change", drawOverlay);	
    showGrid.addEventListener("change", drawOverlay);

    viewRotation = loadRotation();
    setToolbarCollapsed(loadToolbarCollapsed());
    statusVisible = loadStatusVisible();
    updateAngleInfo();
    applyTransform();
    switchMode("view");
    fetchState();
    fetchSlamStatus();
    setInterval(fetchState, 1000);
    setInterval(fetchSlamStatus, 3000);
  </script>
</body>
</html>
"""

