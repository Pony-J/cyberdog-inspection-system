/* ── State ────────────────────────────────────────────────────────────────── */
const state = {
  currentMode: "services",
  maps: [],
  selectedScene: "",
  mapMeta: null,
  mapImage: null,
  liveMapMeta: null,
  liveMapImage: null,
  liveMapVersion: 0,
  liveMapFetchVersion: -1,
  costmaps: { global: { meta: null, image: null }, local: { meta: null, image: null } },
  showCostmap: false,
  robotPose: null,
  globalPath: [],
  localPath: [],
  plannedPath: [],
  inspectionStatus: {},
  interactionMode: "idle",
  visionData: { status: {}, runtime: {}, detections: {}, annotated: {} },
  alarmHistory: [],
  alarmLoaded: false,
  dragStart: null,
  dragCurrent: null,
  reconnectTimer: null,
  serviceStatus: {},
  robotStatus: {},
  tasks: {},
  meterPoints: [],
  meterReadingHistory: [],
  meterRoute: {
    pointIds: [],
    currentIndex: -1,
    phase: "idle",
  },
  maxLinVel: 0.15,
  maxLatVel: 0.10,
  maxAngVel: 0.60,
  lastSentNonZero: false,
  // View (pan/zoom)
  viewScale: 0,
  viewOffsetX: 0,
  viewOffsetY: 0,
  lastMapW: 0,
  lastMapH: 0,
  // Pan drag
  panning: false,
  panStartX: 0,
  panStartY: 0,
  panStartOffX: 0,
  panStartOffY: 0,
};

const DRAG_MIN_DISTANCE = 0.08;

/* ── DOM ──────────────────────────────────────────────────────────────────── */
const $ = (id) => document.getElementById(id);
const el = {
  modeTabs: document.querySelectorAll(".mode-tab"),
  servicesPanels: $("services-panels"),
  slamPanels: $("slam-panels"),
  navPanels: $("nav-panels"),
  inspectPanels: $("inspect-panels"),
  robotPanels: $("robot-panels"),
  viewTitle: $("view-title"),
  viewSubtitle: $("view-subtitle"),
  slamMapDot: $("slam-map-dot"),
  slamMapText: $("slam-map-text"),
  slamCostmapToggle: $("slam-costmap-toggle"),
  navMapDot: $("nav-map-dot"),
  navMapText: $("nav-map-text"),
  navCostmapToggle: $("nav-costmap-toggle"),
  navGoalBtn: $("btn-nav-goal"),
  navAmclBtn: $("btn-nav-amcl"),
  navCancelBtn: $("btn-nav-cancel"),
  navHint: $("nav-interaction-hint"),
  sceneSelect: $("scene-select"),
  refreshMapsBtn: $("btn-refresh-maps"),
  initializeBtn: $("btn-initialize"),
  startBtn: $("btn-start"),
  pauseBtn: $("btn-pause"),
  resumeBtn: $("btn-resume"),
  stopBtn: $("btn-stop"),
  costmapToggle: $("costmap-toggle"),
  connectionDot: $("connection-dot"),
  connectionText: $("connection-text"),
  statusBox: $("status-box"),
  posePreview: $("pose-preview"),
  debugBox: $("debug-box"),
  mapShell: $("map-shell"),
  canvas: $("map-canvas"),
  loading: $("canvas-loading"),
  robotStatusBox: $("robot-status-box"),
  nav2Dot: $("nav2-dot"),
  nav2Text: $("nav2-text"),
  inspSvcDot: $("insp-svc-dot"),
  inspSvcText: $("insp-svc-text"),
  rNav2Dot: $("r-nav2-dot"),
  rNav2Text: $("r-nav2-text"),
  rInspDot: $("r-insp-dot"),
  rInspText: $("r-insp-text"),
  joystickCanvas: $("joystick-canvas"),
  sensorPanels: $("sensor-panels"),
  sensorSvcDot: $("sensor-svc-dot"),
  sensorSvcText: $("sensor-svc-text"),
  sensorTemp: $("sensor-temp"),
  sensorHumi: $("sensor-humi"),
  sensorLight: $("sensor-light"),
  sensorSound: $("sensor-sound"),
  sensorIr: $("sensor-ir"),
  sensorTime: $("sensor-time"),
  sensorStats: $("sensor-stats"),
  // vision
  visionPanels: $("vision-panels"),
  visionDot: $("vision-dot"),
  visionText: $("vision-text"),
  visionFps: $("vision-fps"),
  visionLatency: $("vision-latency"),
  visionEnabled: $("vision-enabled"),
  visionCounts: $("vision-counts"),
  visionDetections: $("vision-detections"),
  visionDebug: $("vision-debug"),
  alarmHistoryList: $("alarm-history-list"),
  alarmModal: $("alarm-modal"),
  alarmModalImg: $("alarm-modal-img"),
  alarmModalTitle: $("alarm-modal-title"),
  alarmModalInfo: $("alarm-modal-info"),
  alarmModalClose: $("alarm-modal-close"),
  alarmModalOverlay: $("alarm-modal-overlay"),
  meterPointsList: $("meter-points-list"),
  meterHistoryList: $("meter-history-list"),
  meterRouteStatus: $("meter-route-status"),
  meterRouteList: $("meter-route-list"),
  meterPointName: $("meter-point-name"),
  meterPointType: $("meter-point-type"),
  btnRecordMeterPoint: $("btn-record-meter-point"),
  btnMeterRouteStart: $("btn-meter-route-start"),
  btnMeterRouteArrived: $("btn-meter-route-arrived"),
  btnMeterRouteNext: $("btn-meter-route-next"),
  btnMeterRouteClear: $("btn-meter-route-clear"),
};
const ctx = el.canvas.getContext("2d");

/* ── Utilities ────────────────────────────────────────────────────────────── */
function unwrap(p) { return p && typeof p === "object" && "data" in p ? p.data : p; }
function setStatus(msg, tone = "neutral") { el.statusBox.textContent = msg; el.statusBox.dataset.tone = tone; }
function updateConnectionState(ok) {
  el.connectionDot.classList.toggle("online", ok);
  el.connectionDot.classList.toggle("offline", !ok);
  el.connectionText.textContent = ok ? "实时连接正常" : "连接中断";
}
function setLoading(on, msg = "正在加载...") {
  el.loading.classList.toggle("hidden", !on);
  const s = el.loading.querySelector("span");
  if (s) s.textContent = msg;
  const spinner = $("loading-spinner");
  const icon = $("loading-icon");
  const isIdle = msg.includes("暂无");
  if (spinner) spinner.classList.toggle("hidden", isIdle);
  if (icon) icon.classList.toggle("hidden", !isIdle);
}

async function requestJson(url, opts = {}) {
  const r = await fetch(url, opts);
  let p = null;
  try { p = await r.json(); } catch { p = null; }
  if (!r.ok || (p && p.success === false))
    throw new Error(p?.detail || p?.message || `${r.status} ${r.statusText}`);
  return p;
}
async function requestPost(url, body = null) {
  return requestJson(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body ? JSON.stringify(body) : null,
  });
}
async function requestDelete(url) {
  return requestJson(url, { method: "DELETE" });
}
function loadImage(url) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => resolve(img);
    img.onerror = () => reject(new Error(`图片加载失败: ${url}`));
    img.src = `${url}${url.includes("?") ? "&" : "?"}t=${Date.now()}`;
  });
}

function visionClassLabel(detectorName, classId, description = "") {
  const desc = String(description || "").trim();
  if (detectorName === "meter") {
    return desc || classId || detectorName || "meter";
  }
  if (detectorName === "fire_alarm") {
    if (desc && desc !== classId) return `${desc} (${classId})`;
    if (classId === "fire") return "火焰 (fire)";
    if (classId === "white_smoke") return "烟雾 (white_smoke)";
  }
  return desc && desc !== classId ? `${desc} (${classId})` : (classId || detectorName || "unknown");
}

function visionDetectionValueText(d) {
  if ((d?.detector || "") === "meter") {
    const reading = String(d?.reading_text || "").trim();
    if (reading) return `读数 ${reading}`;
    if (typeof d?.reading_value === "number" && Number.isFinite(d.reading_value)) {
      const unit = String(d?.reading_unit || "").trim();
      return `读数 ${d.reading_value}${unit ? " " + unit : ""}`;
    }
    return "读数 --";
  }
  return d?.score > 0 ? `${(d.score * 100).toFixed(0)}%` : "N/A";
}

function getMeterPointById(pointId) {
  return (state.meterPoints || []).find(p => p?.point_id === pointId) || null;
}

function getActiveMeterRoutePoint() {
  const route = state.meterRoute || {};
  if (!Array.isArray(route.pointIds)) return null;
  if (route.currentIndex < 0 || route.currentIndex >= route.pointIds.length) return null;
  return getMeterPointById(route.pointIds[route.currentIndex]);
}

function getLatestMeterDetections() {
  const dets = state.visionData?.detections?.detections || [];
  return dets.filter(d => (d?.detector || "") === "meter");
}

function getLatestMeterReading() {
  const meterDets = getLatestMeterDetections();
  if (!meterDets.length) return null;
  const preferred = meterDets.find(d => {
    const txt = String(d?.reading_text || "").trim();
    return txt || (typeof d?.reading_value === "number" && Number.isFinite(d.reading_value));
  }) || meterDets[0];
  const readingText = String(preferred?.reading_text || "").trim();
  const readingValue = typeof preferred?.reading_value === "number" && Number.isFinite(preferred.reading_value)
    ? preferred.reading_value
    : null;
  const readingUnit = String(preferred?.reading_unit || "").trim();
  return {
    readingText: readingText || (readingValue != null ? `${readingValue}${readingUnit ? " " + readingUnit : ""}` : ""),
    readingValue,
    readingUnit,
    detector: preferred?.detector || "meter",
    description: String(preferred?.description || "").trim(),
  };
}

function getLatestMeterReadingText() {
  const meterDets = getLatestMeterDetections();
  if (!meterDets.length) return "";
  return meterDets.map(d => {
    const txt = String(d?.reading_text || "").trim();
    if (txt) return txt;
    if (typeof d?.reading_value === "number" && Number.isFinite(d.reading_value)) {
      const unit = String(d?.reading_unit || "").trim();
      return `${d.reading_value}${unit ? " " + unit : ""}`;
    }
    return String(d?.description || "").trim();
  }).filter(Boolean).join(" / ");
}

async function ensureMeterDetectorEnabled() {
  const rt = state.visionData?.runtime || {};
  const available = new Set(rt.available || []);
  if (available.size && !available.has("meter")) {
    throw new Error("meter 检测器当前不可用");
  }
  const desired = new Set((rt.desired || []).slice());
  if (!desired.has("meter")) {
    desired.add("meter");
    await requestPost("/api/v1/vision/detectors", { detectors: Array.from(desired) });
  }
}

async function navigateToMeterPoint(point) {
  if (!point?.pose) throw new Error("点位缺少 pose");
  await ensureMeterDetectorEnabled();
  await requestPost("/api/v1/navigation/goal", {
    x: Number(point.pose.x || 0),
    y: Number(point.pose.y || 0),
    yaw: Number(point.pose.yaw || 0),
  });
}

function formatMeterRecordTime(ts) {
  if (!ts) return "--";
  const parts = String(ts).split("_");
  if (parts.length >= 2 && parts[0].length === 8) {
    return `${parts[0].slice(0, 4)}-${parts[0].slice(4, 6)}-${parts[0].slice(6, 8)} ${parts[1].slice(0, 2)}:${parts[1].slice(2, 4)}:${parts[1].slice(4, 6)}`;
  }
  return String(ts);
}

async function recordCurrentMeterReading(point, routeIndex = -1) {
  const reading = getLatestMeterReading();
  if (!point) throw new Error("当前点位不存在");
  if (!reading || !reading.readingText) {
    throw new Error("当前没有可确认的表盘读数");
  }
  const pose = point.pose || {};
  const res = await requestPost("/api/v1/meter-history/record", {
    point_id: point.point_id || "",
    point_name: point.name || point.point_id || "",
    meter_type: point.meter_type || "pressure",
    reading_value: reading.readingValue,
    reading_unit: reading.readingUnit,
    reading_text: reading.readingText,
    route_index: routeIndex,
    source: "route_confirm",
    pose: {
      x: Number(pose.x || 0),
      y: Number(pose.y || 0),
      yaw: Number(pose.yaw || 0),
    },
  });
  await fetchMeterReadingHistory();
  return res?.data || null;
}

function addMeterRoutePoint(pointId) {
  const route = state.meterRoute;
  if (!route.pointIds.includes(pointId)) {
    route.pointIds.push(pointId);
  }
  renderMeterRoute();
}

function removeMeterRoutePoint(pointId) {
  const route = state.meterRoute;
  const idx = route.pointIds.indexOf(pointId);
  if (idx < 0) return;
  route.pointIds.splice(idx, 1);
  if (route.currentIndex >= route.pointIds.length) {
    route.currentIndex = route.pointIds.length - 1;
  }
  if (!route.pointIds.length) {
    route.currentIndex = -1;
    route.phase = "idle";
  }
  renderMeterRoute();
}

function clearMeterRoute() {
  state.meterRoute = {
    pointIds: [],
    currentIndex: -1,
    phase: "idle",
  };
  renderMeterRoute();
}

async function goToMeterRouteIndex(index) {
  const route = state.meterRoute;
  if (index < 0 || index >= route.pointIds.length) {
    throw new Error("路线索引无效");
  }
  const point = getMeterPointById(route.pointIds[index]);
  if (!point) {
    throw new Error("路线点位不存在");
  }
  route.currentIndex = index;
  route.phase = "navigating";
  renderMeterRoute();
  await navigateToMeterPoint(point);
  setStatus(`已发送导航：${point.name || point.point_id}。到点后可切到“控制”页微调，再点“到点微调”`, "success");
}

function renderMeterRoute() {
  const statusEl = el.meterRouteStatus;
  const listEl = el.meterRouteList;
  if (!statusEl || !listEl) return;

  const route = state.meterRoute || { pointIds: [], currentIndex: -1, phase: "idle" };
  const activePoint = getActiveMeterRoutePoint();
  const latestReading = getLatestMeterReadingText();

  if (!route.pointIds.length) {
    statusEl.textContent = "当前没有表盘路线任务";
    statusEl.dataset.tone = "neutral";
    listEl.innerHTML = '<div class="status-box subtle-box" data-tone="neutral">从上面的表盘点位里点“加入路线”，就可以按顺序前往。</div>';
  } else {
    let statusText = `路线共 ${route.pointIds.length} 个点`;
    let tone = "neutral";
    if (route.phase === "navigating" && activePoint) {
      statusText = `正在前往：${activePoint.name || activePoint.point_id}`;
      tone = "info";
    } else if (route.phase === "adjusting" && activePoint) {
      statusText = `已到点：${activePoint.name || activePoint.point_id}，现在可以微调画面`;
      tone = "success";
    } else if (route.phase === "done") {
      statusText = "表盘路线已完成";
      tone = "success";
    }
    if (latestReading) {
      statusText += ` | 当前读数：${latestReading}`;
    }
    statusEl.textContent = statusText;
    statusEl.dataset.tone = tone;

    listEl.innerHTML = route.pointIds.map((pointId, index) => {
      const point = getMeterPointById(pointId);
      if (!point) return "";
      const isCurrent = index === route.currentIndex;
      const phaseLabel = isCurrent
        ? (route.phase === "navigating" ? "前往中" : route.phase === "adjusting" ? "微调中" : route.phase === "done" ? "完成" : "当前")
        : index < route.currentIndex ? "已完成" : "待执行";
      return `<div class="alarm-card ${isCurrent ? "is-route-active" : ""}" data-route-point-id="${pointId}">
        <div class="alarm-card-icon">${isCurrent ? "&#x1F916;" : "&#x1F4CD;"}</div>
        <div class="alarm-card-body">
          <div class="alarm-card-title">${index + 1}. ${point.name || point.point_id}</div>
          <div class="alarm-card-meta">${phaseLabel} &middot; ${point.meter_type || "pressure"}</div>
        </div>
        <button class="small-btn danger" data-remove-route-point="${pointId}">移出</button>
      </div>`;
    }).join("");

    listEl.querySelectorAll("[data-remove-route-point]").forEach(btn => {
      btn.addEventListener("click", (ev) => {
        ev.stopPropagation();
        removeMeterRoutePoint(btn.dataset.removeRoutePoint);
      });
    });
  }

  if (el.btnMeterRouteStart) {
    el.btnMeterRouteStart.disabled = route.pointIds.length === 0;
  }
  if (el.btnMeterRouteArrived) {
    el.btnMeterRouteArrived.disabled = route.currentIndex < 0 || route.phase !== "navigating";
  }
  if (el.btnMeterRouteNext) {
    el.btnMeterRouteNext.disabled = route.currentIndex < 0 || !["adjusting", "navigating"].includes(route.phase);
  }
  if (el.btnMeterRouteClear) {
    el.btnMeterRouteClear.disabled = route.pointIds.length === 0;
  }
}

async function fetchMeterReadingHistory() {
  try {
    const res = await requestJson("/api/v1/meter-history?limit=30");
    state.meterReadingHistory = res.data?.records || [];
    renderMeterReadingHistory();
  } catch (e) {
    state.meterReadingHistory = [];
    renderMeterReadingHistory();
    setStatus(`加载表盘记录失败: ${e.message}`, "error");
  }
}

function renderMeterReadingHistory() {
  const list = el.meterHistoryList;
  if (!list) return;
  const records = state.meterReadingHistory || [];
  if (!records.length) {
    list.innerHTML = '<div class="status-box subtle-box" data-tone="neutral">暂无已确认的表盘读数</div>';
    return;
  }
  list.innerHTML = records.map(record => {
    const pointName = record.point_name || record.point_id || "未命名点位";
    const readingText = String(record.reading_text || "").trim() || "--";
    const meterType = String(record.meter_type || "pressure");
    const routeText = Number.isInteger(record.route_index) && record.route_index >= 0
      ? `路线点 ${record.route_index + 1}`
      : "手动确认";
    return `<div class="alarm-card is-meter-record" data-meter-record-id="${record.id}">
      <div class="alarm-card-icon meter-record-icon"><i class="ri-dashboard-3-line"></i></div>
      <div class="alarm-card-body">
        <div class="alarm-card-title">${pointName}</div>
        <div class="alarm-card-meta">${formatMeterRecordTime(record.timestamp)} &middot; ${meterType} &middot; ${routeText}</div>
      </div>
      <div class="meter-record-reading">${readingText}</div>
    </div>`;
  }).join("");
}

function visionDetectorSummary(detectorName, detectorInfo = {}) {
  const total = detectorInfo.count ?? 0;
  const classEntries = Object.entries(detectorInfo.classes || {});
  if (classEntries.length === 0) return `${detectorName}: ${total}`;
  const classesText = classEntries
    .map(([classId, count]) => `${visionClassLabel(detectorName, classId)} ${count}`)
    .join(", ");
  return `${detectorName}: ${total} [${classesText}]`;
}

/* ── Mode Switching ───────────────────────────────────────────────────────── */
const MODE_TITLES = {
  services: ["服务管理", "启动和管理 Nav2、巡检等核心服务"],
  slam: ["SLAM 建图", "实时查看 Cartographer 建图进度"],
  nav: ["导航控制", "拖拽设定目标点和朝向"],
  inspect: ["巡检任务", "选择地图场景并控制自动巡检"],
  robot: ["机器人控制", "遥控移动、模式切换、动作指令"],
  sensor: ["传感器数据", "查看 Type-C 传感器实时数据"],
  vision: ["视觉检测", "查看实时目标检测状态和结果"],
};

function switchMode(mode) {
  state.currentMode = mode;
  el.modeTabs.forEach(t => t.classList.toggle("active", t.dataset.mode === mode));
  [el.servicesPanels, el.slamPanels, el.navPanels, el.inspectPanels, el.robotPanels, el.sensorPanels, el.visionPanels].forEach(p => p.classList.add("hidden"));
  const panelMap = { services: el.servicesPanels, slam: el.slamPanels, nav: el.navPanels, inspect: el.inspectPanels, robot: el.robotPanels, sensor: el.sensorPanels, vision: el.visionPanels };
  panelMap[mode]?.classList.remove("hidden");
  const [t, s] = MODE_TITLES[mode] || ["", ""];
  el.viewTitle.textContent = t;
  el.viewSubtitle.textContent = s;
  setInteractionMode("idle");
  renderCanvas();
}

/* ── Active Map / Meta ────────────────────────────────────────────────────── */
function activeImage() {
  if (state.currentMode === "inspect") return state.mapImage || state.liveMapImage;
  return state.liveMapImage;
}
function activeMeta() {
  if (state.currentMode === "inspect") return state.mapMeta || state.liveMapMeta;
  return state.liveMapMeta;
}

/* ── View (Pan / Zoom) ────────────────────────────────────────────────────── */
function resetView() {
  const img = activeImage();
  if (!img) { state.viewScale = 0; return; }
  const pad = 40, cw = el.canvas.clientWidth, ch = el.canvas.clientHeight;
  if (!cw || !ch) return;
  const sc = Math.min((cw - pad * 2) / img.width, (ch - pad * 2) / img.height);
  state.viewScale = sc;
  state.viewOffsetX = (cw - img.width * sc) / 2;
  state.viewOffsetY = (ch - img.height * sc) / 2;
  state.lastMapW = img.width;
  state.lastMapH = img.height;
}

function ensureView() {
  const img = activeImage();
  if (!img) return;
  if (state.viewScale <= 0 || img.width !== state.lastMapW || img.height !== state.lastMapH) {
    resetView();
  }
}

function getFit() {
  const img = activeImage();
  if (!img || state.viewScale <= 0) return null;
  return {
    scale: state.viewScale,
    offsetX: state.viewOffsetX,
    offsetY: state.viewOffsetY,
    drawWidth: img.width * state.viewScale,
    drawHeight: img.height * state.viewScale,
  };
}

function zoomAtPoint(canvasX, canvasY, factor) {
  const oldScale = state.viewScale;
  const newScale = Math.max(0.3, Math.min(80, oldScale * factor));
  const ratio = newScale / oldScale;
  state.viewOffsetX = canvasX - (canvasX - state.viewOffsetX) * ratio;
  state.viewOffsetY = canvasY - (canvasY - state.viewOffsetY) * ratio;
  state.viewScale = newScale;
  renderCanvas();
}

/* ── Canvas sizing ────────────────────────────────────────────────────────── */
function resizeCanvas() {
  const r = el.mapShell.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width)), h = Math.max(1, Math.round(r.height));
  el.canvas.width = Math.round(w * dpr);
  el.canvas.height = Math.round(h * dpr);
  el.canvas.style.width = `${w}px`;
  el.canvas.style.height = `${h}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  resetView();
  renderCanvas();
}

/* ── Coordinate transforms ────────────────────────────────────────────────── */
function normalizeOrigin(o) {
  if (Array.isArray(o)) return { x: o[0] || 0, y: o[1] || 0 };
  return { x: o?.x || 0, y: o?.y || 0 };
}
function rotateVec(x, y, a) {
  const c = Math.cos(a), s = Math.sin(a);
  return { x: x * c - y * s, y: x * s + y * c };
}
function worldToImage(w, meta) {
  const o = normalizeOrigin(meta.origin);
  return { x: (w.x - o.x) / meta.resolution, y: meta.height - (w.y - o.y) / meta.resolution };
}
function imageToWorld(ip, meta) {
  const o = normalizeOrigin(meta.origin);
  return { x: o.x + ip.x * meta.resolution, y: o.y + (meta.height - ip.y) * meta.resolution };
}
function worldToCanvas(w) {
  const meta = activeMeta();
  const fit = getFit();
  if (!meta || !fit) return { x: 0, y: 0 };
  const ip = worldToImage(w, meta);
  return { x: fit.offsetX + ip.x * fit.scale, y: fit.offsetY + ip.y * fit.scale };
}
function canvasToImage(cx, cy) {
  const fit = getFit();
  if (!fit) return null;
  return { x: (cx - fit.offsetX) / fit.scale, y: (cy - fit.offsetY) / fit.scale };
}
function eventToCanvas(e) {
  const r = el.canvas.getBoundingClientRect();
  return { x: e.clientX - r.left, y: e.clientY - r.top };
}
function eventToWorld(e) {
  const cp = eventToCanvas(e);
  const ip = canvasToImage(cp.x, cp.y);
  const meta = activeMeta();
  if (!ip || !meta) return null;
  if (ip.x < 0 || ip.y < 0 || ip.x > meta.width || ip.y > meta.height) return null;
  return imageToWorld(ip, meta);
}

/* ── Pose helpers ─────────────────────────────────────────────────────────── */
function normPose(raw) {
  if (!raw) return null;
  if (typeof raw.x === "number") return { x: raw.x, y: raw.y, yaw: raw.yaw || 0 };
  return null;
}
function normPath(raw) {
  if (!Array.isArray(raw)) return [];
  return raw.map(i => Array.isArray(i) && i.length >= 2 ? { x: i[0], y: i[1] } : normPose(i)).filter(Boolean);
}

/* ── Interaction mode ─────────────────────────────────────────────────────── */
function updateCanvasCursorClass() {
  el.canvas.classList.remove("mode-idle", "mode-nav", "mode-amcl");
  el.canvas.classList.add(`mode-${state.interactionMode}`);
}
function setInteractionMode(mode) {
  state.interactionMode = mode;
  state.dragStart = null;
  state.dragCurrent = null;
  updateCanvasCursorClass();
  if (state.currentMode === "nav") {
    el.navGoalBtn?.classList.toggle("active", mode === "nav");
    el.navAmclBtn?.classList.toggle("active", mode === "amcl");
    el.navCancelBtn?.classList.toggle("active", mode === "idle");
    if (el.navHint) {
      if (mode === "amcl") el.navHint.textContent = "拖拽设置位置和朝向。起点=位置，方向=朝向。";
      else if (mode === "nav") el.navHint.textContent = "拖拽设置目标点和朝向。起点=目标，方向=朝向。";
      else el.navHint.textContent = "滚轮缩放，拖拽平移地图。";
    }
  }
  renderCanvas();
}

/* ── Drawing ──────────────────────────────────────────────────────────────── */
function drawPolyline(pts, color, lw, dashed = false) {
  const meta = activeMeta();
  const fit = getFit();
  if (!pts.length || !meta || !fit) return;
  ctx.save();
  ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.lineJoin = "round"; ctx.lineCap = "round";
  if (dashed) ctx.setLineDash([8, 8]);
  ctx.beginPath();
  pts.forEach((wp, i) => {
    const ip = worldToImage(wp, meta);
    const x = fit.offsetX + ip.x * fit.scale;
    const y = fit.offsetY + ip.y * fit.scale;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawSingleCostmap(cm, alpha) {
  const meta = activeMeta();
  const fit = getFit();
  if (!cm?.image || !cm?.meta || !fit || !meta) return;
  const co = normalizeOrigin(cm.meta.origin);
  const oy = cm.meta.origin_yaw || 0;
  const wm = cm.meta.width * cm.meta.resolution, hm = cm.meta.height * cm.meta.resolution;
  const tl = worldToCanvas({ x: co.x + rotateVec(0, hm, oy).x, y: co.y + rotateVec(0, hm, oy).y });
  const tr = worldToCanvas({ x: co.x + rotateVec(wm, hm, oy).x, y: co.y + rotateVec(wm, hm, oy).y });
  const bl = worldToCanvas({ x: co.x, y: co.y });
  const ax = { x: (tr.x - tl.x) / cm.image.width, y: (tr.y - tl.y) / cm.image.width };
  const ay = { x: (bl.x - tl.x) / cm.image.height, y: (bl.y - tl.y) / cm.image.height };
  ctx.save(); ctx.globalAlpha = alpha;
  ctx.transform(ax.x, ax.y, ay.x, ay.y, tl.x, tl.y);
  ctx.drawImage(cm.image, 0, 0);
  ctx.restore();
}

function drawRobot() {
  const meta = activeMeta();
  const fit = getFit();
  if (!state.robotPose || !fit || !meta) return;
  const ip = worldToImage(state.robotPose, meta);
  const x = fit.offsetX + ip.x * fit.scale;
  const y = fit.offsetY + ip.y * fit.scale;

  const ppm = fit.scale / meta.resolution;
  const halfL = 0.25 * ppm;
  const halfW = 0.15 * ppm;

  if (halfL < 2) { return; }

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-(state.robotPose.yaw || 0));

  const lw = Math.max(1, 0.018 * ppm);

  ctx.fillStyle = "rgba(28,28,30,0.10)";
  ctx.beginPath();
  ctx.ellipse(0, halfW * 0.3, halfL * 0.9, halfW * 0.5, 0, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#2563eb";
  ctx.strokeStyle = "rgba(255,255,255,0.92)";
  ctx.lineWidth = lw;
  ctx.beginPath();
  ctx.roundRect(-halfL * 0.75, -halfW, halfL * 1.5, halfW * 2, halfW * 0.35);
  ctx.fill(); ctx.stroke();

  ctx.fillStyle = "#60a5fa";
  ctx.beginPath();
  ctx.arc(halfL * 0.95, 0, halfW * 0.42, 0, Math.PI * 2);
  ctx.fill(); ctx.stroke();

  ctx.strokeStyle = "#1d4ed8";
  ctx.lineWidth = Math.max(1.2, 0.022 * ppm);
  const lx = halfL * 0.5, ly = halfW;
  [[-lx, -ly, -lx * 1.2, -ly * 1.55], [lx, -ly, lx * 1.2, -ly * 1.55],
   [-lx, ly, -lx * 1.2, ly * 1.55], [lx, ly, lx * 1.2, ly * 1.55]].forEach(([x1, y1, x2, y2]) => {
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  });
  ctx.beginPath(); ctx.moveTo(-halfL * 0.75, 0); ctx.lineTo(-halfL * 1.1, -halfW * 0.3); ctx.stroke();

  ctx.strokeStyle = "#ff3b30"; ctx.fillStyle = "#ff3b30";
  ctx.lineWidth = Math.max(1.5, 0.02 * ppm);
  const aLen = halfL * 0.9;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(aLen, 0); ctx.stroke();
  const ah = halfW * 0.35;
  ctx.beginPath(); ctx.moveTo(aLen + ah * 1.3, 0); ctx.lineTo(aLen - ah * 0.3, ah); ctx.lineTo(aLen - ah * 0.3, -ah); ctx.closePath(); ctx.fill();

  ctx.restore();
}

function drawDragPreview() {
  const meta = activeMeta();
  const fit = getFit();
  if (!state.dragStart || !state.dragCurrent || !fit || !meta) return;
  const si = worldToImage(state.dragStart, meta), ci = worldToImage(state.dragCurrent, meta);
  const sx = fit.offsetX + si.x * fit.scale, sy = fit.offsetY + si.y * fit.scale;
  const cx = fit.offsetX + ci.x * fit.scale, cy = fit.offsetY + ci.y * fit.scale;
  const dist = Math.hypot(cx - sx, cy - sy);

  ctx.save();
  // Start point marker
  ctx.fillStyle = "rgba(0,122,255,0.25)";
  ctx.beginPath(); ctx.arc(sx, sy, 8, 0, Math.PI * 2); ctx.fill();

  if (dist > 4) {
    const a = Math.atan2(cy - sy, cx - sx);
    ctx.strokeStyle = "rgba(0,122,255,0.85)"; ctx.fillStyle = "rgba(0,122,255,0.85)"; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(cx, cy); ctx.stroke();
    ctx.translate(cx, cy); ctx.rotate(a);
    ctx.beginPath(); ctx.moveTo(10, 0); ctx.lineTo(-5, 6); ctx.lineTo(-5, -6); ctx.closePath(); ctx.fill();
  }
  ctx.restore();
}

function renderCanvas() {
  const w = el.canvas.clientWidth, h = el.canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);
  const img = activeImage(), meta = activeMeta();
  if (!img || !meta) return;
  ensureView();
  const fit = getFit();
  if (!fit) return;
  ctx.drawImage(img, fit.offsetX, fit.offsetY, fit.drawWidth, fit.drawHeight);
  if (state.showCostmap) {
    drawSingleCostmap(state.costmaps.global, 0.42);
    drawSingleCostmap(state.costmaps.local, 0.72);
  }
  if (state.currentMode === "inspect") drawPolyline(state.plannedPath, "#007aff", 4);
  drawPolyline(state.globalPath, "#34c759", 3.5);
  drawPolyline(state.localPath, "#ff9f0a", 3, true);
  drawRobot();
  drawDragPreview();
}

/* ── Pose preview ─────────────────────────────────────────────────────────── */
function updatePosePreview(pose = null) {
  if (!pose) { el.posePreview.textContent = "等待定位数据..."; return; }
  const deg = ((pose.yaw || 0) * 180 / Math.PI).toFixed(1);
  el.posePreview.textContent = `x: ${pose.x.toFixed(3)}\ny: ${pose.y.toFixed(3)}\nyaw: ${deg}°`;
}
function updateDebug() {
  el.debugBox.textContent = JSON.stringify({
    mode: state.currentMode, interaction: state.interactionMode,
      robotPose: state.robotPose,
    zoom: state.viewScale.toFixed(4),
    liveMapVersion: state.liveMapVersion,
  }, null, 2);
}

/* ── Live map ─────────────────────────────────────────────────────────────── */
function updateLiveMapIndicator() {
  const dots = [el.slamMapDot, el.navMapDot];
  const texts = [el.slamMapText, el.navMapText];
  const ok = !!state.liveMapImage;
  dots.forEach(d => { if (d) { d.classList.toggle("online", ok); d.classList.toggle("offline", !ok); } });
  if (state.liveMapMeta) {
    const m = state.liveMapMeta;
    texts.forEach(t => { if (t) t.textContent = `地图 ${m.width}×${m.height} @ ${m.resolution}m`; });
  } else {
    texts.forEach(t => { if (t) t.textContent = "等待地图数据..."; });
  }
}

async function fetchLiveMap() {
  if (state.currentMode === "inspect" && state.mapImage) return;
  try {
    const metaPayload = await requestJson("/api/v1/live-map/metadata");
    const meta = unwrap(metaPayload);
    const v = meta.version ?? 0;
    if (state.liveMapFetchVersion >= v && state.liveMapImage) return;
    const img = await loadImage("/api/v1/live-map/image");
    state.liveMapMeta = meta;
    state.liveMapImage = img;
    state.liveMapFetchVersion = v;
    updateLiveMapIndicator();
    setLoading(false);
    renderCanvas();
  } catch {
    updateLiveMapIndicator();
    if (!state.liveMapImage && !state.mapImage) {
      setLoading(true, "暂无地图 — 请先启动建图或导航服务");
    }
  }
}

/* ── Inspection ───────────────────────────────────────────────────────────── */
async function loadMaps() {
  const payload = await requestJson("/api/v1/maps");
  const data = unwrap(payload);
  state.maps = Array.isArray(data?.maps) ? data.maps : [];
  el.sceneSelect.innerHTML = "";
  state.maps.forEach(m => {
    const o = document.createElement("option");
    o.value = m.scene_name; o.textContent = m.scene_name;
    el.sceneSelect.appendChild(o);
  });
  if (!state.selectedScene && state.maps.length) state.selectedScene = state.maps[0].scene_name;
  if (state.selectedScene) el.sceneSelect.value = state.selectedScene;
}
async function loadScene(name) {
  if (!name) return;
  state.selectedScene = name;
  setLoading(true, "正在加载地图...");
  const md = unwrap(await requestJson(`/api/v1/maps/${encodeURIComponent(name)}/metadata`));
  const url = md.image_url || `/api/v1/maps/${encodeURIComponent(name)}/image`;
  state.mapMeta = md;
  state.mapImage = await loadImage(url);
  state.costmaps = { global: { meta: null, image: null }, local: { meta: null, image: null } };
  state.viewScale = 0;
  if (state.showCostmap) await updateCostmap();
  setLoading(false);
  setStatus(`已加载地图 ${name}`, "success");
  renderCanvas();
}

async function updateCostmap() {
  if (!state.showCostmap) {
    state.costmaps = { global: { meta: null, image: null }, local: { meta: null, image: null } };
    renderCanvas(); return;
  }
  await Promise.all(["global", "local"].map(async t => {
    try {
      state.costmaps[t].meta = unwrap(await requestJson(`/api/v1/costmap/${t}/metadata`));
      state.costmaps[t].image = await loadImage(`/api/v1/costmap/${t}/image`);
    } catch { state.costmaps[t] = { meta: null, image: null }; }
  }));
  renderCanvas();
}

async function refreshInspectionStatus() {
  try {
    const d = unwrap(await requestJson("/api/v1/inspection/planned-path"));
    state.plannedPath = normPath(d?.planned_path);
    renderCanvas();
  } catch {}
}

function nav2StartStatus(mode, alreadyRunning = false) {
  const label = mode === "slam" ? "Nav2 SLAM" : `Nav2 (${mode})`;
  return {
    message: alreadyRunning ? `${label} 已在运行` : `${label} 启动中...`,
    tone: alreadyRunning ? "info" : "success",
  };
}

/* ── Actions ──────────────────────────────────────────────────────────────── */
function computeDragYaw() {
  if (!state.dragStart || !state.dragCurrent) return 0;
  const dx = state.dragCurrent.x - state.dragStart.x;
  const dy = state.dragCurrent.y - state.dragStart.y;
  const dist = Math.hypot(dx, dy);
  if (dist < DRAG_MIN_DISTANCE) return state.robotPose?.yaw ?? 0;
  return Math.atan2(dy, dx);
}

async function sendInitialPose() {
  if (!state.dragStart) return;
  const yaw = computeDragYaw();
  await requestPost("/api/v1/localization/initial-pose", { x: state.dragStart.x, y: state.dragStart.y, yaw });
  setStatus("已发送初始位姿", "success");
  setInteractionMode("idle");
}

async function sendNavGoal() {
  if (!state.dragStart) return;
  const yaw = computeDragYaw();
  await requestPost("/api/v1/navigation/goal", { x: state.dragStart.x, y: state.dragStart.y, yaw });
  setStatus("导航目标已发送", "success");
  setInteractionMode("idle");
}

/* ── Service indicators ───────────────────────────────────────────────────── */
let _prevInspRunning = false;

function updateServiceIndicators() {
  const ss = state.serviceStatus;
  const nav2 = ss.nav2, insp = ss.inspection;
  const n2r = nav2?.running === true, ir = insp?.running === true;
  [el.nav2Dot, el.rNav2Dot].forEach(d => { d.classList.toggle("online", n2r); d.classList.toggle("offline", !n2r); });
  const n2m = nav2?.mode ? ` (${nav2.mode})` : "";
  el.nav2Text.textContent = n2r ? `运行中${n2m}` : "未运行";
  el.rNav2Text.textContent = `Nav2: ${n2r ? "运行中" + n2m : "未运行"}`;
  [el.inspSvcDot, el.rInspDot].forEach(d => { d.classList.toggle("online", ir); d.classList.toggle("offline", !ir); });
  el.inspSvcText.textContent = ir ? "运行中" : "未运行";
  el.rInspText.textContent = `巡检: ${ir ? "运行中" : "未运行"}`;

  // Inline inspection panel in inspect tab
  const iDot = $("inspect-inline-dot");
  const iTxt = $("inspect-inline-text");
  const iHint = $("inspect-svc-hint");
  const iStartBtn = $("btn-inspect-quick-start");
  const iStopBtn = $("btn-inspect-quick-stop");
  if (iDot) { iDot.classList.toggle("online", ir); iDot.classList.toggle("offline", !ir); }
  if (iTxt) iTxt.textContent = ir ? "运行中" : "未运行";
  if (iHint) {
    iHint.classList.toggle("hidden", ir);
    if (!ir) { iHint.dataset.tone = "error"; iHint.textContent = "巡检服务未启动，请先启动后才能选择地图和控制巡检。"; }
  }
  if (iStartBtn) iStartBtn.classList.toggle("hidden", ir);
  if (iStopBtn) iStopBtn.classList.toggle("hidden", !ir);

  // Voice brain
  const vb = ss.voice_brain;
  const vbr = vb?.running === true;
  ["voice-brain-dot", "r-voice-brain-dot"].forEach(id => {
    const d = $(id);
    if (d) { d.classList.toggle("online", vbr); d.classList.toggle("offline", !vbr); }
  });
  if ($("voice-brain-text")) $("voice-brain-text").textContent = vbr ? "运行中" : "未运行";
  if ($("r-voice-brain-text")) $("r-voice-brain-text").textContent = `语音: ${vbr ? "运行中" : "未运行"}`;

  // Auto-load maps when inspection service just came online
  if (ir && !_prevInspRunning) {
    setTimeout(async () => {
      try { await loadMaps(); } catch {}
    }, 2000);
  }
  _prevInspRunning = ir;
}

function updateRobotStatusBox() {
  const rs = state.robotStatus;
  if (!rs || !rs.mode_name) { el.robotStatusBox.textContent = "等待状态..."; return; }
  const v = rs.velocity || {};
  el.robotStatusBox.textContent =
    `模式: ${rs.mode_name} (${rs.control_mode},${rs.mode_type})\n` +
    `步态: ${rs.gait_name || "?"} (${rs.gait})\n` +
    `速度: vx=${(v.vx||0).toFixed(3)} vy=${(v.vy||0).toFixed(3)} wz=${(v.wz||0).toFixed(3)}\n` +
    `足触: ${rs.foot_contact ?? "?"}`;
}

function updateProgressBar(elId, task) {
  const bar = $(elId);
  if (!bar) return;
  if (!task || (!task.running && task.progress !== 100)) { bar.classList.add("hidden"); return; }
  bar.classList.remove("hidden");
  const fill = bar.querySelector(".progress-fill");
  const text = bar.querySelector(".progress-text");
  if (fill) fill.style.width = `${task.progress || 0}%`;
  if (text) text.textContent = task.step || "";
}

/* ── Virtual Joystick ─────────────────────────────────────────────────────── */
class VirtualJoystick {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.cx = canvas.width / 2;
    this.cy = canvas.height / 2;
    this.maxR = canvas.width / 2 - 16;
    this.thumbR = 18;
    this.tx = this.cx; this.ty = this.cy;
    this.active = false;
    this.nx = 0; this.ny = 0;
    canvas.addEventListener("pointerdown", e => this._down(e));
    window.addEventListener("pointermove", e => this._move(e));
    window.addEventListener("pointerup", () => this._up());
    this.draw();
  }
  _down(e) { this.active = true; this._update(e); }
  _move(e) { if (this.active) this._update(e); }
  _up() { this.active = false; this.tx = this.cx; this.ty = this.cy; this.nx = 0; this.ny = 0; this.draw(); }
  _update(e) {
    const rect = this.canvas.getBoundingClientRect();
    let dx = (e.clientX - rect.left) * (this.canvas.width / rect.width) - this.cx;
    let dy = (e.clientY - rect.top) * (this.canvas.height / rect.height) - this.cy;
    const d = Math.hypot(dx, dy);
    if (d > this.maxR) { dx *= this.maxR / d; dy *= this.maxR / d; }
    this.tx = this.cx + dx; this.ty = this.cy + dy;
    this.nx = dx / this.maxR; this.ny = dy / this.maxR;
    this.draw();
  }
  draw() {
    const c = this.ctx, w = this.canvas.width, h = this.canvas.height;
    c.clearRect(0, 0, w, h);
    c.beginPath(); c.arc(this.cx, this.cy, this.maxR, 0, Math.PI * 2);
    c.fillStyle = "rgba(0,0,0,0.04)"; c.fill();
    c.strokeStyle = "rgba(0,0,0,0.12)"; c.lineWidth = 2; c.stroke();
    c.beginPath();
    c.moveTo(this.cx, this.cy - this.maxR + 4); c.lineTo(this.cx, this.cy + this.maxR - 4);
    c.moveTo(this.cx - this.maxR + 4, this.cy); c.lineTo(this.cx + this.maxR - 4, this.cy);
    c.strokeStyle = "rgba(0,0,0,0.06)"; c.lineWidth = 1; c.stroke();
    c.beginPath(); c.arc(this.tx, this.ty, this.thumbR, 0, Math.PI * 2);
    c.fillStyle = this.active ? "#2563eb" : "#94a3b8"; c.fill();
    c.strokeStyle = "#fff"; c.lineWidth = 2; c.stroke();
  }
  getVelocity(maxLin, maxLat) { return { vx: -this.ny * maxLin, vy: -this.nx * maxLat }; }
}
let joystick = null;

/* ── Keyboard teleop ──────────────────────────────────────────────────────── */
const keyState = {};
let turnLeftActive = false, turnRightActive = false;

/* ── Velocity loop ────────────────────────────────────────────────────────── */
let velTimer = null;
function startVelLoop() {
  if (velTimer) return;
  velTimer = setInterval(() => {
    if (state.currentMode !== "robot") return;
    let vx = 0, vy = 0, wz = 0;
    if (joystick?.active) { const v = joystick.getVelocity(state.maxLinVel, state.maxLatVel); vx = v.vx; vy = v.vy; }
    if (keyState.w) vx = state.maxLinVel;
    if (keyState.s) vx = -state.maxLinVel;
    if (keyState.a) vy = state.maxLatVel;
    if (keyState.d) vy = -state.maxLatVel;
    if (keyState.q) wz = state.maxAngVel;
    if (keyState.e) wz = -state.maxAngVel;
    if (turnLeftActive) wz = state.maxAngVel;
    if (turnRightActive) wz = -state.maxAngVel;
    const nonZero = vx !== 0 || vy !== 0 || wz !== 0;
    if (nonZero || state.lastSentNonZero) {
      requestPost("/api/v1/robot/velocity", { vx, vy, wz }).catch(() => {});
      state.lastSentNonZero = nonZero;
    }
  }, 100);
}

/* ── Control bindings ─────────────────────────────────────────────────────── */
function bindControls() {
  el.modeTabs.forEach(t => t.addEventListener("click", () => switchMode(t.dataset.mode)));

  // Services
  $("btn-quick-slam")?.addEventListener("click", async () => {
    try {
      const nav2 = unwrap(await requestPost("/api/v1/services/nav2/start", { mode: "slam" }));
      const status = nav2StartStatus("slam", nav2?.already_running === true);
      setStatus(status.message, status.tone);
      switchMode("slam");
      await requestPost("/api/v1/robot/prepare");
    }
    catch (e) { setStatus(`失败: ${e.message}`, "error"); }
  });
  $("btn-quick-nav")?.addEventListener("click", async () => {
    try {
      const nav2 = unwrap(await requestPost("/api/v1/services/nav2/start", { mode: "nav" }));
      const status = nav2StartStatus("nav", nav2?.already_running === true);
      setStatus(status.message, status.tone);
      switchMode("nav");
    }
    catch (e) { setStatus(`失败: ${e.message}`, "error"); }
  });
  $("btn-quick-inspect")?.addEventListener("click", async () => {
    try {
      const nav2 = unwrap(await requestPost("/api/v1/services/nav2/start", { mode: "nav" }));
      await requestPost("/api/v1/services/inspection/start");
      setStatus(
        nav2?.already_running === true ? "导航已在运行，巡检启动中..." : "导航+巡检启动中...",
        nav2?.already_running === true ? "info" : "success",
      );
      switchMode("inspect");
    }
    catch (e) { setStatus(`失败: ${e.message}`, "error"); }
  });
  $("btn-nav2-start")?.addEventListener("click", async () => {
    try {
      const mode = $("nav2-mode-select").value;
      const nav2 = unwrap(await requestPost("/api/v1/services/nav2/start", { mode }));
      const status = nav2StartStatus(mode, nav2?.already_running === true);
      setStatus(status.message, status.tone);
    }
    catch (e) { setStatus(`失败: ${e.message}`, "error"); }
  });
  $("btn-nav2-stop")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/nav2/stop"); setStatus("Nav2 已停止", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  $("btn-insp-svc-start")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/inspection/start"); setStatus("巡检服务启动中...", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  $("btn-insp-svc-stop")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/inspection/stop"); setStatus("巡检服务已停止", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  $("btn-inspect-quick-start")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/inspection/start"); setStatus("巡检服务启动中...", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  $("btn-inspect-quick-stop")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/inspection/stop"); setStatus("巡检服务已停止", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  el.btnRecordMeterPoint?.addEventListener("click", async () => {
    try {
      const name = (el.meterPointName?.value || "").trim();
      const meterType = el.meterPointType?.value || "pressure";
      const res = await requestPost("/api/v1/meter-points/record", {
        name,
        meter_type: meterType,
      });
      if (el.meterPointName) el.meterPointName.value = "";
      setStatus(res?.message || "点位已记录", "success");
      await fetchMeterPoints();
    } catch (e) {
      setStatus(`记录点位失败: ${e.message}`, "error");
    }
  });
  el.btnMeterRouteStart?.addEventListener("click", async () => {
    try {
      const route = state.meterRoute;
      const startIndex = route.currentIndex >= 0 && route.currentIndex < route.pointIds.length ? route.currentIndex : 0;
      await goToMeterRouteIndex(startIndex);
    } catch (e) {
      setStatus(`开始路线失败: ${e.message}`, "error");
    }
  });
  el.btnMeterRouteArrived?.addEventListener("click", () => {
    const point = getActiveMeterRoutePoint();
    if (!point) {
      setStatus("当前没有正在执行的路线点位", "error");
      return;
    }
    state.meterRoute.phase = "adjusting";
    renderMeterRoute();
    setStatus(`已到 ${point.name || point.point_id}。可切到“控制”页微调画面，确认后点“完成本点/下一点”`, "info");
  });
  el.btnMeterRouteNext?.addEventListener("click", async () => {
    const route = state.meterRoute;
    if (route.currentIndex < 0) {
      setStatus("当前没有活动路线点位", "error");
      return;
    }
    const finishedPoint = getActiveMeterRoutePoint();
    let recorded = null;
    try {
      recorded = await recordCurrentMeterReading(finishedPoint, route.currentIndex);
    } catch (e) {
      setStatus(`记录本点读数失败: ${e.message}`, "error");
      return;
    }
    const nextIndex = route.currentIndex + 1;
    if (nextIndex >= route.pointIds.length) {
      route.phase = "done";
      renderMeterRoute();
      setStatus(`路线完成：${finishedPoint?.name || "最后一个点"} 已记录 ${recorded?.reading_text || getLatestMeterReadingText() || "当前读数"}`, "success");
      return;
    }
    try {
      await goToMeterRouteIndex(nextIndex);
      setStatus(`已记录 ${finishedPoint?.name || "当前点"}：${recorded?.reading_text || getLatestMeterReadingText() || "当前读数"}，正在前往下一点`, "success");
    } catch (e) {
      setStatus(`切到下一点失败: ${e.message}`, "error");
    }
  });
  el.btnMeterRouteClear?.addEventListener("click", () => {
    clearMeterRoute();
    setStatus("表盘路线已清空", "success");
  });
  $("btn-voice-brain-start")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/voice-brain/start"); setStatus("语音控制启动中...", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  $("btn-voice-brain-stop")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/voice-brain/stop"); setStatus("语音控制已停止", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  $("btn-stop-all")?.addEventListener("click", async () => { try { await requestPost("/api/v1/services/stop-all"); setStatus("所有服务已停止", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });

  // Vision detector toggles
  $("btn-det-all")?.addEventListener("click", async () => {
    const rt = state.visionData?.runtime || {};
    try {
      const res = await requestPost("/api/v1/vision/detectors", { detectors: rt.available || [] });
      setStatus(res?.message || "检测器已全开", "success");
    }
    catch (e) { setStatus(`全选失败: ${e.message}`, "error"); }
  });
  $("btn-det-none")?.addEventListener("click", async () => {
    try {
      const res = await requestPost("/api/v1/vision/detectors", { detectors: [] });
      setStatus(res?.message || "检测器已全关", "success");
    }
    catch (e) { setStatus(`全关失败: ${e.message}`, "error"); }
  });

  // SLAM
  el.slamCostmapToggle?.addEventListener("change", async e => { state.showCostmap = e.target.checked; await updateCostmap(); });
  $("btn-save-map")?.addEventListener("click", async () => {
    const filename = $("save-map-path")?.value || "cyberdog_map";
    const exportPgm = $("save-export-pgm")?.checked ?? true;
    try { await requestPost("/api/v1/mapping/save", { filename, export_pgm: exportPgm }); setStatus("地图保存中...", "info"); }
    catch (e) { setStatus(`失败: ${e.message}`, "error"); }
  });

  // Nav
  el.navGoalBtn?.addEventListener("click", () => { setInteractionMode("nav"); setStatus("拖拽设置目标点和朝向", "info"); });
  el.navAmclBtn?.addEventListener("click", () => { setInteractionMode("amcl"); setStatus("拖拽设置初始位姿", "info"); });
  el.navCancelBtn?.addEventListener("click", () => setInteractionMode("idle"));
  el.navCostmapToggle?.addEventListener("change", async e => { state.showCostmap = e.target.checked; await updateCostmap(); });

  // Inspect
  el.sceneSelect?.addEventListener("change", async e => { try { await loadScene(e.target.value); } catch (err) { setLoading(false); setStatus(`切换失败: ${err.message}`, "error"); } });
  el.refreshMapsBtn?.addEventListener("click", async () => { try { await loadMaps(); if (state.selectedScene) await loadScene(state.selectedScene); } catch (err) { setLoading(false); setStatus(`刷新失败: ${err.message}`, "error"); } });
  el.initializeBtn?.addEventListener("click", async () => {
    if (!state.selectedScene) { setStatus("请先选择地图", "error"); return; }
    try { await requestPost("/api/v1/inspection/initialize", { scene_name: state.selectedScene }); setStatus("初始化已发送", "success"); await refreshInspectionStatus(); }
    catch (err) { setStatus(`失败: ${err.message}`, "error"); }
  });
  el.startBtn?.addEventListener("click", async () => { try { await requestPost("/api/v1/inspection/start"); setStatus("开始巡检", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  el.pauseBtn?.addEventListener("click", async () => { try { await requestPost("/api/v1/inspection/pause"); setStatus("已暂停", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  el.resumeBtn?.addEventListener("click", async () => { try { await requestPost("/api/v1/inspection/resume"); setStatus("已恢复", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  el.stopBtn?.addEventListener("click", async () => { try { await requestPost("/api/v1/inspection/stop"); setStatus("已停止", "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  el.costmapToggle?.addEventListener("change", async e => { state.showCostmap = e.target.checked; await updateCostmap(); });

  // Robot
  $("btn-prepare")?.addEventListener("click", async () => { try { await requestPost("/api/v1/robot/prepare"); setStatus("预备序列启动", "info"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } });
  $("btn-estop")?.addEventListener("click", async () => { try { await requestPost("/api/v1/robot/stop"); setStatus("紧急停止已发送", "error"); } catch (e) { setStatus(`停止失败: ${e.message}`, "error"); } });
  document.querySelectorAll("[data-mode-preset]").forEach(btn => btn.addEventListener("click", async () => { try { await requestPost("/api/v1/robot/mode", { preset: btn.dataset.modePreset }); setStatus(`模式 → ${btn.textContent}`, "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } }));
  document.querySelectorAll("[data-order-preset]").forEach(btn => btn.addEventListener("click", async () => { try { await requestPost("/api/v1/robot/order", { preset: btn.dataset.orderPreset }); setStatus(`动作 → ${btn.textContent.trim()}`, "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } }));
  document.querySelectorAll("[data-gait]").forEach(btn => btn.addEventListener("click", async () => { try { await requestPost("/api/v1/robot/gait", { gait: parseInt(btn.dataset.gait) }); setStatus(`步态 → ${btn.textContent}`, "success"); } catch (e) { setStatus(`失败: ${e.message}`, "error"); } }));

  const tl = $("btn-turn-left"), tr = $("btn-turn-right");
  if (tl) { tl.addEventListener("pointerdown", () => { turnLeftActive = true; }); tl.addEventListener("pointerup", () => { turnLeftActive = false; }); tl.addEventListener("pointerleave", () => { turnLeftActive = false; }); }
  if (tr) { tr.addEventListener("pointerdown", () => { turnRightActive = true; }); tr.addEventListener("pointerup", () => { turnRightActive = false; }); tr.addEventListener("pointerleave", () => { turnRightActive = false; }); }

  $("btn-spd-down")?.addEventListener("click", () => { state.maxLinVel = Math.max(0.05, state.maxLinVel - 0.02); state.maxLatVel = Math.max(0.03, state.maxLatVel - 0.02); state.maxAngVel = Math.max(0.15, state.maxAngVel - 0.05); $("spd-display").textContent = state.maxLinVel.toFixed(2); });
  $("btn-spd-up")?.addEventListener("click", () => { state.maxLinVel = Math.min(0.60, state.maxLinVel + 0.02); state.maxLatVel = Math.min(0.40, state.maxLatVel + 0.02); state.maxAngVel = Math.min(2.00, state.maxAngVel + 0.05); $("spd-display").textContent = state.maxLinVel.toFixed(2); });

  $("camera-toggle")?.addEventListener("change", async e => { try { await requestPost("/api/v1/robot/camera", { enable: e.target.checked }); setStatus(e.target.checked ? "相机已启用" : "相机已禁用", "success"); } catch (err) { setStatus(`相机操作失败: ${err.message}`, "error"); } });

  document.addEventListener("keydown", e => {
    if (state.currentMode !== "robot") return;
    const k = e.key.toLowerCase();
    if (["w", "a", "s", "d", "q", "e"].includes(k)) { keyState[k] = true; e.preventDefault(); }
    if (k === " " || k === "x") { requestPost("/api/v1/robot/stop").catch(() => {}); e.preventDefault(); }
  });
  document.addEventListener("keyup", e => { const k = e.key.toLowerCase(); if (k in keyState) keyState[k] = false; });
}

/* ── Map interaction (pan / zoom / nav / amcl) ────────────────────────────── */
function bindMapInteraction() {
  el.canvas.addEventListener("contextmenu", e => e.preventDefault());

  // Wheel → zoom at mouse
  el.canvas.addEventListener("wheel", e => {
    e.preventDefault();
    if (state.viewScale <= 0) return;
    const cp = eventToCanvas(e);
    const factor = e.deltaY > 0 ? 0.88 : 1.14;
    zoomAtPoint(cp.x, cp.y, factor);
  }, { passive: false });

  el.canvas.addEventListener("pointerdown", e => {
    const btn = e.button;

    // Right or middle button → pan always
    if (btn === 1 || btn === 2) {
      startPan(e);
      return;
    }

    // Left button in idle or non-interactive modes → pan
    const isInteractive = (state.interactionMode === "nav" || state.interactionMode === "amcl")
      && !["slam", "services", "robot"].includes(state.currentMode);

    if (!isInteractive) {
      startPan(e);
      return;
    }

    // Left button in nav/amcl → start drag for goal/pose
    const wp = eventToWorld(e);
    if (!wp) return;
    state.dragStart = wp;
    state.dragCurrent = wp;
    renderCanvas();
  });

  window.addEventListener("pointermove", e => {
    if (state.panning) { updatePan(e); return; }
    if (!state.dragStart) return;
    if (state.interactionMode !== "amcl" && state.interactionMode !== "nav") return;
    const wp = eventToWorld(e);
    if (!wp) return;
    state.dragCurrent = wp;
    renderCanvas();
  });

  window.addEventListener("pointerup", async () => {
    if (state.panning) { stopPan(); return; }
    if (!state.dragStart || !state.dragCurrent) return;

    try {
      if (state.interactionMode === "amcl") await sendInitialPose();
      else if (state.interactionMode === "nav") await sendNavGoal();
    } catch (err) {
      setStatus(`失败: ${err.message}`, "error");
    }
      state.dragStart = null;
      state.dragCurrent = null;
      renderCanvas();
  });
}

function startPan(e) {
  state.panning = true;
  state.panStartX = e.clientX;
  state.panStartY = e.clientY;
  state.panStartOffX = state.viewOffsetX;
  state.panStartOffY = state.viewOffsetY;
}
function updatePan(e) {
  state.viewOffsetX = state.panStartOffX + (e.clientX - state.panStartX);
  state.viewOffsetY = state.panStartOffY + (e.clientY - state.panStartY);
  renderCanvas();
}
function stopPan() { state.panning = false; }

/* ── WebSocket ────────────────────────────────────────────────────────────── */
function applyLive(d) {
  state.robotPose = normPose(d.robot_pose) || state.robotPose;
  state.globalPath = normPath(d.global_path);
  state.localPath = normPath(d.local_path);
  state.plannedPath = normPath(d.planned_path);
  state.inspectionStatus = d.inspection_status || {};
  if (d.live_map_version != null) state.liveMapVersion = d.live_map_version;
  if (d.robot_status) state.robotStatus = d.robot_status;
  if (d.service_status) state.serviceStatus = d.service_status;
  if (d.tasks) state.tasks = d.tasks;
  if (d.vision_data) state.visionData = d.vision_data;

  if (state.robotPose && state.interactionMode === "idle" && !state.dragStart) updatePosePreview(state.robotPose);
  updateServiceIndicators();
  updateRobotStatusBox();
  applySensorData(d.sensor_data);
  updateVisionPanel();
  renderMeterRoute();
  updateProgressBar("prepare-progress", state.tasks?.prepare);
  updateProgressBar("save-map-progress", state.tasks?.save_map);
  renderCanvas();
  updateDebug();
}

function connectWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/live`);
  ws.addEventListener("open", () => { updateConnectionState(true); setStatus("实时连接成功", "success"); });
  ws.addEventListener("message", e => { try { applyLive(JSON.parse(e.data)); } catch {} });
  ws.addEventListener("close", () => {
    updateConnectionState(false); setStatus("连接断开，重连中...", "error");
    clearTimeout(state.reconnectTimer);
    state.reconnectTimer = setTimeout(connectWS, 1500);
  });
  ws.addEventListener("error", () => updateConnectionState(false));
}

/* ── Vision panel ─────────────────────────────────────────────── */
function updateVisionPanel() {
  const vd = state.visionData;
  const st = vd.status || {};
  const rt = vd.runtime || {};
  const det = vd.detections || {};

  // Connection dot — show green if we have any vision data
  const hasData = Object.keys(st).length > 0 || Object.keys(rt).length > 0 || Object.keys(det).length > 0;
  el.visionDot.classList.toggle("online", hasData);
  el.visionDot.classList.toggle("offline", !hasData);
  el.visionText.textContent = hasData ? "已连接" : "未连接";

  // FPS / latency
  el.visionFps.textContent = st.fps != null ? st.fps.toFixed(1) : "--";
  el.visionLatency.textContent = st.latency_ms != null ? `${st.latency_ms.toFixed(0)}ms` : "--";

  // Runtime sources
  if (rt.desired && rt.running) {
    const desiredText = rt.desired.length > 0 ? rt.desired.join(", ") : "无";
    const runningText = rt.running.length > 0 ? rt.running.join(", ") : "无";
    const unavailableText = (rt.unavailable || []).length > 0 ? rt.unavailable.join(", ") : "无";
    el.visionEnabled.classList.remove("subtle-box");
    el.visionEnabled.textContent = `目标: ${desiredText}  |  运行: ${runningText}  |  不可用: ${unavailableText}`;
  } else if (st.enabled && st.available) {
    el.visionEnabled.classList.remove("subtle-box");
    el.visionEnabled.textContent = `聚合启用: ${st.enabled.join(", ")}  |  聚合可见: ${st.available.join(", ")}`;
  } else {
    el.visionEnabled.textContent = "等待数据...";
  }

  // Detector toggles
  const togglesEl = document.getElementById("vision-detector-toggles");
  const runtimeDetectors = rt.detectors || {};
  const runtimeOrder = Object.keys(runtimeDetectors);
  if (togglesEl && runtimeOrder.length > 0) {
    const desired = new Set(rt.desired || []);
    const running = new Set(rt.running || []);
    const unavailable = new Set(rt.unavailable || []);
    togglesEl.innerHTML = runtimeOrder.map(d => {
      const count = st.detectors?.[d]?.count ?? det.counts?.[d] ?? null;
      const badge = count != null ? ` <small style="opacity:0.6">(${count})</small>` : '';
      const info = runtimeDetectors[d] || {};
      const runtimeState = unavailable.has(d)
        ? "unavailable"
        : running.has(d)
          ? "running"
          : desired.has(d)
            ? "desired"
            : "stopped";
      const title = info.last_error ? ` title="${String(info.last_error).replace(/"/g, "&quot;")}"` : "";
      return `<button class="chip ${runtimeState === 'running' ? 'active' : ''}" data-runtime-state="${runtimeState}" data-detector="${d}"${title}>${d}${badge}</button>`;
    }).join("");
    togglesEl.querySelectorAll("[data-detector]").forEach(btn => {
      btn.addEventListener("click", async () => {
        const detectorName = btn.dataset.detector;
        const curDesired = new Set((state.visionData?.runtime?.desired || []).slice());
        const unavailableSet = new Set(state.visionData?.runtime?.unavailable || []);
        const willEnable = !curDesired.has(detectorName);
        if (willEnable && unavailableSet.has(detectorName)) {
          setStatus(`检测器 ${detectorName} 当前不可用`, "error");
          return;
        }
        if (willEnable) curDesired.add(detectorName);
        else curDesired.delete(detectorName);
        const orderedDesired = runtimeOrder.filter(name => curDesired.has(name));
        btn.disabled = true;
        try {
          const res = await requestPost("/api/v1/vision/detectors", { detectors: orderedDesired });
          setStatus(res?.message || `检测器 ${detectorName} 已${willEnable ? '启用' : '禁用'}`, "success");
        } catch (e) {
          setStatus(`切换失败: ${e.message}`, "error");
        } finally {
          btn.disabled = false;
        }
      });
    });
  } else if (togglesEl) {
    togglesEl.innerHTML = "";
  }

  // Detection counts
  if (det.counts && Object.keys(det.counts).length > 0) {
    const detectorStats = st.detectors || {};
    const entries = Object.keys(detectorStats).length > 0
      ? Object.entries(detectorStats).map(([detectorName, info]) => visionDetectorSummary(detectorName, info))
      : Object.entries(det.counts).map(([detectorName, n]) => `${detectorName}: ${n}`);
    const alarmCount = (det.detections || []).filter(d => d?.is_alarm).length;
    const prefix = alarmCount > 0 ? `报警 ${alarmCount}` : `总计 ${det.count || 0}`;
    el.visionCounts.textContent = `${prefix} — ${entries.join(" | ")}`;
    el.visionCounts.dataset.tone = alarmCount > 0 ? "error" : "neutral";
  } else {
    el.visionCounts.textContent = "无检测数据";
    el.visionCounts.dataset.tone = "neutral";
  }

  // Detection details
  if (det.detections && det.detections.length > 0) {
    el.visionDetections.textContent = det.detections.map((d, i) =>
      `#${i + 1} ${d.is_alarm ? '[ALARM] ' : ''}${d.detector || 'detector'}/${visionClassLabel(d.detector, d.class_id, d.description)} (${visionDetectionValueText(d)}) @ [${(d.roi?.x || 0).toFixed(0)}, ${(d.roi?.y || 0).toFixed(0)}] ${(d.roi?.width || 0).toFixed(0)}×${(d.roi?.height || 0).toFixed(0)}`
    ).join("\n");
  } else {
    el.visionDetections.textContent = "无检测结果";
  }

  // Debug
  el.visionDebug.textContent = JSON.stringify({
    status_keys: Object.keys(st),
    runtime_keys: Object.keys(rt),
    desired: rt.desired,
    running: rt.running,
    available: rt.available,
    unavailable: rt.unavailable,
    enabled: st.enabled,
    detectors_count: Object.keys(st.detectors || {}).length,
    detections_count: det.count || 0,
    has_annotated: !!vd.annotated?.image_base64,
    errors: st.errors || {},
  }, null, 2);
}

/* ── Alarm History ──────────────────────────────────────────────── */
async function fetchAlarmHistory() {
  try {
    const res = await fetch("/api/v1/vision/alarms?limit=20");
    const data = await res.json();
    state.alarmHistory = (data.data?.alarms || []);
    state.alarmLoaded = true;
    updateAlarmBadge();
    renderAlarmHistory();
  } catch {}
}

function updateAlarmBadge() {
  const badge = document.getElementById("alarm-badge");
  if (!badge) return;
  const count = state.alarmHistory.length;
  if (count > 0) {
    badge.textContent = count;
    badge.classList.remove("hidden");
  } else {
    badge.classList.add("hidden");
  }
}

function renderAlarmHistory() {
  const list = el.alarmHistoryList;
  if (!list) return;
  if (!state.alarmLoaded || state.alarmHistory.length === 0) {
    list.innerHTML = '<div class="status-box subtle-box" data-tone="neutral">暂无报警记录</div>';
    return;
  }
  list.innerHTML = state.alarmHistory.map(a => {
    const icon = a.class_id === "fire" ? "&#x1F525;" : a.class_id === "smoke" ? "&#x1F6AC;" : "&#x26A0;";
    const label = a.description || a.class_id;
    const timeStr = formatAlarmTime(a.timestamp);
    const score = a.score > 0 ? `${(a.score * 100).toFixed(0)}%` : "";
    return `<div class="alarm-card" data-alarm-id="${a.id}">
      <div class="alarm-card-icon">${icon}</div>
      <div class="alarm-card-body">
        <div class="alarm-card-title">${a.detector} / ${label}</div>
        <div class="alarm-card-meta">${timeStr} ${score ? "&middot; " + score : ""}</div>
      </div>
      <span class="alarm-card-arrow">&rsaquo;</span>
    </div>`;
  }).join("");

  list.querySelectorAll(".alarm-card").forEach(card => {
    card.addEventListener("click", () => openAlarmModal(card.dataset.alarmId));
  });
}

function formatAlarmTime(ts) {
  if (!ts || !ts.startsWith("alarm_")) return ts;
  const parts = ts.replace("alarm_", "").split("_");
  if (parts.length >= 2) return `${parts[0].slice(0,4)}-${parts[0].slice(4,6)}-${parts[0].slice(6,8)} ${parts[1].slice(0,2)}:${parts[1].slice(2,4)}:${parts[1].slice(4,6)}`;
  return ts;
}

function openAlarmModal(alarmId) {
  el.alarmModalTitle.textContent = "报警详情";
  el.alarmModalImg.src = `/api/v1/vision/alarms/${alarmId}`;
  const entry = state.alarmHistory.find(a => a.id === alarmId);
  if (entry) {
    const score = entry.score > 0 ? `${(entry.score * 100).toFixed(1)}%` : "";
    el.alarmModalInfo.innerHTML = `${entry.detector} / ${entry.description || entry.class_id} ${score ? "&middot; 置信度 " + score : ""} <br>${formatAlarmTime(entry.timestamp)}`;
  }
  el.alarmModal.classList.remove("hidden");
}

function closeAlarmModal() {
  el.alarmModal.classList.add("hidden");
  el.alarmModalImg.src = "";
}

async function fetchMeterPoints() {
  try {
    const res = await requestJson("/api/v1/meter-points");
    state.meterPoints = res.data?.points || [];
    renderMeterPoints();
  } catch (e) {
    setStatus(`加载表盘点位失败: ${e.message}`, "error");
  }
}

function renderMeterPoints() {
  const list = el.meterPointsList;
  if (!list) return;
  if (!state.meterPoints || state.meterPoints.length === 0) {
    list.innerHTML = '<div class="status-box subtle-box" data-tone="neutral">暂无表盘点位，先把狗开到表盘前再点“记录当前点位”</div>';
    renderMeterRoute();
    return;
  }
  list.innerHTML = state.meterPoints.map(p => {
    const pose = p.pose || {};
    return `<div class="alarm-card" data-point-id="${p.point_id}">
      <div class="alarm-card-icon">&#x1F4CD;</div>
      <div class="alarm-card-body">
        <div class="alarm-card-title">${p.name || p.point_id}</div>
        <div class="alarm-card-meta">${p.meter_type || 'pressure'} &middot; x=${Number(pose.x || 0).toFixed(2)}, y=${Number(pose.y || 0).toFixed(2)}, yaw=${Number(pose.yaw || 0).toFixed(2)}</div>
      </div>
      <button class="small-btn info" data-go-meter-point="${p.point_id}">前往</button>
      <button class="small-btn success" data-add-route-point="${p.point_id}">加入路线</button>
      <button class="small-btn danger" data-delete-point="${p.point_id}">删除</button>
    </div>`;
  }).join("");
  list.querySelectorAll("[data-go-meter-point]").forEach(btn => {
    btn.addEventListener("click", async (ev) => {
      ev.stopPropagation();
      const point = getMeterPointById(btn.dataset.goMeterPoint);
      if (!point) return;
      try {
        await navigateToMeterPoint(point);
        state.meterRoute.currentIndex = -1;
        state.meterRoute.phase = "idle";
        renderMeterRoute();
        setStatus(`已发送导航：${point.name || point.point_id}`, "success");
      } catch (e) {
        setStatus(`前往点位失败: ${e.message}`, "error");
      }
    });
  });
  list.querySelectorAll("[data-add-route-point]").forEach(btn => {
    btn.addEventListener("click", (ev) => {
      ev.stopPropagation();
      addMeterRoutePoint(btn.dataset.addRoutePoint);
      const point = getMeterPointById(btn.dataset.addRoutePoint);
      setStatus(`已加入路线：${point?.name || btn.dataset.addRoutePoint}`, "success");
    });
  });
  list.querySelectorAll("[data-delete-point]").forEach(btn => {
    btn.addEventListener("click", async (ev) => {
      ev.stopPropagation();
      const pointId = btn.dataset.deletePoint;
      try {
        removeMeterRoutePoint(pointId);
        await requestDelete(`/api/v1/meter-points/${encodeURIComponent(pointId)}`);
        setStatus(`已删除点位 ${pointId}`, "success");
        await fetchMeterPoints();
      } catch (e) {
        setStatus(`删除点位失败: ${e.message}`, "error");
      }
    });
  });
  renderMeterRoute();
}

el.alarmModalClose.addEventListener("click", closeAlarmModal);
el.alarmModalOverlay.addEventListener("click", closeAlarmModal);

/* ── Sensor (from WS) ────────────────────────────────────────────────────── */
function fmtVal(v, unit, decimals = 1) {
  if (v == null) return "--";
  return typeof v === "number" ? v.toFixed(decimals) + unit : String(v) + unit;
}

function applySensorData(sd) {
  if (!sd) {
    el.sensorSvcDot.classList.remove("online");
    el.sensorSvcDot.classList.add("offline");
    el.sensorSvcText.textContent = "等待数据...";
    return;
  }
  el.sensorSvcDot.classList.add("online");
  el.sensorSvcDot.classList.remove("offline");
  el.sensorSvcText.textContent = "数据接收中";
  const d = sd.data || sd;
  el.sensorTemp.textContent = fmtVal(d.temperature_c, "°C");
  el.sensorHumi.textContent = fmtVal(d.humidity_pct, "%");
  el.sensorLight.textContent = fmtVal(d.light_lux, " lux", 0);
  el.sensorSound.textContent = fmtVal(d.sound_level, "", 0);
  el.sensorIr.textContent = fmtVal(d.infrared_c, "°C");
  const ts = sd.timestamp;
  if (ts) {
    const t = new Date(ts);
    el.sensorTime.textContent = `更新: ${t.toLocaleTimeString()}`;
  }
  if (el.sensorStats) {
    el.sensorStats.textContent =
      `序号: ${sd.seq ?? "?"}\n` +
      `原始帧: ${sd.raw_frame ?? "--"}`;
  }
}

/* ── Camera window ─────────────────────────────────────────────────────────── */
let _cameraTimer = null;
const CAMERA_REFRESH_MS = 200;

function initCameraWindow() {
  const win = document.getElementById("camera-window");
  const titlebar = document.getElementById("camera-titlebar");
  const closeBtn = document.getElementById("camera-close");
  const img = document.getElementById("camera-img");
  const toggleBtn = document.getElementById("btn-camera-toggle");

  // restore position
  try {
    const pos = JSON.parse(localStorage.getItem("cameraWinPos"));
    if (pos) { win.style.left = pos.left + "px"; win.style.top = pos.top + "px"; win.style.right = "auto"; }
  } catch {}

  function open() {
    win.classList.remove("hidden");
    toggleBtn.innerHTML = '<i class="ri-camera-off-line"></i>关闭画面窗口';
    startCameraStream(img);
  }
  function close() {
    win.classList.add("hidden");
    toggleBtn.innerHTML = '<i class="ri-camera-line"></i>打开画面窗口';
    stopCameraStream();
  }

  toggleBtn.addEventListener("click", () => {
    if (win.classList.contains("hidden")) open(); else close();
  });
  closeBtn.addEventListener("click", close);

  // drag
  let dragging = false, dx = 0, dy = 0;
  titlebar.addEventListener("mousedown", (e) => {
    dragging = true;
    dx = e.clientX - win.offsetLeft;
    dy = e.clientY - win.offsetTop;
    document.body.style.userSelect = "none";
  });
  window.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    win.style.left = (e.clientX - dx) + "px";
    win.style.top = (e.clientY - dy) + "px";
    win.style.right = "auto";
  });
  window.addEventListener("mouseup", () => {
    if (!dragging) return;
    dragging = false;
    document.body.style.userSelect = "";
    localStorage.setItem("cameraWinPos", JSON.stringify({ left: win.offsetLeft, top: win.offsetTop }));
  });
}

function startCameraStream(img) {
  stopCameraStream();
  img.src = "/api/v1/vision/frame?t=" + Date.now();
  _cameraTimer = setInterval(() => {
    img.src = "/api/v1/vision/frame?t=" + Date.now();
  }, CAMERA_REFRESH_MS);
}

function stopCameraStream() {
  if (_cameraTimer) { clearInterval(_cameraTimer); _cameraTimer = null; }
}

/* ── Init ─────────────────────────────────────────────────────────────────── */
async function init() {
  bindControls();
  bindMapInteraction();
  switchMode("services");
  resizeCanvas();
  initCameraWindow();
  window.addEventListener("resize", resizeCanvas);
  setLoading(true, "暂无地图 — 请先启动建图或导航服务");

  if (el.joystickCanvas) joystick = new VirtualJoystick(el.joystickCanvas);
  startVelLoop();

  try { await requestJson("/api/v1/health"); } catch {}
  try { await loadMaps(); } catch {}
  try { await fetchMeterPoints(); } catch {}
  try { await fetchMeterReadingHistory(); } catch {}
  await fetchLiveMap();
  connectWS();
  setInterval(fetchLiveMap, 1000);
  setInterval(() => { if (state.showCostmap) updateCostmap(); }, 2000);
  fetchAlarmHistory();
  setInterval(fetchAlarmHistory, 30000);
}

init();
