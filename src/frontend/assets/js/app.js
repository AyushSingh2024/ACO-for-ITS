/* ================================================================
   Driver HUD — Vanilla JavaScript
   Self-contained dashboard with map logic & API polling.
================================================================ */

(function () {
  'use strict';

  // ── Config ──
  const API_BASE = 'http://localhost:8000';
  const POLL_INTERVAL_MS = 800;   // how often we hit the API
  const INTERP_MS = 800;          // interpolation window = same as poll
  const COMM_RADIUS_M = 150;
  const MAX_STEPS_SHOWN = 6;

  // ── State ──
  const state = {
    connected: false,
    retryCount: 0,
    metaLoaded: false,
    minTime: 0, maxTime: 1000, currentTime: 0,
    isPlaying: false, speedMult: 1,
    lastFrameMs: 0,   // rAF delta-time for smooth playback clock

    driverId: null, driverVehicle: null,
    vehicles: [], instructions: [],

    // ── Smooth interpolation ──
    vehicleFrom: {},    // id -> {x, y}  start of current lerp
    vehicleTo:   {},    // id -> {x, y, speed, congestion, lane}  target
    interpStartMs: 0,   // Date.now() when this lerp began

    xMin: 0, xMax: 100, yMin: 0, yMax: 100,
    networkEdges: [], networkLoaded: false,
    edgesList: [], plannedRoute: null, plannedRouteShapes: [], driverRouteShapes: [],

    mapZoom: 1.0, mapOffsetX: 0, mapOffsetY: 0,
    mapDragging: false, mapDragStartX: 0, mapDragStartY: 0,
    mapDragOffsetStartX: 0, mapDragOffsetStartY: 0,
    autoCenter: true,

    // canvas resize cache
    _canvasW: 0, _canvasH: 0,
    // nav panel diff-cache
    _lastInstructionKey: '',
    _lastPrimary: null,
    
    // Ghost car for ACO visualization
    acoCar: { active: false, currentSeg: 0, progress: 0, x: 0, y: 0 },
  };

  let pollTimer = null;
  const dom = {};

  // ── Easing ──
  function easeInOut(t) {
    t = Math.max(0, Math.min(1, t));
    return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
  }

  function lerp(a, b, t) { return a + (b - a) * t; }

  const $ = (sel) => document.querySelector(sel);

  function cacheDom() {
    dom.statusDot = $('#status-dot');
    dom.vehicleIdBadge = $('#vehicle-id');
    dom.v2vStatusBadge = $('#v2v-status');
    dom.timeBadge = $('#time-badge');

    dom.navIcon = $('#nav-icon');
    dom.navAction = $('#nav-action');
    dom.navRoad = $('#nav-road');
    dom.metricDist = $('#metric-dist');
    dom.metricEta = $('#metric-eta');

    dom.speedValue = $('#speed-value');
    dom.congestionFill = $('#congestion-fill');
    dom.congestionLevel = $('#congestion-level');
    dom.laneInfo = $('#lane-info');

    dom.stepsList = $('#steps-list');
    dom.navMapCanvas = $('#nav-map');
    dom.mapInfoOverlay = $('#map-info-overlay');
    
    dom.mapZoomIn = $('#map-zoom-in');
    dom.mapZoomOut = $('#map-zoom-out');
    dom.mapCenter = $('#map-center');

    dom.originSelect = $('#origin-select');
    dom.destSelect = $('#dest-select');
    dom.planRouteBtn = $('#plan-route-btn');
    dom.randomTripBtn = $('#random-trip-btn');
    dom.routeResult = $('#route-result');

    dom.playBtn = $('#play-btn');
    dom.timeSlider = $('#time-slider');
    dom.timeDisplay = $('#time-display');
    dom.speedSelect = $('#speed-select');

    dom.reconnectOverlay = $('#reconnect-overlay');
    dom.reconnectMsg = $('#reconnect-msg');
    
    dom.toastContainer = $('#v2v-toast-container');
  }

  // ── Utils ──
  function getIconForAction(action) {
    if (!action) return '📍';
    const a = action.toLowerCase();
    if (a.includes('left')) return '↰';
    if (a.includes('right')) return '↱';
    if (a.includes('u-turn')) return '↩';
    if (a.includes('head') || a.includes('straight')) return '⇧';
    return '📍';
  }

  function dist(v1, v2) {
    const dx = v1.x - v2.x, dy = v1.y - v2.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  function truncate(str, len) {
    if (!str) return '';
    return str.length > len ? str.slice(0, len) + '…' : str;
  }

  // ── API ──
  async function apiGet(path) {
    const res = await fetch(API_BASE + path);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return res.json();
  }

  function setConnected(yes) {
    state.connected = yes;
    if (yes) {
      state.retryCount = 0;
      dom.reconnectOverlay.classList.remove('visible');
      dom.statusDot.classList.remove('offline');
      dom.v2vStatusBadge.textContent = 'Connected';
      dom.v2vStatusBadge.className = 'status-online';
    } else {
      state.retryCount++;
      dom.reconnectOverlay.classList.add('visible');
      dom.reconnectMsg.textContent = `Attempt ${state.retryCount}… Looking for server on port 8000`;
      dom.statusDot.classList.add('offline');
      dom.v2vStatusBadge.textContent = 'Offline';
      dom.v2vStatusBadge.className = 'status-offline';
    }
  }

  async function fetchMeta() {
    try {
      const data = await apiGet('/api/meta');
      state.minTime = data.time_min; state.maxTime = data.time_max;
      state.xMin = data.x_min; state.xMax = data.x_max;
      state.yMin = data.y_min; state.yMax = data.y_max;
      state.currentTime = data.time_min;
      state.metaLoaded = true;

      dom.timeSlider.min = data.time_min;
      dom.timeSlider.max = data.time_max;
      dom.timeSlider.value = data.time_min;

      setConnected(true);
      return true;
    } catch (e) {
      setConnected(false);
      return false;
    }
  }

  async function fetchNetworkGeometry() {
    if (state.networkLoaded) return;
    try {
      const data = await apiGet('/api/network-geometry');
      state.networkEdges = data.edges || [];
      state.networkLoaded = true;
    } catch (e) { console.warn(e.message); }
  }

  async function fetchEdgesList() {
    if (state.edgesList.length > 0) return;
    try {
      const data = await apiGet('/api/edges');
      state.edgesList = data.edges || [];
      const makeOptions = (select, placeholder) => {
        select.innerHTML = `<option value="">${placeholder}</option>`;
        for (const edge of state.edgesList) {
          const opt = document.createElement('option');
          opt.value = edge; opt.textContent = edge;
          select.appendChild(opt);
        }
      };
      makeOptions(dom.originSelect, '— Select Origin —');
      makeOptions(dom.destSelect, '— Select Destination —');
      
    } catch (e) { console.warn(e.message); }
  }

  async function fetchVehicles() {
    try {
      const data = await apiGet('/api/vehicles?timestep=' + Math.floor(state.currentTime));
      const incoming = data.vehicles || [];
      setConnected(true);

      // ── Snapshot current interpolated positions as the new "from" ──
      const now = Date.now();
      const alpha = easeInOut(Math.min(1, (now - state.interpStartMs) / INTERP_MS));

      const newFrom = {};
      const newTo   = {};
      for (const v of incoming) {
        const id = String(v.id);
        const from = state.vehicleFrom[id];
        const to   = state.vehicleTo[id];
        if (from && to) {
          // Capture where we are RIGHT NOW (mid-lerp) as the new start
          newFrom[id] = { x: lerp(from.x, to.x, alpha), y: lerp(from.y, to.y, alpha) };
        } else {
          // First time seeing this vehicle — start at target (no lerp needed)
          newFrom[id] = { x: v.x, y: v.y };
        }
        newTo[id] = v;
      }
      state.vehicleFrom   = newFrom;
      state.vehicleTo     = newTo;
      state.interpStartMs = now;

      // Rebuild flat vehicles list for V2V / UI (use current interpolated pos)
      state.vehicles = incoming;

      // Removed auto-select of the first vehicle (blue dot)
    } catch (e) { setConnected(false); }
  }

  async function fetchDriverAdvice() {
    if (!state.driverId || state.driverId === "ACO Agent") return;
    try {
      const data = await apiGet(`/api/driver-advice/${state.driverId}?timestep=${Math.floor(state.currentTime)}`);
      if (data.instructions) state.instructions = data.instructions;
      else state.instructions = [];
      
      state.driverRouteShapes = [];
      if (data.path && data.path.length > 0) {
        const edgeMap = {};
        for (const edge of state.networkEdges) edgeMap[edge.id] = edge.shape;
        for (const edgeId of data.path) {
          if (edgeMap[edgeId]) state.driverRouteShapes.push({ id: edgeId, shape: edgeMap[edgeId] });
        }
      }
    } catch (e) { console.warn('Fetch advice failed'); }
  }

  async function planRoute() {
    const origin = dom.originSelect.value, dest = dom.destSelect.value;
    if (!origin || !dest) return;
    
    dom.planRouteBtn.disabled = true;
    dom.planRouteBtn.textContent = '⏳ Planning...';
    try {
      const data = await apiGet(`/api/route-plan?origin=${encodeURIComponent(origin)}&destination=${encodeURIComponent(dest)}&timestep=${Math.floor(state.currentTime)}`);
      if (!data.error) {
        state.plannedRoute = data;
        state.plannedRouteShapes = [];
        const edgeMap = {};
        for (const edge of state.networkEdges) edgeMap[edge.id] = edge.shape;
        for (const edgeId of data.path) {
          if (edgeMap[edgeId]) state.plannedRouteShapes.push({ id: edgeId, shape: edgeMap[edgeId] });
        }
        dom.routeResult.innerHTML = `<div style="color:var(--success); font-size: 0.8rem; text-align:center; padding: 5px;">Route Acquired: ${data.total_distance_m}m ETA: ${data.total_eta_seconds}s</div>`;
        
        // Update the driver HUD with the ACO instructions
        state.instructions = data.instructions || [];
        state.driverId = "ACO Agent";
        state.driverVehicle = { speed: 18, congestion: "Low", lane: "ACO Optimized Path" };

        // Spawn the ACO Ghost Car at the start of the route
        if (state.plannedRouteShapes.length > 0 && state.plannedRouteShapes[0].shape.length > 0) {
            state.acoCar.active = true;
            state.acoCar.currentSeg = 0;
            state.acoCar.progress = 0;
            state.acoCar.edgeDetails = data.edge_details || [];
            state.acoCar.x = state.plannedRouteShapes[0].shape[0][0];
            state.acoCar.y = state.plannedRouteShapes[0].shape[0][1];
        }
      } else {
        // Clear previous route
        state.plannedRoute = null;
        state.plannedRouteShapes = [];
        dom.routeResult.innerHTML = `<div style="color:var(--danger); font-size: 0.8rem; text-align:center; padding: 5px;">Error: ${data.error}</div>`;
      }
    } catch (e) {
      state.plannedRoute = null;
      state.plannedRouteShapes = [];
      dom.routeResult.innerHTML = `<div style="color:var(--danger); font-size: 0.8rem; text-align:center; padding: 5px;">Failed to plan route</div>`;
    } finally {
      dom.planRouteBtn.disabled = false;
      dom.planRouteBtn.textContent = 'Start Navigation';
    }
  }
  async function randomTrip() {
    if (state.edgesList.length < 10 || state.networkEdges.length === 0) {
      showToast('Network not loaded yet', 'warn'); return;
    }

    try {
      dom.randomTripBtn.disabled = true;
      const data = await apiGet('/api/random-trip');
      dom.originSelect.value = data.origin;
      dom.destSelect.value = data.destination;
      showToast(`🎲 Random trip selected`, 'info');
      planRoute();
    } catch (e) {
      // Fallback if endpoint fails
      let origIdx = Math.floor(Math.random() * state.edgesList.length);
      let destIdx = Math.floor(Math.random() * state.edgesList.length);
      while (origIdx === destIdx) {
        destIdx = Math.floor(Math.random() * state.edgesList.length);
      }
      dom.originSelect.value = state.edgesList[origIdx];
      dom.destSelect.value = state.edgesList[destIdx];
      showToast(`🎲 Random trip selected (fallback)`, 'info');
      planRoute();
    } finally {
      dom.randomTripBtn.disabled = false;
    }
  }

  function showToast(msg, severity) {
    const el = document.createElement('div');
    el.className = `toast ${severity}`;
    el.innerHTML = `<strong>T=${state.currentTime.toFixed(1)}</strong>: ${msg}`;
    dom.toastContainer.appendChild(el);
    setTimeout(() => {
      el.style.animation = 'toastFade 0.3s ease-in forwards';
      setTimeout(() => el.remove(), 300);
    }, 4000);
  }

  function generateV2VAlerts() {
    if (!state.driverVehicle || state.vehicles.length < 2) return;
    const me = state.driverVehicle;
    const nearby = state.vehicles.filter(v => String(v.id) !== String(me.id) && dist(me, v) < COMM_RADIUS_M);
    
    let added = 0;
    for (const neighbor of nearby) {
      if (added >= 1) break;
      if (neighbor.congestion === 'High' && Math.random() < 0.2) {
        showToast(`⚠ Heavy traffic on ${truncate(neighbor.lane, 12)}`, 'danger');
        added++;
      } else if (neighbor.congestion === 'Medium' && Math.random() < 0.1) {
        showToast(`Moderate traffic on ${truncate(neighbor.lane, 12)}`, 'warn');
        added++;
      }
    }
  }

  // ── Renders ──
  // Helper: only set textContent if the value changed (avoids layout thrash)
  function setText(el, val) {
    if (el && el.textContent !== String(val)) el.textContent = val;
  }

  function renderUI() {
    setText(dom.vehicleIdBadge, state.driverId || '\u2014');

    // ── Primary nav instruction ──
    if (state.instructions.length > 0) {
      const p = state.instructions[0];
      const key = p.action + p.road + p.distance_m;
      if (key !== (state._lastPrimary || '')) {
        state._lastPrimary = key;
        setText(dom.navIcon,   getIconForAction(p.action));
        setText(dom.navAction, p.action);
        setText(dom.navRoad,   p.road);
        setText(dom.metricDist, p.distance_m + ' m');
        setText(dom.metricEta,  p.eta_seconds + ' s');
        // Briefly highlight primary card so user notices the change
        dom.navAction.classList.remove('nav-flash');
        void dom.navAction.offsetWidth;   // reflow trick to restart animation
        dom.navAction.classList.add('nav-flash');
      }
    } else {
      if (state._lastPrimary !== '') {
        state._lastPrimary = '';
        setText(dom.navAction, 'Waiting for Path');
        setText(dom.navRoad,   '...');
      }
    }

    // ── Speed / congestion gauge ──
    if (state.driverVehicle) {
      const kmh  = Math.round(state.driverVehicle.speed * 3.6);
      const cong = state.driverVehicle.congestion || 'Low';
      setText(dom.speedValue, kmh);
      if (dom.congestionLevel.textContent !== cong) {
        dom.congestionLevel.textContent = cong;
        dom.congestionLevel.className = 'fw-bold ' +
          (cong === 'High' ? 'text-danger' : cong === 'Medium' ? 'text-warning' : 'text-success');
        dom.congestionFill.style.width = (cong === 'High' ? 90 : cong === 'Medium' ? 55 : 25) + '%';
        dom.congestionFill.className   = 'congestion-fill ' + cong.toLowerCase();
      }
      setText(dom.laneInfo, 'Lane: ' + truncate(state.driverVehicle.lane, 20));
    }

    // ── Steps list ──
    // Build a cheap string key to detect if instructions changed
    const upcoming = state.instructions.slice(1, MAX_STEPS_SHOWN + 1);
    const instrKey = upcoming.map(i => i.action + i.road + i.distance_m).join('|');
    if (instrKey !== state._lastInstructionKey) {
      state._lastInstructionKey = instrKey;
      dom.stepsList.innerHTML = '';
      upcoming.forEach((inst, idx) => {
        const card = document.createElement('div');
        card.className = 'step-card';
        card.style.animationDelay = (idx * 40) + 'ms';
        card.innerHTML =
          `<span>${getIconForAction(inst.action)} ${inst.action} onto ${inst.road}</span>` +
          `<span style="color:var(--success)">${inst.distance_m}m</span>`;
        dom.stepsList.appendChild(card);
      });
    }
  }

  function renderMap() {
    const canvas = dom.navMapCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    // ── Resize only when needed (avoid thrash every rAF frame) ──
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr  = window.devicePixelRatio || 1;
    const cw   = Math.round(rect.width  * dpr);
    const ch   = Math.round(rect.height * dpr);
    if (canvas.width !== cw || canvas.height !== ch) {
      canvas.width  = cw;
      canvas.height = ch;
      canvas.style.width  = rect.width  + 'px';
      canvas.style.height = rect.height + 'px';
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const w = rect.width, h = rect.height;

    ctx.clearRect(0, 0, w, h);
    if (!state.metaLoaded || !state.networkLoaded) return;

    // ── Compute interpolation alpha (0→1) with easing ──
    const alpha = easeInOut(Math.min(1, (Date.now() - state.interpStartMs) / INTERP_MS));

    // ── Build smoothed position map for this frame ──
    const smoothed = {};   // id -> {x, y, data}
    for (const id in state.vehicleTo) {
      const from = state.vehicleFrom[id] || state.vehicleTo[id];
      const to   = state.vehicleTo[id];
      smoothed[id] = {
        x:         lerp(from.x, to.x, alpha),
        y:         lerp(from.y, to.y, alpha),
        speed:     to.speed,
        congestion: to.congestion,
        lane:      to.lane,
      };
    }

    // ── Map projection ──
    const netW = state.xMax - state.xMin, netH = state.yMax - state.yMin;
    const baseScale = Math.min(w / netW, h / netH);

    // Center on smoothed driver position for truly smooth auto-center
    let centerX, centerY;
    const driverId = state.driverId ? String(state.driverId) : null;
    const driverSmooth = driverId ? smoothed[driverId] : null;
    if (state.autoCenter && driverSmooth) {
      centerX = driverSmooth.x; centerY = driverSmooth.y;
    } else {
      centerX = (state.xMin + state.xMax) / 2 + state.mapOffsetX / (baseScale * state.mapZoom);
      centerY = (state.yMin + state.yMax) / 2 - state.mapOffsetY / (baseScale * state.mapZoom);
    }

    const zoom  = state.mapZoom;
    const scale = baseScale * zoom;
    const sx = (x) => w / 2 + (x - centerX) * scale;
    const sy = (y) => h / 2 - (y - centerY) * scale;

    // ── Grid ──
    ctx.strokeStyle = 'rgba(56,189,248,0.05)';
    ctx.lineWidth = 1;
    const gridSize = 100 * scale;
    const offX = sx(0) % gridSize;
    const offY = sy(0) % gridSize;
    ctx.beginPath();
    for (let x = offX; x < w; x += gridSize) { ctx.moveTo(x, 0); ctx.lineTo(x, h); }
    for (let y = offY; y < h; y += gridSize) { ctx.moveTo(0, y); ctx.lineTo(w, y); }
    ctx.stroke();

    // ── Road network ──
    ctx.lineWidth = Math.max(2, 2 * zoom);
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    for (const edge of state.networkEdges) {
      const shape = edge.shape;
      if (!shape || shape.length < 2) continue;
      const firstSx = sx(shape[0][0]), firstSy = sy(shape[0][1]);
      const lastSx  = sx(shape[shape.length - 1][0]), lastSy = sy(shape[shape.length - 1][1]);
      if (Math.max(firstSx, lastSx) < -50 || Math.min(firstSx, lastSx) > w + 50) continue;
      if (Math.max(firstSy, lastSy) < -50 || Math.min(firstSy, lastSy) > h + 50) continue;
      ctx.beginPath();
      ctx.moveTo(firstSx, firstSy);
      for (let i = 1; i < shape.length; i++) ctx.lineTo(sx(shape[i][0]), sy(shape[i][1]));
      ctx.strokeStyle = 'rgba(71, 85, 105, 0.55)';
      ctx.stroke();
    }

    // ── ACO Planned route ──
    if (state.plannedRouteShapes.length > 0) {
      ctx.lineWidth = Math.max(4, 5 * zoom);
      ctx.strokeStyle = 'rgba(52, 211, 153, 0.9)';
      for (const seg of state.plannedRouteShapes) {
        if (seg.shape.length < 2) continue;
        ctx.beginPath();
        ctx.moveTo(sx(seg.shape[0][0]), sy(seg.shape[0][1]));
        for (let i = 1; i < seg.shape.length; i++) ctx.lineTo(sx(seg.shape[i][0]), sy(seg.shape[i][1]));
        ctx.stroke();
      }
    }

    // ── Driver true route ──
    if (state.driverRouteShapes && state.driverRouteShapes.length > 0) {
      ctx.lineWidth = Math.max(3, 4 * zoom);
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.6)';
      for (const seg of state.driverRouteShapes) {
        if (seg.shape.length < 2) continue;
        ctx.beginPath();
        ctx.moveTo(sx(seg.shape[0][0]), sy(seg.shape[0][1]));
        for (let i = 1; i < seg.shape.length; i++) ctx.lineTo(sx(seg.shape[i][0]), sy(seg.shape[i][1]));
        ctx.stroke();
      }
    }

    // ── Vehicles (interpolated positions) ──
    const dotR = Math.max(2, 2.5 * zoom);
    for (const id in smoothed) {
      if (id === driverId) continue;
      const v  = smoothed[id];
      const vx = sx(v.x), vy = sy(v.y);
      if (vx < -10 || vx > w + 10 || vy < -10 || vy > h + 10) continue;
      ctx.beginPath();
      ctx.arc(vx, vy, dotR, 0, Math.PI * 2);
      ctx.fillStyle = v.congestion === 'High' ? '#f87171'
                    : v.congestion === 'Medium' ? '#fbbf24' : '#64748b';
      ctx.fill();
    }

    // ── Driver vehicle (interpolated + pulsing ring) ──
    if (driverSmooth) {
      const dx = sx(driverSmooth.x), dy = sy(driverSmooth.y);
      const pulse = 10 + 4 * Math.sin(Date.now() / 300);

      ctx.beginPath();
      ctx.arc(dx, dy, pulse, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(56, 189, 248, 0.15)';
      ctx.fill();

      ctx.beginPath();
      ctx.arc(dx, dy, 7, 0, Math.PI * 2);
      ctx.fillStyle = '#38bdf8';
      ctx.fill();
    }

    // ── ACO Ghost Car (Smart Vehicle) ──
    if (state.acoCar.active) {
      const ax = sx(state.acoCar.x), ay = sy(state.acoCar.y);
      const pulse = 12 + 4 * Math.sin(Date.now() / 150);

      ctx.beginPath();
      ctx.arc(ax, ay, pulse, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(16, 185, 129, 0.4)'; // glowing emerald
      ctx.fill();

      ctx.beginPath();
      ctx.arc(ax, ay, 6, 0, Math.PI * 2);
      ctx.fillStyle = '#10b981';
      ctx.fill();
      
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText('ACO', ax + 12, ay + 4);
    }

    dom.mapInfoOverlay.textContent = `${state.vehicles.length} vehicles | Zoom: ${zoom.toFixed(1)}x`;
  }

  // ── Start/Events ──
  async function poll() {
    if (!state.metaLoaded) { const ok = await fetchMeta(); if (!ok) return; }
    if (!state.networkLoaded) await fetchNetworkGeometry();
    if (state.edgesList.length === 0) await fetchEdgesList();
    
    await fetchVehicles();
    await fetchDriverAdvice();
    generateV2VAlerts();
    renderUI();
  }

  // Playback clock runs inside rAF so time advances at true 60fps
  function mapAnimLoop(nowMs) {
    if (state.isPlaying && state.metaLoaded) {
      const dt = state.lastFrameMs > 0 ? (nowMs - state.lastFrameMs) / 1000 : 0;
      state.currentTime += dt * state.speedMult;
      
      // ── Move the ACO Ghost Car ──
      if (state.acoCar.active && state.plannedRouteShapes.length > 0) {
          const route = state.plannedRouteShapes;
          let seg = state.acoCar.currentSeg;
          if (seg < route.length) {
              const shape = route[seg].shape;
              let pIdx = Math.floor(state.acoCar.progress);
              
              if (pIdx < shape.length - 1) {
                  const currentDetails = state.acoCar.edgeDetails && state.acoCar.edgeDetails[seg] ? state.acoCar.edgeDetails[seg] : null;
                  const speedKmh = currentDetails ? currentDetails.speed_kmh : 65;
                  const speedMps = speedKmh / 3.6;
                  const congestion = currentDetails ? currentDetails.congestion : "Low";
                  
                  if (state.driverId === "ACO Agent") {
                     state.driverVehicle = {
                         speed: speedMps,
                         congestion: congestion,
                         lane: currentDetails ? currentDetails.edge : "ACO Optimized Path"
                     };
                  }

                  const p1 = shape[pIdx];
                  const p2 = shape[pIdx + 1];
                  const pFrac = state.acoCar.progress - pIdx;
                  state.acoCar.x = lerp(p1[0], p2[0], pFrac);
                  state.acoCar.y = lerp(p1[1], p2[1], pFrac);
                  
                  const dx = p2[0] - p1[0], dy = p2[1] - p1[1];
                  const distP = Math.sqrt(dx*dx + dy*dy) || 1;
                  const fracPerSec = speedMps / distP;
                  
                  state.acoCar.progress += (dt * state.speedMult * fracPerSec);
              } else {
                  state.acoCar.currentSeg++;
                  state.acoCar.progress = 0;
              }
          } else {
              state.acoCar.active = false; // Reached destination!
          }
      }

      if (state.currentTime > state.maxTime) {
        state.currentTime = state.minTime;
        state.isPlaying = false;
        dom.playBtn.textContent = '▶ Play';
      }
      dom.timeDisplay.textContent = `T = ${state.currentTime.toFixed(1)}`;
      dom.timeSlider.value = state.currentTime;
    }
    state.lastFrameMs = state.isPlaying ? nowMs : 0;
    renderMap();
    requestAnimationFrame(mapAnimLoop);
  }

  function init() {
    cacheDom();

    if (dom.playBtn) dom.playBtn.addEventListener('click', () => {
      state.isPlaying = !state.isPlaying;
      state.lastFrameMs = 0;   // reset delta so first frame doesn't jump
      dom.playBtn.textContent = state.isPlaying ? '⏸ Pause' : '▶ Play';
    });

    if (dom.timeSlider) dom.timeSlider.addEventListener('input', e => state.currentTime = parseFloat(e.target.value));
    if (dom.speedSelect) dom.speedSelect.addEventListener('change', e => {
      state.speedMult = parseFloat(e.target.value);
    });

    if (dom.planRouteBtn) dom.planRouteBtn.addEventListener('click', planRoute);
    if (dom.randomTripBtn) dom.randomTripBtn.addEventListener('click', randomTrip);
    if (dom.mapZoomIn) dom.mapZoomIn.addEventListener('click', () => { state.mapZoom*=1.3; state.autoCenter=false; });
    if (dom.mapZoomOut) dom.mapZoomOut.addEventListener('click', () => { state.mapZoom*=0.75; state.autoCenter=false; });
    if (dom.mapCenter) dom.mapCenter.addEventListener('click', () => { state.autoCenter=true; state.mapZoom=3.0; state.mapOffsetX=0; state.mapOffsetY=0; });

    const cvs = dom.navMapCanvas;
    if (cvs) {
      cvs.addEventListener('wheel', e => { e.preventDefault(); state.mapZoom *= (e.deltaY>0?0.85:1.18); state.autoCenter=false; }, {passive:false});
      cvs.addEventListener('mousedown', e => { state.mapDragging=true; state.mapDragStartX=e.clientX; state.mapDragStartY=e.clientY; state.mapDragOffsetStartX=state.mapOffsetX; state.mapDragOffsetStartY=state.mapOffsetY; state.autoCenter=false; });
    }
    window.addEventListener('mousemove', e => { if(!state.mapDragging)return; state.mapOffsetX=state.mapDragOffsetStartX+(e.clientX-state.mapDragStartX); state.mapOffsetY=state.mapDragOffsetStartY+(e.clientY-state.mapDragStartY); });
    window.addEventListener('mouseup', () => state.mapDragging=false);

    window.addEventListener('resize', renderMap);
    
    mapAnimLoop();
    poll();
    setInterval(poll, POLL_INTERVAL_MS);
  }

  document.addEventListener('DOMContentLoaded', init);
})();
