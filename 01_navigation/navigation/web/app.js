(() => {
  'use strict';

  const canvas = document.querySelector('#map');
  const context = canvas.getContext('2d');
  const view = { map: null, zoom: 1, panX: 0, panY: 0 };
  const activeStatuses = new Set(['planning', 'starting', 'running', 'cancelling', 'resetting']);
  const labels = {
    idle: '就绪', planning: '规划路线', starting: '准备启动', running: '正在导航',
    cancelling: '正在停车', resetting: '正在重置', needs_pose: '待初始化',
    arrived: '已到达', cancelled: '已取消',
    unreachable: '目标不可达', busy: '仿真被占用', error: '运行异常'
  };
  const phaseLabels = { align: '对准', track: '行驶', final_align: '终点对准',
    docking: '直线停靠', 'acquiring station plane': '寻找工位平面', settling: '停稳', cancelling: '停稳' };
  let pickedStation = null;
  let pickedGoal = null;
  let picking = false;
  let dragging = null;
  let resizeObserver;
  let pollTimer = 0;
  let lastStatus = null;
  let latestState = null;
  let lastJobId = null;
  let receivedState = false;
  let stateConnected = false;
  let actionInFlight = false;
  let actionRevision = 0;
  let snapshotKey = null;
  let snapshotUrl = null;
  let lastEpisode = null;

  const $ = (selector) => document.querySelector(selector);
  const elements = {
    connection: $('#connection-dot'), connectionLabel: $('#connection-label'),
    statusIndicator: $('#status-indicator'), status: $('#run-status'), phase: $('#phase-status'),
    detail: $('#status-detail'), resolution: $('#map-resolution'), hint: $('#map-hint'), mapError: $('#map-error'),
    cursor: $('#cursor-coordinate'), target: $('#target-coordinate'), start: $('#start-run'), cancel: $('#cancel-run'),
    panelError: $('#panel-error'), reset: $('#reset-sim'), poseX: $('#pose-x'), poseY: $('#pose-y'),
    poseYaw: $('#pose-yaw'), simTime: $('#sim-time'), sequence: $('#sequence'), footer: $('#footer-world'),
    pick: $('#pick-goal'), scaleLabel: $('#scale-label'), scaleBar: $('#scale-bar')
  };

  function active(state = latestState) {
    return Boolean(state && activeStatuses.has(state.status));
  }

  function fmt(value, places = 2) {
    return Number.isFinite(value) ? value.toFixed(places) : '—';
  }

  function formatClock(seconds) {
    if (!Number.isFinite(seconds)) return '—';
    const minutes = Math.floor(seconds / 60);
    return `${String(minutes).padStart(2, '0')}:${(seconds % 60).toFixed(1).padStart(4, '0')}`;
  }

  async function request(path, options = {}) {
    const response = await fetch(path, { cache: 'no-store', ...options,
      headers: { Accept: 'application/json', ...(options.body ? { 'Content-Type': 'application/json' } : {}), ...options.headers } });
    let payload = {};
    try { payload = await response.json(); } catch (_) { /* The API may return an empty body. */ }
    if (!response.ok) throw Object.assign(new Error(payload.error || `请求失败 (${response.status})`), { status: response.status });
    return payload;
  }

  function validMap(data) {
    if (!Array.isArray(data.origin) || data.origin.length !== 2 || !Number.isFinite(data.resolution) ||
        data.resolution <= 0 || !Number.isInteger(data.width) || !Number.isInteger(data.height) ||
        data.width <= 0 || data.height <= 0 || typeof data.pixels !== 'string') {
      throw new Error('地图数据格式不正确');
    }
    const binary = atob(data.pixels);
    if (binary.length !== data.width * data.height) throw new Error('地图数据长度不匹配');
    const pixels = new Uint8ClampedArray(binary.length * 4);
    const colors = new Map([[0, [52, 70, 84, 255]], [205, [233, 237, 241, 255]], [254, [255, 255, 255, 255]]]);
    for (let i = 0; i < binary.length; i += 1) {
      const color = colors.get(binary.charCodeAt(i)) || colors.get(205);
      pixels.set(color, i * 4);
    }
    const raster = document.createElement('canvas');
    raster.width = data.width;
    raster.height = data.height;
    raster.getContext('2d').putImageData(new ImageData(pixels, data.width, data.height), 0, 0);
    return { ...data, raster };
  }

  function worldBounds() {
    const map = view.map;
    return { xmin: map.origin[0], ymin: map.origin[1],
      xmax: map.origin[0] + map.width * map.resolution,
      ymax: map.origin[1] + map.height * map.resolution };
  }

  function transform() {
    const rect = canvas.getBoundingClientRect();
    const bounds = worldBounds();
    const margin = { left: 53, right: 27, top: 35, bottom: 43 };
    const availableWidth = Math.max(10, rect.width - margin.left - margin.right);
    const availableHeight = Math.max(10, rect.height - margin.top - margin.bottom);
    const fit = Math.min(availableWidth / (bounds.xmax - bounds.xmin), availableHeight / (bounds.ymax - bounds.ymin));
    const scale = fit * view.zoom;
    const mapWidth = (bounds.xmax - bounds.xmin) * scale;
    const mapHeight = (bounds.ymax - bounds.ymin) * scale;
    const x = margin.left + (availableWidth - mapWidth) / 2 + view.panX;
    const y = margin.top + (availableHeight - mapHeight) / 2 + view.panY;
    return { x, y, scale, width: mapWidth, height: mapHeight, bounds, rect };
  }

  function project(x, y, t) {
    return [t.x + (x - t.bounds.xmin) * t.scale,
      t.y + (t.bounds.ymax - y) * t.scale];
  }

  function unproject(clientX, clientY) {
    const t = transform();
    const x = (clientX - t.rect.left - t.x) / t.scale + t.bounds.xmin;
    const y = t.bounds.ymax - (clientY - t.rect.top - t.y) / t.scale;
    return { x, y };
  }

  function fitView() { view.zoom = 1; view.panX = 0; view.panY = 0; render(); }

  function drawAxes(t) {
    const ctx = context;
    const rect = t.rect;
    const left = Math.max(37, t.x - 9);
    const right = Math.min(rect.width - 8, t.x + t.width + 8);
    const top = Math.max(7, t.y - 8);
    const bottom = Math.min(rect.height - 28, t.y + t.height + 7);
    ctx.save();
    ctx.font = '10px "DM Mono", "Cascadia Code", monospace';
    ctx.fillStyle = '#74828a';
    ctx.strokeStyle = '#d6ddda';
    ctx.lineWidth = 1;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const distance = 44 / t.scale;
    const magnitude = Math.pow(10, Math.floor(Math.log10(distance)));
    const options = [1, 2, 2.5, 5, 10].map((n) => n * magnitude);
    const spacing = options.find((value) => value * t.scale >= 44) || 10 * magnitude;
    const startX = Math.ceil(t.bounds.xmin / spacing) * spacing;
    for (let x = startX; x <= t.bounds.xmax + 1e-9; x += spacing) {
      const px = project(x, t.bounds.ymin, t)[0];
      if (px < left || px > right) continue;
      ctx.beginPath(); ctx.moveTo(px, bottom); ctx.lineTo(px, bottom + 4); ctx.stroke();
      ctx.fillText(Number(x.toFixed(4)).toString(), px, bottom + 7);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    const startY = Math.ceil(t.bounds.ymin / spacing) * spacing;
    for (let y = startY; y <= t.bounds.ymax + 1e-9; y += spacing) {
      const py = project(t.bounds.xmin, y, t)[1];
      if (py < top || py > bottom) continue;
      ctx.beginPath(); ctx.moveTo(left - 4, py); ctx.lineTo(left, py); ctx.stroke();
      ctx.fillText(Number(y.toFixed(4)).toString(), left - 8, py);
    }
    ctx.fillStyle = '#687880';
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom'; ctx.fillText('X  /  m', (left + right) / 2, rect.height - 5);
    ctx.save(); ctx.translate(12, (top + bottom) / 2); ctx.rotate(-Math.PI / 2);
    ctx.fillText('Y  /  m', 0, 0); ctx.restore(); ctx.restore();
    const barMeters = spacing;
    const barPixels = barMeters * t.scale;
    elements.scaleLabel.textContent = `${fmt(barMeters, barMeters < 0.1 ? 2 : 1)} m`;
    elements.scaleBar.style.width = `${barPixels}px`;
  }

  function path(points, color, width, dash, t) {
    if (!Array.isArray(points) || points.length < 2) return;
    context.save(); context.beginPath();
    points.forEach((point, index) => {
      if (!Array.isArray(point) || !Number.isFinite(point[0]) || !Number.isFinite(point[1])) return;
      const [x, y] = project(point[0], point[1], t);
      if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
    });
    context.strokeStyle = color; context.lineWidth = width; context.lineCap = 'round'; context.lineJoin = 'round';
    if (dash) context.setLineDash(dash); context.stroke(); context.restore();
  }

  function drawTarget(t) {
    const state = latestState;
    const target = pickedGoal || (state && state.goal);
    const station = target?.station;
    if (view.map.stations) {
      Object.entries(view.map.stations).forEach(([name, stationPose]) => {
        const [x, y] = project(stationPose.x, stationPose.y, t);
        context.save(); context.beginPath(); context.arc(x, y, 9, 0, Math.PI * 2);
        context.fillStyle = '#fbfbf8'; context.fill(); context.strokeStyle = '#cb8b53'; context.lineWidth = 1.4; context.stroke();
        context.fillStyle = '#a76534'; context.font = '11px "DM Mono", monospace'; context.textAlign = 'center'; context.textBaseline = 'middle';
        context.fillText(name, x, y + 0.4); context.restore();
      });
    }
    if (!target || !Number.isFinite(target.x) || !Number.isFinite(target.y)) return;
    const [x, y] = project(target.x, target.y, t);
    context.save(); context.strokeStyle = '#d47b37'; context.fillStyle = '#fff8ed'; context.lineWidth = 2;
    context.beginPath(); context.arc(x, y, 8, 0, Math.PI * 2); context.fill(); context.stroke();
    context.beginPath(); context.moveTo(x - 12, y); context.lineTo(x + 12, y); context.moveTo(x, y - 12); context.lineTo(x, y + 12); context.stroke();
    if (station && view.map.stations && view.map.stations[station]) {
      const yaw = view.map.stations[station].yaw; context.beginPath();
      context.moveTo(x, y); context.lineTo(x + Math.cos(yaw) * 21, y - Math.sin(yaw) * 21); context.stroke();
    }
    context.restore();
  }

  function render() {
    if (!view.map) return;
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.round(rect.width * dpr); const height = Math.round(rect.height * dpr);
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.clearRect(0, 0, rect.width, rect.height);
    const t = transform();
    context.fillStyle = '#f8f9f6'; context.fillRect(0, 0, rect.width, rect.height);
    context.fillStyle = '#e9edf1'; context.fillRect(t.x, t.y, t.width, t.height);
    context.imageSmoothingEnabled = false;
    context.drawImage(view.map.raster, t.x, t.y, t.width, t.height);
    context.save(); context.beginPath(); context.rect(t.x, t.y, t.width, t.height); context.clip();
    for (const box of (view.map.keepouts || [])) {
      if (!Array.isArray(box) || box.length !== 4) continue;
      const a = project(box[0], box[3], t); const b = project(box[2], box[1], t);
      context.fillStyle = '#d1783214'; context.strokeStyle = '#bf783e80'; context.lineWidth = 1;
      context.fillRect(a[0], a[1], b[0] - a[0], b[1] - a[1]); context.strokeRect(a[0], a[1], b[0] - a[0], b[1] - a[1]);
    }
    if (latestState && latestState.path) path(latestState.path, '#286aa8', 2.3, [], t);
    if (latestState && latestState.trajectory) path(latestState.trajectory, '#ad427b', 1.8, [5, 4], t);
    drawTarget(t);
    if (latestState && latestState.pose && Number.isFinite(latestState.pose.x)) {
      const pose = latestState.pose; const [x, y] = project(pose.x, pose.y, t);
      context.save(); context.translate(x, y); context.rotate(-pose.yaw);
      context.beginPath(); context.moveTo(13, 0); context.lineTo(-9, -8); context.lineTo(-6, 0); context.lineTo(-9, 8); context.closePath();
      context.fillStyle = '#a83c78'; context.strokeStyle = '#fff'; context.lineWidth = 1.4; context.fill(); context.stroke(); context.restore();
    }
    context.restore(); drawAxes(t);
  }

  function setGoal(goal, station = null) {
    pickedGoal = goal ? { x: goal.x, y: goal.y, station } : null;
    pickedStation = station;
    document.querySelectorAll('.station-button').forEach((button) => button.classList.toggle('selected', button.dataset.station === station));
    elements.target.textContent = pickedGoal ? `${fmt(pickedGoal.x, 3)}, ${fmt(pickedGoal.y, 3)} m` : '尚未选择';
    updateActions();
    render(); updateMapHint();
  }

  function updateMapHint() {
    if (picking) elements.hint.textContent = '点击地图设置位置 · Esc 退出';
    else if (active()) elements.hint.textContent = latestState.status === 'running' ? '显示估计位姿与行驶轨迹' : labels[latestState.status];
    else elements.hint.textContent = pickedGoal ? '点击地图可更换目标' : '点击空闲区域设置目标';
  }

  function renderState(state) {
    if (state.episode !== lastEpisode) {
      lastEpisode = state.episode;
      snapshotKey = null;
      if (snapshotUrl) URL.revokeObjectURL(snapshotUrl);
      snapshotUrl = null;
      $('#snapshot-card').hidden = true;
    }
    if (state.status === 'arrived' && state.id && snapshotKey !== `${state.episode}:${state.id}`) {
      const key = `${state.episode}:${state.id}`;
      snapshotKey = key;
      fetch('/api/snapshot', { cache: 'no-store' }).then((response) => {
        if (!response.ok) return null;
        return response.blob();
      }).then((blob) => {
        if (!blob || snapshotKey !== key) return;
        if (snapshotUrl) URL.revokeObjectURL(snapshotUrl);
        snapshotUrl = URL.createObjectURL(blob);
        $('#snapshot-image').src = snapshotUrl;
        $('#snapshot-link').href = snapshotUrl;
        $('#snapshot-card').hidden = false;
      }).catch(() => {});
    }
    const jobChanged = Boolean(state.id && state.id !== lastJobId);
    const enteringRun = active(state) && !active(latestState);
    const restoreInitialGoal = !receivedState && !pickedGoal;
    if (state.goal && (jobChanged || enteringRun || restoreInitialGoal)) {
      pickedGoal = { x: state.goal.x, y: state.goal.y, station: state.goal.station || null };
      pickedStation = pickedGoal.station;
    }
    lastJobId = state.id || null;
    receivedState = true;
    latestState = state;
    document.querySelectorAll('.station-button').forEach((button) => button.classList.toggle('selected', button.dataset.station === (pickedGoal && pickedGoal.station)));
    elements.target.textContent = pickedGoal ? `${fmt(pickedGoal.x, 3)}, ${fmt(pickedGoal.y, 3)} m` : '尚未选择';
    const status = labels[state.status] || state.status || '未知状态';
    elements.status.textContent = status;
    elements.statusIndicator.className = `status-indicator ${active(state) ? 'busy' : ['error', 'unreachable', 'busy'].includes(state.status) ? 'error' : ['arrived', 'cancelled'].includes(state.status) ? 'active' : ''}`;
    elements.phase.textContent = active(state) && state.phase ? (phaseLabels[state.phase] || state.phase) : '';
    const details = {
      idle: state.pose ? '选择目标，从当前位置继续。' : '选择地图位置或工位。',
      planning: '正在检查目标可达性。',
      starting: '正在核对当前位姿。',
      running: '正在跟踪路径。',
      cancelling: '正在确认底盘停稳。',
      resetting: '正在从出生位建立定位。',
      arrived: '已停稳，可继续选择下一目标。',
      cancelled: '已停稳，可继续选择下一目标。',
      unreachable: '请在已知空闲区域选择目标。',
      busy: '稍后可重试。',
      error: '请检查本机仿真连接。',
    };
    elements.detail.textContent = state.detail || details[state.status] || '';
    elements.poseX.textContent = state.pose ? fmt(state.pose.x) : '—';
    elements.poseY.textContent = state.pose ? fmt(state.pose.y) : '—';
    elements.poseYaw.textContent = state.pose ? fmt(state.pose.yaw * 180 / Math.PI, 1) : '—';
    elements.simTime.textContent = formatClock(state.sim_time);
    elements.sequence.textContent = Number.isFinite(state.seq) ? `SEQ ${state.seq}` : 'SEQ —';
    updateActions();
    clearError();
    if (lastStatus !== state.status && ['arrived', 'cancelled', 'unreachable', 'busy', 'error', 'needs_pose'].includes(state.status)) picking = false;
    lastStatus = state.status;
    updateMapHint(); render(); schedulePoll(active(state) ? 200 : 1000);
  }

  function showError(message) { elements.panelError.textContent = message; elements.panelError.hidden = false; }
  function clearError() { elements.panelError.textContent = ''; elements.panelError.hidden = true; }

  function schedulePoll(delay) { window.clearTimeout(pollTimer); pollTimer = window.setTimeout(poll, delay); }

  async function poll() {
    let failed = false;
    const revision = actionRevision;
    try {
      const state = await request('/api/state');
      if (revision !== actionRevision || actionInFlight) {
        schedulePoll(active() ? 200 : 1000);
        return;
      }
      stateConnected = true;
      elements.connection.className = 'connection-dot connected'; elements.connectionLabel.textContent = '控制台在线';
      renderState(state);
    } catch (error) {
      if (revision !== actionRevision || actionInFlight) {
        schedulePoll(active() ? 200 : 1000);
        return;
      }
      stateConnected = false;
      updateActions();
      failed = true; elements.connection.className = 'connection-dot error'; elements.connectionLabel.textContent = '连接中断';
      showError(`无法读取运行状态：${error.message}`);
    }
    if (failed) schedulePoll(active() ? 200 : 1000);
  }

  async function initialLoad() {
    try {
      view.map = validMap(await request('/api/map'));
      elements.resolution.textContent = `${fmt(view.map.resolution, 3)} m / 格`;
      elements.footer.textContent = `占据栅格 · ${view.map.width} × ${view.map.height}`;
      const template = $('#station-button-template').content.firstElementChild;
      const stations = $('.station-buttons');
      for (const name of Object.keys(view.map.stations || {}).sort()) {
        const button = template.cloneNode(true);
        button.id = `station-${name.toLowerCase()}`;
        button.dataset.station = name;
        button.querySelector('.station-id').textContent = name;
        button.querySelector('strong').textContent = `目标 ${name}`;
        button.addEventListener('click', () => setStation(name));
        stations.append(button);
      }
      render(); resizeObserver = new ResizeObserver(render); resizeObserver.observe(canvas.parentElement);
      await poll();
    } catch (error) {
      elements.connection.className = 'connection-dot error'; elements.connectionLabel.textContent = '地图不可用';
      elements.mapError.textContent = `无法载入占据栅格：${error.message}`; elements.mapError.hidden = false;
      updateActions();
      schedulePoll(1000);
    }
  }

  function canOperate() {
    return !active() && !actionInFlight && stateConnected && Boolean(view.map);
  }

  function canStart() {
    return canOperate() && Boolean(pickedGoal) && latestState?.status !== 'needs_pose';
  }

  function updateActions() {
    elements.start.disabled = !canStart();
    elements.reset.hidden = active();
    elements.reset.disabled = !canOperate();
    const canCancel = active() && !['cancelling', 'resetting'].includes(latestState.status);
    elements.cancel.hidden = !canCancel;
    elements.cancel.disabled = !canCancel || actionInFlight || !stateConnected;
  }

  async function submitAction(path, payload) {
    actionInFlight = true;
    actionRevision += 1;
    clearError();
    updateActions();
    try {
      renderState(await request(path, { method: 'POST', body: JSON.stringify(payload) }));
    } catch (error) {
      showError(error.message);
    } finally {
      actionInFlight = false;
      updateActions();
      schedulePoll(200);
    }
  }

  function startRun() {
    if (!canStart()) return;
    picking = false;
    elements.pick.classList.remove('picking');
    const goal = pickedStation ? { station: pickedStation } : { x: pickedGoal.x, y: pickedGoal.y };
    submitAction('/api/run', goal);
  }

  function cancelRun() {
    if (elements.cancel.disabled || !latestState?.id) return;
    submitAction('/api/cancel', { id: latestState.id });
  }

  function resetSim() {
    if (canOperate()) submitAction('/api/reset', {});
  }

  function setStation(name) {
    if (!view.map || !view.map.stations || !view.map.stations[name] || active() || actionInFlight) return;
    const station = view.map.stations[name]; setGoal(station, name); picking = false; elements.pick.classList.remove('picking');
  }

  function canvasPoint(event) {
    return unproject(event.clientX, event.clientY);
  }

  canvas.addEventListener('pointerdown', (event) => {
    if (!view.map || dragging || event.button < 0 || event.button > 2) return;
    dragging = {
      id: event.pointerId,
      x: event.clientX,
      y: event.clientY,
      panX: view.panX,
      panY: view.panY,
      panOnly: event.button !== 0 || event.shiftKey,
      moved: false,
    };
    canvas.setPointerCapture(event.pointerId);
    if (event.button !== 0) event.preventDefault();
  });

  canvas.addEventListener('pointermove', (event) => {
    if (!view.map) return;
    if (dragging?.id === event.pointerId) {
      const dx = event.clientX - dragging.x;
      const dy = event.clientY - dragging.y;
      if (dragging.moved || Math.hypot(dx, dy) >= 5) {
        dragging.moved = true;
        canvas.classList.add('dragging');
        view.panX = dragging.panX + dx;
        view.panY = dragging.panY + dy;
        render();
      }
    }
    const point = canvasPoint(event);
    elements.cursor.textContent = '地图坐标  ' + fmt(point.x, 3) + ', ' + fmt(point.y, 3);
  });

  function finishPointer(event) {
    if (dragging?.id !== event.pointerId) return;
    const wasPan = dragging.moved || dragging.panOnly;
    dragging = null;
    canvas.classList.remove('dragging');
    if (wasPan || event.type !== 'pointerup' || active() || actionInFlight) return;

    const point = canvasPoint(event);
    const { bounds } = transform();
    if (point.x < bounds.xmin || point.x > bounds.xmax ||
        point.y < bounds.ymin || point.y > bounds.ymax) return;
    picking = false;
    elements.pick.classList.remove('picking');
    setGoal(point);
  }

  canvas.addEventListener('pointerup', finishPointer);
  canvas.addEventListener('pointercancel', finishPointer);
  canvas.addEventListener('lostpointercapture', finishPointer);
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());
  canvas.addEventListener('wheel', (event) => {
    if (!view.map) return; event.preventDefault(); zoomAt(Math.exp(-event.deltaY * 0.001), event.clientX, event.clientY);
  }, { passive: false });

  function zoomAt(factor, clientX, clientY) {
    const rect = canvas.getBoundingClientRect(); const before = unproject(clientX, clientY);
    const oldZoom = view.zoom; view.zoom = Math.min(8, Math.max(0.65, oldZoom * factor));
    const t = transform(); const after = project(before.x, before.y, t);
    view.panX += clientX - rect.left - after[0]; view.panY += clientY - rect.top - after[1]; render();
  }

  $('#zoom-in').addEventListener('click', () => { const r = canvas.getBoundingClientRect(); zoomAt(1.25, r.left + r.width / 2, r.top + r.height / 2); });
  $('#zoom-out').addEventListener('click', () => { const r = canvas.getBoundingClientRect(); zoomAt(0.8, r.left + r.width / 2, r.top + r.height / 2); });
  $('#fit-map').addEventListener('click', fitView);
  elements.pick.addEventListener('click', () => { if (active()) return; picking = !picking; elements.pick.classList.toggle('picking', picking); updateMapHint(); canvas.focus(); });
  elements.start.addEventListener('click', startRun);
  elements.cancel.addEventListener('click', cancelRun);
  elements.reset.addEventListener('click', resetSim);
  document.addEventListener('keydown', (event) => { if (event.key === 'Escape' && picking) { picking = false; elements.pick.classList.remove('picking'); updateMapHint(); } });
  initialLoad();
})();
