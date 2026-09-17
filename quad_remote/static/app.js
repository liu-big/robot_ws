'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const baseLimits = { vx: .5, vy: .5, wz: 1 };
  const limits = { ...baseLimits };
  const ui = { state: null, dirty: true, ws: null, connected: false, canControl: false,
    ready: false, lastState: 0, rtt: null, pendingMode: null, pendingUntil: 0 };
  const details = $('details');
  const poseButtons = [...document.querySelectorAll('[data-pose]')];
  const speedSlider = $('speedSlider');
  const speedValue = $('speedValue');
  const portrait = matchMedia('(orientation: portrait) and (max-width: 700px)');
  let reconnectTimer = 0, backoff = 500, toastTimer = 0, lastTelemetryPaint = 0;
  let suspended = false;
  let speedScale = .8;
  let speedMin = .4;
  let speedMax = 1.5;
  let minBody = .4;
  let minYaw = .4;
  let lastDriveJson = '';
  let lastLayout = { o: '' };
  let wasDriving = false;
  let stateTick = 0;
  let lastControlAt = 0;

  try {
    const saved = Number(sessionStorage.getItem('quad_speed'));
    if (saved >= speedMin && saved <= speedMax) speedScale = saved;
  } catch { /* private mode */ }

  function applySpeedScale() {
    speedScale = Math.min(speedMax, Math.max(speedMin, speedScale));
    limits.vx = baseLimits.vx * speedScale;
    limits.vy = baseLimits.vy * speedScale;
    limits.wz = baseLimits.wz * speedScale;
    speedSlider.min = String(speedMin);
    speedSlider.max = String(speedMax);
    speedSlider.step = '0.05';
    speedSlider.value = String(speedScale);
    speedSlider.setAttribute('aria-valuemin', String(speedMin));
    speedSlider.setAttribute('aria-valuemax', String(speedMax));
    speedSlider.setAttribute('aria-valuenow', String(speedScale));
    text(speedValue, `${speedScale.toFixed(2)}×`);
    try { sessionStorage.setItem('quad_speed', String(speedScale)); } catch { /* ignore */ }
    ui.dirty = true;
  }

  function text(element, value) { if (element.textContent !== value) element.textContent = value; }
  function toast(message) {
    text($('toast'), message); $('toast').hidden = false;
    clearTimeout(toastTimer); toastTimer = setTimeout(() => { $('toast').hidden = true; }, 2600);
  }
  function canDrive() {
    return ui.connected && ui.canControl && !document.hidden &&
      !portrait.matches && !details.open && !suspended;
  }
  function canTouch() { return ui.connected && ui.canControl && !portrait.matches && !details.open; }

  function createStick(id, thumbId, yaw = false) {
    const element = $(id), thumb = $(thumbId);
    const stick = { element, thumb, pointer: null, x: 0, y: 0, radius: 1,
      cx: 0, cy: 0, dirty: true, paintedActive: false, yaw };

    const measure = () => {
      const bounds = element.getBoundingClientRect();
      stick.cx = bounds.left + bounds.width / 2;
      stick.cy = bounds.top + bounds.height / 2;
      stick.radius = Math.max(36, bounds.width / 2 - thumb.offsetWidth / 2 - 8);
    };

    const position = event => {
      let x = (event.clientX - stick.cx) / stick.radius;
      let y = yaw ? 0 : (event.clientY - stick.cy) / stick.radius;
      const length = Math.hypot(x, y);
      if (length > 1) { x /= length; y /= length; }
      stick.x = x; stick.y = y; stick.dirty = true;
    };

    const begin = event => {
      if (stick.pointer !== null || event.button !== 0 || !canTouch()) return;
      measure();
      stick.pointer = event.pointerId;
      try { element.setPointerCapture(event.pointerId); } catch { /* older browsers */ }
      position(event);
      event.preventDefault();
    };

    const move = event => {
      if (event.pointerId !== stick.pointer) return;
      position(event);
      event.preventDefault();
    };

    const release = event => {
      if (event.pointerId !== stick.pointer) return;
      if (event.type === 'pointerup' || event.type === 'pointercancel') resetStick(stick);
    };

    element.addEventListener('pointerdown', begin, { passive: false });
    element.addEventListener('pointermove', move, { passive: false });
    element.addEventListener('pointerup', release);
    element.addEventListener('pointercancel', release);
    element.addEventListener('contextmenu', event => event.preventDefault());
    return stick;
  }

  function resetStick(stick) {
    const pointer = stick.pointer;
    stick.pointer = null; stick.x = 0; stick.y = 0; stick.dirty = true;
    if (pointer !== null && stick.element.hasPointerCapture(pointer)) {
      try { stick.element.releasePointerCapture(pointer); } catch { /* ignore */ }
    }
  }

  const move = createStick('moveStick', 'moveThumb');
  const yaw = createStick('yawStick', 'yawThumb', true);
  const sticks = [move, yaw];

  function resetAll() { for (const stick of sticks) resetStick(stick); }

  function sendDrive(vx, vy, wz) {
    const ws = ui.ws;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const active = vx !== 0 || vy !== 0 || wz !== 0;
    const payload = `{"type":"drive","vx":${vx},"vy":${vy},"wz":${wz}}`;
    if (!active && payload === lastDriveJson) return;
    lastDriveJson = payload;
    ws.send(payload);
  }

  function sendNow(data) {
    const ws = ui.ws;
    if (!ws || ws.readyState !== WebSocket.OPEN) return false;
    ws.send(JSON.stringify(data));
    return true;
  }

  function stop() { resetAll(); lastDriveJson = ''; sendNow({ type: 'stop' }); }

  function shape(stick) {
    const length = Math.min(1, Math.hypot(stick.x, stick.y));
    if (length <= .08) return { x: 0, y: 0, frac: 0 };
    const t = (length - .08) / .92;
    const frac = .7 * t + .3 * t * t * t;
    const gain = frac / length;
    return { x: stick.x * gain, y: stick.y * gain, frac };
  }

  function mapMove(vx, vy, frac) {
    if (frac <= 0) return { vx: 0, vy: 0 };
    const maxMag = Math.hypot(limits.vx, limits.vy);
    const floor = Math.min(minBody * speedScale, maxMag * 0.5);
    const targetMag = floor + (maxMag - floor) * Math.min(1, frac);
    const mag = Math.hypot(vx, vy);
    if (mag <= 0) return { vx: 0, vy: 0 };
    const scale = targetMag / mag;
    return { vx: vx * scale, vy: vy * scale };
  }

  function mapYaw(wz, frac) {
    if (frac <= 0 || wz === 0) return 0;
    const floor = Math.min(minYaw * speedScale, limits.wz * 0.5);
    const target = floor + (limits.wz - floor) * Math.min(1, frac);
    return Math.sign(wz) * target;
  }

  const rounded = value => Math.round(value * 10000) / 10000;

  function buildDrive() {
    const left = shape(move);
    const right = shape(yaw);
    const cmd = mapMove(-left.y * limits.vx, -left.x * limits.vy, left.frac);
    const wz = mapYaw(-right.x * limits.wz, right.frac);
    return [rounded(cmd.vx), rounded(cmd.vy), rounded(wz)];
  }

  function controlLoop(now) {
    if (now - lastControlAt >= 20) {
      lastControlAt = now;
      const driving = canDrive();
      if (driving) {
        wasDriving = true;
        const [vx, vy, wz] = buildDrive();
        sendDrive(vx, vy, wz);
      } else if (wasDriving) {
        wasDriving = false;
        stop();
      }
    }
    requestAnimationFrame(controlLoop);
  }
  requestAnimationFrame(controlLoop);

  function paintStick(stick) {
    if (!stick.dirty) return;
    const px = stick.x * stick.radius;
    const py = stick.y * stick.radius;
    stick.thumb.style.transform = `translate3d(${px}px,${py}px,0)`;
    const active = stick.pointer !== null;
    if (active !== stick.paintedActive) {
      stick.element.dataset.active = String(active);
      stick.paintedActive = active;
    }
    stick.dirty = false;
  }

  const rows = new Map();
  function addRow(container, name, label) {
    const row = document.createElement('tr'); row.dataset.status = 'unavailable';
    const labelCell = document.createElement('td');
    const dot = document.createElement('i'); dot.className = 'row-dot';
    labelCell.append(dot, document.createTextNode(label)); row.append(labelCell);
    const cells = ['position', 'velocity', 'effort'].map(() => {
      const cell = document.createElement('td'); cell.textContent = '—'; row.append(cell); return cell;
    });
    container.append(row); rows.set(name, { row, cells });
  }
  for (let leg = 1; leg <= 4; leg++) {
    for (const axis of ['lift', 'crouch']) addRow($('jointRows'), `leg${leg}_${axis}`, `L${leg} ${axis === 'lift' ? 'Lift' : 'Crouch'}`);
  }
  ['FL', 'FR', 'RR', 'RL'].forEach((label, i) => addRow($('wheelRows'), `leg${i + 1}_wheel`, label));
  const statusNames = { receiving: '反馈更新中', stale: '反馈超时', disabled: '未配置', unavailable: '暂无反馈' };
  const format = (value, decimals = 2) => typeof value === 'number' && Number.isFinite(value) ? (Math.abs(value) < .5 * 10 ** -decimals ? 0 : value).toFixed(decimals) : '—';

  function paintTelemetry() {
    const state = ui.state;
    const fresh = ui.connected && performance.now() - ui.lastState < 5000;
    const linked = ui.ready;
    $('rosLink').classList.toggle('connected', ui.connected && linked);
    text($('rosText'), !ui.connected ? 'ROS 离线' : linked ? 'ROS 已连接' : 'ROS 等待');
    text($('ping'), ui.connected && ui.rtt !== null ? String(Math.round(ui.rtt)) : '—');
    const mode = fresh && state ? state.mode : 'unknown';
    text($('mode'), mode.toUpperCase()); text($('detailMode'), mode.toUpperCase());
    $('modeDot').dataset.known = String(mode !== 'unknown' && mode !== 'estop');
    const descriptions = { stand: '站立模式', neutral: '中立模式', crouch: '蹲伏模式', unknown: '等待模式反馈', estop: '已急停' };
    text($('modeDescription'), descriptions[mode] || '等待模式反馈');
    for (const axis of ['vx', 'vy', 'wz']) text($(axis), fresh && state ? format(state[axis]) : '—');
    if (ui.pendingMode && (ui.pendingMode === mode || performance.now() > ui.pendingUntil)) {
      ui.pendingMode = null;
    }
    for (const button of poseButtons) {
      button.disabled = !ui.connected || !ui.canControl;
      button.classList.toggle('active', button.dataset.pose === mode);
      button.classList.toggle('pending', button.dataset.pose === ui.pendingMode);
      button.setAttribute('aria-pressed', String(button.dataset.pose === mode));
    }
    const speedLabel = `${speedScale.toFixed(2)}× · 起步 ${minBody}`;
    let session = !ui.connected ? '连接中 · 自动重连' : !ui.canControl ? '只读状态 · 关闭其他遥控页' : !linked ? '遥控节点启动中' : `双摇杆就绪 · ${speedLabel}`;
    if (state?.wheels && ui.canControl && linked) {
      const count = state.wheels.filter(w => w.status === 'receiving').length;
      if (count) session += ` · ${count}/4 轮反馈`;
    }
    text($('sessionLabel'), session);
    if (!state?.wheels) return;
    for (const item of state.wheels) {
      const group = $(`wheel${item.label}`);
      if (!group) continue;
      const status = fresh ? item.status : item.status === 'disabled' ? 'disabled' : 'stale';
      if (group.dataset.status !== status) group.dataset.status = status;
      text($(`speed${item.label}`), status === 'receiving' ? format(item.velocity, 1) : status === 'disabled' ? 'OFF' : '—');
    }
    if (details.open) {
      for (const item of [...state.joints, ...state.wheels]) {
        const view = rows.get(item.name); if (!view) continue;
        const status = fresh ? item.status : item.status === 'disabled' ? 'disabled' : 'stale';
        view.row.dataset.status = status; view.row.title = statusNames[status];
        ['position', 'velocity', 'effort'].forEach((key, index) => text(view.cells[index], status === 'receiving' ? format(item[key], 3) : '—'));
      }
    }
  }

  function frame(now) {
    paintStick(move); paintStick(yaw);
    if (ui.dirty && now - lastTelemetryPaint >= 150) {
      paintTelemetry(); ui.dirty = false; lastTelemetryPaint = now;
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);

  function connect() {
    clearTimeout(reconnectTimer);
    if (document.hidden || suspended || ui.ws && ui.ws.readyState < WebSocket.CLOSING) return;
    const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws`);
    ui.ws = ws;
    ws.addEventListener('open', () => {
      if (ui.ws !== ws) return;
      ui.connected = true; ui.lastState = performance.now(); ui.rtt = null;
      backoff = 500; resetAll(); ui.dirty = true;
      sendNow({ type: 'ping', t: performance.now() });
    });
    ws.addEventListener('message', event => {
      if (ui.ws !== ws) return;
      let message;
      try { message = JSON.parse(event.data); } catch { return; }
      if (message.type === 'hello') {
        const source = message.base_limits || message.limits;
        if (source) Object.assign(baseLimits, source);
        speedMin = message.speed_min || speedMin;
        speedMax = message.speed_max || speedMax;
        minBody = message.min_body || minBody;
        minYaw = message.min_yaw || minYaw;
        applySpeedScale();
        $('demoBadge').hidden = !message.demo;
      } else if (message.type === 'state') {
        const hadControl = ui.canControl;
        if (!ui.state) {
          ui.state = message;
        } else {
          ui.state.vx = message.vx;
          ui.state.vy = message.vy;
          ui.state.wz = message.wz;
          ui.state.mode = message.mode;
          ui.state.teleop_connected = message.teleop_connected;
          if (message.joints) {
            ui.state.joints = message.joints;
            ui.state.wheels = message.wheels;
          }
        }
        ui.canControl = message.can_control;
        ui.ready = message.teleop_connected;
        ui.lastState = performance.now();
        if ((stateTick++ & 3) === 0) ui.dirty = true;
        if (hadControl && !ui.canControl) resetAll();
      } else if (message.type === 'pong') {
        ui.rtt = Math.max(0, performance.now() - message.t); ui.dirty = true;
      }
    });
    ws.addEventListener('close', () => {
      if (ui.ws !== ws) return;
      ui.connected = ui.canControl = ui.ready = false; ui.ws = null; lastDriveJson = '';
      resetAll(); ui.dirty = true;
      if (!document.hidden && !suspended) {
        reconnectTimer = setTimeout(connect, backoff); backoff = Math.min(backoff * 1.6, 5000);
      }
    });
    ws.addEventListener('error', () => ws.close());
  }

  setInterval(() => {
    if (ui.connected && performance.now() - ui.lastState > 12000) {
      ui.ws?.close();
    }
  }, 1000);

  setInterval(() => {
    if (ui.connected) sendNow({ type: 'ping', t: performance.now(), rtt: ui.rtt });
  }, 3000);

  speedSlider.addEventListener('input', () => {
    speedScale = Number(speedSlider.value);
    if (!Number.isFinite(speedScale)) return;
    applySpeedScale();
  }, { passive: true });

  poseButtons.forEach(button => button.addEventListener('click', () => {
    if (!ui.connected || !ui.canControl) return;
    resetAll();
    const mode = button.dataset.pose;
    if (ui.state) ui.state.mode = mode;
    text($('mode'), mode.toUpperCase());
    ui.dirty = true;
    sendNow({ type: 'pose', mode });
  }));

  $('robotOpen').addEventListener('click', () => { stop(); details.showModal(); ui.dirty = true; });
  $('closeDetails').addEventListener('click', () => details.close());
  details.addEventListener('close', () => { resetAll(); ui.dirty = true; });
  details.addEventListener('click', event => { if (event.target === details && event.clientY < details.getBoundingClientRect().top) details.close(); });

  $('fullscreen').addEventListener('click', async () => {
    stop();
    try {
      if (document.fullscreenElement) await document.exitFullscreen();
      else if (document.documentElement.requestFullscreen) {
        await document.documentElement.requestFullscreen();
        try { await screen.orientation.lock('landscape'); } catch { /* optional */ }
      } else toast('此浏览器不支持全屏；可从浏览器菜单添加到主屏幕');
    } catch { toast('请在浏览器菜单中启用全屏或横屏'); }
  });

  function onLayoutChange() {
    const orientation = screen.orientation?.type || '';
    if (orientation && orientation !== lastLayout.o) {
      lastLayout.o = orientation;
      stop();
    }
  }

  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { suspended = true; stop(); }
    else { suspended = false; }
  });
  window.addEventListener('pagehide', () => { suspended = true; stop(); });
  window.addEventListener('pageshow', () => { suspended = false; connect(); });
  window.addEventListener('resize', onLayoutChange, { passive: true });
  document.addEventListener('fullscreenchange', () => {
    $('fullscreen').setAttribute('aria-label', document.fullscreenElement ? '退出全屏' : '进入全屏');
    onLayoutChange();
  });

  applySpeedScale();
  connect();
})();
