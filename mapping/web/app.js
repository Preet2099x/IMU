import { createScene } from './scene.js';
import { eulerToQuat, quatToEuler, relativeTo, slerp, wrap360 } from './orientation.js';

const $ = (id) => document.getElementById(id);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const IDENTITY = [1, 0, 0, 0];

const view = createScene($('view'));

const state = {
  mode: 'flat', // 'flat': first reading is level, heading 0. 'abs': angles as reported.
  ref: null, // pose that counts as flat
  latest: null, // last measured pose
  shown: IDENTITY, // pose on screen right now
  seg: { from: IDENTITY, to: IDENTITY, t0: 0, dur: 100 },
  interval: 100, // smoothed ms between readings
  lastArrival: 0,
  raw: null,
};

const targetPose = () => {
  if (!state.latest) return IDENTITY;
  if (state.mode === 'abs' || !state.ref) return state.latest;
  return relativeTo(state.latest, state.ref);
};

// Readings arrive ~10 times a second. Each one starts a short glide from what is
// on screen toward the new pose, so motion stays smooth and never jumps.
const retarget = () => {
  state.seg = { from: state.shown, to: targetPose(), t0: performance.now(), dur: clamp(state.interval, 30, 250) };
};

function onAttitude(ev) {
  const now = performance.now();
  if (state.lastArrival && now - state.lastArrival < 1000) {
    state.interval += 0.2 * (now - state.lastArrival - state.interval);
  }
  state.lastArrival = now;
  state.raw = ev;
  state.latest = eulerToQuat(ev.roll, ev.pitch, ev.heading);
  if (!state.ref) state.ref = state.latest;
  view.setGyro(ev.gyro || null);
  retarget();
}

const BADGE = { connected: 'ok', demo: 'demo', busy: 'warn', waiting: 'warn' };
let lastStatus = null;
let statusAt = 0;
let serverDown = false;

function setBadge(cls, text) {
  $('conn').className = `badge ${cls}`;
  $('connText').textContent = text;
}

function renderBadge() {
  if (serverDown) return setBadge('warn', 'Viewer server not reachable, retrying…');
  if (!lastStatus) return;
  const idleMs = performance.now() - Math.max(state.lastArrival, statusAt);
  if (lastStatus.state === 'connected' && idleMs > 2500) {
    return setBadge('warn', 'Connected, but no readings (board calibrating?)');
  }
  setBadge(BADGE[lastStatus.state] || 'warn', lastStatus.state === 'connected' ? `Connected · ${lastStatus.port}` : lastStatus.message);
}

let session = null;

function onStatus(ev) {
  lastStatus = ev;
  statusAt = performance.now();
  // A new connection means the board may have restarted (heading and all), so
  // the next reading becomes the new flat reference.
  if (ev.session !== undefined && ev.session !== session) {
    session = ev.session;
    state.ref = null;
    view.clearTrail();
  }
  renderBadge();
}

const logLines = [];
function onLog(text) {
  logLines.push(text);
  if (logLines.length > 8) logLines.shift();
  $('log').textContent = logLines.join('\n');
}

function connect() {
  const es = new EventSource('/events');
  es.onmessage = (m) => {
    serverDown = false;
    const ev = JSON.parse(m.data);
    if (ev.type === 'att') onAttitude(ev);
    else if (ev.type === 'status') onStatus(ev);
    else if (ev.type === 'log') onLog(ev.text);
  };
  es.onerror = () => {
    serverDown = true;
    renderBadge();
  };
}

// One decimal, and no "-0.0" for tiny negative values.
const fmt = (v) => (Math.abs(v) < 0.05 ? 0 : v).toFixed(1);

function updateHud() {
  const e = quatToEuler(state.shown);
  $('roll').textContent = fmt(e.roll);
  $('pitch').textContent = fmt(e.pitch);
  $('yaw').textContent = fmt(wrap360(e.yaw)) === '360.0' ? '0.0' : fmt(wrap360(e.yaw));
  const r = state.raw;
  $('raw').textContent = r ? `board: H ${r.heading.toFixed(1)}  P ${r.pitch.toFixed(1)}  R ${r.roll.toFixed(1)}` : '';
  $('gyro').textContent = r && r.gyro ? `gyro: ${r.gyro.map((v) => v.toFixed(1)).join('  ')} °/s` : '';
  $('modeNote').textContent = state.mode === 'flat' ? 'Angles from the start pose (flat, heading 0)' : 'Angles exactly as the board reports them';
}

let lastStaleCheck = 0;
function frame(now) {
  const { from, to, t0, dur } = state.seg;
  state.shown = slerp(from, to, Math.min(1, (now - t0) / dur));
  view.setPose(state.shown);
  updateHud();
  view.render();

  if (now - lastStaleCheck > 500) {
    lastStaleCheck = now;
    renderBadge();
  }
  requestAnimationFrame(frame);
}

function setMode(mode) {
  state.mode = mode;
  for (const b of document.querySelectorAll('#mode button')) b.classList.toggle('on', b.dataset.mode === mode);
  retarget();
}

function rezero() {
  if (!state.latest) return;
  state.ref = state.latest;
  view.clearTrail();
  setMode('flat');
}

async function skipCalibration() {
  try {
    const res = await fetch('/send', { method: 'POST', headers: { 'X-IMU-Viewer': '1' }, body: 's' });
    onLog(res.ok ? '> sent a key press to the board' : `> could not send (${res.status}): demo mode has no board`);
  } catch {
    onLog('> could not send: viewer server not reachable');
  }
}

for (const b of document.querySelectorAll('#mode button')) b.onclick = () => setMode(b.dataset.mode);
for (const b of document.querySelectorAll('[data-view]')) b.onclick = () => view.setView(b.dataset.view);
$('rezero').onclick = rezero;
$('clearTrail').onclick = () => view.clearTrail();
$('skipCal').onclick = skipCalibration;
for (const [id, name] of [['optGimbal', 'gimbal'], ['optTrail', 'trail'], ['optGyro', 'gyro']]) {
  $(id).onchange = (e) => view.setOption(name, e.target.checked);
}

const toggle = (id) => { const el = $(id); el.checked = !el.checked; el.dispatchEvent(new Event('change')); };
window.addEventListener('keydown', (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (k === 'z') rezero();
  else if (k === 'g') toggle('optGimbal');
  else if (k === 't') toggle('optTrail');
  else if (k === 'c') view.clearTrail();
  else if (['1', '2', '3', '4'].includes(k)) view.setView(['iso', 'top', 'side', 'rear'][k - 1]);
});

window.__imu = { state, view, onAttitude, rezero, setMode }; // handy for poking at from the browser console
connect();
requestAnimationFrame(frame);
