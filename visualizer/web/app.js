import { createScene, ROOM } from './scene.js';
import { Lift } from './lift.js';
import { quatToEuler, wrap360 } from './orientation.js';
import { IDENTITY_CAL, applyCal, faceOf, fitAccelCal, FACES } from './calibration.js';

const $ = (id) => document.getElementById(id);

const view = createScene($('view'));
const tracker = new Lift();
const state = { started: false, latestQ: null, last: null, lastArrival: 0, displayQ: [1, 0, 0, 0] };

// --- accelerometer calibration ---------------------------------------------

let cal = IDENTITY_CAL;
const isCalibrated = () => cal.offset.some((v) => v !== 0) || cal.scale.some((v) => v !== 1);

async function loadCal() {
  try {
    const res = await fetch('/api/cal');
    if (res.ok) cal = await res.json();
  } catch { /* the server is not there yet: keep going uncalibrated */ }
  $('calState').textContent = isCalibrated() ? 'Accelerometer: calibrated' : 'Accelerometer: not calibrated (less accurate)';
  $('calState').classList.toggle('warn', !isCalibrated());
}

const FACE_LABELS = {
  '+X': 'Nose up', '-X': 'Nose down', '+Y': 'Green arrow up', '-Y': 'Green arrow down', '+Z': 'Flat, right way up', '-Z': 'Upside down',
};
const HOLD_SAMPLES = 60; // 1.2 s of stillness at 50 Hz
const rawBuf = [];
let calib = null; // { faces: [mean | null] x 6, still: n } while calibrating

function drawFaces() {
  $('faces').innerHTML = FACES.map((name, i) => `<span class="face${calib.faces[i] ? ' done' : ''}">${calib.faces[i] ? '✓ ' : ''}${FACE_LABELS[name]}</span>`).join('');
}

function startCalibration() {
  calib = { faces: FACES.map(() => null), still: 0 };
  $('calib').hidden = false;
  $('calibMsg').textContent = 'Put the board down on any side and hold it still.';
  drawFaces();
}

function stopCalibration(message) {
  calib = null;
  if (message) onLog(message);
  $('calib').hidden = true;
}

async function finishCalibration() {
  let fit;
  try {
    fit = fitAccelCal(calib.faces);
  } catch (e) {
    $('calibMsg').textContent = `${e.message}. Start again.`;
    calib.faces = FACES.map(() => null);
    drawFaces();
    return;
  }
  if (fit.rms > 0.01) {
    $('calibMsg').textContent = `The readings did not agree well (${(fit.rms * 100).toFixed(1)}% off). Hold each side still and start again.`;
    calib.faces = FACES.map(() => null);
    drawFaces();
    return;
  }
  try {
    const res = await fetch('/api/cal', {
      method: 'POST',
      headers: { 'X-IMU-Viewer': '1', 'Content-Type': 'application/json' },
      body: JSON.stringify({ offset: fit.offset, scale: fit.scale }),
    });
    if (!res.ok) throw new Error(`server said ${res.status}`);
    cal = await res.json();
  } catch (e) {
    $('calibMsg').textContent = `Could not save the calibration (${e.message}).`;
    return;
  }
  $('calState').textContent = 'Accelerometer: calibrated';
  $('calState').classList.remove('warn');
  stopCalibration(`Calibrated: readings are within ${(fit.rms * 100).toFixed(2)}% of 1 g. Hold still to start tracking.`);
  state.started = false; // start over with the corrected readings
}

// Called with every reading while calibrating: grabs a side once it has been still long enough.
function calibrationStep(detector) {
  calib.still = detector < 6 ? calib.still + 1 : 0;
  if (calib.still === 1) $('calibMsg').textContent = 'Hold still…';
  if (calib.still === 0) $('calibMsg').textContent = 'Turn the board onto a side you have not done yet.';
  if (calib.still !== HOLD_SAMPLES) return;

  const mean = [0, 1, 2].map((i) => rawBuf.reduce((s, f) => s + f[i], 0) / rawBuf.length);
  const face = faceOf(mean);
  if (face < 0) {
    $('calibMsg').textContent = 'Not quite on a side. Tilt it until one arrow points straight up or down, then hold still.';
  } else if (calib.faces[face]) {
    $('calibMsg').textContent = `Already have "${FACE_LABELS[FACES[face]]}". Try another side.`;
  } else {
    calib.faces[face] = mean;
    drawFaces();
    $('calibMsg').textContent = `Got "${FACE_LABELS[FACES[face]]}". Turn it onto another side.`;
    if (calib.faces.every(Boolean)) finishCalibration();
  }
}

// --- incoming data ----------------------------------------------------------

// A new connection means the board may have restarted, so the first reading
// after it becomes the middle of the room, facing front.
function onViz(ev) {
  const nums = [...ev.q, ...ev.f, ...ev.g];
  if (!nums.every(Number.isFinite)) return;
  if (!state.started) {
    tracker.reset();
    tracker.zeroHeading(ev.q);
    view.clearTrail();
    state.started = true;
    state.last = null;
    const dq = tracker.displayQuat(ev.q);
    view.setPose([0, 0, 0], dq);
    view.snap();
  }
  const r = tracker.update({ ms: ev.ms, q: ev.q, f: applyCal(ev.f, cal), g: ev.g });
  state.latestQ = ev.q;
  state.last = r;
  state.lastArrival = performance.now();
  state.displayQ = tracker.displayQuat(ev.q);
  view.setPose(r.pos, state.displayQ);

  rawBuf.push(ev.f);
  if (rawBuf.length > HOLD_SAMPLES) rawBuf.shift();
  if (calib) calibrationStep(r.detector);
}

const BADGE = { connected: 'ok', demo: 'demo', busy: 'warn', waiting: 'warn' };
let lastStatus = null;
let statusAt = 0;
let serverDown = false;
let session = null;

function setBadge(cls, text) {
  $('conn').className = `badge ${cls}`;
  $('connText').textContent = text;
}

function renderBadge() {
  if (serverDown) return setBadge('warn', 'Server not reachable, retrying…');
  if (!lastStatus) return;
  const idleMs = performance.now() - Math.max(state.lastArrival, statusAt);
  if (lastStatus.state === 'connected' && idleMs > 2500) {
    return setBadge('warn', 'Connected, waiting for data (board starting up?)');
  }
  setBadge(BADGE[lastStatus.state] || 'warn', lastStatus.state === 'connected' ? `Connected · ${lastStatus.port}` : lastStatus.message);
}

function onStatus(ev) {
  lastStatus = ev;
  statusAt = performance.now();
  if (ev.session !== undefined && ev.session !== session) {
    session = ev.session;
    state.started = false;
  }
  renderBadge();
}

const logLines = [];
function onLog(text) {
  logLines.push(text);
  if (logLines.length > 6) logLines.shift();
  $('log').textContent = logLines.join('\n');
}

function connect() {
  const es = new EventSource('/events');
  es.onmessage = (m) => {
    serverDown = false;
    const ev = JSON.parse(m.data);
    if (ev.type === 'viz') onViz(ev);
    else if (ev.type === 'status') onStatus(ev);
    else if (ev.type === 'log') onLog(ev.text);
    else if (ev.type === 'acal') onLog('The board is now using the accelerometer calibration.');
  };
  es.onerror = () => {
    serverDown = true;
    renderBadge();
  };
}

// --- display ------------------------------------------------------------------

// One decimal, no "-0.0", and the label follows the sign (Front / Back, ...).
function axis(valueId, labelId, metres, plus, minus) {
  const cm = metres * 100;
  $(labelId).textContent = cm >= 0 ? plus : minus;
  $(valueId).textContent = Math.abs(cm) < 0.05 ? '0.0' : Math.abs(cm).toFixed(1);
}

const fmt = (v) => (Math.abs(v) < 0.05 ? 0 : v).toFixed(1);
const fmtHeading = (deg) => (fmt(wrap360(deg)) === '360.0' ? '0.0' : fmt(wrap360(deg)));

function updateHud() {
  const pos = state.last ? state.last.pos : [0, 0, 0];
  axis('pz', 'kz', pos[2], 'Up', 'Down');

  const settling = state.last && state.last.settling;
  const v = state.last ? state.last.vel[2] : 0;
  const moving = !settling && state.last && !state.last.still;
  $('motion').textContent = !state.last ? 'Waiting for data' : settling ? 'Hold still to start' : !moving ? 'Still' : v > 0.03 ? '▲ Going up' : v < -0.03 ? '▼ Going down' : 'Moving';
  $('motion').classList.toggle('moving', Boolean(moving || settling));

  const e = quatToEuler(state.displayQ);
  $('angles').textContent = `roll ${fmt(e.roll)}  pitch ${fmt(e.pitch)}  heading ${fmtHeading(e.yaw)}`;
  $('edge').textContent = view.atEdge() ? `Reached the edge of the ${ROOM.half * 2} m room` : '';
}

let prev = performance.now();
let lastBadge = 0;
function frame(now) {
  const dt = Math.min(0.1, (now - prev) / 1000);
  prev = now;
  view.render(dt);
  updateHud();
  if (now - lastBadge > 500) {
    lastBadge = now;
    renderBadge();
  }
  requestAnimationFrame(frame);
}

// --- controls -----------------------------------------------------------------

function zeroPosition() {
  tracker.zeroPosition();
  view.clearTrail();
  if (state.latestQ) view.setPose([0, 0, 0], state.displayQ);
  view.snap();
}

function zeroHeading() {
  if (!state.latestQ) return;
  tracker.zeroHeading(state.latestQ);
  state.displayQ = tracker.displayQuat(state.latestQ);
  zeroPosition();
}

for (const b of document.querySelectorAll('[data-view]')) b.onclick = () => view.setView(b.dataset.view);
$('zeroPos').onclick = zeroPosition;
$('zeroHead').onclick = zeroHeading;
$('clearTrail').onclick = () => view.clearTrail();
$('optTrail').onchange = (e) => view.setTrail(e.target.checked);
$('calibrate').onclick = startCalibration;
$('calibCancel').onclick = () => stopCalibration();

window.addEventListener('keydown', (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (k === 'z') zeroPosition();
  else if (k === 'h') zeroHeading();
  else if (k === 'c') view.clearTrail();
  else if (k === 't') { const el = $('optTrail'); el.checked = !el.checked; view.setTrail(el.checked); }
  else if (['1', '2', '3', '4'].includes(k)) view.setView(['iso', 'top', 'side', 'rear'][k - 1]);
});

window.__viz = { tracker, state, onViz, view, get cal() { return cal; }, startCalibration }; // handy for poking at from the browser console
loadCal();
connect();
requestAnimationFrame(frame);
