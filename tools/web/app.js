// Live 3D view of the IMU board. All the measuring happens on the Teensy; this
// only draws what the firmware's '@' lines say, as relayed by
// tools/motion_viewer.py (see the "Dead reckoning" comment in src/main.cpp).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

// The firmware's earth frame is X forward, Y left, Z up, in cm. three.js is
// Y-up, so forward becomes -z, left -x and up +y (still right-handed).
const toScene = (x, y, z) => new THREE.Vector3(-y, z, -x);

const LABELLED_MOVES = 6;  // only the latest moves get numbers and arrowheads
const FADE_MOVES = 5;      // moves back over which an old path fades out
const MAP_MOVES = 10;      // the top view frames this many of the latest moves
const LIST_MOVES = 9;      // rows in the recent-moves panel
const SMOOTH_S = 0.06;     // smoothing between the board's 25 Hz updates
const BG = new THREE.Color('#070b14');
const MAP_BG = new THREE.Color('#0b1222');
const COLOR = {
  latestStart: new THREE.Color('#7c3aed'),
  latestEnd: new THREE.Color('#f472b6'),
  recent: new THREE.Color('#60a5fa'),
  old: new THREE.Color('#5470b8'), // muted blue: faint, but never the grid's grey
  liveStart: new THREE.Color('#b45309'),
  liveEnd: new THREE.Color('#fde047'),
  accent: new THREE.Color('#a78bfa'),
};

// ---------------------------------------------------------------------------
// Renderer, scene, camera

const container = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
container.appendChild(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.setSize(innerWidth, innerHeight);
Object.assign(labelRenderer.domElement.style, { position: 'fixed', top: '0', left: '0', pointerEvents: 'none' });
container.appendChild(labelRenderer.domElement);

const scene = new THREE.Scene();
scene.background = BG;
const fog = new THREE.Fog(BG, 300, 1500);
scene.fog = fog;

const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.5, 100000);
camera.position.set(90, 110, 190); // behind the start, to the right and above
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.zoomToCursor = true;
controls.screenSpacePanning = true;
controls.minDistance = 8;
controls.maxDistance = 30000;

scene.add(new THREE.HemisphereLight(0xc7d2fe, 0x0b1020, 1.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.8);
sun.position.set(300, 600, 200);
scene.add(sun);

// ---------------------------------------------------------------------------
// Ground: a fine and a coarse grid that travel with the board, faded by fog

function makeGrid(size, divisions, color, opacity) {
  const grid = new THREE.GridHelper(size, divisions, color, color);
  grid.material.transparent = true;
  grid.material.opacity = opacity;
  grid.material.depthWrite = false;
  return grid;
}
const ground = new THREE.Group();
ground.add(makeGrid(8000, 800, 0x1e293b, 0.4)); // 10 cm
ground.add(makeGrid(8000, 80, 0x334155, 0.5));  // 1 m
scene.add(ground);

function label(text, className) {
  const outer = document.createElement('div');
  const inner = document.createElement('span');
  inner.className = className;
  inner.textContent = text;
  outer.appendChild(inner);
  return new CSS2DObject(outer);
}

// Start marker and a compass, scaled with the camera so they stay readable.
const origin = new THREE.Group();
{
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(3.2, 4.2, 64),
    new THREE.MeshBasicMaterial({ color: 0xe2e8f0, transparent: true, opacity: 0.7, side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2;
  origin.add(ring);
  const start = label('Start', 'ground-label');
  start.position.set(0, 0, 9);
  origin.add(start);
  for (const [dir, text] of [[toScene(1, 0, 0), 'Forward'], [toScene(0, 1, 0), 'Left']]) {
    origin.add(new THREE.ArrowHelper(dir, new THREE.Vector3(), 22, 0x94a3b8, 4, 2.6));
    const tag = label(text, 'ground-label');
    tag.position.copy(dir.clone().multiplyScalar(30));
    origin.add(tag);
  }
}
scene.add(origin);

// ---------------------------------------------------------------------------
// The board: a small PCB whose x, y, z axes are the sensor's

function makeBoard() {
  const model = new THREE.Group();
  const pcb = new THREE.Mesh(
    new THREE.BoxGeometry(10, 5, 0.6),
    new THREE.MeshStandardMaterial({ color: 0x1f2a44, metalness: 0.35, roughness: 0.45 }));
  model.add(pcb);
  model.add(new THREE.LineSegments(
    new THREE.EdgesGeometry(pcb.geometry),
    new THREE.LineBasicMaterial({ color: 0xa78bfa })));
  const chip = new THREE.Mesh(
    new THREE.BoxGeometry(2.6, 2.6, 0.5),
    new THREE.MeshStandardMaterial({ color: 0x0b0f19, metalness: 0.2, roughness: 0.3 }));
  chip.position.set(1.6, 0, 0.55);
  model.add(chip);
  const led = new THREE.Mesh(new THREE.SphereGeometry(0.4, 16, 12), new THREE.MeshBasicMaterial({ color: 0x34d399 }));
  led.position.set(-3.7, 1.6, 0.45);
  model.add(led);
  const axes = [[new THREE.Vector3(1, 0, 0), 0xef4444], [new THREE.Vector3(0, 1, 0), 0x22c55e],
                [new THREE.Vector3(0, 0, 1), 0x3b82f6]];
  for (const [dir, color] of axes) {
    model.add(new THREE.ArrowHelper(dir, new THREE.Vector3(), 9, color, 2.2, 1.3));
  }
  return model;
}
const board = new THREE.Group();
board.add(makeBoard());
board.visible = false;
scene.add(board);

// Where the board is over the ground: a ring below it and a dashed drop line.
const marker = new THREE.Mesh(
  new THREE.RingGeometry(4, 5.4, 64),
  new THREE.MeshBasicMaterial({ color: 0xa78bfa, transparent: true, opacity: 0.6, side: THREE.DoubleSide, depthWrite: false }));
marker.rotation.x = -Math.PI / 2;
marker.visible = false;
scene.add(marker);
const dropMaterial = new THREE.LineDashedMaterial({ color: 0xa78bfa, dashSize: 3, gapSize: 2, transparent: true, opacity: 0.55 });
const drop = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), dropMaterial);
drop.visible = false;
scene.add(drop);

// ---------------------------------------------------------------------------
// Thick lines (WebGL lines are 1 px otherwise)

const lineMaterials = new Set(); // their resolution must follow the viewport

function fatLine(points, colors, width, opacity, additive = false) {
  const geometry = new LineGeometry();
  geometry.setPositions(points.flatMap((p) => [p.x, p.y, p.z]));
  geometry.setColors(colors.flatMap((c) => [c.r, c.g, c.b]));
  const material = new LineMaterial({
    linewidth: width,
    vertexColors: true,
    transparent: true,
    opacity,
    depthWrite: false,
    blending: additive ? THREE.AdditiveBlending : THREE.NormalBlending,
  });
  material.resolution.set(innerWidth, innerHeight);
  lineMaterials.add(material);
  const line = new Line2(geometry, material);
  line.computeLineDistances();
  return line;
}

function disposeLine(line) {
  if (!line) return;
  line.removeFromParent();
  line.geometry.dispose();
  line.material.dispose();
  lineMaterials.delete(line.material);
}

function gradient(n, from, to) {
  return Array.from({ length: n }, (_, i) => from.clone().lerp(to, n > 1 ? i / (n - 1) : 1));
}

// ---------------------------------------------------------------------------
// Finished moves

let moves = []; // { n, start, end, text, points, group, line, glow, cone, label }
const paths = new Map(); // n -> path from @PATH, earth cm

function addMove(n, start, end, text) {
  const raw = paths.get(n) ?? [start, end];
  paths.delete(n);
  const points = raw.map((p) => toScene(...p));
  const group = new THREE.Group();
  const line = fatLine(points, gradient(points.length, COLOR.recent, COLOR.recent), 3, 1);
  const glow = fatLine(points, gradient(points.length, COLOR.latestStart, COLOR.latestEnd), 12, 0.22, true);
  group.add(line, glow);

  // Arrowhead along the last stretch of the path.
  const tip = points[points.length - 1];
  let back = points[0];
  for (let i = points.length - 2; i >= 0; i--) {
    back = points[i];
    if (back.distanceTo(tip) > 2) break;
  }
  const dir = tip.clone().sub(back);
  if (dir.lengthSq() < 1e-6) dir.set(0, 0, -1);
  const cone = new THREE.Mesh(
    new THREE.ConeGeometry(1.1, 3.2, 20),
    new THREE.MeshStandardMaterial({ color: 0xffffff, transparent: true, emissive: 0x000000, roughness: 0.4 }));
  cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  cone.position.copy(tip);
  group.add(cone);

  // The firmware numbers measured moves as steps ("Step 5: ..."); tag with that.
  const step = /^Step (\d+)/.exec(text)?.[1] ?? String(n);
  const tag = label(step, 'move-label');
  tag.position.copy(tip);
  group.add(tag);

  scene.add(group);
  moves.push({ n, step, start, end, text, points, group, line, glow, cone, label: tag });
  restyleMoves();
  updateMoveList();
}

// The latest move stands out; older ones fade toward faint, thin lines.
function restyleMoves() {
  moves.forEach((m, i) => {
    const age = moves.length - 1 - i;
    const fade = age === 0 ? 1 : Math.exp(-(age - 1) / FADE_MOVES);
    const count = m.points.length;
    let colors;
    if (age === 0) {
      colors = gradient(count, COLOR.latestStart, COLOR.latestEnd);
    } else {
      const base = COLOR.old.clone().lerp(COLOR.recent, fade);
      colors = gradient(count, base.clone().multiplyScalar(0.55), base);
    }
    m.line.geometry.setColors(colors.flatMap((c) => [c.r, c.g, c.b]));
    // Old moves stay as a faint trace, so the whole session still reads.
    m.line.material.linewidth = age === 0 ? 4 : 2 + 1.2 * fade;
    m.line.material.opacity = age === 0 ? 1 : 0.4 + 0.5 * fade;
    m.glow.visible = age === 0;

    const shown = age < LABELLED_MOVES;
    m.cone.visible = shown;
    m.cone.material.color.copy(colors[count - 1]);
    m.cone.material.emissive.copy(colors[count - 1]).multiplyScalar(age === 0 ? 0.6 : 0.2);
    m.cone.material.opacity = m.line.material.opacity;
    m.label.visible = shown;
    m.label.element.firstChild.classList.toggle('latest', age === 0);
    m.label.element.style.opacity = String(age === 0 ? 1 : 0.35 + 0.65 * fade);
  });
}

function clearMoves() {
  for (const m of moves) {
    disposeLine(m.line);
    disposeLine(m.glow);
    m.cone.geometry.dispose();
    m.cone.material.dispose();
    m.label.removeFromParent();
    m.label.element.remove();
    m.group.removeFromParent();
  }
  moves = [];
  paths.clear();
  updateMoveList();
}

// ---------------------------------------------------------------------------
// The move under way: a live trail from the firmware's running estimate

let trail = [];
let trailLine = null;
let trailGlow = null;

function redrawTrail() {
  disposeLine(trailLine);
  disposeLine(trailGlow);
  trailLine = trailGlow = null;
  if (trail.length < 2) return;
  const colors = gradient(trail.length, COLOR.liveStart, COLOR.liveEnd);
  trailLine = fatLine(trail, colors, 3.5, 1);
  trailGlow = fatLine(trail, colors, 12, 0.25, true);
  scene.add(trailLine, trailGlow);
}

function clearTrail() {
  trail = [];
  redrawTrail();
}

// ---------------------------------------------------------------------------
// Live board pose, smoothed between the firmware's 25 Hz updates

const targetPos = new THREE.Vector3();
const targetQuat = new THREE.Quaternion();
const shownPos = new THREE.Vector3();
const shownQuat = new THREE.Quaternion();
const basis = new THREE.Matrix4();
let haveLive = false;
let lastLive = [0, 0, 0];

function onLive(v) {
  targetPos.copy(toScene(v[0], v[1], v[2]));
  basis.makeBasis(toScene(v[3], v[4], v[5]), toScene(v[6], v[7], v[8]), toScene(v[9], v[10], v[11]));
  targetQuat.setFromRotationMatrix(basis).normalize();
  if (!haveLive) {
    shownPos.copy(targetPos);
    shownQuat.copy(targetQuat);
    haveLive = true;
    board.visible = marker.visible = drop.visible = true;
  }
  if (state === 'MOVING') {
    trail.push(targetPos.clone());
    redrawTrail();
  }
  lastLive = v.slice(0, 3);
  setReadouts();
}

// ---------------------------------------------------------------------------
// Panels

const $ = (id) => document.getElementById(id);
const signed = (v) => `${v >= 0 ? '+' : '−'}${Math.abs(v).toFixed(1)}`;

// The firmware counts the steps and keeps their average; total distance is
// steps times the average step.
function setSteps(count, averageCm, totalCm) {
  $('dist-total').textContent = totalCm.toFixed(1);
  $('dist-steps').textContent = String(count);
  $('dist-avg').textContent = averageCm.toFixed(1);
  $('dist-formula').textContent = count
    ? `${count} step${count === 1 ? '' : 's'} × ${averageCm.toFixed(1)} cm average`
    : 'No steps yet';
}

function setReadouts() {
  const [x, y, z] = lastLive;
  $('px').textContent = signed(x);
  $('py').textContent = signed(y);
  $('pz').textContent = signed(z);
  $('pdist').textContent = Math.hypot(x, y, z).toFixed(1);
  $('pcount').textContent = String(moves.length);
}

function setPill(el, kind, text, pulse = false) {
  el.className = `pill ${kind}${pulse ? ' pulse' : ''}`;
  el.querySelector('.text').textContent = text;
}

let state = 'WAITING';
let linkPort = '';
let gotBoardLines = false;

function setState(name, message) {
  state = name;
  const look = {
    READY: ['ok', 'Ready'],
    MOVING: ['warn', 'Measuring move', true],
    CALIBRATING: ['info', 'Calibrating', true],
    SETTLING: ['info', 'Settling', true],
  }[name] ?? ['', name];
  setPill($('state'), look[0], look[1], look[2]);
  // Only show the firmware's words when they ask for something.
  const quiet = name === 'MOVING' || (name === 'READY' && message.startsWith('move it hard'));
  $('state-note').textContent = quiet ? '' : message;
  if (name !== 'MOVING') clearTrail();
}

function setLink(message) {
  const el = $('link');
  if (message.startsWith('connected on ')) {
    linkPort = message.slice('connected on '.length);
    gotBoardLines = false;
    setPill(el, 'info', `Connected · ${linkPort}`, true);
  } else if (message.startsWith('replay of ')) {
    setPill(el, 'info', `Replay · ${message.slice('replay of '.length)}`);
  } else if (message.startsWith("can't use ")) {
    setPill(el, 'bad', 'Port busy · close other serial monitors');
  } else if (message.startsWith('Teensy not found')) {
    setPill(el, 'bad', 'Board not found · plug it in');
  } else {
    setPill(el, '', message, true);
  }
}

function markLive() {
  if (gotBoardLines || !linkPort) return;
  gotBoardLines = true;
  setPill($('link'), 'ok', `Live · ${linkPort}`);
}

function moveIcon(start, end) {
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const dz = end[2] - start[2];
  if (Math.abs(dz) > Math.hypot(dx, dy)) return { glyph: dz > 0 ? '⤒' : '⤓', angle: 0 };
  // Forward is up the screen and left is left, like the top view.
  return { glyph: '↑', angle: (Math.atan2(-dy, dx) * 180) / Math.PI };
}

function updateMoveList() {
  const list = $('move-list');
  list.replaceChildren();
  const recent = moves.slice(-LIST_MOVES).reverse();
  recent.forEach((m, age) => {
    const parsed = /:\s*([\d.]+) cm (.+?), in ([\d.]+) s/.exec(m.text);
    const li = document.createElement('li');
    if (age === 0) li.className = 'latest';
    li.style.opacity = String(Math.max(0.3, 1 - age * 0.09));
    const icon = moveIcon(m.start, m.end);
    li.innerHTML = `
      <div class="move-icon"><span style="transform: rotate(${icon.angle.toFixed(0)}deg)">${icon.glyph}</span></div>
      <div><div class="move-dir"></div><div class="move-sub"></div></div>
      <div class="move-dist"></div>`;
    li.querySelector('.move-dir').textContent = parsed ? parsed[2] : m.text;
    li.querySelector('.move-sub').textContent = parsed ? `Step ${m.step} · ${parsed[3]} s` : `Step ${m.step}`;
    li.querySelector('.move-dist').textContent = parsed ? `${parsed[1]} cm` : '';
    list.appendChild(li);
  });
  $('move-empty').hidden = moves.length > 0;
  $('pcount').textContent = String(moves.length);
}

function toast(text) {
  const kind = /forward set/i.test(text) ? 'ok'
    : /not measured|lost|spun|harder|longer/i.test(text) ? 'warn' : 'info';
  const el = document.createElement('div');
  el.className = `toast ${kind}`;
  el.innerHTML = '<span class="dot"></span><span></span>';
  el.lastChild.textContent = text;
  const box = $('toasts');
  box.prepend(el);
  while (box.children.length > 3) box.lastChild.remove();
  setTimeout(() => {
    el.classList.add('out');
    setTimeout(() => el.remove(), 450);
  }, 5000);
}

// ---------------------------------------------------------------------------
// The firmware's lines

function resetAll() {
  clearMoves();
  clearTrail();
}

let catchingUp = false;

function handle(line) {
  if (line === '#hello') {
    resetAll(); // the server is about to replay everything this page needs
    catchingUp = true;
    return;
  }
  if (line === '#caught-up') {
    catchingUp = false;
    return;
  }
  if (line.startsWith('#link ')) {
    setLink(line.slice(6));
    return;
  }
  if (!line.startsWith('@')) return;
  markLive();
  const space = line.indexOf(' ');
  const kind = space < 0 ? line.slice(1) : line.slice(1, space);
  const rest = space < 0 ? '' : line.slice(space + 1);

  if (kind === 'LIVE') {
    onLive(rest.trim().split(/\s+/).map(Number));
  } else if (kind === 'STATE') {
    const cut = rest.indexOf(' ');
    setState(cut < 0 ? rest : rest.slice(0, cut), cut < 0 ? '' : rest.slice(cut + 1));
  } else if (kind === 'PATH') {
    const [n, ...v] = rest.trim().split(/\s+/).map(Number);
    const pts = [];
    for (let i = 0; i + 2 < v.length; i += 3) pts.push([v[i], v[i + 1], v[i + 2]]);
    paths.set(n, pts);
  } else if (kind === 'MOVE') {
    const [numbers, text = ''] = rest.split(' | ');
    const v = numbers.trim().split(/\s+/).map(Number);
    clearTrail();
    addMove(v[0], v.slice(1, 4), v.slice(4, 7), text);
  } else if (kind === 'STEPS') {
    const [count, averageCm, totalCm] = rest.trim().split(/\s+/).map(Number);
    setSteps(count, averageCm, totalCm);
  } else if (kind === 'INFO') {
    if (!catchingUp) toast(rest);
  } else if (kind === 'RESET') {
    resetAll();
    setReadouts();
  }
}

const events = new EventSource('/events');
events.onmessage = (e) => handle(e.data);
events.onerror = () => setPill($('link'), 'bad', 'Viewer server stopped · restart motion_viewer.py');

// ---------------------------------------------------------------------------
// Camera: follow the board, fit everything, top view, 3D view

let follow = true;
let flight = null; // { t, from: {pos, target}, to: {pos, target} }

function setFollow(on) {
  follow = on;
  $('btn-follow').classList.toggle('on', on);
}

function flyTo(target, position) {
  flight = {
    t: 0,
    from: { pos: camera.position.clone(), target: controls.target.clone() },
    to: { pos: position, target },
  };
}

function fitAll() {
  const box = new THREE.Box3().expandByPoint(new THREE.Vector3());
  for (const m of moves) for (const p of m.points) box.expandByPoint(p);
  if (haveLive) box.expandByPoint(shownPos);
  const sphere = box.getBoundingSphere(new THREE.Sphere());
  const radius = Math.max(sphere.radius, 30);
  const distance = (radius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2))) * 1.1;
  const dir = camera.position.clone().sub(controls.target).normalize();
  setFollow(false);
  flyTo(sphere.center.clone(), sphere.center.clone().add(dir.multiplyScalar(distance)));
}

function topView() {
  const target = controls.target.clone();
  const distance = camera.position.distanceTo(target);
  flyTo(target, target.clone().add(new THREE.Vector3(0, distance, 0.01))); // forward ends up at the top
}

function view3d() {
  const target = controls.target.clone();
  const distance = camera.position.distanceTo(target);
  flyTo(target, target.clone().add(new THREE.Vector3(0.45, 0.55, 0.7).normalize().multiplyScalar(distance)));
}

// Panning means the user wants to look somewhere else, so stop following.
renderer.domElement.addEventListener('pointerdown', (e) => {
  if (e.button !== 0 || e.ctrlKey || e.metaKey || e.shiftKey) setFollow(false);
});

// ---------------------------------------------------------------------------
// Buttons and keys

function send(key) {
  fetch('/cmd', { method: 'POST', body: key }).catch(() => {});
}

document.querySelectorAll('[data-cmd]').forEach((b) => b.addEventListener('click', () => send(b.dataset.cmd)));
$('btn-follow').addEventListener('click', () => setFollow(!follow));
$('btn-fit').addEventListener('click', fitAll);
$('btn-top').addEventListener('click', topView);
$('btn-3d').addEventListener('click', view3d);
addEventListener('keydown', (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const key = e.key.toLowerCase();
  if (key === 'f' || key === 'z' || key === 'r') send(key);
  else if (key === 'a') setFollow(!follow);
  else if (key === 'w') fitAll();
  else if (key === 't') topView();
  else if (key === '3') view3d();
});

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
  labelRenderer.setSize(innerWidth, innerHeight);
  for (const m of lineMaterials) m.resolution.set(innerWidth, innerHeight);
});

// ---------------------------------------------------------------------------
// Top view inset: the same scene from straight above, framing recent moves

// It has its own canvas inside its panel; drawn into the main canvas it would
// sit under the panel's frosted glass.
const mapView = $('minimap-view');
const mapRenderer = new THREE.WebGLRenderer({ antialias: true });
mapRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
mapRenderer.toneMapping = THREE.ACESFilmicToneMapping;
mapView.appendChild(mapRenderer.domElement);
const mapCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 1, 200000);
mapCamera.up.set(0, 0, -1); // forward up the inset, left on the left

function renderMap() {
  const rect = mapView.getBoundingClientRect();
  if (rect.width < 10) return; // hidden on narrow screens
  if (mapRenderer.domElement.width !== Math.round(rect.width * mapRenderer.getPixelRatio())) {
    mapRenderer.setSize(rect.width, rect.height);
  }
  const box = new THREE.Box3();
  if (haveLive) box.expandByPoint(shownPos);
  for (const m of moves.slice(-MAP_MOVES)) for (const p of m.points) box.expandByPoint(p);
  for (const p of trail) box.expandByPoint(p);
  if (box.isEmpty()) box.expandByPoint(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const aspect = rect.width / rect.height;
  let half = Math.max(size.z / 2, size.x / 2 / aspect, 40) * 1.2;
  mapCamera.left = -half * aspect;
  mapCamera.right = half * aspect;
  mapCamera.top = half;
  mapCamera.bottom = -half;
  mapCamera.position.set(center.x, center.y + 50000, center.z);
  mapCamera.lookAt(center.x, center.y, center.z);
  mapCamera.updateProjectionMatrix();

  // The board and markers are sized for the main camera; size them for the
  // inset instead while it draws (the board about a twelfth of its width).
  setMarkerScale((half * aspect * 2) / 12 / 10);
  drop.visible = false;
  for (const m of lineMaterials) m.resolution.set(rect.width, rect.height);
  scene.fog = null;
  scene.background = MAP_BG;
  mapRenderer.render(scene, mapCamera);
  scene.fog = fog;
  scene.background = BG;
  for (const m of lineMaterials) m.resolution.set(innerWidth, innerHeight);
  drop.visible = haveLive;
}

// ---------------------------------------------------------------------------
// Frame loop

// Board, start marker, ground ring and arrowheads are drawn at a size that
// suits the camera, not their real size.
function setMarkerScale(s) {
  board.scale.setScalar(s);
  origin.scale.setScalar(s);
  marker.scale.setScalar(s);
  for (const m of moves) if (m.cone.visible) m.cone.scale.setScalar(s);
}

const clock = new THREE.Clock();
const scratch = new THREE.Vector3();

// The floor sits at the start height, or just under the lowest point of the
// recent moves if the board has gone lower, so it stays in view.
let floorY = 0;
function floorTarget() {
  let low = haveLive ? shownPos.y : 0;
  for (const m of moves.slice(-MAP_MOVES)) for (const p of m.points) low = Math.min(low, p.y);
  for (const p of trail) low = Math.min(low, p.y);
  return Math.min(0, low - 15);
}

function frame() {
  requestAnimationFrame(frame);
  const dt = Math.min(clock.getDelta(), 0.1);
  const k = 1 - Math.exp(-dt / SMOOTH_S);
  shownPos.lerp(targetPos, k);
  shownQuat.slerp(targetQuat, k);
  board.position.copy(shownPos);
  board.quaternion.copy(shownQuat);

  // Floor marker and drop line under the board; the grid travels along.
  floorY += (floorTarget() - floorY) * (1 - Math.exp(-dt * 3));
  marker.position.set(shownPos.x, floorY + 0.05, shownPos.z);
  drop.geometry.setFromPoints([shownPos, scratch.set(shownPos.x, floorY, shownPos.z)]);
  drop.computeLineDistances();
  ground.position.set(Math.round(shownPos.x / 100) * 100, floorY, Math.round(shownPos.z / 100) * 100);

  if (flight) {
    flight.t = Math.min(1, flight.t + dt / 0.7);
    const e = 1 - Math.pow(1 - flight.t, 3); // ease out
    camera.position.lerpVectors(flight.from.pos, flight.to.pos, e);
    controls.target.lerpVectors(flight.from.target, flight.to.target, e);
    if (flight.t >= 1) flight = null;
  } else if (follow && haveLive) {
    const step = scratch.copy(shownPos).sub(controls.target).multiplyScalar(1 - Math.exp(-dt * 4));
    controls.target.add(step);
    camera.position.add(step);
  }
  controls.update();

  // Keep the board, arrowheads and markers a readable size at any zoom.
  const distance = camera.position.distanceTo(controls.target);
  const s = THREE.MathUtils.clamp(distance / 120, 0.5, 120);
  setMarkerScale(s);
  dropMaterial.dashSize = 3 * s;
  dropMaterial.gapSize = 2 * s;
  marker.material.opacity = state === 'MOVING' ? 0.45 + 0.35 * Math.sin(clock.elapsedTime * 8) : 0.6;
  fog.near = distance * 1.3;
  fog.far = distance * 5;

  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
  renderMap();
}

$('loading').hidden = true;
setReadouts();
frame();
