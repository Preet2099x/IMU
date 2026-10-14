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
// One square is exactly 1 m (80 m across in 80 squares), so a distance can be
// read straight off the floor. The group is moved in whole metres below, so the
// lines always stay on whole metres of the board's own coordinates.
ground.add(makeGrid(8000, 80, 0x4a5a78, 0.8));
// Each square is split into ten, 10 cm apart. Only drawn when zoomed in: from
// further away these lines are a few pixels apart and blur into a haze.
const FINE_GRID_OPACITY = 0.5;
const fineGrid = makeGrid(8000, 800, 0x34425f, FINE_GRID_OPACITY);
fineGrid.renderOrder = -1; // under the 1 m lines
ground.add(fineGrid);

function setFineGrid(fade) {
  fineGrid.visible = fade > 0.01;
  fineGrid.material.opacity = FINE_GRID_OPACITY * fade;
  for (const m of axisMinor) {
    m.visible = fade > 0.01;
    m.material.opacity = 0.6 * fade;
  }
}
scene.add(ground);

function label(text, className) {
  const outer = document.createElement('div');
  const inner = document.createElement('span');
  inner.className = className;
  inner.textContent = text;
  outer.appendChild(inner);
  return new CSS2DObject(outer);
}

// Start marker, scaled with the camera so it stays readable. The axes are
// separate (below), drawn in true metres.
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
// World axes through the start: X forward, Y left, Z up. The ticks sit on the
// same whole metres as the floor squares, with a number on each, so a position
// can be read straight off. They grow in 5 m steps if the board goes past an end.

const AXES = [
  { name: 'X · Forward', dir: toScene(1, 0, 0), tick: toScene(0, 1, 0), color: new THREE.Color('#f87171'), css: '#f87171' },
  { name: 'Y · Left', dir: toScene(0, 1, 0), tick: toScene(1, 0, 0), color: new THREE.Color('#4ade80'), css: '#4ade80' },
  { name: 'Z · Up', dir: toScene(0, 0, 1), tick: toScene(1, 0, 0), color: new THREE.Color('#60a5fa'), css: '#60a5fa' },
];
const axesGroup = new THREE.Group();
scene.add(axesGroup);
let axisHalf = { h: 10, v: 5 }; // metres each way: horizontal axes, vertical axis
let axisTicks = [];             // { obj, m }: the number labels, thinned out with distance
let axisCones = [];             // arrowheads, kept a readable size
let axisNames = [];             // the X / Y / Z name labels
let axisMinor = [];             // the 10 cm tick sets, faded with the fine grid
let axesOn = true;              // the Axes button; remembered between visits
try {
  axesOn = localStorage.getItem('imu.axes') !== 'off';
} catch (e) { /* storage can be blocked; the default is on */ }

// Show or hide the axes, their ticks, numbers and names.
function setAxes(on) {
  axesOn = on;
  axesGroup.visible = on;
  const button = document.getElementById('btn-axes');
  if (button) button.classList.toggle('on', on);
  try {
    localStorage.setItem('imu.axes', on ? 'on' : 'off');
  } catch (e) { /* not remembered, still works */ }
}
const axisLines = [];

function clearAxes() {
  for (const l of axisLines) disposeLine(l);
  axisLines.length = 0;
  for (const child of [...axesGroup.children]) {
    axesGroup.remove(child);
    if (child.isCSS2DObject) child.element.remove();
    if (child.geometry) child.geometry.dispose();
    if (child.material) child.material.dispose();
  }
  axisTicks = [];
  axisCones = [];
  axisNames = [];
  axisMinor = [];
}

function buildAxes() {
  clearAxes();
  AXES.forEach((a, i) => {
    const half = i === 2 ? axisHalf.v : axisHalf.h;
    const L = half * 100; // cm each way
    const zero = new THREE.Vector3();
    const neg = a.dir.clone().multiplyScalar(-L);
    const pos = a.dir.clone().multiplyScalar(L);
    const faint = a.color.clone().multiplyScalar(0.55);

    const back = fatLine([neg, zero], [faint, faint], 1.6, 0.45);
    const fwd = fatLine([zero, pos], [a.color, a.color], 2.4, 0.95);
    axesGroup.add(back, fwd);
    axisLines.push(back, fwd);

    // a tick and a number on every metre, the ticks longer on every fifth
    const verts = [];
    for (let m = -half; m <= half; m++) {
      if (m === 0) continue;
      const len = m % 5 === 0 ? 10 : 5;
      const p = a.dir.clone().multiplyScalar(m * 100);
      verts.push(p.clone().addScaledVector(a.tick, -len), p.clone().addScaledVector(a.tick, len));
      const tag = label(String(m), 'axis-tick');
      tag.element.firstChild.style.color = a.css;
      tag.position.copy(p).addScaledVector(a.tick, 16);
      axesGroup.add(tag);
      axisTicks.push({ obj: tag, m });
    }
    axesGroup.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(verts),
      new THREE.LineBasicMaterial({ color: a.color, transparent: true, opacity: 0.75 })));

    // a short tick every 10 cm between the metres, shown only when zoomed in
    const minorVerts = [];
    for (let k = -half * 10; k <= half * 10; k++) {
      if (k % 10 === 0) continue;
      const p = a.dir.clone().multiplyScalar(k * 10);
      minorVerts.push(p.clone().addScaledVector(a.tick, -2.5), p.clone().addScaledVector(a.tick, 2.5));
    }
    const minor = new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(minorVerts),
      new THREE.LineBasicMaterial({ color: a.color, transparent: true, opacity: 0 }));
    minor.visible = false;
    axesGroup.add(minor);
    axisMinor.push(minor);

    // arrowhead and name at the positive end
    const cone = new THREE.Mesh(new THREE.ConeGeometry(2.2, 9, 16), new THREE.MeshBasicMaterial({ color: a.color }));
    cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), a.dir);
    cone.position.copy(pos);
    axesGroup.add(cone);
    axisCones.push(cone);
    const name = label(a.name, 'axis-name');
    name.element.firstChild.style.color = a.css;
    name.position.copy(pos).addScaledVector(a.dir, 40);
    axesGroup.add(name);
    axisNames.push(name);
  });
}

// Lengthen the axes when a position (cm, earth frame) is near an end.
function ensureAxes(x, y, z) {
  const h = Math.max(Math.abs(x), Math.abs(y)) / 100;
  const v = Math.abs(z) / 100;
  let H = axisHalf.h;
  let V = axisHalf.v;
  while (h > H - 1) H += 5;
  while (v > V - 1) V += 5;
  if (H !== axisHalf.h || V !== axisHalf.v) {
    axisHalf = { h: H, v: V };
    buildAxes();
  }
}
buildAxes();

// ---------------------------------------------------------------------------
// Finished moves

let moves = []; // { n, start, end, text, points, group, line, glow, cone, label }
const paths = new Map(); // n -> path from @PATH, earth cm

function addMove(n, start, end, text) {
  const raw = paths.get(n) ?? [start, end];
  paths.delete(n);
  for (const p of raw) ensureAxes(p[0], p[1], p[2]);
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
  ensureAxes(v[0], v[1], v[2]);
  setReadouts();
  setHeading(v.slice(3, 6), v.slice(6, 9), v.slice(9, 12));
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

// The acceleration the board is experiencing, as the firmware works it out
// (gravity removed, so about 0 at rest). The bar is logarithmic so a gentle
// push and a hard knock both show, up to the sensor's 24 g limit.
const ACC_LIMIT_G = 24;

// The Accel button: show or hide the whole readout. Remembered between visits.
let accelShown = true;
try {
  accelShown = localStorage.getItem('imu.accel') !== 'off';
} catch (e) { /* storage blocked; the default is on */ }

function setAccelShown(on) {
  accelShown = on;
  document.getElementById('accel').style.display = on ? '' : 'none';
  document.getElementById('btn-accel').classList.toggle('on', on);
  try {
    localStorage.setItem('imu.accel', on ? 'on' : 'off');
  } catch (e) { /* not remembered, still works */ }
}
const accFraction = (g) => Math.min(1, Math.log2(1 + Math.max(0, g)) / Math.log2(1 + ACC_LIMIT_G));
const signed2 = (v) => `${v >= 0 ? '+' : '−'}${Math.abs(v).toFixed(2)}`;

function setAccel([forward, left, up, linear, felt, hold]) {
  $('acc-lin').textContent = linear.toFixed(2);
  $('acc-hold').textContent = hold.toFixed(1);
  $('acc-x').textContent = signed2(forward);
  $('acc-y').textContent = signed2(left);
  $('acc-z').textContent = signed2(up);
  $('acc-felt').textContent = felt.toFixed(2);
  const fill = $('acc-fill');
  fill.style.width = `${(accFraction(linear) * 100).toFixed(1)}%`;
  // green while gentle, amber for a hard push, red as it nears the sensor's limit
  fill.style.background = linear < 1 ? '#34d399' : linear < 8 ? '#fbbf24' : '#f87171';
}

// Which edge of the board counts as its front: "<axis index>:<sign>", e.g. "2:-1"
// is -z. The default, +x, is the firmware's own heading. On a foot the board's x
// axis can point almost straight down, where yaw means nothing, so this can be
// changed to the edge that points along the toes. It is remembered.
let frontAxisChoice = '0:1';
try {
  frontAxisChoice = localStorage.getItem('imu.front') || '0:1';
} catch (e) { /* storage blocked; the default is fine */ }

// Yaw and tilt of that edge, from the board's axes in the earth frame (the
// columns of the rotation matrix, as the firmware sends them). For +x the yaw
// is exactly the heading the firmware prints. Yaw is measured from the forward
// direction set with F, turning anticlockwise (to the left), and shown as a
// full 0-360: 0 forward, 90 left, 180 behind, 270 right. Tilt is how far the
// edge points above (+) or below (-) the horizontal.
let lastAxes = null; // the latest board axes, so a changed front edge shows at once
function setHeading(xAxis, yAxis, zAxis) {
  lastAxes = [xAxis, yAxis, zAxis];
  const deg = 180 / Math.PI;
  const [index, sign] = frontAxisChoice.split(':').map(Number);
  const edge = [xAxis, yAxis, zAxis][index].map((c) => c * sign);
  const yaw = Math.atan2(edge[1], edge[0]) * deg;
  const tilt = Math.asin(Math.max(-1, Math.min(1, edge[2]))) * deg;
  const steep = Math.abs(tilt) > 75; // nearly straight up or down: yaw is meaningless
  // 0-360 on the card; the note keeps the plain "turned 30 left / 30 right".
  const yawFull = (((Math.round(yaw * 10) / 10) % 360) + 360) % 360;
  $('yaw').textContent = yawFull.toFixed(1);
  $('tilt').textContent = signed(tilt);
  $('yaw-note').textContent = steep ? 'edge is vertical: pick another front edge'
    : Math.abs(yaw) < 1 ? 'facing forward'
    : `turned ${Math.abs(yaw).toFixed(0)}° ${yaw > 0 ? 'left' : 'right'}`;
  const arrow = document.getElementById('dial-arrow');
  // On the dial, forward is up and left is anticlockwise, like the top view.
  arrow.setAttribute('transform', `rotate(${-yaw.toFixed(1)})`);
  arrow.style.opacity = steep ? '0.25' : '1';
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
    const parsed = /:\s*([\d.]+) cm (.+?), in ([\d.]+) s(?:, peak ([\d.]+) g)?/.exec(m.text);
    const li = document.createElement('li');
    if (age === 0) li.className = 'latest';
    li.style.opacity = String(Math.max(0.3, 1 - age * 0.09));
    const icon = moveIcon(m.start, m.end);
    li.innerHTML = `
      <div class="move-icon"><span style="transform: rotate(${icon.angle.toFixed(0)}deg)">${icon.glyph}</span></div>
      <div><div class="move-dir"></div><div class="move-sub"></div></div>
      <div class="move-right"><div class="move-dist"></div><div class="move-acc"></div></div>`;
    li.querySelector('.move-dir').textContent = parsed ? parsed[2] : m.text;
    li.querySelector('.move-sub').textContent = parsed ? `Step ${m.step} · ${parsed[3]} s` : `Step ${m.step}`;
    li.querySelector('.move-dist').textContent = parsed ? `${parsed[1]} cm` : '';
    // The hardest push during the step, coloured like the acceleration bar.
    const acc = li.querySelector('.move-acc');
    if (parsed && parsed[4] !== undefined) {
      const g = Number(parsed[4]);
      acc.textContent = `peak ${g.toFixed(1)} g`;
      acc.classList.add(g < 1 ? 'low' : g < 8 ? 'mid' : 'high');
    }
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
  } else if (kind === 'ACC') {
    setAccel(rest.trim().split(/\s+/).map(Number));
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
$('btn-axes').addEventListener('click', () => setAxes(!axesOn));
$('btn-accel').addEventListener('click', () => setAccelShown(!accelShown));
setAccelShown(accelShown); // apply the remembered choice
setAxes(axesOn); // apply the remembered choice
const frontSelect = $('front-axis');
frontSelect.value = frontAxisChoice;
frontSelect.addEventListener('change', () => {
  frontAxisChoice = frontSelect.value;
  try {
    localStorage.setItem('imu.front', frontAxisChoice);
  } catch (e) { /* not remembered, still works */ }
  if (lastAxes) setHeading(...lastAxes);
});
// Typing in the selector must not trigger the single-key shortcuts below.
frontSelect.addEventListener('keydown', (e) => e.stopPropagation());
addEventListener('keydown', (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const key = e.key.toLowerCase();
  if (key === 'f' || key === 'z' || key === 'r') send(key);
  else if (key === 'a') setFollow(!follow);
  else if (key === 'w') fitAll();
  else if (key === 't') topView();
  else if (key === '3') view3d();
  else if (key === 'x') setAxes(!axesOn);
  else if (key === 'g') setAccelShown(!accelShown);
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
  // 10 cm lines only when the inset is a few metres wide (the main view sets it
  // again at the start of every frame).
  setFineGrid(THREE.MathUtils.clamp((500 - half * aspect * 2) / 250, 0, 1));
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
  setFineGrid(THREE.MathUtils.clamp((1600 - distance) / 1000, 0, 1));
  const tickStep = distance < 1500 ? 1 : distance < 3500 ? 2 : distance < 8000 ? 5 : 10;
  for (const t of axisTicks) t.obj.visible = axesOn && t.m % tickStep === 0;
  for (const n of axisNames) n.visible = axesOn;
  for (const c of axisCones) c.scale.setScalar(s * 0.5); // smaller than the board's, so the Z one isn't a blob from above
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
