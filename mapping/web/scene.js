import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { buildBoard, buildAxes, buildGimbal, makeLabel, COLORS } from './model.js';
import { quatToEuler } from './orientation.js';

// The room is z-up, like the sensor's earth frame.
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const HOVER = 6.6; // height of the board above the floor
const RAD = Math.PI / 180;
const TRAIL_MAX = 360;
const NOSE_TIP = new THREE.Vector3(4.2, 0, 0);

const VIEWS = {
  iso: [15, -17, 13],
  top: [0, -0.01, 34],
  side: [0, -30, HOVER + 1], // nose points right: pitch reads as nose up/down
  rear: [-30, 0, HOVER + 1], // looking along the nose: roll reads as a tilt
};

function buildFloor(scene) {
  const grid = new THREE.GridHelper(30, 30, 0x2e4468, 0x1a2841);
  grid.rotation.x = Math.PI / 2; // GridHelper is XZ by default
  scene.add(grid);

  const pts = [];
  for (let i = 0; i <= 128; i++) pts.push(new THREE.Vector3(Math.cos((i / 128) * 2 * Math.PI) * 10, Math.sin((i / 128) * 2 * Math.PI) * 10, 0.02));
  scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0x8fa6c9 })));

  const ticks = [];
  for (let deg = 0; deg < 360; deg += 10) {
    const major = deg % 30 === 0;
    const a = deg * RAD, r0 = major ? 9.3 : 9.6;
    ticks.push(Math.cos(a) * r0, Math.sin(a) * r0, 0.02, Math.cos(a) * 10, Math.sin(a) * 10, 0.02);
    if (major) {
      const big = deg % 90 === 0;
      const label = makeLabel(`${deg}°`, big ? '#f1f5ff' : '#8fa6c9', big ? 1.9 : 1.3);
      const r = big ? 12.2 : 11.4; // keep the big labels clear of the corner posts
      label.position.set(Math.cos(a) * r, Math.sin(a) * r, 0.1);
      scene.add(label);
    }
  }
  const tickGeo = new THREE.BufferGeometry();
  tickGeo.setAttribute('position', new THREE.Float32BufferAttribute(ticks, 3));
  scene.add(new THREE.LineSegments(tickGeo, new THREE.LineBasicMaterial({ color: 0x8fa6c9 })));

  const post = { 0: COLORS.x, 90: COLORS.y, 180: 0x66758c, 270: 0x66758c };
  for (const [deg, color] of Object.entries(post)) {
    const p = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 1.6, 12), new THREE.MeshStandardMaterial({ color, roughness: 0.5 }));
    p.rotation.x = Math.PI / 2;
    p.position.set(Math.cos(deg * RAD) * 10, Math.sin(deg * RAD) * 10, 0.8);
    scene.add(p);
  }
}

export function createScene(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 300);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, HOVER * 0.7);
  controls.enableDamping = true;
  controls.minDistance = 12;
  controls.maxDistance = 70;

  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const sun = new THREE.DirectionalLight(0xffffff, 2.4);
  sun.position.set(8, -6, 16);
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0x88aaff, 0.9);
  fill.position.set(-10, 8, 4);
  scene.add(fill);

  buildFloor(scene);

  // Rig: gimbal rings (optional) around the board, hovering above the floor.
  const rig = new THREE.Group();
  rig.position.z = HOVER;
  scene.add(rig);
  const gimbal = buildGimbal();
  rig.add(gimbal.yaw);
  const board = buildBoard();
  board.add(buildAxes());
  gimbal.roll.add(board);

  const gyroArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.3), 3, 0xd65cff, 0.8, 0.5);
  gyroArrow.visible = false;
  board.add(gyroArrow);

  // Flat copy of the board on the floor, so tilt and turn are easy to read.
  const shadow = new THREE.Mesh(
    new THREE.BoxGeometry(6.4, 4.2, 0.2),
    new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.4, depthWrite: false }),
  );
  shadow.matrixAutoUpdate = false;
  scene.add(shadow);
  const flatten = new THREE.Matrix4().set(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0.03, 0, 0, 0, 1);

  // Floor needle showing where the nose points, read against the compass ring.
  const needle = new THREE.Group();
  needle.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.04), 9.6, COLORS.z, 1.1, 0.6));
  scene.add(needle);

  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX * 3), 3));
  trailGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX * 3), 3));
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ vertexColors: true }));
  trail.frustumCulled = false;
  scene.add(trail);
  const trailPts = [];
  const oldColor = new THREE.Color(0x24344f), newColor = new THREE.Color(0xffb020), tmpColor = new THREE.Color();

  const pose = { roll: 0, pitch: 0, yaw: 0 };
  const opts = { gimbal: true, trail: true, gyro: true };
  const tip = new THREE.Vector3();

  function setPose(q) {
    const e = quatToEuler(q);
    Object.assign(pose, e);
    needle.rotation.z = e.yaw * RAD;
    gimbal.yaw.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), e.yaw * RAD);
    gimbal.pitch.quaternion.setFromAxisAngle(new THREE.Vector3(0, 1, 0), e.pitch * RAD);
    gimbal.roll.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), e.roll * RAD);
  }

  function updateTrail() {
    rig.updateMatrixWorld(true);
    tip.copy(NOSE_TIP);
    board.localToWorld(tip);
    const last = trailPts[trailPts.length - 1];
    if (!last || last.distanceToSquared(tip) > 0.0009) {
      trailPts.push(tip.clone());
      if (trailPts.length > TRAIL_MAX) trailPts.shift();
    }
    const pos = trailGeo.attributes.position.array, col = trailGeo.attributes.color.array;
    trailPts.forEach((p, i) => {
      pos.set([p.x, p.y, p.z], i * 3);
      tmpColor.copy(oldColor).lerp(newColor, i / Math.max(1, trailPts.length - 1));
      col.set([tmpColor.r, tmpColor.g, tmpColor.b], i * 3);
    });
    trailGeo.setDrawRange(0, trailPts.length);
    trailGeo.attributes.position.needsUpdate = true;
    trailGeo.attributes.color.needsUpdate = true;
  }

  function render() {
    controls.update();
    rig.updateMatrixWorld(true);
    shadow.matrix.multiplyMatrices(flatten, board.matrixWorld);
    shadow.matrixWorldNeedsUpdate = true;
    if (opts.trail) updateTrail();
    renderer.render(scene, camera);
  }

  function resize() {
    const w = container.clientWidth, h = container.clientHeight;
    renderer.setSize(w, h);
    camera.aspect = w / Math.max(1, h);
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(container);
  resize();

  const setView = (name) => {
    const p = VIEWS[name];
    if (!p) return;
    camera.position.set(...p);
    controls.update();
  };
  setView('iso');

  return {
    setPose,
    getPose: () => ({ ...pose }),
    render,
    setView,
    setGyro(v) {
      const mag = v ? Math.hypot(...v) : 0;
      gyroArrow.visible = opts.gyro && mag > 4;
      if (!gyroArrow.visible) return;
      gyroArrow.setDirection(new THREE.Vector3(...v).normalize());
      gyroArrow.setLength(Math.min(8, 1.5 + mag / 25), 0.8, 0.5);
    },
    setOption(name, on) {
      opts[name] = on;
      if (name === 'gimbal') {
        for (const group of [gimbal.yaw, gimbal.pitch, gimbal.roll]) {
          for (const child of group.children) if (child.isMesh) child.visible = on;
        }
      }
      if (name === 'trail') trail.visible = on;
      if (name === 'gyro' && !on) gyroArrow.visible = false;
    },
    clearTrail() { trailPts.length = 0; },
    domElement: renderer.domElement,
  };
}
