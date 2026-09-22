import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { buildBoard, buildAxes, makeLabel } from './model.js';

// z is up, x is front (the way the board faced when zeroed), y is left.
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

export const ROOM = { half: 1.0, height: 1.5 }; // metres: floor is 2 m x 2 m
const START = new THREE.Vector3(0, 0, 0.75); // where the board hangs when position is zero
const MODEL_SCALE = 0.036; // the board model is 6.4 units long, so about 23 cm
const TRAIL_MAX = 600;

const VIEWS = {
  iso: [2.7, -3.1, 1.9],
  top: [0, -0.01, 4.4],
  side: [0, -4.2, 0.9], // front is on the right
  rear: [-4.2, 0, 0.9], // looking the way the board faces
};

function buildRoom(scene) {
  const grid = new THREE.GridHelper(ROOM.half * 2, 20, 0x35507e, 0x1b2b47);
  grid.rotation.x = Math.PI / 2; // GridHelper is XZ by default
  scene.add(grid);

  const box = new THREE.BoxGeometry(ROOM.half * 2, ROOM.half * 2, ROOM.height);
  box.translate(0, 0, ROOM.height / 2);
  scene.add(new THREE.LineSegments(new THREE.EdgesGeometry(box), new THREE.LineBasicMaterial({ color: 0x4a6ba3, transparent: true, opacity: 0.7 })));

  const label = (text, x, y, z, color = '#a9bddd') => {
    const s = makeLabel(text, color, 0.55);
    s.position.set(x, y, z);
    scene.add(s);
  };
  const e = ROOM.half + 0.22;
  label('FRONT', e, 0, 0.03, '#ff8a8a');
  label('BACK', -e, 0, 0.03);
  label('LEFT', 0, e, 0.03, '#8ae6a3');
  label('RIGHT', 0, -e, 0.03);
  label('UP', 0, 0, ROOM.height + 0.15, '#8ab4ff');
}

export function createScene(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 100);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.7);
  controls.enableDamping = true;
  controls.minDistance = 1.2;
  controls.maxDistance = 9;

  scene.add(new THREE.AmbientLight(0xffffff, 0.75));
  const sun = new THREE.DirectionalLight(0xffffff, 2.3);
  sun.position.set(2, -1.5, 4);
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0x88aaff, 0.8);
  fill.position.set(-2, 2, 1);
  scene.add(fill);

  buildRoom(scene);

  const imu = new THREE.Group();
  const body = new THREE.Group();
  body.scale.setScalar(MODEL_SCALE);
  body.add(buildBoard());
  body.add(buildAxes(4.6));
  imu.add(body);
  scene.add(imu);

  // A line down to the floor and a mark where it lands make height easy to read.
  const dropGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  scene.add(new THREE.Line(dropGeo, new THREE.LineBasicMaterial({ color: 0x7f93b8, transparent: true, opacity: 0.8 })));
  const mark = new THREE.Mesh(new THREE.RingGeometry(0.05, 0.068, 40), new THREE.MeshBasicMaterial({ color: 0xffb020, side: THREE.DoubleSide }));
  const dot = new THREE.Mesh(new THREE.CircleGeometry(0.018, 20), new THREE.MeshBasicMaterial({ color: 0xffb020 }));
  mark.add(dot);
  scene.add(mark);

  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX * 3), 3));
  trailGeo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(TRAIL_MAX * 3), 3));
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ vertexColors: true }));
  trail.frustumCulled = false;
  scene.add(trail);
  const trailPts = [];
  const oldColor = new THREE.Color(0x24344f), newColor = new THREE.Color(0xffb020), tmp = new THREE.Color();

  const shown = new THREE.Vector3().copy(START);
  const want = new THREE.Vector3().copy(START);
  const wantQuat = new THREE.Quaternion();
  let atEdge = false;

  function setPose(pos, q) {
    want.set(START.x + pos[0], START.y + pos[1], START.z + pos[2]);
    const c = want.clone();
    want.x = THREE.MathUtils.clamp(want.x, -ROOM.half + 0.08, ROOM.half - 0.08);
    want.y = THREE.MathUtils.clamp(want.y, -ROOM.half + 0.08, ROOM.half - 0.08);
    want.z = THREE.MathUtils.clamp(want.z, 0.06, ROOM.height - 0.06);
    atEdge = !c.equals(want);
    wantQuat.set(q[1], q[2], q[3], q[0]);
  }

  function updateTrail() {
    const last = trailPts[trailPts.length - 1];
    if (!last || last.distanceToSquared(shown) > 1e-5) {
      trailPts.push(shown.clone());
      if (trailPts.length > TRAIL_MAX) trailPts.shift();
    }
    const pos = trailGeo.attributes.position.array, col = trailGeo.attributes.color.array;
    trailPts.forEach((p, i) => {
      pos.set([p.x, p.y, p.z], i * 3);
      tmp.copy(oldColor).lerp(newColor, i / Math.max(1, trailPts.length - 1));
      col.set([tmp.r, tmp.g, tmp.b], i * 3);
    });
    trailGeo.setDrawRange(0, trailPts.length);
    trailGeo.attributes.position.needsUpdate = true;
    trailGeo.attributes.color.needsUpdate = true;
  }

  function render(dt) {
    const k = 1 - Math.exp(-dt / 0.035); // a little smoothing between the 50 Hz samples
    shown.lerp(want, k);
    imu.position.copy(shown);
    imu.quaternion.slerp(wantQuat, Math.min(1, k * 1.5));
    const a = dropGeo.attributes.position;
    a.setXYZ(0, shown.x, shown.y, shown.z);
    a.setXYZ(1, shown.x, shown.y, 0.002);
    a.needsUpdate = true;
    mark.position.set(shown.x, shown.y, 0.004);
    updateTrail();
    controls.update();
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
    if (!VIEWS[name]) return;
    camera.position.set(...VIEWS[name]);
    controls.update();
  };
  setView('iso');

  return {
    setPose,
    render,
    setView,
    atEdge: () => atEdge,
    clearTrail: () => { trailPts.length = 0; },
    setTrail: (on) => { trail.visible = on; },
    // Snap to the current target (used after zeroing, so nothing glides across the room).
    snap: () => { shown.copy(want); imu.quaternion.copy(wantQuat); },
  };
}
