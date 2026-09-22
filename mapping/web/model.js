import * as THREE from 'three';

// Body frame used everywhere: +X forward (red), +Y left (green), +Z up (blue),
// the same axes as the chip, so what you see matches the numbers on serial.
export const COLORS = { x: 0xff5c5c, y: 0x5cd67c, z: 0x5c9bff };

export function makeLabel(text, color = '#dfe8f5', scale = 1.5) {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 64;
  const ctx = canvas.getContext('2d');
  ctx.font = '600 40px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillStyle = color;
  ctx.fillText(text, 64, 34);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
  sprite.scale.set(scale, scale / 2, 1);
  sprite.renderOrder = 10;
  return sprite;
}

const box = (w, h, d, color, x, y, z, extra = {}) => {
  const m = new THREE.Mesh(
    new THREE.BoxGeometry(w, h, d),
    new THREE.MeshStandardMaterial({ color, roughness: 0.5, ...extra }),
  );
  m.position.set(x, y, z);
  return m;
};

// A generic IMU board: green top with chips, slate underside, orange nose.
export function buildBoard() {
  const g = new THREE.Group();
  const edge = new THREE.MeshStandardMaterial({ color: 0x0b5c3c, roughness: 0.7 });
  const top = new THREE.MeshStandardMaterial({ color: 0x14a06b, roughness: 0.55 });
  const under = new THREE.MeshStandardMaterial({ color: 0x2b3a52, roughness: 0.7 });
  // The real board is long along Y (the nose) and short along X, so the box and
  // every part on it are laid out that way too, or a real long edge would show
  // up as the model's short edge and vice versa.
  // BoxGeometry face order: +x, -x, +y, -y, +z, -z
  g.add(new THREE.Mesh(new THREE.BoxGeometry(4.2, 6.4, 0.2), [edge, edge, edge, edge, top, under]));

  g.add(box(1.1, 1.1, 0.22, 0x14171c, -0.6, 0.7, 0.21, { metalness: 0.3 })); // BMI088
  g.add(box(0.7, 0.7, 0.14, 0x1b1f26, 0.8, -1.5, 0.17, { metalness: 0.3 })); // BMM350
  for (let i = 0; i < 6; i++) g.add(box(0.28, 0.28, 0.06, 0xd9b44a, -1.7, -2.7 + i * 0.5, 0.13, { metalness: 0.8 }));

  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.42, 1.0, 20),
    new THREE.MeshStandardMaterial({ color: 0xffb020, roughness: 0.4 }),
  );
  nose.position.set(0, 3.7, 0); // cone axis is already +Y, so no rotation needed
  g.add(nose);
  return g;
}

export function buildAxes(length = 4.4) {
  const g = new THREE.Group();
  const dirs = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
  for (const [name, d] of Object.entries(dirs)) {
    const v = new THREE.Vector3(...d);
    g.add(new THREE.ArrowHelper(v, new THREE.Vector3(), length, COLORS[name], 0.7, 0.4));
    const label = makeLabel(name.toUpperCase(), '#' + COLORS[name].toString(16), 1.3);
    label.position.copy(v).multiplyScalar(length + 0.8);
    g.add(label);
  }
  return g;
}

const ring = (radius, color, tilt) => {
  const geo = new THREE.TorusGeometry(radius, 0.09, 10, 128);
  tilt(geo);
  return new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color, roughness: 0.35, metalness: 0.4 }));
};

const pin = (color, from, to) => {
  const dir = new THREE.Vector3().subVectors(to, from);
  const m = new THREE.Mesh(
    new THREE.CylinderGeometry(0.16, 0.16, dir.length(), 12),
    new THREE.MeshStandardMaterial({ color, roughness: 0.4 }),
  );
  m.position.copy(from).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
  return m;
};

// Three nested rings, one per Euler angle. The outer ring turns with yaw about
// the vertical, the middle one pitches about the (turned) Y axis, the inner
// one rolls about X, and the board is fixed to the inner ring.
export function buildGimbal() {
  const yaw = new THREE.Group();
  const pitch = new THREE.Group();
  const roll = new THREE.Group();
  yaw.add(pitch);
  pitch.add(roll);

  const R = { yaw: 5.8, pitch: 5.2, roll: 4.6 };
  yaw.add(ring(R.yaw, COLORS.z, (g) => g.rotateY(Math.PI / 2))); // circle in the YZ plane
  pitch.add(ring(R.pitch, COLORS.y, () => {})); // circle in the XY plane
  roll.add(ring(R.roll, COLORS.x, (g) => g.rotateX(Math.PI / 2))); // circle in the XZ plane

  for (const s of [-1, 1]) {
    yaw.add(pin(COLORS.y, new THREE.Vector3(0, s * R.pitch, 0), new THREE.Vector3(0, s * R.yaw, 0)));
    pitch.add(pin(COLORS.x, new THREE.Vector3(s * R.roll, 0, 0), new THREE.Vector3(s * R.pitch, 0, 0)));
  }
  return { yaw, pitch, roll, radius: R.yaw };
}
