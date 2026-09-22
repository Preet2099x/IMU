import * as THREE from 'three';

// Body frame: +X forward (red), +Y left (green), +Z up (blue), the chip's own axes.
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
  // BoxGeometry face order: +x, -x, +y, -y, +z, -z
  g.add(new THREE.Mesh(new THREE.BoxGeometry(6.4, 4.2, 0.2), [edge, edge, edge, edge, top, under]));

  g.add(box(1.1, 1.1, 0.22, 0x14171c, 0.7, -0.6, 0.21, { metalness: 0.3 })); // BMI088
  g.add(box(0.7, 0.7, 0.14, 0x1b1f26, -1.5, 0.8, 0.17, { metalness: 0.3 })); // BMM350
  for (let i = 0; i < 6; i++) g.add(box(0.28, 0.28, 0.06, 0xd9b44a, -2.7 + i * 0.5, -1.7, 0.13, { metalness: 0.8 }));

  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.42, 1.0, 20),
    new THREE.MeshStandardMaterial({ color: 0xffb020, roughness: 0.4 }),
  );
  nose.rotation.z = -Math.PI / 2; // cone axis +Y -> +X
  nose.position.set(3.7, 0, 0);
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
