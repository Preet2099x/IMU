// Quaternion helpers, stored as [w, x, y, z].
//
// Conventions match the firmware in src/main.cpp: q rotates the sensor frame
// into the earth frame (z up), and the printed angles are the ZYX Euler angles
// of that rotation: q = Rz(yaw) * Ry(pitch) * Rx(roll), all right-handed.

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;

export function eulerToQuat(rollDeg, pitchDeg, yawDeg) {
  const cr = Math.cos(rollDeg * DEG / 2), sr = Math.sin(rollDeg * DEG / 2);
  const cp = Math.cos(pitchDeg * DEG / 2), sp = Math.sin(pitchDeg * DEG / 2);
  const cy = Math.cos(yawDeg * DEG / 2), sy = Math.sin(yawDeg * DEG / 2);
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

// Same extraction as quaternionToEuler() in the firmware.
export function quatToEuler([w, x, y, z]) {
  const sinp = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * RAD,
    pitch: Math.asin(sinp) * RAD,
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * RAD,
  };
}

export function mul([aw, ax, ay, az], [bw, bx, by, bz]) {
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

export const conj = ([w, x, y, z]) => [w, -x, -y, -z];

export function normalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return q.map((v) => v / n);
}

export function slerp(a, b, t) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let bb = b;
  if (d < 0) { bb = b.map((v) => -v); d = -d; } // take the short way round
  if (d > 0.9995) return normalize(a.map((v, i) => v + t * (bb[i] - v)));
  const theta = Math.acos(d);
  const s = Math.sin(theta);
  const wa = Math.sin((1 - t) * theta) / s;
  const wb = Math.sin(t * theta) / s;
  return a.map((v, i) => wa * v + wb * bb[i]);
}

// Orientation of `cur` measured from the pose `ref`, which counts as flat with
// heading 0 (even if it was upside down or facing any way). The turn between
// the two poses is a rotation in the earth frame, so it is re-expressed in a
// room turned to face the reference heading. That way pitching the board reads
// as pitch, and turning it reads as heading change, whatever it started facing.
export function relativeTo(cur, ref) {
  const faceRef = eulerToQuat(0, 0, quatToEuler(ref).yaw); // rotation about vertical by the reference heading
  return normalize(mul(mul(mul(conj(faceRef), cur), conj(ref)), faceRef));
}

export const wrap360 = (deg) => ((deg % 360) + 360) % 360;
