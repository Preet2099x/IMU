// Run with: node mapping/web/test_orientation.mjs
import assert from 'node:assert/strict';
import { eulerToQuat, quatToEuler, mul, conj, slerp, relativeTo, wrap360 } from './orientation.js';

const near = (a, b, tol = 1e-6, msg = '') =>
  assert.ok(Math.abs(a - b) <= tol, `${msg} expected ${b}, got ${a}`);

const angleDiff = (a, b) => ((a - b + 540) % 360) - 180;

// Euler -> quaternion -> Euler round trip, away from gimbal lock.
for (const [r, p, y] of [[0, 0, 0], [30, -20, 100], [-135.9, -6.3, 0.2], [170, 45, -170], [12, 80, 300]]) {
  const e = quatToEuler(eulerToQuat(r, p, y));
  near(angleDiff(e.roll, r), 0, 1e-6, `roll ${r}/${p}/${y}`);
  near(e.pitch, p, 1e-6, `pitch ${r}/${p}/${y}`);
  near(angleDiff(e.yaw, y), 0, 1e-6, `yaw ${r}/${p}/${y}`);
}

// Same quaternion the firmware seeds from a flat-ish accel reading (yaw 0).
{
  const roll = 25 * Math.PI / 180, pitch = -10 * Math.PI / 180;
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2), cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const fw = [cr * cp, sr * cp, cr * sp, -sr * sp];
  const q = eulerToQuat(25, -10, 0);
  fw.forEach((v, i) => near(q[i], v, 1e-12, `seed q${i}`));
}

// Unit length is preserved by composition.
{
  const q = mul(eulerToQuat(10, 20, 30), eulerToQuat(-5, 40, 200));
  near(Math.hypot(...q), 1, 1e-12, 'norm');
}

// Reference pose = flat: the starting pose shows as level, heading 0.
{
  const ref = eulerToQuat(-135.9, -6.3, 0.2);
  const e = quatToEuler(relativeTo(ref, ref));
  near(e.roll, 0); near(e.pitch, 0); near(e.yaw, 0);
}

// Chip mounted upside down (roll 180 at rest). A 10 degree tilt about the
// room's y axis must still read as pitch 10 in the display, not mirrored.
{
  const ref = eulerToQuat(180, 0, 0);
  const tilt = eulerToQuat(0, 10, 0);
  const e = quatToEuler(relativeTo(mul(tilt, ref), ref));
  near(e.roll, 0, 1e-6, 'upside-down roll');
  near(e.pitch, 10, 1e-6, 'upside-down pitch');
  near(e.yaw, 0, 1e-6, 'upside-down yaw');
}

// The start pose can face any heading. Pitching or rolling the board from
// there must still read as pure pitch / pure roll, not as a diagonal tilt.
{
  const ref = eulerToQuat(0, 0, 100);
  let e = quatToEuler(relativeTo(eulerToQuat(0, 10, 100), ref));
  near(e.roll, 0, 1e-6, 'pitch@100 roll'); near(e.pitch, 10, 1e-6, 'pitch@100 pitch'); near(e.yaw, 0, 1e-6, 'pitch@100 yaw');
  e = quatToEuler(relativeTo(eulerToQuat(15, 0, 100), ref));
  near(e.roll, 15, 1e-6, 'roll@100 roll'); near(e.pitch, 0, 1e-6, 'roll@100 pitch'); near(e.yaw, 0, 1e-6, 'roll@100 yaw');
}

// Same, starting upside down and facing 100 degrees: a tilt of the whole board
// about the axis across its heading (10 degrees) still reads as pitch 10.
{
  const ref = eulerToQuat(180, 0, 100);
  const cur = mul(eulerToQuat(0, 0, 100), mul(eulerToQuat(0, 10, 0), eulerToQuat(180, 0, 0)));
  const e = quatToEuler(relativeTo(cur, ref));
  near(e.roll, 0, 1e-6, 'flipped@100 roll'); near(e.pitch, 10, 1e-6, 'flipped@100 pitch'); near(e.yaw, 0, 1e-6, 'flipped@100 yaw');
}

// Turning by 30 degrees from a 100 degree reference heading reads as yaw 30.
{
  const e = quatToEuler(relativeTo(eulerToQuat(0, 0, 130), eulerToQuat(0, 0, 100)));
  near(e.yaw, 30, 1e-6, 'relative yaw');
}

// Heading wrap: 359 -> 1 goes through 0, not the long way round.
{
  const mid = quatToEuler(slerp(eulerToQuat(0, 0, 359), eulerToQuat(0, 0, 1), 0.5));
  near(angleDiff(mid.yaw, 0), 0, 1e-6, 'slerp wrap');
}

near(wrap360(-0.1), 359.9, 1e-9);
near(wrap360(360), 0, 1e-9);
near(mul(eulerToQuat(20, 0, 0), conj(eulerToQuat(20, 0, 0)))[0], 1, 1e-12);

console.log('orientation tests passed');
