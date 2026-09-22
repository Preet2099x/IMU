// Accelerometer calibration.
//
// A real accelerometer reads gravity a little off: each axis has an offset and
// a scale error. Left uncorrected, that error looks like a steady push, so a
// board sitting still appears to accelerate and its position runs away.
//
// At rest the corrected reading must be exactly 1 g long, whichever way the
// board points. Collecting still readings pointing in different directions and
// fitting offset and scale per axis so they all land on a sphere of radius 1
// finds the correction (the usual ellipsoid-fit calibration, axis-aligned).

export const IDENTITY_CAL = { offset: [0, 0, 0], scale: [1, 1, 1] };

export const applyCal = (f, cal) => [0, 1, 2].map((i) => (f[i] - cal.offset[i]) * cal.scale[i]);

export const FACES = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']; // the axis that points up
const FACE_MIN = 0.8; // how much of the reading must be on one axis for it to count as a face

// Which face is up for this reading, or -1 if the board is tilted between faces.
export function faceOf(f) {
  const len = Math.hypot(f[0], f[1], f[2]);
  if (len < 0.5) return -1;
  for (let i = 0; i < 3; i++) {
    if (Math.abs(f[i]) / len > FACE_MIN) return i * 2 + (f[i] < 0 ? 1 : 0);
  }
  return -1;
}

// Solves A x = b by Gaussian elimination with partial pivoting.
function solve(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-9) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const k = M[r][c] / M[c][c];
      for (let j = c; j <= n; j++) M[r][j] -= k * M[c][j];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let j = r + 1; j < n; j++) s -= M[r][j] * x[j];
    x[r] = s / M[r][r];
  }
  return x;
}

// samples: still readings [x, y, z] in g, pointing in different directions.
// Fits  a_i f_i^2 + d_i f_i  summed over the axes  = 1  by least squares, then
// completes the square: offset o_i = -d_i / (2 a_i), scale s_i = sqrt(a_i / K).
export function fitAccelCal(samples) {
  if (samples.length < 6) throw new Error('Need at least 6 still readings');
  const AtA = Array.from({ length: 6 }, () => new Array(6).fill(0));
  const Atb = new Array(6).fill(0);
  for (const [x, y, z] of samples) {
    const row = [x * x, y * y, z * z, x, y, z];
    for (let i = 0; i < 6; i++) {
      Atb[i] += row[i];
      for (let j = 0; j < 6; j++) AtA[i][j] += row[i] * row[j];
    }
  }
  const sol = solve(AtA, Atb);
  if (!sol) throw new Error('The readings are too alike: turn the board to different sides');
  const a = sol.slice(0, 3), d = sol.slice(3);
  if (!a.every((v) => v > 0)) throw new Error('The readings do not fit: hold each side still and try again');
  const offset = a.map((ai, i) => -d[i] / (2 * ai));
  const K = 1 + a.reduce((s, ai, i) => s + ai * offset[i] * offset[i], 0);
  const scale = a.map((ai) => Math.sqrt(ai / K));

  const cal = { offset, scale };
  let sumSq = 0, maxErr = 0;
  for (const f of samples) {
    const c = applyCal(f, cal);
    const err = Math.hypot(c[0], c[1], c[2]) - 1;
    sumSq += err * err;
    maxErr = Math.max(maxErr, Math.abs(err));
  }
  return { ...cal, rms: Math.sqrt(sumSq / samples.length), maxErr };
}
