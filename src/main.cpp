#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// I2C addresses, confirmed by chip-ID readback on this board:
//   0x18 = BMI088 accelerometer (chip ID 0x1E)
//   0x68 = BMI088 gyroscope     (chip ID 0x0F)
//   0x15 = BMM350 magnetometer  (ADSEL high)
constexpr uint8_t ACC_ADDR = 0x18;
constexpr uint8_t GYR_ADDR = 0x68;
constexpr uint8_t MAG_ADDR = 0x15;

// Scale factors must match the ranges configured in setup().
// Only hard moves (over 0.5 g) count, and a hand jab can peak at several g, so
// leave plenty of headroom above that.
constexpr float ACC_RANGE_G = 12.0f;                        // ACC_RANGE = 0x02
constexpr float ACC_LSB_PER_G = 32768.0f / ACC_RANGE_G;
// A narrow gyro range clips on taps and bumps, and clipped spikes integrate
// into a permanent heading error, so run at full scale.
constexpr float GYR_RANGE_DPS = 2000.0f;                    // GYRO_RANGE = 0x00
constexpr float GYR_LSB_PER_DPS = 32768.0f / GYR_RANGE_DPS;

// Every gyro sample is pulled from the BMI088's on-chip FIFO and integrated
// over its own sample period. Polling the rate register instead means a loop
// stall stretches one reading across the gap, and during fast motion that
// integrates into a permanent heading error.
constexpr float GYRO_ODR_HZ = 400.0f;     // GYRO_BANDWIDTH 0x03: ODR 400 Hz, 47 Hz filter
constexpr uint8_t GYRO_FIFO_MODE = 0x40;  // FIFO_CONFIG_1: stop-at-full, overrun flagged
constexpr int GYRO_FIFO_CHUNK_FRAMES = 22; // 132 bytes, fits Teensy 4 Wire's 136-byte buffer
constexpr uint32_t MAG_READ_INTERVAL_US = 5000;

// Gyro bias drifts as the BMI088 warms up, and accel can't observe yaw, so any
// leftover Z bias integrates straight into heading. The bias is measured over
// any one-second window in which the gyro holds steady, judged by its spread
// rather than its value: a bias learned while the board was being handled is
// off by however fast it turned, and once off it reads as a constant rotation
// that a value-based "is it still?" check would never let it unlearn.
constexpr float GYRO_STEADY_STD_DPS = 0.4f; // per-axis spread; a still board shows ~0.2
constexpr float GYRO_MAX_BIAS_DPS = 3.0f;   // BMI088 zero-rate offset is about ±1 dps
constexpr int GYRO_BIAS_WINDOW = (int)GYRO_ODR_HZ;

// At rest, heading is held fixed (see holdYaw in mahonyUpdate).
constexpr float STILL_GYRO_DPS = 0.5f;       // smoothed rate below this counts as still
constexpr float STILL_ACCEL_TOL_G = 0.1f;    // |accel| must be this close to 1g
constexpr float STILL_FILTER_TAU_S = 0.1f;   // smoothing of the rate used for detection
constexpr float STILL_TIME_REQUIRED_S = 1.0f;
constexpr uint32_t PRINT_INTERVAL_MS = 1000; // slow, so move reports stand out

// Mahony proportional gain: how hard accel/mag pull the estimate toward
// vertical/north. There is deliberately no integral term: bias is learned
// explicitly while at rest, and an integral learned in one orientation turns
// into a false heading rate once the board is turned.
constexpr float TWO_KP = 2.0f * 0.5f;

// While the board is being bumped or moved, accel measures more than gravity.
// Feeding that in tilts the estimate and can leak into heading, so skip it.
constexpr float ACCEL_REJECT_TOL_G = 0.15f;

// Hard/soft iron correction, in raw magnetometer counts, filled in by the
// startup calibration. Yaw is only meaningful once these are valid.
bool useMagInFusion = false;
float magOffset[3] = {0.0f, 0.0f, 0.0f};
float magScale[3] = {1.0f, 1.0f, 1.0f};

// The BMM350 is a separate die from the BMI088 and need not share its axis
// orientation. Solved for during calibration.
int magAxisMap[3] = {0, 1, 2};
float magAxisSign[3] = {1.0f, 1.0f, 1.0f};

constexpr uint32_t MAG_CAL_DURATION_MS = 30000;
constexpr uint32_t MAG_CAL_SAMPLE_INTERVAL_MS = 20; // 50 Hz
constexpr int MAG_CAL_MAX_SAMPLES = 1600;

struct CalSample {
  float a[3];
  float m[3];
};
CalSample calSamples[MAG_CAL_MAX_SAMPLES];
int calSampleCount = 0;

float gyroBiasDps[3] = {0.0f, 0.0f, 0.0f};
float gravityG = 1.0f; // |accel| of the still board, measured with the gyro bias
uint32_t gyroSaturations = 0; // samples that hit the gyro's ±2000 dps limit
float magNoiseFloor[3] = {0.0f, 0.0f, 0.0f};

// Quaternion of the sensor frame relative to the earth frame.
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void setupBMI088Accel() {
  writeReg(ACC_ADDR, 0x7E, 0xB6); // ACC_SOFTRESET
  delay(5);
  writeReg(ACC_ADDR, 0x7D, 0x04); // ACC_PWR_CTRL: enable accelerometer
  delay(5);
  writeReg(ACC_ADDR, 0x7C, 0x00); // ACC_PWR_CONF: active mode
  writeReg(ACC_ADDR, 0x40, 0xAA); // ACC_CONF: normal filter, ODR 400 Hz (matches the gyro)
  writeReg(ACC_ADDR, 0x41, 0x02); // ACC_RANGE: ±12g
  delay(50);
}

void setupBMI088Gyro() {
  writeReg(GYR_ADDR, 0x14, 0xB6); // GYRO_SOFTRESET
  delay(35);
  writeReg(GYR_ADDR, 0x0F, 0x00);           // GYRO_RANGE: ±2000dps
  writeReg(GYR_ADDR, 0x10, 0x03);           // GYRO_BANDWIDTH: ODR 400 Hz, 47 Hz filter
  writeReg(GYR_ADDR, 0x11, 0x00);           // GYRO_LPM1: normal mode
  writeReg(GYR_ADDR, 0x15, 0x40);           // GYRO_INT_CTRL: fifo_en
  writeReg(GYR_ADDR, 0x3D, 0x00);           // FIFO_CONFIG_0: no tags (6-byte frames)
  writeReg(GYR_ADDR, 0x3E, GYRO_FIFO_MODE); // FIFO_CONFIG_1: FIFO mode
  delay(30);
}

// Writing FIFO_CONFIG_1 clears the buffer and its overrun flag.
void flushGyroFifo() {
  writeReg(GYR_ADDR, 0x3E, GYRO_FIFO_MODE);
}

// Drains up to maxFrames raw samples from the gyro FIFO. Returns the count;
// overrun reports that samples were lost since the last flush.
int readGyroFifo(int16_t frames[][3], int maxFrames, bool &overrun) {
  uint8_t status;
  overrun = false;
  if (!readBytes(GYR_ADDR, 0x0E, &status, 1)) return 0;
  overrun = status & 0x80;

  int available = status & 0x7F;
  if (available > maxFrames) available = maxFrames;

  int got = 0;
  uint8_t buf[GYRO_FIFO_CHUNK_FRAMES * 6];
  while (got < available) {
    int chunk = min(available - got, GYRO_FIFO_CHUNK_FRAMES);
    // FIFO_DATA doesn't advance the register address, so a burst read keeps
    // popping consecutive frames.
    if (!readBytes(GYR_ADDR, 0x3F, buf, chunk * 6)) break;
    for (int i = 0; i < chunk; i++) {
      const uint8_t *b = buf + i * 6;
      frames[got + i][0] = (int16_t)(b[0] | (b[1] << 8));
      frames[got + i][1] = (int16_t)(b[2] | (b[3] << 8));
      frames[got + i][2] = (int16_t)(b[4] | (b[5] << 8));
    }
    got += chunk;
  }
  return got;
}

void setupBMM350() {
  writeReg(MAG_ADDR, 0x7E, 0xB6); // CMD: soft reset
  delay(50);
  writeReg(MAG_ADDR, 0x06, 0x01); // PMU_CMD: normal mode
  writeReg(MAG_ADDR, 0x04, 0x04); // PMU_CMD_AGGR_SET: ODR/averaging
  delay(10);
  writeReg(MAG_ADDR, 0x06, 0x07); // PMU_CMD: BR (magnetic reset)
  delay(20);
}

bool readAccelG(float &x, float &y, float &z) {
  uint8_t b[6];
  if (!readBytes(ACC_ADDR, 0x12, b, 6)) return false;
  x = (int16_t)(b[0] | (b[1] << 8)) / ACC_LSB_PER_G;
  y = (int16_t)(b[2] | (b[3] << 8)) / ACC_LSB_PER_G;
  z = (int16_t)(b[4] | (b[5] << 8)) / ACC_LSB_PER_G;
  return true;
}

// Raw rate, bias not removed.
bool readGyroDps(float g[3]) {
  uint8_t b[6];
  if (!readBytes(GYR_ADDR, 0x02, b, 6)) return false;
  for (int k = 0; k < 3; k++) g[k] = (int16_t)(b[2 * k] | (b[2 * k + 1] << 8)) / GYR_LSB_PER_DPS;
  return true;
}

int32_t sext24(uint32_t v) {
  if (v & 0x00800000) v |= 0xFF000000;
  return (int32_t)v;
}

bool readMagRawCounts(float out[3]) {
  uint8_t b[11]; // 2 dummy bytes + 9 data bytes
  if (!readBytes(MAG_ADDR, 0x31, b, 11)) return false;
  out[0] = sext24((uint32_t)b[2] | ((uint32_t)b[3] << 8) | ((uint32_t)b[4] << 16));
  out[1] = sext24((uint32_t)b[5] | ((uint32_t)b[6] << 8) | ((uint32_t)b[7] << 16));
  out[2] = sext24((uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16));
  return true;
}

// Applies hard/soft iron correction, then maps the magnetometer's axes onto the
// accel/gyro body frame.
void applyMagCalibration(const float raw[3], float out[3]) {
  float c[3];
  for (int i = 0; i < 3; i++) c[i] = (raw[i] - magOffset[i]) * magScale[i];
  for (int i = 0; i < 3; i++) out[i] = magAxisSign[i] * c[magAxisMap[i]];
}

bool readMag(float &x, float &y, float &z) {
  float raw[3], cal[3];
  if (!readMagRawCounts(raw)) return false;
  applyMagCalibration(raw, cal);
  x = cal[0];
  y = cal[1];
  z = cal[2];
  return true;
}

// Collects raw gyro samples and says whether they look like a board at rest.
struct GyroWindow {
  double sum[3], sumSq[3];
  int n;

  void reset() {
    for (int k = 0; k < 3; k++) sum[k] = sumSq[k] = 0.0;
    n = 0;
  }

  void add(const float g[3]) {
    for (int k = 0; k < 3; k++) {
      sum[k] += g[k];
      sumSq[k] += (double)g[k] * g[k];
    }
    n++;
  }

  // Fills mean and the largest per-axis spread; true if the board was steady.
  bool steady(float mean[3], float &spread) const {
    spread = 0.0f;
    bool plausible = n >= 100;
    for (int k = 0; k < 3; k++) {
      mean[k] = n ? sum[k] / n : 0.0f;
      float var = n ? sumSq[k] / n - (double)mean[k] * mean[k] : 0.0f;
      spread = fmaxf(spread, sqrtf(fmaxf(var, 0.0f)));
      if (fabsf(mean[k]) > GYRO_MAX_BIAS_DPS) plausible = false;
    }
    return plausible && spread < GYRO_STEADY_STD_DPS;
  }
};

// Averages the gyro at rest so the constant part of the bias is removed before
// it gets integrated into the attitude estimate. Waits, however long it takes,
// for a window in which the board is actually still.
void calibrateGyroBias() {
  while (true) {
    GyroWindow w;
    w.reset();
    double accelSum = 0.0;
    int accelCount = 0;
    for (int i = 0; i < GYRO_BIAS_WINDOW; i++) {
      float g[3], ax, ay, az;
      if (readGyroDps(g)) w.add(g);
      if (readAccelG(ax, ay, az)) {
        accelSum += sqrtf(ax * ax + ay * ay + az * az);
        accelCount++;
      }
      delayMicroseconds(2500);
    }

    float mean[3], spread;
    if (w.steady(mean, spread)) {
      for (int k = 0; k < 3; k++) gyroBiasDps[k] = mean[k];
      if (accelCount > 0) gravityG = accelSum / accelCount;
      Serial.printf("gyro bias (dps): x=%.3f y=%.3f z=%.3f  (spread %.2f), gravity reads %.4f g\n",
                    mean[0], mean[1], mean[2], spread, gravityG);
      return;
    }
    Serial.printf("@STATE CALIBRATING board is moving (gyro spread %.1f dps, mean %.1f %.1f %.1f) - keep it still\n",
                  spread, mean[0], mean[1], mean[2]);
  }
}

// Mahony complementary filter. gx/gy/gz in rad/s, accel and mag in any unit
// (both get normalized). Pass mx=my=mz=0 to run accel+gyro only.
void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz,
                  float dt, bool holdYaw) {
  float recipNorm;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;

  float accNorm = sqrtf(ax * ax + ay * ay + az * az);
  bool useAcc = fabsf(accNorm - 1.0f) < ACCEL_REJECT_TOL_G;
  bool useMag = !(mx == 0.0f && my == 0.0f && mz == 0.0f);

  float q0q0 = q0 * q0, q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

  if (useAcc) {
    ax /= accNorm;
    ay /= accNorm;
    az /= accNorm;

    // Estimated direction of gravity in the sensor frame.
    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3q3;

    // Error is the cross product between estimated and measured directions.
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  if (useMag) {
    recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Earth's magnetic field projected into the sensor frame.
    float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    float halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    float halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    float halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    halfex += (my * halfwz - mz * halfwy);
    halfey += (mz * halfwx - mx * halfwz);
    halfez += (mx * halfwy - my * halfwx);
  }

  gx += TWO_KP * halfex;
  gy += TWO_KP * halfey;
  gz += TWO_KP * halfez;

  // With the board at rest, any rotation about the earth's vertical is gyro
  // bias or filter feedback, not real motion. Removing that component keeps
  // heading fixed while still letting roll/pitch settle.
  if (holdYaw) {
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float along = gx * vx + gy * vy + gz * vz;
    gx -= along * vx;
    gy -= along * vy;
    gz -= along * vz;
  }

  // Integrate the rate of change of the quaternion.
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// Records how far the magnetometer wanders while the board is held still, so
// the rotation sweep can be judged against this board's real noise instead of a
// guessed threshold.
void measureMagNoiseFloor() {
  float lo[3] = {INFINITY, INFINITY, INFINITY};
  float hi[3] = {-INFINITY, -INFINITY, -INFINITY};

  for (int i = 0; i < 200; i++) {
    float raw[3];
    if (readMagRawCounts(raw)) {
      for (int k = 0; k < 3; k++) {
        if (raw[k] < lo[k]) lo[k] = raw[k];
        if (raw[k] > hi[k]) hi[k] = raw[k];
      }
    }
    delay(5);
  }

  for (int k = 0; k < 3; k++) {
    magNoiseFloor[k] = isfinite(hi[k] - lo[k]) ? (hi[k] - lo[k]) : 0.0f;
  }
  Serial.printf("mag noise floor (counts): x=%.0f y=%.0f z=%.0f\n",
                magNoiseFloor[0], magNoiseFloor[1], magNoiseFloor[2]);
}

// The angle between gravity and the earth's magnetic field is fixed, so with
// the axes mapped correctly accel·mag stays constant no matter how the board is
// turned. Whichever of the 24 possible axis orientations holds that dot product
// steadiest across the captured rotations is the real one.
void solveMagAxisMapping() {
  static const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  static const int parity[6] = {1, -1, -1, 1, 1, -1};

  int bestPerm[3] = {0, 1, 2};
  float bestSign[3] = {1.0f, 1.0f, 1.0f};
  float bestScore = INFINITY;
  float identityScore = INFINITY;
  float runnerUpScore = INFINITY;

  for (int p = 0; p < 6; p++) {
    for (int s = 0; s < 8; s++) {
      float sign[3] = {(s & 1) ? -1.0f : 1.0f, (s & 2) ? -1.0f : 1.0f, (s & 4) ? -1.0f : 1.0f};
      if (parity[p] * sign[0] * sign[1] * sign[2] < 0.0f) continue; // keep proper rotations only

      double sum = 0.0, sumSq = 0.0;
      int n = 0;
      for (int i = 0; i < calSampleCount; i++) {
        const float *a = calSamples[i].a;
        const float *m = calSamples[i].m;

        float aNorm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (aNorm < 0.8f || aNorm > 1.2f) continue; // reject samples with motion accel

        float mapped[3];
        for (int k = 0; k < 3; k++) mapped[k] = sign[k] * m[perms[p][k]];
        float mNorm = sqrtf(mapped[0] * mapped[0] + mapped[1] * mapped[1] + mapped[2] * mapped[2]);
        if (mNorm < 1e-6f) continue;

        float dot = (a[0] * mapped[0] + a[1] * mapped[1] + a[2] * mapped[2]) / (aNorm * mNorm);
        sum += dot;
        sumSq += (double)dot * dot;
        n++;
      }
      if (n < 50) continue;

      float mean = sum / n;
      float score = sqrtf((float)(sumSq / n - (double)mean * mean));

      bool isIdentity = (perms[p][0] == 0 && perms[p][1] == 1 && perms[p][2] == 2 &&
                         sign[0] > 0 && sign[1] > 0 && sign[2] > 0);
      if (isIdentity) identityScore = score;

      if (score < bestScore) {
        runnerUpScore = bestScore;
        bestScore = score;
        for (int k = 0; k < 3; k++) {
          bestPerm[k] = perms[p][k];
          bestSign[k] = sign[k];
        }
      } else if (score < runnerUpScore) {
        runnerUpScore = score;
      }
    }
  }

  if (!isfinite(bestScore)) {
    Serial.println("  axis mapping: not enough usable samples, keeping identity");
    return;
  }

  // A real mapping should stand clearly apart from the next best candidate.
  if (runnerUpScore < bestScore * 1.5f) {
    Serial.printf("  axis mapping: ambiguous (best %.4f vs next %.4f), keeping identity\n",
                  bestScore, runnerUpScore);
    return;
  }

  for (int k = 0; k < 3; k++) {
    magAxisMap[k] = bestPerm[k];
    magAxisSign[k] = bestSign[k];
  }
  Serial.printf("  axis mapping: x=%c%c y=%c%c z=%c%c  (spread %.4f, next best %.4f, identity %.4f)\n",
                bestSign[0] < 0 ? '-' : '+', 'x' + bestPerm[0],
                bestSign[1] < 0 ? '-' : '+', 'x' + bestPerm[1],
                bestSign[2] < 0 ? '-' : '+', 'x' + bestPerm[2],
                bestScore, runnerUpScore, identityScore);
}

// Collects magnetometer extremes while the board is rotated, then derives
// hard-iron offsets (centre of the swept sphere) and soft-iron scales
// (equalising each axis's swing).
void calibrateMagnetometer() {
  Serial.println();
  Serial.println("Mag calibration: rotate the board slowly through ALL orientations");
  Serial.println("(figure-8s, plus turning it over) for the next 30 seconds.");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  float lo[3] = {INFINITY, INFINITY, INFINITY};
  float hi[3] = {-INFINITY, -INFINITY, -INFINITY};
  calSampleCount = 0;

  uint32_t startMs = millis();
  uint32_t lastSampleMs = 0;
  uint32_t lastReportMs = 0;

  while (millis() - startMs < MAG_CAL_DURATION_MS) {
    if (millis() - lastSampleMs < MAG_CAL_SAMPLE_INTERVAL_MS) continue;
    lastSampleMs = millis();

    float raw[3], ax, ay, az;
    if (!readMagRawCounts(raw) || !readAccelG(ax, ay, az)) continue;

    for (int i = 0; i < 3; i++) {
      if (raw[i] < lo[i]) lo[i] = raw[i];
      if (raw[i] > hi[i]) hi[i] = raw[i];
    }

    if (calSampleCount < MAG_CAL_MAX_SAMPLES) {
      calSamples[calSampleCount].a[0] = ax;
      calSamples[calSampleCount].a[1] = ay;
      calSamples[calSampleCount].a[2] = az;
      calSamples[calSampleCount].m[0] = raw[0];
      calSamples[calSampleCount].m[1] = raw[1];
      calSamples[calSampleCount].m[2] = raw[2];
      calSampleCount++;
    }

    if (millis() - lastReportMs >= 2000) {
      lastReportMs = millis();
      Serial.printf("  %2lus left | swing x=%.0f y=%.0f z=%.0f\n",
                    (unsigned long)((MAG_CAL_DURATION_MS - (millis() - startMs)) / 1000),
                    hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
    }
  }

  float half[3];
  for (int i = 0; i < 3; i++) half[i] = 0.5f * (hi[i] - lo[i]);

  // Did the board actually get turned through a range of orientations? Gravity
  // sweeping across all three accel axes is the scale-free way to tell.
  float aLo[3] = {INFINITY, INFINITY, INFINITY};
  float aHi[3] = {-INFINITY, -INFINITY, -INFINITY};
  for (int i = 0; i < calSampleCount; i++) {
    for (int k = 0; k < 3; k++) {
      if (calSamples[i].a[k] < aLo[k]) aLo[k] = calSamples[i].a[k];
      if (calSamples[i].a[k] > aHi[k]) aHi[k] = calSamples[i].a[k];
    }
  }
  float minAccelSpread = INFINITY;
  for (int k = 0; k < 3; k++) minAccelSpread = fminf(minAccelSpread, aHi[k] - aLo[k]);

  if (!isfinite(minAccelSpread) || minAccelSpread < 0.8f) {
    Serial.printf("  FAILED: board did not cover enough orientations (min accel swing %.2fg).\n",
                  minAccelSpread);
    Serial.println("  Mag stays out of the fusion. Reset and rotate it through all");
    Serial.println("  orientations, including turning it upside down.");
    return;
  }

  // Each axis has to swing well clear of its own noise, otherwise we are just
  // fitting a sphere to sensor jitter.
  for (int k = 0; k < 3; k++) {
    if ((hi[k] - lo[k]) < 8.0f * magNoiseFloor[k] || (hi[k] - lo[k]) < 500.0f) {
      Serial.printf("  FAILED: axis %c swing %.0f counts is not clear of noise (%.0f).\n",
                    'x' + k, hi[k] - lo[k], magNoiseFloor[k]);
      Serial.println("  Mag stays out of the fusion. Reset and rotate more thoroughly.");
      return;
    }
  }

  float smallest = fminf(half[0], fminf(half[1], half[2]));
  float largest = fmaxf(half[0], fmaxf(half[1], half[2]));
  if (largest > smallest * 4.0f) {
    Serial.println("  WARNING: very uneven coverage, yaw may be poor. Consider redoing this.");
  }

  float avgHalf = (half[0] + half[1] + half[2]) / 3.0f;
  for (int i = 0; i < 3; i++) {
    magOffset[i] = 0.5f * (hi[i] + lo[i]);
    magScale[i] = avgHalf / half[i];
  }

  // Re-express the stored samples in corrected counts so the axis search sees
  // the same values the fusion will.
  for (int i = 0; i < calSampleCount; i++) {
    for (int k = 0; k < 3; k++) {
      calSamples[i].m[k] = (calSamples[i].m[k] - magOffset[k]) * magScale[k];
    }
  }

  solveMagAxisMapping();

  Serial.printf("  offsets: %.1f %.1f %.1f\n", magOffset[0], magOffset[1], magOffset[2]);
  Serial.printf("  scales : %.4f %.4f %.4f\n", magScale[0], magScale[1], magScale[2]);
  Serial.println("  To skip this next boot, paste these into magOffset/magScale");
  Serial.println("  (and magAxisMap/magAxisSign) and set useMagInFusion = true.");

  useMagInFusion = true;
  Serial.println("  Mag calibration done, yaw is now absolute.");
}

// Starting from the identity quaternion makes the filter converge through a
// large initial error, which winds up the integral term and then takes a long
// time to unwind. Seeding from the first accel sample avoids that transient.
void seedOrientationFromAccel() {
  float ax, ay, az;
  if (!readAccelG(ax, ay, az)) return;

  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

  q0 = cr * cp;
  q1 = sr * cp;
  q2 = cr * sp;
  q3 = -sr * sp;
}

void quaternionToEuler(float &rollDeg, float &pitchDeg, float &yawDeg) {
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;

  rollDeg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  pitchDeg = asinf(sinp) * RAD_TO_DEG;
  yawDeg = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

// ---------------------------------------------------------------------------
// Dead reckoning
//
// Integrating accel twice drifts away within seconds, so instead of tracking
// continuously this measures discrete moves that start and end with the board
// at rest. Rest at both ends is what makes it usable: the earth-frame accel
// just before the move is the gravity reference, and since the board is known
// to be stopped afterwards, any velocity left over at the end is drift.
//
// Everything is computed here. Lines starting with '@' are for the plotting
// tool, which only draws what they say:
//   @RESET                                   position is back at zero
//   @STATE <name> <message>                  CALIBRATING, SETTLING, READY or MOVING
//   @PATH <n> <x y z> <x y z> ...            the path move n took, sent just before its @MOVE
//   @MOVE <n> <x0 y0 z0> <x1 y1 z1> | <text> a move from start to end, earth frame, cm
//   @INFO <text>                             something worth showing that isn't a move
//   @LIVE <x y z> <board x axis> <board y axis> <board z axis>
//                                            25 times a second: where the board is now (cm)
//                                            and which way its axes point, earth frame
// Sending 'z' zeroes the position; sending 'f' makes the next push define forward.

constexpr float GRAVITY_MS2 = 9.80665f;

// Only big, deliberate moves count. Detection looks at linear acceleration,
// earth-frame accel minus gravity, so it reads the same however the board is
// tilted. Over 0.5 g in any direction is motion; anything under (hand
// tremor, slow drifting, tilting the board in hand) is rest. A move ends once
// it has been rest for a while.
constexpr float DR_FAST_TAU_S = 0.02f; // smoothing so single noisy samples don't decide
constexpr float MOTION_ACCEL_G = 0.5f;

constexpr int MOVE_END_SAMPLES = (int)(0.40f * GYRO_ODR_HZ); // this long at rest ends a move
// The trigger fires partway up the push, which can build gently for a while
// before crossing the threshold. Missing that build-up loses the early
// velocity and can flip a move's direction, so the move is taken to begin at
// the last moment the board was truly quiet, looking back up to HIST_SAMPLES.
// The samples before that, and the tail of the move, give the gravity
// reference when the board was steady in them (see steadyMean).
constexpr int HIST_SAMPLES = (int)(1.2f * GYRO_ODR_HZ);
constexpr float QUIET_ACCEL_G = 0.1f;
constexpr int QUIET_RUN_SAMPLES = (int)(0.05f * GYRO_ODR_HZ);
constexpr int MOVE_DEFAULT_PRE_SAMPLES = (int)(0.25f * GYRO_ODR_HZ); // if it never was quiet
constexpr int MOVE_REF_SAMPLES = (int)(0.15f * GYRO_ODR_HZ);
constexpr int MOVE_TAIL_SAMPLES = (int)(0.15f * GYRO_ODR_HZ);
constexpr float STEADY_ACCEL_G = 0.05f;
constexpr int MOVE_MAX_SAMPLES = (int)(10.0f * GYRO_ODR_HZ); // longer drifts too far to report
constexpr float MOVE_MIN_REPORT_M = 0.01f;
constexpr float TURN_REPORT_DEG = 10.0f; // a smaller turn with no movement is just a bump

// Turning the board moves its accel bias and tilt error to a new direction, so
// the gravity reference after a move can differ from the one before. The shift
// is spread over the move in proportion to how far the board had turned; with
// almost no turning, in proportion to time.
constexpr float REF_BLEND_MIN_ROT_RAD = 2.0f * DEG_TO_RAD;

// Stillness judged without the attitude: the accelerometer reads just gravity
// and the board isn't turning. A spin past the gyro's ±2000 dps range leaves
// rotation uncounted and the attitude wrong, and the earth-frame tests above
// then can't see that the board has stopped; this one still can.
constexpr float BODY_STILL_ACCEL_G = 0.05f;
constexpr float BODY_STILL_GYRO_DPS = 10.0f;
// Once still that long, an attitude this far from gravity is levelled at once
// rather than left to the filter's slow pull, and the move is not trusted.
constexpr int LEVEL_STILL_SAMPLES = (int)(0.2f * GYRO_ODR_HZ);
constexpr float LEVEL_SNAP_DEG = 15.0f;

// Without a magnetometer nothing says which way is forward, so the user sets
// it: send 'f', then push the board forward once, at least this far.
constexpr float ALIGN_MIN_M = 0.10f;

struct MoveSample {
  float e[3]; // accel in the earth frame, m/s^2, gravity still included
  float rot;  // how far the board turned during this sample, rad
  float lin;  // linear accel (gravity removed), g
};

enum class DrState { Settling, Rest, Moving };
DrState drState = DrState::Settling;

MoveSample hist[HIST_SAMPLES]; // latest samples; once full, the oldest is at histHead
int histHead = 0, histCount = 0;

MoveSample moveBuf[MOVE_MAX_SAMPLES];
float moveSpeed[MOVE_MAX_SAMPLES]; // corrected speed through the move, m/s

// The corrected path of the last move, relative to its start, thinned to at
// most this many points for the plot.
constexpr int PATH_MAX_POINTS = 60;
float pathBuf[PATH_MAX_POINTS + 2][3];
int pathLen = 0;
int moveLen = 0;
bool moveOverflow = false;
float moveRef[3];    // earth-frame accel at rest before the move
uint32_t moveStartSaturations;
bool alignPending = false; // the next move sets which way is forward
float moveStartQ[4];
int moveCount = 0;
float position[3] = {0.0f, 0.0f, 0.0f}; // sum of every reported move, m

// Running estimate while a move is under way, so the plot can follow it live.
// It uses the gravity reference from before the move but can't apply the
// end-of-move drift correction yet, so it wanders until finishMove replaces
// it with the corrected result.
float liveVel[3], liveDisp[3];
constexpr uint32_t LIVE_INTERVAL_MS = 40;

float eFast[3];       // smoothed earth-frame accel, m/s^2
float aBodyFast[3];   // smoothed sensor-frame accel, g
float gBodyFast[3];   // smoothed sensor-frame rate, dps
int bodyStillCount = 0; // consecutive samples still by the attitude-free test
float linNowG = 0.0f; // linear accel right now, g
int restCount = 0;    // consecutive samples below MOTION_ACCEL_G
float peakLinG = 0.0f; // highest linear accel since the last status print, for tuning

float norm3(const float v[3]) {
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float dist3(const float a[3], const float b[3]) {
  float d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  return norm3(d);
}

// Rotates a sensor-frame vector into the earth frame by quaternion qq, or with
// inverse set, an earth-frame vector into the sensor frame.
void rotateByQuat(const float qq[4], const float v[3], float out[3], bool inverse) {
  float w = qq[0], x = qq[1], y = qq[2], z = qq[3];
  float r[3][3] = {
      {1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - w * z), 2.0f * (x * z + w * y)},
      {2.0f * (x * y + w * z), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - w * x)},
      {2.0f * (x * z - w * y), 2.0f * (y * z + w * x), 1.0f - 2.0f * (x * x + y * y)}};
  for (int i = 0; i < 3; i++) {
    out[i] = inverse ? r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2]
                     : r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2];
  }
}

// Names the earth axes a displacement mostly lies along, e.g. "forward + down".
void describeDirection(const float d[3], char *out, size_t len) {
  static const char *const posName[3] = {"forward", "left", "up"};
  static const char *const negName[3] = {"back", "right", "down"};

  int order[3] = {0, 1, 2};
  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (fabsf(d[order[j]]) > fabsf(d[order[i]])) {
        int t = order[i];
        order[i] = order[j];
        order[j] = t;
      }
    }
  }

  float total = norm3(d);
  size_t used = 0;
  out[0] = '\0';
  for (int i = 0; i < 3 && used < len; i++) {
    int k = order[i];
    if (i > 0 && fabsf(d[k]) < 0.4f * total) break; // closer than ~24 deg to the main axis
    used += snprintf(out + used, len - used, "%s%s", i > 0 ? " + " : "",
                     d[k] > 0.0f ? posName[k] : negName[k]);
  }
}

// Tells the plotting tool what the board is doing and what to do next.
void reportState() {
  switch (drState) {
    case DrState::Settling:
      Serial.printf("@STATE SETTLING waiting for the board to settle "
                    "(accel now %.2f g; under %.1f g counts as rest)\n", linNowG, MOTION_ACCEL_G);
      break;
    case DrState::Rest:
      if (alignPending) {
        Serial.printf("@STATE READY push the board forward once (over %.1f g) to set forward\n", MOTION_ACCEL_G);
      } else {
        Serial.printf("@STATE READY move it hard (over %.1f g) to register a move\n", MOTION_ACCEL_G);
      }
      break;
    case DrState::Moving:
      Serial.printf("@STATE MOVING measuring the move (accel now %.2f g; under %.2f g for %.1f s ends it)\n",
                    linNowG, MOTION_ACCEL_G, MOVE_END_SAMPLES / GYRO_ODR_HZ);
      break;
  }
}

// Where the board is now and which way its axes point (the columns of its
// rotation into the earth frame), for the plot to draw as-is.
void reportLive() {
  bool moving = drState == DrState::Moving;
  float w = q0, x = q1, y = q2, z = q3;
  Serial.printf("@LIVE %.1f %.1f %.1f  %.3f %.3f %.3f  %.3f %.3f %.3f  %.3f %.3f %.3f\n",
                (position[0] + (moving ? liveDisp[0] : 0.0f)) * 100.0f,
                (position[1] + (moving ? liveDisp[1] : 0.0f)) * 100.0f,
                (position[2] + (moving ? liveDisp[2] : 0.0f)) * 100.0f,
                1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y + w * z), 2.0f * (x * z - w * y),
                2.0f * (x * y - w * z), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z + w * x),
                2.0f * (x * z + w * y), 2.0f * (y * z - w * x), 1.0f - 2.0f * (x * x + y * y));
}

void enterRest() {
  drState = DrState::Rest;
  reportState();
}

// Earth-frame accel of the still board: straight up, as strong as gravity read
// during calibration.
void gravityVector(float out[3]) {
  out[0] = 0.0f;
  out[1] = 0.0f;
  out[2] = gravityG * GRAVITY_MS2;
}

// Mean earth-frame accel over count samples, and whether the board was truly
// steady through them. "Rest" covers anything under MOTION_ACCEL_G, so a window can hold
// real motion, and only a steady one is a fair gravity reference.
template <typename SampleAt>
bool steadyMean(int count, SampleAt at, float mean[3]) {
  for (int k = 0; k < 3; k++) mean[k] = 0.0f;
  for (int i = 0; i < count; i++) {
    for (int k = 0; k < 3; k++) mean[k] += at(i).e[k];
  }
  for (int k = 0; k < 3; k++) mean[k] /= count;
  for (int i = 0; i < count; i++) {
    if (dist3(at(i).e, mean) > STEADY_ACCEL_G * GRAVITY_MS2) return false;
  }
  return true;
}

bool moveRefMeasured; // whether moveRef came from a steady window, for the report

// Re-expresses the attitude in an earth frame rotated by quaternion r.
void rotateEarthFrame(const float r[4]) {
  float n0 = r[0] * q0 - r[1] * q1 - r[2] * q2 - r[3] * q3;
  float n1 = r[0] * q1 + r[1] * q0 + r[2] * q3 - r[3] * q2;
  float n2 = r[0] * q2 - r[1] * q3 + r[2] * q0 + r[3] * q1;
  float n3 = r[0] * q3 + r[1] * q2 - r[2] * q1 + r[3] * q0;
  float inv = 1.0f / sqrtf(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
  q0 = n0 * inv;
  q1 = n1 * inv;
  q2 = n2 * inv;
  q3 = n3 * inv;

  // Recent samples are in the old frame; the next move may reach back into them.
  for (int i = 0; i < HIST_SAMPLES; i++) {
    float e[3] = {hist[i].e[0], hist[i].e[1], hist[i].e[2]};
    rotateByQuat(r, e, hist[i].e, false);
  }
  float e[3] = {eFast[0], eFast[1], eFast[2]};
  rotateByQuat(r, e, eFast, false);
}

// Turns the earth frame about the vertical by angle (rad): the direction at
// heading -angle becomes forward. Gravity is untouched, so tilt stays right.
void rotateHeading(float angle) {
  float r[4] = {cosf(0.5f * angle), 0.0f, 0.0f, sinf(0.5f * angle)};
  rotateEarthFrame(r);
}

// How far the attitude's "down" is from the way gravity pulls, in degrees.
// Only meaningful while the board is still.
float tiltErrorDeg() {
  float qNow[4] = {q0, q1, q2, q3};
  float up[3];
  rotateByQuat(qNow, aBodyFast, up, false);
  float n = norm3(up);
  if (n < 1e-6f) return 0.0f;
  return acosf(fmaxf(-1.0f, fminf(1.0f, up[2] / n))) * RAD_TO_DEG;
}

// If the board is still and the attitude is far from gravity, turn it
// straight to match, about a horizontal axis so heading moves as little as
// possible.
void levelFromGravity() {
  if (bodyStillCount < LEVEL_STILL_SAMPLES) return;
  float errDeg = tiltErrorDeg();
  if (errDeg < LEVEL_SNAP_DEG) return;

  float qNow[4] = {q0, q1, q2, q3};
  float up[3];
  rotateByQuat(qNow, aBodyFast, up, false);
  float axis[3] = {up[1], -up[0], 0.0f}; // up x vertical
  float an = norm3(axis);
  if (an < 1e-6f) {
    axis[0] = 1.0f; // exactly upside down: any horizontal axis will do
    an = 1.0f;
  }
  float half = 0.5f * errDeg * DEG_TO_RAD;
  float r[4] = {cosf(half), sinf(half) * axis[0] / an, sinf(half) * axis[1] / an, 0.0f};
  rotateEarthFrame(r);
  Serial.printf("Attitude was %.0f deg off once still; levelled it from gravity.\n", errDeg);
}

void integrateLive(const float e[3], float dt) {
  for (int k = 0; k < 3; k++) {
    liveVel[k] += (e[k] - moveRef[k]) * dt;
    liveDisp[k] += liveVel[k] * dt;
  }
}

void beginMove(float dt) {
  // History index 0 is the oldest sample, HIST_SAMPLES - 1 the one that
  // tripped the trigger.
  auto at = [](int j) -> const MoveSample & { return hist[(histHead + j) % HIST_SAMPLES]; };
  const int newest = HIST_SAMPLES - 1;

  // Walk back to the last quiet stretch before the push; the move starts there.
  int start = newest + 1 - MOVE_DEFAULT_PRE_SAMPLES;
  int quiet = 0;
  for (int j = newest; j >= MOVE_REF_SAMPLES; j--) {
    quiet = at(j).lin < QUIET_ACCEL_G ? quiet + 1 : 0;
    if (quiet == QUIET_RUN_SAMPLES) {
      start = j;
      break;
    }
  }

  // A steady window just before the start also captures whatever tilt error
  // the attitude has; otherwise use plain gravity.
  moveRefMeasured = steadyMean(
      MOVE_REF_SAMPLES, [&](int j) -> const MoveSample & { return at(start - MOVE_REF_SAMPLES + j); },
      moveRef);
  if (!moveRefMeasured) gravityVector(moveRef);

  moveLen = 0;
  for (int k = 0; k < 3; k++) liveVel[k] = liveDisp[k] = 0.0f;
  for (int j = start; j <= newest; j++) {
    moveBuf[moveLen] = at(j);
    integrateLive(moveBuf[moveLen].e, dt);
    moveLen++;
  }
  moveStartSaturations = gyroSaturations;

  moveStartQ[0] = q0;
  moveStartQ[1] = q1;
  moveStartQ[2] = q2;
  moveStartQ[3] = q3;
  moveOverflow = false;
  // The rest that ends a move is counted from here, which also guarantees the
  // move holds MOVE_END_SAMPLES for finishMove's end reference.
  restCount = 0;
  drState = DrState::Moving;
  reportState();
}

// Turns the recorded move into a displacement and reports it.
void finishMove(float dt) {
  uint32_t solveStartUs = micros();
  moveCount++;
  float qNow[4] = {q0, q1, q2, q3};
  float qDot = moveStartQ[0] * qNow[0] + moveStartQ[1] * qNow[1] +
               moveStartQ[2] * qNow[2] + moveStartQ[3] * qNow[3];
  float turnedDeg = 2.0f * acosf(fminf(1.0f, fabsf(qDot))) * RAD_TO_DEG;

  Serial.println();
  if (moveOverflow) {
    Serial.printf("MOVE %d: longer than %.0f s, drifted too far to measure. Not counted.\n",
                  moveCount, MOVE_MAX_SAMPLES * dt);
    Serial.printf("@INFO Move %d: longer than %.0f s, not measured\n", moveCount, MOVE_MAX_SAMPLES * dt);
    return;
  }

  // Either the gyro maxed out, leaving rotation uncounted, or the board has
  // stopped with the attitude far from what gravity shows. Both mean the
  // acceleration was integrated pointing the wrong way.
  bool spunOut = gyroSaturations != moveStartSaturations;
  float tiltErr = tiltErrorDeg();
  if (spunOut || (bodyStillCount >= LEVEL_STILL_SAMPLES && tiltErr > LEVEL_SNAP_DEG)) {
    if (spunOut) {
      Serial.printf("MOVE %d: not measured, it spun past the gyro's 2000 dps range.\n", moveCount);
      Serial.printf("@INFO Move %d: spun faster than the gyro can follow (2000 dps), not measured."
                    " Heading may be off, F re-aligns\n", moveCount);
    } else {
      Serial.printf("MOVE %d: not measured, attitude was %.0f deg off at the end.\n", moveCount, tiltErr);
      Serial.printf("@INFO Move %d: lost track of orientation, not measured. Heading may be off, F re-aligns\n",
                    moveCount);
    }
    return;
  }

  // If the board was truly steady at the end, that gives the gravity reference
  // after the move; otherwise assume it didn't shift.
  float refEnd[3];
  bool refEndMeasured = steadyMean(
      MOVE_TAIL_SAMPLES,
      [](int j) -> const MoveSample & { return moveBuf[moveLen - MOVE_TAIL_SAMPLES + j]; }, refEnd);
  if (!refEndMeasured) {
    for (int k = 0; k < 3; k++) refEnd[k] = moveRef[k];
  }

  float totalRot = 0.0f;
  for (int i = 0; i < moveLen; i++) totalRot += moveBuf[i].rot;
  bool blendByRot = totalRot > REF_BLEND_MIN_ROT_RAD;

  auto linearAccel = [&](int i, float rotSoFar, float out[3]) {
    float w = blendByRot ? rotSoFar / totalRot : (float)(i + 1) / moveLen;
    for (int k = 0; k < 3; k++) {
      out[k] = moveBuf[i].e[k] - (moveRef[k] + w * (refEnd[k] - moveRef[k]));
    }
  };

  // First pass: the velocity left at the end, which should be zero.
  float v[3] = {0.0f, 0.0f, 0.0f};
  float rotSoFar = 0.0f;
  for (int i = 0; i < moveLen; i++) {
    float a[3];
    rotSoFar += moveBuf[i].rot;
    linearAccel(i, rotSoFar, a);
    for (int k = 0; k < 3; k++) v[k] += a[k] * dt;
  }
  float vEnd[3] = {v[0], v[1], v[2]};

  // Second pass: a constant accel error makes velocity drift linearly, so take
  // that ramp back out and integrate again for position.
  float d[3] = {0.0f, 0.0f, 0.0f};
  float peakSpeed = 0.0f;
  for (int k = 0; k < 3; k++) v[k] = 0.0f;
  rotSoFar = 0.0f;
  const int pathStep = moveLen / PATH_MAX_POINTS + 1;
  pathLen = 0;
  for (int k = 0; k < 3; k++) pathBuf[pathLen][k] = 0.0f;
  pathLen++;
  for (int i = 0; i < moveLen; i++) {
    float a[3], vc[3];
    rotSoFar += moveBuf[i].rot;
    linearAccel(i, rotSoFar, a);
    float frac = (float)(i + 1) / moveLen;
    for (int k = 0; k < 3; k++) {
      v[k] += a[k] * dt;
      vc[k] = v[k] - vEnd[k] * frac;
      d[k] += vc[k] * dt;
    }
    moveSpeed[i] = norm3(vc);
    peakSpeed = fmaxf(peakSpeed, moveSpeed[i]);
    if ((i + 1) % pathStep == 0 || i == moveLen - 1) {
      for (int k = 0; k < 3; k++) pathBuf[pathLen][k] = d[k];
      pathLen++;
    }
  }

  uint32_t solveUs = micros() - solveStartUs;
  float distM = norm3(d);
  // How long the board was really moving: the span where its speed was above
  // a fifth of the peak. The trigger and the rest that ends a move both sit
  // at arbitrary distances from the motion itself.
  int firstMoving = moveLen, lastMoving = -1;
  for (int i = 0; i < moveLen; i++) {
    if (moveSpeed[i] > 0.2f * peakSpeed) {
      if (firstMoving == moveLen) firstMoving = i;
      lastMoving = i;
    }
  }
  float activeS = lastMoving >= firstMoving ? (lastMoving - firstMoving + 1) * dt : 0.0f;

  if (alignPending) {
    float horiz = sqrtf(d[0] * d[0] + d[1] * d[1]);
    if (horiz < ALIGN_MIN_M) {
      Serial.printf("Alignment push was only %.0f cm sideways.\n", horiz * 100.0f);
      Serial.printf("@INFO That push was only %.0f cm - push forward harder to set forward\n", horiz * 100.0f);
      return;
    }
    rotateHeading(-atan2f(d[1], d[0]));
    alignPending = false;
    for (int k = 0; k < 3; k++) position[k] = 0.0f;
    Serial.printf("Forward set from a %.0f cm push.\n", horiz * 100.0f);
    Serial.println("@RESET");
    Serial.println("@INFO Forward set: that push is forward now, left is to its left");
    return;
  }

  if (distM < MOVE_MIN_REPORT_M) {
    // Bumps and table knocks land here; only a real turn is worth showing.
    Serial.printf("MOVE %d: no net movement (%.1f cm), turned %.0f deg\n",
                  moveCount, distM * 100.0f, turnedDeg);
    if (turnedDeg >= TURN_REPORT_DEG) {
      Serial.printf("@INFO Move %d: turned %.0f deg in place, no net movement\n", moveCount, turnedDeg);
    }
  } else {
    float start[3] = {position[0], position[1], position[2]};
    for (int k = 0; k < 3; k++) position[k] += d[k];

    char dir[48];
    describeDirection(d, dir, sizeof dir);
    float dBoard[3];
    rotateByQuat(moveStartQ, d, dBoard, true);

    Serial.printf("MOVE %d: %.1f cm %s, in %.2f s\n", moveCount, distM * 100.0f, dir, activeS);
    Serial.printf("  earth (cm): X %+6.1f  Y %+6.1f  Z %+6.1f    board axes (cm): x %+6.1f  y %+6.1f  z %+6.1f\n",
                  d[0] * 100.0f, d[1] * 100.0f, d[2] * 100.0f,
                  dBoard[0] * 100.0f, dBoard[1] * 100.0f, dBoard[2] * 100.0f);
    Serial.printf("@PATH %d", moveCount);
    for (int j = 0; j < pathLen; j++) {
      Serial.printf(" %.1f %.1f %.1f", (start[0] + pathBuf[j][0]) * 100.0f,
                    (start[1] + pathBuf[j][1]) * 100.0f, (start[2] + pathBuf[j][2]) * 100.0f);
    }
    Serial.println();
    Serial.printf("@MOVE %d %.2f %.2f %.2f %.2f %.2f %.2f | Move %d: %.1f cm %s, in %.2f s\n",
                  moveCount, start[0] * 100.0f, start[1] * 100.0f, start[2] * 100.0f,
                  position[0] * 100.0f, position[1] * 100.0f, position[2] * 100.0f,
                  moveCount, distM * 100.0f, dir, activeS);
  }
  Serial.printf("  peak %.2f m/s | end drift %.1f cm/s | gravity ref %s/%s, shift %.1f mg | turned %.0f deg"
                " | solved on the Teensy in %lu us\n",
                peakSpeed, norm3(vEnd) * 100.0f, moveRefMeasured ? "steady" : "assumed",
                refEndMeasured ? "steady" : "assumed", dist3(refEnd, moveRef) / GRAVITY_MS2 * 1000.0f,
                turnedDeg, (unsigned long)solveUs);
  Serial.printf("  POSITION (cm): X %+7.1f  Y %+7.1f  Z %+7.1f\n",
                position[0] * 100.0f, position[1] * 100.0f, position[2] * 100.0f);
}

// Runs once per gyro sample, after the attitude update. gDps is bias-corrected
// gyro in deg/s, aG is accel in g.
void deadReckonStep(const float gDps[3], const float aG[3], float dt) {
  MoveSample s;
  float qNow[4] = {q0, q1, q2, q3};
  float aMs2[3] = {aG[0] * GRAVITY_MS2, aG[1] * GRAVITY_MS2, aG[2] * GRAVITY_MS2};
  rotateByQuat(qNow, aMs2, s.e, false);
  s.rot = norm3(gDps) * DEG_TO_RAD * dt;
  float gravity[3];
  gravityVector(gravity);
  s.lin = dist3(s.e, gravity) / GRAVITY_MS2;

  static bool primed = false;
  if (!primed) {
    for (int k = 0; k < 3; k++) {
      eFast[k] = s.e[k];
      aBodyFast[k] = aG[k];
      gBodyFast[k] = 0.0f;
    }
    primed = true;
  }
  float kFast = dt / DR_FAST_TAU_S;
  for (int k = 0; k < 3; k++) {
    eFast[k] += kFast * (s.e[k] - eFast[k]);
    aBodyFast[k] += kFast * (aG[k] - aBodyFast[k]);
    gBodyFast[k] += kFast * (gDps[k] - gBodyFast[k]);
  }
  bool bodyStill = fabsf(norm3(aBodyFast) - gravityG) < BODY_STILL_ACCEL_G &&
                   norm3(gBodyFast) < BODY_STILL_GYRO_DPS;
  bodyStillCount = bodyStill ? bodyStillCount + 1 : 0;

  // During a move, measure against the same reference the move is integrated
  // with; otherwise against plain gravity.
  linNowG = dist3(eFast, drState == DrState::Moving ? moveRef : gravity) / GRAVITY_MS2;
  peakLinG = fmaxf(peakLinG, linNowG);
  restCount = linNowG < MOTION_ACCEL_G ? restCount + 1 : 0;

  hist[histHead] = s;
  histHead = (histHead + 1) % HIST_SAMPLES;
  if (histCount < HIST_SAMPLES) histCount++;

  switch (drState) {
    case DrState::Settling:
      if ((restCount >= MOVE_END_SAMPLES || bodyStillCount >= MOVE_END_SAMPLES) &&
          histCount == HIST_SAMPLES) {
        Serial.println("DR: board is at rest, ready to measure moves.");
        levelFromGravity();
        enterRest();
      }
      break;

    case DrState::Rest:
      if (linNowG > MOTION_ACCEL_G) beginMove(dt);
      break;

    case DrState::Moving:
      if (moveLen < MOVE_MAX_SAMPLES) {
        moveBuf[moveLen++] = s;
        integrateLive(s.e, dt);
      } else {
        moveOverflow = true;
      }
      // The attitude-free test lets a move end even if the attitude went wrong.
      if (moveOverflow || restCount >= MOVE_END_SAMPLES || bodyStillCount >= MOVE_END_SAMPLES) {
        finishMove(dt);
        levelFromGravity();
        enterRest();
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 15000) {} // wait for the serial monitor to attach
  delay(300);

  Wire.begin();
  Wire.setClock(400000);

  setupBMI088Accel();
  setupBMI088Gyro();
  setupBMM350();

  Serial.println();
  Serial.println("=== IMU startup ===");
  Serial.println("@RESET");
  Serial.println("@STATE CALIBRATING keep the board still - measuring gyro bias");
  Serial.println("Keep the board STILL for gyro bias calibration...");
  delay(1500);
  calibrateGyroBias();
  measureMagNoiseFloor();

  Serial.println();
  Serial.println("Send any character within 5s to SKIP mag calibration.");
  bool skip = false;
  uint32_t waitStart = millis();
  while (millis() - waitStart < 5000) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      skip = true;
      break;
    }
  }

  if (skip) {
    Serial.println("Skipped. Mag stays out of the fusion, yaw is relative and will drift.");
  } else {
    Serial.println("@STATE CALIBRATING mag calibration - rotate the board through all orientations for 30 s");
    calibrateMagnetometer();
  }

  seedOrientationFromAccel();
  flushGyroFifo(); // it filled up and overran while calibration was running
  Serial.println();
  Serial.println("=== running ===");
  Serial.printf("Dead reckoning: only hard moves (over %.1f g) count, everything gentler is rest.\n",
                MOTION_ACCEL_G);
  Serial.println("Each move is reported once its acceleration has died down for 0.4 s.");
  Serial.println("Earth axes: X (forward) = where the board's +x axis pointed at startup,");
  Serial.println("Y (left) = 90 deg to the left of that, Z (up) = up.");
  if (useMagInFusion) {
    Serial.println("Mag is in the fusion, so X slowly swings toward magnetic north.");
  }
}

void loop() {
  static uint32_t lastPrintMs = millis();
  static uint32_t lastMagUs = 0;
  static float ax = 0, ay = 0, az = 0;
  static float gx = 0, gy = 0, gz = 0;
  static float mx = 0, my = 0, mz = 0;
  static bool accelValid = false, magValid = false;

  // The sensor's real ODR can sit a percent or so off nominal, which would
  // scale every integrated angle; measure it against the Teensy's crystal.
  static float dtSample = 1.0f / GYRO_ODR_HZ;
  static uint32_t odrWindowStartUs = micros();
  static uint32_t odrWindowFrames = 0;

  static float gxLp = 0, gyLp = 0, gzLp = 0;
  static float stillTime = 0;
  static bool atRest = false;

  static uint32_t overrunCount = 0;
  static int maxBatch = 0;
  static uint32_t accelNewCount = 0;
  static GyroWindow biasWindow = {};
  static uint32_t biasUpdates = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      for (int k = 0; k < 3; k++) position[k] = 0.0f;
      Serial.println("Position zeroed.");
      Serial.println("@RESET");
      reportState();
    } else if (c == 'f') {
      if (useMagInFusion) {
        Serial.println("@INFO Forward follows magnetic north while the magnetometer is in use");
      } else {
        alignPending = true;
        Serial.println("@INFO Push the board forward once to set which way is forward");
        reportState();
      }
    }
  }

  uint32_t nowUs = micros();
  if (useMagInFusion && nowUs - lastMagUs >= MAG_READ_INTERVAL_US) {
    lastMagUs = nowUs;
    magValid = readMag(mx, my, mz);
  }

  static uint32_t lastFifoUs = 0;
  if (nowUs - lastFifoUs < 2000) return; // ~1 new frame per 2.5 ms, no need to hammer I2C
  lastFifoUs = nowUs;

  // Read on every gyro drain, since dead reckoning integrates it at the gyro
  // rate. Noise means a fresh sample never repeats the last one exactly, so
  // counting changes measures the accel's real output rate.
  float prevAx = ax, prevAy = ay, prevAz = az;
  accelValid = readAccelG(ax, ay, az);
  if (accelValid && (ax != prevAx || ay != prevAy || az != prevAz)) accelNewCount++;

  int16_t frames[100][3];
  bool overrun;
  int n = readGyroFifo(frames, 100, overrun);
  if (overrun) {
    // Samples were dropped, so this batch doesn't cover the elapsed time.
    overrunCount++;
    flushGyroFifo();
    odrWindowStartUs = micros();
    odrWindowFrames = 0;
  } else if (n > 0) {
    odrWindowFrames += n;
    uint32_t windowUs = micros() - odrWindowStartUs;
    if (windowUs >= 2000000 && odrWindowFrames > 0) {
      float measured = windowUs * 1e-6f / odrWindowFrames;
      float nominal = 1.0f / GYRO_ODR_HZ;
      if (fabsf(measured - nominal) < 0.05f * nominal) dtSample += 0.2f * (measured - dtSample);
      odrWindowStartUs = micros();
      odrWindowFrames = 0;
    }
  }
  if (n > maxBatch) maxBatch = n;

  for (int i = 0; i < n; i++) {
    for (int k = 0; k < 3; k++) {
      if (frames[i][k] >= 32767 || frames[i][k] <= -32767) {
        gyroSaturations++;
        break;
      }
    }
    float raw[3] = {frames[i][0] / GYR_LSB_PER_DPS, frames[i][1] / GYR_LSB_PER_DPS,
                    frames[i][2] / GYR_LSB_PER_DPS};
    biasWindow.add(raw);
    if (biasWindow.n >= GYRO_BIAS_WINDOW) {
      float mean[3], spread;
      if (biasWindow.steady(mean, spread)) {
        for (int k = 0; k < 3; k++) gyroBiasDps[k] = mean[k];
        biasUpdates++;
      }
      biasWindow.reset();
    }
    gx = raw[0] - gyroBiasDps[0];
    gy = raw[1] - gyroBiasDps[1];
    gz = raw[2] - gyroBiasDps[2];

    float lpAlpha = dtSample / STILL_FILTER_TAU_S;
    gxLp += lpAlpha * (gx - gxLp);
    gyLp += lpAlpha * (gy - gyLp);
    gzLp += lpAlpha * (gz - gzLp);

    float rate = sqrtf(gxLp * gxLp + gyLp * gyLp + gzLp * gzLp);
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    bool still = accelValid && rate < STILL_GYRO_DPS && fabsf(accNorm - 1.0f) < STILL_ACCEL_TOL_G;
    stillTime = still ? stillTime + dtSample : 0.0f;
    atRest = stillTime >= STILL_TIME_REQUIRED_S;

    // During a move accel holds the motion as well as gravity. Letting it pull
    // on the attitude tilts the estimate, and that tilt leaks gravity into the
    // measured motion, so run on the gyro alone until the board stops.
    bool feedback = drState != DrState::Moving;
    bool useAccel = feedback && accelValid;
    bool useMag = feedback && magValid;
    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                 useAccel ? ax : 0.0f, useAccel ? ay : 0.0f, useAccel ? az : 0.0f,
                 useMag ? mx : 0.0f, useMag ? my : 0.0f, useMag ? mz : 0.0f,
                 dtSample, atRest);

    float gDps[3] = {gx, gy, gz};
    float aG[3] = {ax, ay, az};
    deadReckonStep(gDps, aG, dtSample);
  }

  static uint32_t lastLiveMs = 0;
  if (millis() - lastLiveMs >= LIVE_INTERVAL_MS) {
    lastLiveMs = millis();
    reportLive();
  }

  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    uint32_t elapsedMs = millis() - lastPrintMs;
    lastPrintMs = millis();
    reportState(); // repeated so a plot opened mid-session picks the state up
    if (drState != DrState::Moving) {
      float roll, pitch, yaw;
      quaternionToEuler(roll, pitch, yaw);
      float heading = yaw < 0.0f ? yaw + 360.0f : yaw;
      static const char *const stateName[] = {"SETTLING", "REST", "MOVING"};
      Serial.printf("Heading:%6.1f  Pitch:%6.1f  Roll:%6.1f  Gyro(dps) X:%7.2f Y:%7.2f Z:%7.2f\n",
                    heading, pitch, roll, gx, gy, gz);
      Serial.printf("DIAG rest=%d odr=%.1f accHz=%lu maxBatch=%d overruns=%lu saturations=%lu bias=%.4f %.4f %.4f (updates %lu)\n",
                    atRest, 1.0f / dtSample, (unsigned long)(accelNewCount * 1000UL / elapsedMs),
                    maxBatch, (unsigned long)overrunCount, (unsigned long)gyroSaturations,
                    gyroBiasDps[0], gyroBiasDps[1], gyroBiasDps[2], (unsigned long)biasUpdates);
      Serial.printf("DR %s moves=%d | linear accel peak %.3f g (motion above %.1f g, rest below)\n",
                    stateName[(int)drState], moveCount, peakLinG, MOTION_ACCEL_G);
    }
    maxBatch = 0;
    accelNewCount = 0;
    peakLinG = 0.0f;
  }
}
