// ESP32 build of main.cpp (same firmware, same serial protocol). Differences: the
// gyro FIFO read chunk fits the ESP32 Wire buffer. Wiring: SDA=GPIO21, SCL=GPIO22.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "wifi_config.h"

// I2C addresses, confirmed by chip-ID readback on this board:
//   0x18 = BMI088 accelerometer (chip ID 0x1E)
//   0x68 = BMI088 gyroscope     (chip ID 0x0F)
//   0x15 = BMM350 magnetometer  (ADSEL high)
constexpr uint8_t ACC_ADDR = 0x18;
constexpr uint8_t GYR_ADDR = 0x68;
constexpr uint8_t MAG_ADDR = 0x15;

// Scale factors must match the ranges configured in setup().
constexpr float ACC_RANGE_G = 6.0f;                         // ACC_RANGE = 0x01
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
constexpr int GYRO_FIFO_CHUNK_FRAMES = 20; // 120 bytes, fits the ESP32 Wire's 128-byte buffer
constexpr uint32_t ACCEL_READ_INTERVAL_US = 5000;

// Gyro bias drifts as the BMI088 warms up, and accel can't observe yaw, so any
// leftover Z bias integrates straight into heading. While the board is still,
// keep nudging the bias toward what the gyro reads.
constexpr float STILL_GYRO_DPS = 0.5f;       // smoothed rate below this counts as still
constexpr float STILL_ACCEL_TOL_G = 0.1f;    // |accel| must be this close to 1g
constexpr float STILL_FILTER_TAU_S = 0.1f;   // smoothing of the rate used for detection
constexpr float STILL_TIME_REQUIRED_S = 1.0f;
constexpr float BIAS_ADAPT_TAU_S = 2.5f;
constexpr uint32_t PRINT_INTERVAL_MS = 100;  // 10 Hz

// Optional fast stream for the 3D visualizer. Sending 'v' at least every
// VIZ_TIMEOUT_MS swaps the text line for a 50 Hz CSV line; 'h' (or silence)
// switches back, so a plain serial monitor still sees the normal output.
constexpr uint32_t VIZ_INTERVAL_MS = 20;
constexpr uint32_t VIZ_TIMEOUT_MS = 3000;
constexpr uint32_t LOG_INTERVAL_MS = 50;   // 20 Hz for the long-run log

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
float magNoiseFloor[3] = {0.0f, 0.0f, 0.0f};

// Quaternion of the sensor frame relative to the earth frame.
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// The serial protocol is also served over WiFi: TCP port 8888, hostname "imu"
// (imu.local). Everything printed goes to USB serial and to the connected client,
// and commands are accepted from either.
constexpr uint16_t TCP_PORT = 8888;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

void pollTcpClient() {
  if (tcpClient && tcpClient.connected()) return;
  WiFiClient c = tcpServer.available();
  if (c) {
    c.setNoDelay(true);
    tcpClient = c;
  }
}

class Link : public Print {
 public:
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t *buf, size_t n) override {
    Serial.write(buf, n);
    if (tcpClient && tcpClient.connected()) tcpClient.write(buf, n);
    return n;
  }
  int available() {
    pollTcpClient();
    return Serial.available() + (tcpClient && tcpClient.connected() ? tcpClient.available() : 0);
  }
  int read() {
    if (Serial.available()) return Serial.read();
    if (tcpClient && tcpClient.connected() && tcpClient.available()) return tcpClient.read();
    return -1;
  }
};
Link out;

void connectWiFi() {
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    Serial.printf("\nWiFi: disconnected, reason %d\n", info.wifi_sta_disconnected.reason);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  int found = WiFi.scanNetworks();
  for (int i = 0; i < found; i++)
    Serial.printf("WiFi: sees \"%s\" ch%d %ddBm\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
#ifdef WIFI_STATIC_IP
  // A fixed address, so the viewer and OTA uploads always know where the board is.
  WiFi.config(IPAddress(WIFI_STATIC_IP), IPAddress(WIFI_GATEWAY), IPAddress(255, 255, 255, 0),
              IPAddress(WIFI_GATEWAY));
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi: connecting to \"%s\"", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false); // power-save makes the board drop packets and answer slowly
    MDNS.begin("imu");
    ArduinoOTA.setHostname("imu");
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin(); // software upload: pio run -e esp32_ota -t upload
    tcpServer.begin();
    Serial.printf("WiFi: connected, IP %s, TCP port %u (imu.local)\n",
                  WiFi.localIP().toString().c_str(), TCP_PORT);
  } else {
    Serial.println("WiFi: not connected (USB serial still works)");
  }
}

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
  writeReg(ACC_ADDR, 0x40, 0xA9); // ACC_CONF: normal filter, ODR 200 Hz
  writeReg(ACC_ADDR, 0x41, 0x01); // ACC_RANGE: ±6g
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

// Accelerometer calibration, set over serial by the 3D visualizer as
// "c,ox,oy,oz,sx,sy,sz": calibrated = (raw - offset) * scale, per axis, in g.
// Every use of the accelerometer (tilt, motion detection) goes through it.
float accOffset[3] = {0.0f, 0.0f, 0.0f};
float accScale[3] = {1.0f, 1.0f, 1.0f};
float accRaw[3] = {0.0f, 0.0f, 1.0f}; // last reading before the calibration is applied

bool readAccelG(float &x, float &y, float &z) {
  uint8_t b[6];
  if (!readBytes(ACC_ADDR, 0x12, b, 6)) return false;
  accRaw[0] = (int16_t)(b[0] | (b[1] << 8)) / ACC_LSB_PER_G;
  accRaw[1] = (int16_t)(b[2] | (b[3] << 8)) / ACC_LSB_PER_G;
  accRaw[2] = (int16_t)(b[4] | (b[5] << 8)) / ACC_LSB_PER_G;
  x = (accRaw[0] - accOffset[0]) * accScale[0];
  y = (accRaw[1] - accOffset[1]) * accScale[1];
  z = (accRaw[2] - accOffset[2]) * accScale[2];
  return true;
}

bool readGyroDps(float &x, float &y, float &z) {
  uint8_t b[6];
  if (!readBytes(GYR_ADDR, 0x02, b, 6)) return false;
  x = (int16_t)(b[0] | (b[1] << 8)) / GYR_LSB_PER_DPS - gyroBiasDps[0];
  y = (int16_t)(b[2] | (b[3] << 8)) / GYR_LSB_PER_DPS - gyroBiasDps[1];
  z = (int16_t)(b[4] | (b[5] << 8)) / GYR_LSB_PER_DPS - gyroBiasDps[2];
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

// Averages the gyro at rest so the constant part of the bias is removed before
// it gets integrated into the attitude estimate.
void calibrateGyroBias() {
  constexpr int samples = 500;
  double sx = 0, sy = 0, sz = 0;
  int taken = 0;

  for (int i = 0; i < samples; i++) {
    float x, y, z;
    if (readGyroDps(x, y, z)) {
      sx += x;
      sy += y;
      sz += z;
      taken++;
    }
    delay(3);
  }

  if (taken > 0) {
    gyroBiasDps[0] = sx / taken;
    gyroBiasDps[1] = sy / taken;
    gyroBiasDps[2] = sz / taken;
  }
  Serial.printf("gyro bias (dps): x=%.3f y=%.3f z=%.3f\n",
                gyroBiasDps[0], gyroBiasDps[1], gyroBiasDps[2]);
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
    ArduinoOTA.handle();
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
// large initial error. Seeding from the first accel sample starts it at the
// correct tilt instead.
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

// One command line from the PC. Only "c,ox,oy,oz,sx,sy,sz" is understood.
void handleCommand(const char *line) {
  float v[6];
  if (line[0] != 'c' || line[1] != ',') return;
  if (sscanf(line + 2, "%f,%f,%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return;
  for (int i = 0; i < 3; i++) {
    if (!(fabsf(v[i]) < 0.5f) || !(v[3 + i] > 0.5f && v[3 + i] < 2.0f)) return; // not a plausible calibration
  }
  for (int i = 0; i < 3; i++) {
    accOffset[i] = v[i];
    accScale[i] = v[3 + i];
  }
  Serial.printf("ACAL,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
}

void quaternionToEuler(float &rollDeg, float &pitchDeg, float &yawDeg) {
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;

  rollDeg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  pitchDeg = asinf(sinp) * RAD_TO_DEG;
  yawDeg = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 15000) {} // wait for the serial monitor to attach
  delay(300);

  Wire.begin(); // default I2C pins: SDA = GPIO21, SCL = GPIO22
  Wire.setClock(400000);
  connectWiFi();

  setupBMI088Accel();
  setupBMI088Gyro();
  setupBMM350();

  out.println();
  out.println("=== IMU startup ===");
  out.println("Keep the board STILL for gyro bias calibration...");
  delay(1500);
  calibrateGyroBias();
  measureMagNoiseFloor();

  out.println();
  out.println("Send any character within 5s to SKIP mag calibration.");
  bool skip = false;
  uint32_t waitStart = millis();
  while (millis() - waitStart < 5000) {
    ArduinoOTA.handle();
    if (out.available()) {
      while (out.available()) out.read();
      skip = true;
      break;
    }
  }

  if (skip) {
    out.println("Skipped. Mag stays out of the fusion, yaw is relative and will drift.");
  } else {
    calibrateMagnetometer();
  }

  seedOrientationFromAccel();
  flushGyroFifo(); // it filled up and overran while calibration was running
  out.println();
  out.println("=== running ===");
}

void loop() {
  ArduinoOTA.handle();
  static uint32_t lastPrintMs = millis();
  static uint32_t lastAccelUs = 0;
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

  // From the PC: 'v' keeps the fast stream on, 'h' turns it off, and a line
  // starting with "c," sets the accelerometer calibration.
  static uint32_t vizUntilMs = 0, lastVizMs = 0, logUntilMs = 0, lastLogMs = 0;
  static char cmd[96];
  static uint8_t cmdLen = 0;
  while (out.available()) {
    char c = out.read();
    if (cmdLen == 0 && c == 'v') vizUntilMs = millis() + VIZ_TIMEOUT_MS;
    else if (cmdLen == 0 && c == 'h') vizUntilMs = logUntilMs = 0;
    else if (cmdLen == 0 && c == 'l') logUntilMs = millis() + VIZ_TIMEOUT_MS;
    else if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmd[cmdLen] = 0;
        handleCommand(cmd);
      }
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = c;
    }
  }

  uint32_t nowUs = micros();
  if (nowUs - lastAccelUs >= ACCEL_READ_INTERVAL_US) {
    lastAccelUs = nowUs;
    accelValid = readAccelG(ax, ay, az);
    bool magOk = readMag(mx, my, mz); // also read when not fused, so the log mode can report it
    magValid = useMagInFusion && magOk;
  }

  static uint32_t lastFifoUs = 0;
  if (nowUs - lastFifoUs < 2000) return; // ~1 new frame per 2.5 ms, no need to hammer I2C
  lastFifoUs = nowUs;

  int16_t frames[100][3];
  bool overrun;
  int n = readGyroFifo(frames, 100, overrun);
  if (overrun) {
    // Samples were dropped, so this batch doesn't cover the elapsed time.
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
  for (int i = 0; i < n; i++) {
    gx = frames[i][0] / GYR_LSB_PER_DPS - gyroBiasDps[0];
    gy = frames[i][1] / GYR_LSB_PER_DPS - gyroBiasDps[1];
    gz = frames[i][2] / GYR_LSB_PER_DPS - gyroBiasDps[2];

    float lpAlpha = dtSample / STILL_FILTER_TAU_S;
    gxLp += lpAlpha * (gx - gxLp);
    gyLp += lpAlpha * (gy - gyLp);
    gzLp += lpAlpha * (gz - gzLp);

    float rate = sqrtf(gxLp * gxLp + gyLp * gyLp + gzLp * gzLp);
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);
    bool still = accelValid && rate < STILL_GYRO_DPS && fabsf(accNorm - 1.0f) < STILL_ACCEL_TOL_G;
    stillTime = still ? stillTime + dtSample : 0.0f;
    atRest = stillTime >= STILL_TIME_REQUIRED_S;

    if (atRest) {
      float adapt = dtSample / BIAS_ADAPT_TAU_S;
      gyroBiasDps[0] += adapt * gx;
      gyroBiasDps[1] += adapt * gy;
      gyroBiasDps[2] += adapt * gz;
    }

    mahonyUpdate(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD,
                 accelValid ? ax : 0.0f, accelValid ? ay : 0.0f, accelValid ? az : 0.0f,
                 magValid ? mx : 0.0f, magValid ? my : 0.0f, magValid ? mz : 0.0f,
                 dtSample, atRest);
  }

  bool viz = vizUntilMs != 0 && (int32_t)(vizUntilMs - millis()) > 0;
  if (viz) {
    if (millis() - lastVizMs >= VIZ_INTERVAL_MS) {
      lastVizMs = millis();
      // Orientation, the accelerometer exactly as the chip reports it (g, sensor
      // frame, before calibration), and the rotation rate (deg/s).
      out.printf("VIZ,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f\n",
                    (unsigned long)millis(), q0, q1, q2, q3, accRaw[0], accRaw[1], accRaw[2], gx, gy, gz);
    }
  } else if (logUntilMs != 0 && (int32_t)(logUntilMs - millis()) > 0) {
    // Logging: orientation plus every sensor. Accelerometer in g (calibrated if a
    // calibration was set), gyro in deg/s (bias removed), magnetometer as the
    // fusion sees it (counts, calibrated once the mag calibration has run).
    if (millis() - lastLogMs >= LOG_INTERVAL_MS) {
      lastLogMs = millis();
      float roll, pitch, yaw;
      quaternionToEuler(roll, pitch, yaw);
      float heading = yaw < 0.0f ? yaw + 360.0f : yaw;
      out.printf("LOG,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    (unsigned long)millis(), heading, pitch, roll, ax, ay, az, gx, gy, gz, mx, my, mz);
    }
  } else if (millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);
    float heading = yaw < 0.0f ? yaw + 360.0f : yaw;
    out.printf("Heading:%6.1f  Pitch:%6.1f  Roll:%6.1f  Gyro(dps) X:%7.2f Y:%7.2f Z:%7.2f\n",
                  heading, pitch, roll, gx, gy, gz);
  }
}
