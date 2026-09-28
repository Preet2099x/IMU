// ESP32 firmware for 3D tracking with the BMI088 (accelerometer + gyro) and BMM350
// (magnetometer). Wiring: SDA = GPIO21, SCL = GPIO22.
//
// What it does differently from the orientation firmware (src/main.cpp):
//   * Both the gyro and the accelerometer are read from their on-chip FIFOs at
//     400 Hz, so no sample is lost and every sample gets its own time.
//   * Everything is tracked on the board, sample by sample (src/tracker3d.h): the
//     orientation from the gyro, and the position from the accelerometer with
//     gravity removed. The accelerometer only corrects tilt while the board is
//     still, never during a move.
//   * Sensor reading and tracking run in their own task, so WiFi or a slow
//     connection can never make it miss samples.
//   * No magnetometer calibration at startup: heading is relative to where the
//     board faced when it first settled. The magnetometer is still read and
//     reported (uncalibrated) in the LOG stream.
//
// Talking to it: USB serial (115200) and TCP port 8888 over WiFi carry the same
// text protocol. Single letters at the start of a line:
//   t  keep the tracking stream on (TRK at 50 Hz, STAT at 1 Hz) for 3 s
//   r  as t, plus every raw sample (A/G lines, WiFi only) for replaying offline
//   v  the orientation stream the older viewers read (VIZ at 50 Hz) for 3 s
//   l  the dashboard stream (LOG at 20 Hz) for 3 s
//   h  back to the plain text line
//   s  ignored (the older viewers send it to skip the Teensy's magnetometer calibration)
// With no stream requested it prints, 10 times a second, the same line as the Teensy
// firmware (so the mapping viewer reads it), with the tracked position after it:
//   Heading: 12.3  Pitch: -1.0  Roll: 2.0  Gyro(dps) X: 0.01 Y: -0.02 Z: 0.00   x ... m
//   z  position back to zero        y  current heading becomes heading 0
// Lines:
//   c,ox,oy,oz,sx,sy,sz  accelerometer calibration for this session (g units)
//   C,ox,oy,oz,sx,sy,sz  the same, and remembered across restarts
// Software update over WiFi: pio run -e esp32_ota -t upload
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include <math.h>

#include "tracker3d.h"
#include "wifi_config.h"

using trk::Q;
using trk::V3;

// ---------------------------------------------------------------- sensors

constexpr uint8_t ACC_ADDR = 0x18;  // BMI088 accelerometer (chip ID 0x1E)
constexpr uint8_t GYR_ADDR = 0x68;  // BMI088 gyroscope (chip ID 0x0F)
constexpr uint8_t MAG_ADDR = 0x15;  // BMM350

constexpr float G0 = 9.80665f;
constexpr float ACC_LSB_PER_G = 32768.0f / 12.0f;        // ACC_RANGE 0x02: +-12 g
constexpr float GYR_LSB_PER_DPS = 32768.0f / 2000.0f;    // GYRO_RANGE 0x00: +-2000 deg/s
constexpr float DEG = 0.01745329252f;
constexpr float ODR_HZ = 400.0f;  // both sensors
constexpr int GYRO_CHUNK = 20;    // frames per I2C read: 120 bytes, fits the 128-byte Wire buffer
constexpr int ACC_CHUNK = 17;     // 119 bytes

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

void setupAccel() {
  writeReg(ACC_ADDR, 0x7E, 0xB6);  // soft reset
  delay(5);
  writeReg(ACC_ADDR, 0x7D, 0x04);  // ACC_PWR_CTRL: on
  delay(5);
  writeReg(ACC_ADDR, 0x7C, 0x00);  // ACC_PWR_CONF: active
  // ODR 400 Hz with 4x oversampling: a 37 Hz filter, close to the gyro's 47 Hz,
  // so both see a movement with about the same delay.
  writeReg(ACC_ADDR, 0x40, 0x8A);  // ACC_CONF
  writeReg(ACC_ADDR, 0x41, 0x02);  // ACC_RANGE: +-12 g (a knock on the table reached 6 g and clipped)
  writeReg(ACC_ADDR, 0x45, 0x80);  // FIFO_DOWNS: filtered data, no downsampling
  writeReg(ACC_ADDR, 0x48, 0x02);  // FIFO_CONFIG_0: stream mode (oldest dropped if full)
  writeReg(ACC_ADDR, 0x49, 0x50);  // FIFO_CONFIG_1: accelerometer data into the FIFO
  delay(50);
}

void setupGyro() {
  writeReg(GYR_ADDR, 0x14, 0xB6);  // soft reset
  delay(35);
  writeReg(GYR_ADDR, 0x0F, 0x00);  // GYRO_RANGE: +-2000 deg/s (a narrower range clips on bumps)
  writeReg(GYR_ADDR, 0x10, 0x03);  // GYRO_BANDWIDTH: ODR 400 Hz, 47 Hz filter
  writeReg(GYR_ADDR, 0x11, 0x00);  // GYRO_LPM1: normal mode
  writeReg(GYR_ADDR, 0x15, 0x40);  // GYRO_INT_CTRL: fifo_en
  writeReg(GYR_ADDR, 0x3D, 0x00);  // FIFO_CONFIG_0: no tags (6-byte frames)
  writeReg(GYR_ADDR, 0x3E, 0x40);  // FIFO_CONFIG_1: stop when full, overrun flagged
  delay(30);
}

void flushGyroFifo() { writeReg(GYR_ADDR, 0x3E, 0x40); }  // rewriting the mode clears it

void setupMag() {
  writeReg(MAG_ADDR, 0x7E, 0xB6);  // soft reset
  delay(50);
  writeReg(MAG_ADDR, 0x06, 0x01);  // PMU_CMD: normal mode
  writeReg(MAG_ADDR, 0x04, 0x04);  // PMU_CMD_AGGR_SET: ODR/averaging
  delay(10);
  writeReg(MAG_ADDR, 0x06, 0x07);  // PMU_CMD: magnetic reset
  delay(20);
}

int32_t sext24(uint32_t v) {
  if (v & 0x00800000) v |= 0xFF000000;
  return (int32_t)v;
}

bool readMagRaw(float out[3]) {
  uint8_t b[11];  // 2 dummy bytes + 9 data bytes
  if (!readBytes(MAG_ADDR, 0x31, b, 11)) return false;
  for (int k = 0; k < 3; k++)
    out[k] = sext24((uint32_t)b[2 + 3 * k] | ((uint32_t)b[3 + 3 * k] << 8) | ((uint32_t)b[4 + 3 * k] << 16));
  return true;
}

// Gyro FIFO: 6-byte frames. Returns the count; overrun means frames were lost.
int readGyroFifo(int16_t (*frames)[3], int maxFrames, bool &overrun) {
  uint8_t status;
  overrun = false;
  if (!readBytes(GYR_ADDR, 0x0E, &status, 1)) return 0;
  overrun = status & 0x80;
  int avail = status & 0x7F;
  if (avail > maxFrames) avail = maxFrames;
  int got = 0;
  uint8_t buf[GYRO_CHUNK * 6];
  while (got < avail) {
    int chunk = min(avail - got, GYRO_CHUNK);
    if (!readBytes(GYR_ADDR, 0x3F, buf, chunk * 6)) break;  // FIFO_DATA keeps popping frames
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

// Accelerometer FIFO: a header byte per frame (0x84 = data, followed by 6 bytes).
// Returns the count of data frames; glitch reports frames that could not be read
// in order (the FIFO is then emptied and the clock re-synced).
int readAccelFifo(int16_t (*frames)[3], int maxFrames, bool &glitch) {
  glitch = false;
  uint8_t lb[2];
  if (!readBytes(ACC_ADDR, 0x24, lb, 2)) return 0;
  int len = lb[0] | ((lb[1] & 0x3F) << 8);
  int got = 0;
  uint8_t buf[ACC_CHUNK * 7];
  while (len >= 7 && got < maxFrames) {
    int n = min(len / 7, min(ACC_CHUNK, maxFrames - got)) * 7;
    if (!readBytes(ACC_ADDR, 0x26, buf, n)) break;
    len -= n;
    int i = 0;
    while (i < n) {
      uint8_t h = buf[i];
      int size;
      if (h == 0x84) size = 7;
      else if (h == 0x40 || h == 0x48 || h == 0x50) size = 2;  // skip, config change, sample drop
      else if (h == 0x44) size = 4;                              // sensor time
      else if (h == 0x80) { size = n; }                          // read past the end
      else { glitch = true; size = n; }
      if (i + size > n && h != 0x80) { glitch = true; break; }  // frame split across reads
      if (h == 0x84 && got < maxFrames) {
        const uint8_t *b = buf + i + 1;
        frames[got][0] = (int16_t)(b[0] | (b[1] << 8));
        frames[got][1] = (int16_t)(b[2] | (b[3] << 8));
        frames[got][2] = (int16_t)(b[4] | (b[5] << 8));
        got++;
      }
      if (h == 0x40 || h == 0x50) glitch = true;  // samples were lost
      i += size;
    }
    if (glitch) break;
  }
  if (glitch) {  // empty it so the next read starts on a frame boundary
    for (int k = 0; k < 12 && readBytes(ACC_ADDR, 0x24, lb, 2); k++) {
      int left = lb[0] | ((lb[1] & 0x3F) << 8);
      if (left == 0) break;
      readBytes(ACC_ADDR, 0x26, buf, min(left, (int)sizeof(buf)));
    }
  }
  return got;
}

// Gives each FIFO frame a time on the board's clock. Frames come at the sensor's
// own rate (measured, since it can sit a percent off nominal), and the stamps are
// pulled gently toward the moment they were read so the two sensors stay aligned.
struct StreamClock {
  double period = 1e6 / ODR_HZ;  // microseconds per frame
  double tLast = 0;
  bool synced = false;
  int64_t winStart = 0;
  uint32_t winFrames = 0;
  uint32_t resyncs = 0;
  float rateHz = 0;

  void resync() { synced = false; }

  void stamp(int n, int64_t now, uint32_t *out) {
    if (n <= 0) return;
    if (!synced) {
      tLast = (double)now - period * n;
      synced = true;
      winStart = now;
      winFrames = 0;
      resyncs++;
    }
    for (int i = 0; i < n; i++) {
      tLast += period;
      out[i] = (uint32_t)(uint64_t)tLast;
    }
    double err = (double)now - tLast;
    if (fabs(err) > 20000.0) synced = false;  // a stall: start again from the clock
    else tLast += 0.02 * err;
    winFrames += n;
    if (now - winStart >= 2000000) {
      double measured = (double)(now - winStart) / winFrames;
      double nominal = 1e6 / ODR_HZ;
      if (fabs(measured - nominal) < 0.05 * nominal) period += 0.2 * (measured - period);
      rateHz = winFrames * 1e6f / (float)(now - winStart);
      winStart = now;
      winFrames = 0;
    }
  }
};

// ---------------------------------------------------------------- shared state

trk::Tracker tracker;
StreamClock accClock, gyrClock;

float accOffset[3] = {0, 0, 0};  // calibration, g: calibrated = (raw - offset) * scale
float accScale[3] = {1, 1, 1};

// Requests from the command side, applied by the sensor task.
volatile bool reqZeroPos = false, reqZeroYaw = false, reqCal = false, reqRaw = false;
float reqCalValues[6];

// What the output side reads, copied under a lock.
struct Snapshot {
  uint32_t ms;
  Q q;        // display frame: heading 0 where the user zeroed it
  V3 p, v;    // display frame, metres and m/s
  uint8_t state;
  uint32_t moves;
  float accStd, gyroMeanDps, gLocal, lastMoveTime, lastMoveVres;
  V3 bgDps;
  float accG[3], gyroDps[3], mag[3];  // latest, sensor axes (accel in g, calibrated)
  float accRawG[3];                   // latest, before calibration
  float accHz, gyrHz;
  uint32_t accGlitches, gyrOverruns, rawDrops;
};
Snapshot snap;
portMUX_TYPE snapLock = portMUX_INITIALIZER_UNLOCKED;

// Raw samples for offline replay: exactly what the tracker was fed.
struct RawSample {
  char kind;  // 'A' accel m/s^2, 'G' gyro rad/s (bias not removed), 'I' the INIT line
  uint32_t t;
  float x, y, z;
};
constexpr int RAW_RING = 1024;
RawSample rawRing[RAW_RING];
volatile uint32_t rawHead = 0, rawTail = 0;
volatile uint32_t rawDrops = 0;

void pushRaw(char kind, uint32_t t, V3 v) {
  uint32_t next = (rawHead + 1) % RAW_RING;
  if (next == rawTail) {
    rawDrops++;
    return;
  }
  rawRing[rawHead] = {kind, t, v.x, v.y, v.z};
  rawHead = next;
}

// ---------------------------------------------------------------- sensor task

float yaw0 = 0;           // heading that counts as 0
V3 origin{0, 0, 0};       // position that counts as 0
char initLine[256];       // tracker state at the start of a raw recording

void sensorTask(void *) {
  static int16_t gFrames[100][3], aFrames[150][3];
  static uint32_t gT[100], aT[150];
  uint32_t accGlitches = 0, gyrOverruns = 0;
  float mag[3] = {0, 0, 0};
  int64_t lastMagUs = 0;
  float accRawG[3] = {0, 0, 1}, accG[3] = {0, 0, 1}, gyroDps[3] = {0, 0, 0};
  bool rawOn = false;

  flushGyroFifo();
  bool dummy;
  readAccelFifo(aFrames, 150, dummy);  // throw away what piled up during setup

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2));  // about one new frame per sensor each time

    if (reqCal) {
      for (int i = 0; i < 3; i++) {
        accOffset[i] = reqCalValues[i];
        accScale[i] = reqCalValues[3 + i];
      }
      reqCal = false;
    }
    if (reqZeroPos) {
      origin = tracker.p;
      reqZeroPos = false;
    }
    if (reqZeroYaw) {
      yaw0 = trk::yawOf(tracker.q);
      reqZeroYaw = false;
    }
    if (reqRaw != rawOn) {
      rawOn = reqRaw;
      if (rawOn) {
        const trk::Tracker &T = tracker;
        snprintf(initLine, sizeof(initLine),
                 "INIT,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.5f,%.5f,%.5f,%.5f,%d,%.5f,%.5f,%.5f,%.6f,%.5f,%.5f,%.5f\n",
                 T.q.w, T.q.x, T.q.y, T.q.z, T.bg.x, T.bg.y, T.bg.z, T.p.x, T.p.y, T.p.z, T.gLocal, (int)T.state,
                 T.fRef.x, T.fRef.y, T.fRef.z, yaw0, origin.x, origin.y, origin.z);
        pushRaw('I', 0, V3{0, 0, 0});  // the samples that follow start from this state
      }
    }

    int64_t now = esp_timer_get_time();
    bool overrun, glitch;
    int ng = readGyroFifo(gFrames, 100, overrun);
    int na = readAccelFifo(aFrames, 150, glitch);
    if (overrun) {
      flushGyroFifo();
      gyrClock.resync();
      gyrOverruns++;
    }
    if (glitch) {
      accClock.resync();
      accGlitches++;
    }
    gyrClock.stamp(ng, now, gT);
    accClock.stamp(na, now, aT);

    // Feed both streams to the tracker in time order.
    int ig = 0, ia = 0;
    while (ig < ng || ia < na) {
      bool takeGyro = ia >= na || (ig < ng && (int32_t)(gT[ig] - aT[ia]) <= 0);
      if (takeGyro) {
        for (int k = 0; k < 3; k++) gyroDps[k] = gFrames[ig][k] / GYR_LSB_PER_DPS;
        V3 w{gyroDps[0] * DEG, gyroDps[1] * DEG, gyroDps[2] * DEG};
        tracker.gyro(gT[ig], w);
        if (rawOn) pushRaw('G', gT[ig], w);
        ig++;
      } else {
        for (int k = 0; k < 3; k++) {
          accRawG[k] = aFrames[ia][k] / ACC_LSB_PER_G;
          accG[k] = (accRawG[k] - accOffset[k]) * accScale[k];
        }
        V3 f{accG[0] * G0, accG[1] * G0, accG[2] * G0};
        tracker.accel(aT[ia], f);
        if (rawOn) pushRaw('A', aT[ia], f);
        ia++;
      }
    }

    if (now - lastMagUs >= 20000) {
      lastMagUs = now;
      readMagRaw(mag);
    }

    // Publish in the display frame.
    Q rz = trk::yawQuat(-yaw0);
    Snapshot s;
    s.ms = (uint32_t)(now / 1000);
    s.q = trk::qmul(rz, tracker.q);
    s.p = trk::rotate(rz, tracker.p - origin);
    s.v = trk::rotate(rz, tracker.v);
    s.state = tracker.lost ? 3 : tracker.state;  // 3: moving too long, position paused
    s.moves = tracker.moves;
    s.accStd = tracker.accStd;
    s.gyroMeanDps = tracker.gyroMean / DEG;
    s.gLocal = tracker.gLocal;
    s.lastMoveTime = tracker.lastMoveTime;
    s.lastMoveVres = trk::norm(tracker.lastMoveVres);
    s.bgDps = tracker.bg * (1.0f / DEG);
    for (int k = 0; k < 3; k++) {
      s.accG[k] = accG[k];
      s.accRawG[k] = accRawG[k];
      s.gyroDps[k] = gyroDps[k];
      s.mag[k] = mag[k];
    }
    s.accHz = accClock.rateHz;
    s.gyrHz = gyrClock.rateHz;
    s.accGlitches = accGlitches;
    s.gyrOverruns = gyrOverruns;
    s.rawDrops = rawDrops;
    portENTER_CRITICAL(&snapLock);
    snap = s;
    portEXIT_CRITICAL(&snapLock);
  }
}

// ---------------------------------------------------------------- WiFi and output

constexpr uint16_t TCP_PORT = 8888;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
bool netStarted = false;
volatile uint8_t lastDisconnectReason = 0;  // why the last connection attempt failed (ESP-IDF reason code)
bool newClient = false;
Preferences prefs;

void startNetworkServices() {
  WiFi.setSleep(false);  // power save makes the board drop packets and answer slowly
  MDNS.begin("imu");
  ArduinoOTA.setHostname("imu");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  netStarted = true;
  Serial.printf("WiFi: connected, IP %s, TCP port %u\n", WiFi.localIP().toString().c_str(), TCP_PORT);
}

void connectWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
#ifdef WIFI_STATIC_IP
  WiFi.config(IPAddress(WIFI_STATIC_IP), IPAddress(WIFI_GATEWAY), IPAddress(255, 255, 255, 0), IPAddress(WIFI_GATEWAY));
#endif
  // The library refuses networks weaker than WPA2 whenever a password is given, which
  // rejects an open hotspot. Allow any; on an open network the password is just ignored.
  WiFi.setMinSecurity(WIFI_AUTH_OPEN);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) { lastDisconnectReason = info.wifi_sta_disconnected.reason; },
               ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Not waited for: the loop starts serving as soon as the connection comes up, and USB
  // works meanwhile. (Waiting here kept USB silent for 15 s after every restart whenever
  // the network could not be reached.)
  Serial.printf("WiFi: connecting to \"%s\" in the background (2.4 GHz networks only)\n", WIFI_SSID);
}

uint32_t tcpDropped = 0;  // bytes that did not fit in the send buffer

// Output to the WiFi viewer goes through this buffer, so a slow patch on the link
// (the hotspot's ping swings up to 350 ms) is ridden out instead of losing data.
// Only whole lines go in; the loop pushes out whatever the link takes, never waiting
// (a blocking write to a viewer that had just been closed once froze this loop).
constexpr size_t TX_SIZE = 24576;
static char txBuf[TX_SIZE];
static size_t txHead = 0, txTail = 0;  // bytes live in [txTail, txHead), wrapping

size_t txUsed() { return (txHead + TX_SIZE - txTail) % TX_SIZE; }

void txFlush() {
  if (!tcpClient) {
    txHead = txTail = 0;
    return;
  }
  int fd = tcpClient.fd();
  if (fd < 0) return;
  while (txUsed() > 0) {
    size_t chunk = txHead >= txTail ? txHead - txTail : TX_SIZE - txTail;
    int sent = send(fd, txBuf + txTail, chunk, MSG_DONTWAIT);
    if (sent < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {  // the viewer went away
        tcpClient.stop();
        txHead = txTail = 0;
      }
      return;
    }
    txTail = (txTail + sent) % TX_SIZE;
    if ((size_t)sent < chunk) return;
  }
}

// Writes to USB serial (dropped rather than waited for if its buffer is full) and
// queues the same for the WiFi viewer.
void emit(const char *buf, size_t n, bool toSerial = true) {
  if (toSerial && Serial.availableForWrite() >= (int)n) Serial.write((const uint8_t *)buf, n);
  if (!tcpClient) return;
  if (txUsed() + n >= TX_SIZE - 1) {
    tcpDropped += n;
    return;
  }
  for (size_t i = 0; i < n; i++) {
    txBuf[txHead] = buf[i];
    txHead = (txHead + 1) % TX_SIZE;
  }
}

void emitf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) emit(buf, min(n, (int)sizeof(buf) - 1));
}

void loadCalibration() {
  prefs.begin("imu", true);
  float v[6];
  if (prefs.getBytes("acal", v, sizeof(v)) == sizeof(v)) {
    for (int i = 0; i < 3; i++) {
      accOffset[i] = v[i];
      accScale[i] = v[3 + i];
    }
    Serial.printf("accelerometer calibration loaded: %.4f %.4f %.4f / %.4f %.4f %.4f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
  } else {
    Serial.println("accelerometer: no stored calibration, using raw readings");
  }
  prefs.end();
}

void handleLine(const char *line) {
  float v[6];
  if ((line[0] != 'c' && line[0] != 'C') || line[1] != ',') return;
  if (sscanf(line + 2, "%f,%f,%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return;
  for (int i = 0; i < 3; i++)
    if (!(fabsf(v[i]) < 0.5f) || !(v[3 + i] > 0.5f && v[3 + i] < 2.0f)) return;  // not a plausible calibration
  for (int i = 0; i < 6; i++) reqCalValues[i] = v[i];
  reqCal = true;
  if (line[0] == 'C') {
    prefs.begin("imu", false);
    prefs.putBytes("acal", v, sizeof(v));
    prefs.end();
  }
  emitf("ACAL,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f%s\n", v[0], v[1], v[2], v[3], v[4], v[5], line[0] == 'C' ? ",saved" : "");
}

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== IMU 3D tracking (ESP32) ===");

  Wire.begin();  // SDA = GPIO21, SCL = GPIO22
  Wire.setClock(400000);
  setupAccel();
  setupGyro();
  setupMag();
  loadCalibration();

  xTaskCreatePinnedToCore(sensorTask, "sensors", 8192, nullptr, 5, nullptr, 1);
  Serial.println("Hold the board still for a second: tracking starts once it has settled.");
  connectWiFi();
}

void loop() {
  if (!netStarted && WiFi.status() == WL_CONNECTED) startNetworkServices();

  // The library retries a dropped connection but gives up if the network was not there
  // when the board started (a hotspot switched on later). So ask again every 15 s until
  // it is connected; a board on a battery then joins whenever the network appears.
  static uint32_t lastJoinTry = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastJoinTry > 15000) {
    lastJoinTry = millis();
    // Say what the board sees, so a network that is on the wrong band or hidden shows up.
    int n = WiFi.scanNetworks();
    int seen = 0;
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == WIFI_SSID) {
        seen++;
        Serial.printf("WiFi: sees \"%s\" channel %d, %d dBm, security %d\n", WIFI_SSID, WiFi.channel(i), WiFi.RSSI(i),
                      (int)WiFi.encryptionType(i));
      }
    }
    Serial.printf("WiFi: not connected (%d networks in range, \"%s\" %s), last disconnect reason %d; trying again\n", n,
                  WIFI_SSID, seen ? "visible" : "NOT visible", lastDisconnectReason);
    WiFi.scanDelete();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
  if (netStarted) ArduinoOTA.handle();

  if (netStarted) {
    WiFiClient c = tcpServer.available();
    if (c) {  // a new viewer always takes over, even if the old one vanished silently
      if (tcpClient) tcpClient.stop();
      txHead = txTail = 0;
      newClient = true;
      c.setNoDelay(true);
      tcpClient = c;
    }
  }

  // Commands, from either side.
  static uint32_t trkUntil = 0, rawUntil = 0, vizUntil = 0, logUntil = 0;
  static char cmd[96];
  static uint8_t cmdLen = 0;
  if (newClient) {  // a half-typed line from an earlier viewer must not swallow this one's commands
    cmdLen = 0;
    newClient = false;
  }
  for (;;) {
    int c = -1;
    if (Serial.available()) c = Serial.read();
    else if (tcpClient && tcpClient.available()) c = tcpClient.read();
    if (c < 0) break;
    uint32_t until = millis() + 3000;
    if (cmdLen == 0 && c == 't') trkUntil = until;
    else if (cmdLen == 0 && c == 'r') rawUntil = trkUntil = until;
    else if (cmdLen == 0 && c == 'v') vizUntil = until;
    else if (cmdLen == 0 && c == 'l') logUntil = until;
    else if (cmdLen == 0 && c == 'h') trkUntil = rawUntil = vizUntil = logUntil = 0;
    else if (cmdLen == 0 && c == 's') continue;
    else if (cmdLen == 0 && c == 'z') reqZeroPos = true;
    else if (cmdLen == 0 && c == 'y') reqZeroYaw = true;
    else if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmd[cmdLen] = 0;
        handleLine(cmd);
      }
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = (char)c;
    }
  }

  uint32_t nowMs = millis();
  auto active = [nowMs](uint32_t until) { return until != 0 && (int32_t)(until - nowMs) > 0; };
  bool trkOn = active(trkUntil), rawOn = active(rawUntil), vizOn = active(vizUntil), logOn = active(logUntil);
  reqRaw = rawOn && tcpClient;

  Snapshot s;
  portENTER_CRITICAL(&snapLock);
  s = snap;
  portEXIT_CRITICAL(&snapLock);

  // Raw samples first, batched into large writes.
  if (rawTail != rawHead) {
    static char big[1400];
    size_t used = 0;
    while (rawTail != rawHead) {
      const RawSample &r = rawRing[rawTail];
      if (r.kind == 'I') {
        if (used) emit(big, used, false);
        used = 0;
        emit(initLine, strlen(initLine), false);
        rawTail = (rawTail + 1) % RAW_RING;
        continue;
      }
      int n = r.kind == 'A' ? snprintf(big + used, sizeof(big) - used, "A,%lu,%.4f,%.4f,%.4f\n", (unsigned long)r.t, r.x, r.y, r.z)
                            : snprintf(big + used, sizeof(big) - used, "G,%lu,%.6f,%.6f,%.6f\n", (unsigned long)r.t, r.x, r.y, r.z);
      if (n <= 0 || used + n >= sizeof(big)) {
        emit(big, used, false);
        used = 0;
        continue;  // retry this sample in an empty buffer
      }
      used += n;
      rawTail = (rawTail + 1) % RAW_RING;
    }
    if (used) emit(big, used, false);
    txFlush();
  }

  static uint32_t lastTrk = 0, lastStat = 0, lastViz = 0, lastLog = 0, lastText = 0;
  float roll = atan2f(2 * (s.q.w * s.q.x + s.q.y * s.q.z), 1 - 2 * (s.q.x * s.q.x + s.q.y * s.q.y)) / DEG;
  float sp = 2 * (s.q.w * s.q.y - s.q.z * s.q.x);
  float pitch = asinf(sp > 1 ? 1 : (sp < -1 ? -1 : sp)) / DEG;
  float heading = trk::yawOf(s.q) / DEG;
  if (heading < 0) heading += 360;

  if (trkOn && nowMs - lastTrk >= 20) {
    lastTrk = nowMs - lastTrk < 60 ? lastTrk + 20 : nowMs;  // steady 50 Hz
    emitf("TRK,%lu,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%u,%lu\n", (unsigned long)s.ms, s.q.w, s.q.x,
          s.q.y, s.q.z, s.p.x, s.p.y, s.p.z, s.v.x, s.v.y, s.v.z, s.state, (unsigned long)s.moves);
  }
  if (trkOn && nowMs - lastStat >= 1000) {
    lastStat = nowMs;
    emitf("STAT,%.1f,%.1f,%lu,%lu,%lu,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.3f\n", s.accHz, s.gyrHz,
          (unsigned long)s.accGlitches, (unsigned long)s.gyrOverruns, (unsigned long)(s.rawDrops + tcpDropped / 30), s.accStd,
          s.gyroMeanDps, s.bgDps.x, s.bgDps.y, s.bgDps.z, s.gLocal, s.lastMoveTime, s.lastMoveVres);
  }
  if (vizOn && nowMs - lastViz >= 20) {
    lastViz = nowMs;
    emitf("VIZ,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f\n", (unsigned long)s.ms, s.q.w, s.q.x, s.q.y,
          s.q.z, s.accRawG[0], s.accRawG[1], s.accRawG[2], s.gyroDps[0], s.gyroDps[1], s.gyroDps[2]);
  }
  if (logOn && nowMs - lastLog >= 50) {
    lastLog = nowMs;
    emitf("LOG,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f\n", (unsigned long)s.ms, heading, pitch,
          roll, s.accG[0], s.accG[1], s.accG[2], s.gyroDps[0], s.gyroDps[1], s.gyroDps[2], s.mag[0], s.mag[1], s.mag[2]);
  }
  if (!trkOn && !vizOn && !logOn && nowMs - lastText >= 100) {
    lastText = nowMs;
    static const char *names[] = {"settling (hold still)", "still", "moving", "lost (hold still)"};
    // bias removed, like the Teensy prints it
    float gx = s.gyroDps[0] - s.bgDps.x, gy = s.gyroDps[1] - s.bgDps.y, gz = s.gyroDps[2] - s.bgDps.z;
    emitf("Heading:%6.1f  Pitch:%6.1f  Roll:%6.1f  Gyro(dps) X:%7.2f Y:%7.2f Z:%7.2f   x %+6.3f y %+6.3f z %+6.3f m  %s, %lu moves\n",
          heading, pitch, roll, gx, gy, gz, s.p.x, s.p.y, s.p.z, names[s.state > 3 ? 0 : s.state], (unsigned long)s.moves);
  }
  txFlush();
  delay(2);
}
