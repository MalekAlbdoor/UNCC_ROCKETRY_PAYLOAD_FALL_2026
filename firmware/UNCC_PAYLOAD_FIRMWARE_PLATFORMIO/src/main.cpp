#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include "stm32yyxx_ll_adc.h"

// 1 = live (10 Hz, SF8), 0 = max range (1 Hz, SF12). Must match the base station.
#define LIVE 1

#define FREQ  915.0
#define BW    500.0
#define TX_DBM 22                               // not PWR: the STM32 headers already use that name
#define TCXO  1.8
#if LIVE
  #define SF  8
  #define CR  5
  #define TX_PERIOD_MS 100
#else
  #define SF  12
  #define CR  8
  #define TX_PERIOD_MS 1000
#endif

const int LED = PC1;                              // silkscreen 13
SX1262 radio = new Module(PB9, PC2, PB8, PC3);    // CS 10, DIO1 12, RST 9, BUSY 11

// sensors: SPI on A0-A5, or I2C over STEMMA QT, picked at boot
SPIClass sensorSPI(PA7, PA6, PA5);                // MOSI A3, MISO A2, SCK A1
SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE3);
Uart gpsSerial(PB11, PB10);                       // GPS UART on the Feather's RX, TX pins

const float G0 = 9.80665f;

enum Bus : uint8_t { NONE, VIA_SPI, VIA_I2C };
struct Sensor {
  const char *name; int cs; uint8_t addrs[2]; uint8_t whoami;
  uint8_t spiInc, i2cInc;                         // auto-increment bit for multi-byte reads
  Bus bus; uint8_t addr;
};
Sensor imu  = {"LSM6DSO32", PA4, {0x6A, 0x6B}, 0x6C, 0x00, 0x00, NONE, 0};   // CS A0
Sensor hig  = {"H3LIS331",  PC4, {0x18, 0x19}, 0x32, 0x40, 0x80, NONE, 0};   // CS A4
Sensor baro = {"MS5611",    PC5, {0x77, 0x76}, 0,    0,    0,    NONE, 0};   // CS A5, no WHO_AM_I
uint8_t shtAddr = 0;                              // SHT45 at 0x44 when found
uint8_t gpsI2C = 0;                               // 0x42 when the GPS answers on I2C, else UART only

enum Phase : uint8_t { PAD, BOOST, COAST, DESCENT, LANDED };

struct __attribute__((packed)) Telem {
  uint8_t  magic;      // 'X' ('W' was 73 bytes without the magnetometer, 'V' 71 without the battery)
  uint8_t  flags;      // bit0 IMU, bit1 high-g, bit2 baro, bit3 SHT45, bit4 GPS talking, bit5 magnetometer
  uint16_t seq;
  uint32_t ms;
  int16_t  acc[3];     // LSM6DSO32 accel, 0.976 mg/LSB (+-32 g)
  int16_t  gyr[3];     // LSM6DSO32 gyro, 70 mdps/LSB (+-2000 dps)
  int16_t  hig[3];     // H3LIS331 accel, 98 mg/LSB (+-200 g)
  uint16_t peak;       // peak |accel| since last packet, 0.1 g/LSB
  int16_t  temp;       // IMU die temp, 0.01 C/LSB
  uint8_t  state;      // Phase
  int32_t  alt;        // altitude above the pad, cm (baro + accel filter)
  int16_t  vel;        // vertical velocity, 0.1 m/s, up is positive
  uint16_t maxAlt;     // highest altitude since launch, m
  uint16_t maxVel;     // fastest climb since launch, 0.1 m/s
  uint32_t pres;       // MS5611 pressure, Pa
  int16_t  airT;       // SHT45 air temp, 0.01 C
  uint16_t rh;         // SHT45 humidity, 0.01 %
  uint8_t  fix;        // GPS fix: 0 none, 2 2D, 3 3D
  uint8_t  sats;
  int32_t  lat, lon;   // 1e-7 deg
  int16_t  gpsAlt;     // m above sea level
  int16_t  gpsVup;     // GPS vertical velocity, 0.1 m/s
  uint16_t gpsSpd;     // GPS ground speed, 0.1 m/s
  int16_t  roll, pitch, yaw;   // attitude, 0.01 deg; all 0 standing on the pad
  uint16_t vbat;       // battery, mV
  uint16_t heading;    // compass bearing the nose leans toward, 0.01 deg from magnetic north; 0xFFFF unknown
  uint16_t magField;   // field strength, mG (the Earth's is about 500)
};
// 77 bytes is the same airtime as 73 at SF12 and 5 symbols (2.6 ms) more at SF8
static_assert(sizeof(Telem) == 77, "base station expects 77 bytes");

Telem pkt = {'X'};
float peakG = 0;
volatile bool txDone = false;
bool txBusy = false;
void onTxDone() { txDone = true; }

// ---------- register access (IMU, high-g) ----------
void readRegs(Sensor &s, uint8_t reg, uint8_t *buf, uint8_t n) {
  if (s.bus == VIA_SPI) {
    sensorSPI.beginTransaction(spiCfg);
    digitalWrite(s.cs, LOW);
    sensorSPI.transfer(reg | 0x80 | (n > 1 ? s.spiInc : 0));
    for (uint8_t i = 0; i < n; i++) buf[i] = sensorSPI.transfer(0);
    digitalWrite(s.cs, HIGH);
    sensorSPI.endTransaction();
  } else {
    Wire.beginTransmission(s.addr);
    Wire.write(reg | (n > 1 ? s.i2cInc : 0));
    Wire.endTransmission(false);
    Wire.requestFrom(s.addr, n);
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  }
}

uint8_t readReg(Sensor &s, uint8_t reg) { uint8_t v; readRegs(s, reg, &v, 1); return v; }

void writeReg(Sensor &s, uint8_t reg, uint8_t val) {
  if (s.bus == VIA_SPI) {
    sensorSPI.beginTransaction(spiCfg);
    digitalWrite(s.cs, LOW);
    sensorSPI.transfer(reg & 0x3F);
    sensorSPI.transfer(val);
    digitalWrite(s.cs, HIGH);
    sensorSPI.endTransaction();
  } else {
    Wire.beginTransmission(s.addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
}

bool detect(Sensor &s) {
  s.bus = VIA_SPI;
  if (readReg(s, 0x0F) == s.whoami) return true;
  s.bus = VIA_I2C;
  for (uint8_t a : s.addrs) { s.addr = a; if (readReg(s, 0x0F) == s.whoami) return true; }
  s.bus = NONE;
  return false;
}

bool i2cAck(uint8_t addr) { Wire.beginTransmission(addr); return Wire.endTransmission() == 0; }

void report(const char *name, Bus bus, uint8_t addr) {
  Serial.print(name); Serial.print(": ");
  if (bus == VIA_SPI) Serial.println("SPI");
  else if (bus == VIA_I2C) { Serial.print("I2C 0x"); Serial.println(addr, HEX); }
  else Serial.println("NOT FOUND");
}

// ---------- MS5611 barometer ----------
// Command based, no registers. A conversion takes 9 ms at OSR 4096, so it runs as a small
// state machine: start a conversion, come back later, read it, start the next. Temperature
// is refreshed every 10th conversion, which gives about 100 pressure readings a second.
uint16_t baroC[8];
uint32_t baroD2 = 0, baroAt = 0;
uint8_t baroStep = 0;

uint32_t baroCmd(uint8_t cmd, uint8_t n) {
  uint32_t v = 0;
  if (baro.bus == VIA_SPI) {
    sensorSPI.beginTransaction(spiCfg);
    digitalWrite(baro.cs, LOW);
    sensorSPI.transfer(cmd);
    for (uint8_t i = 0; i < n; i++) v = v << 8 | sensorSPI.transfer(0);
    digitalWrite(baro.cs, HIGH);
    sensorSPI.endTransaction();
  } else {
    Wire.beginTransmission(baro.addr);
    Wire.write(cmd);
    Wire.endTransmission();
    if (n) {                                      // a short read is no reading, not 0xFF filler
      if (Wire.requestFrom(baro.addr, n) != n) return 0;
      for (uint8_t i = 0; i < n; i++) v = v << 8 | Wire.read();
    }
  }
  return v;
}

// AN520 CRC-4 over the 8 PROM words, compared with the 4 bits stored in word 7
bool baroPromOk() {
  if (baroC[1] == 0 || baroC[1] == 0xFFFF) return false;   // nothing answering reads all 0s or all 1s
  uint16_t rem = 0;
  for (int i = 0; i < 16; i++) {
    uint16_t w = i == 15 ? 0 : (i == 14 ? baroC[7] & 0xFF00 : baroC[i >> 1]);
    rem ^= (i & 1) ? (w & 0xFF) : (w >> 8);
    for (int b = 0; b < 8; b++) rem = (rem & 0x8000) ? (rem << 1) ^ 0x3000 : rem << 1;
  }
  return ((rem >> 12) & 0xF) == (baroC[7] & 0xF);
}

bool baroTry() {
  baroCmd(0x1E, 0); delay(3);                      // reset, reloads the PROM
  for (int i = 0; i < 8; i++) baroC[i] = baroCmd(0xA0 + 2 * i, 2);
  return baroPromOk();
}

bool detectBaro() {
  baro.bus = VIA_SPI;
  if (baroTry()) return true;
  baro.bus = VIA_I2C;
  for (uint8_t a : baro.addrs) { baro.addr = a; if (i2cAck(a) && baroTry()) return true; }
  baro.bus = NONE;
  return false;
}

// datasheet first- and second-order compensation; returns pressure in Pa
uint32_t baroCompute(uint32_t D1, uint32_t D2, int32_t *tempC100) {
  int64_t dT = (int64_t)D2 - ((int64_t)baroC[5] << 8);
  int64_t TEMP = 2000 + (dT * baroC[6] >> 23);
  int64_t OFF = ((int64_t)baroC[2] << 16) + ((int64_t)baroC[4] * dT >> 7);
  int64_t SENS = ((int64_t)baroC[1] << 15) + ((int64_t)baroC[3] * dT >> 8);
  if (TEMP < 2000) {
    int64_t T2 = dT * dT >> 31, d = TEMP - 2000;
    int64_t OFF2 = 5 * d * d / 2, SENS2 = 5 * d * d / 4;
    if (TEMP < -1500) { int64_t e = TEMP + 1500; OFF2 += 7 * e * e; SENS2 += 11 * e * e / 2; }
    TEMP -= T2; OFF -= OFF2; SENS -= SENS2;
  }
  *tempC100 = (int32_t)TEMP;
  return (uint32_t)((((int64_t)D1 * SENS >> 21) - OFF) >> 15);
}

// ---------- SHT45 air temperature and humidity ----------
uint8_t crc8(const uint8_t *d, uint8_t n) {
  uint8_t c = 0xFF;
  for (uint8_t i = 0; i < n; i++) { c ^= d[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0x31 : c << 1; }
  return c;
}

bool shtRead6(uint8_t *b) {
  if (Wire.requestFrom((uint8_t)0x44, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  return crc8(b, 2) == b[2] && crc8(b + 3, 2) == b[5];
}

bool detectSht() {
  if (!i2cAck(0x44)) return false;
  Wire.beginTransmission(0x44); Wire.write(0x89); Wire.endTransmission();   // read serial number
  delay(2);
  uint8_t b[6];
  return shtRead6(b);
}

// ---------- u-blox GPS (SAM-M10Q) ----------
// Set to UBX NAV-PVT only, 4 Hz, airborne dynamics. Works over the UART (as on the PCB) or
// over I2C at 0x42 (SparkFun Qwiic breakouts). The settings live in RAM, so they are sent
// again whenever the GPS goes quiet, which also covers a GPS that powers up late.
uint32_t gpsLastPvt = 0, gpsLastCfg = 0;

void gpsSet(uint32_t key, uint32_t val, uint8_t size) {   // UBX-CFG-VALSET, RAM layer
  uint8_t f[20] = {0xB5, 0x62, 0x06, 0x8A, (uint8_t)(8 + size), 0, 0, 0x01, 0, 0};
  memcpy(f + 10, &key, 4);
  memcpy(f + 14, &val, size);
  uint8_t n = 14 + size, a = 0, b = 0;
  for (uint8_t i = 2; i < n; i++) { a += f[i]; b += a; }
  f[n] = a; f[n + 1] = b;
  gpsSerial.write(f, n + 2);
  if (gpsI2C) { Wire.beginTransmission(gpsI2C); Wire.write(f, n + 2); Wire.endTransmission(); }
}

void gpsConfigure() {
  gpsI2C = i2cAck(0x42) ? 0x42 : 0;
  gpsSet(0x20110021, 8, 1);                       // CFG-NAVSPG-DYNMODEL: airborne < 4 g
  gpsSet(0x30210001, 250, 2);                     // CFG-RATE-MEAS: 250 ms
  gpsSet(0x20910007, 1, 1);                       // CFG-MSGOUT-UBX_NAV_PVT_UART1
  gpsSet(0x20910006, 1, 1);                       // CFG-MSGOUT-UBX_NAV_PVT_I2C
  gpsSet(0x10740002, 0, 1);                       // CFG-UART1OUTPROT-NMEA off
  gpsSet(0x10720002, 0, 1);                       // CFG-I2COUTPROT-NMEA off
}

void gpsPvt(const uint8_t *p) {
  int32_t v[10];
  memcpy(v, p + 24, sizeof(v));                   // lon, lat, height, hMSL, hAcc, vAcc, velN, velE, velD, gSpeed
  uint8_t type = p[20];
  pkt.fix = (p[21] & 1) && type >= 2 && type <= 4 ? (type > 3 ? 3 : type) : 0;
  pkt.sats = p[23];
  pkt.lon = v[0];
  pkt.lat = v[1];
  pkt.gpsAlt = v[3] / 1000;
  pkt.gpsVup = -v[8] / 100;
  pkt.gpsSpd = v[9] / 100;
  gpsLastPvt = millis();
}

void gpsByte(uint8_t c) {                         // keeps only NAV-PVT frames with a good checksum
  static uint8_t f[6 + 92 + 2];
  static uint16_t n = 0;
  if ((n == 0 && c != 0xB5) || (n == 1 && c != 0x62)) { n = c == 0xB5; return; }
  f[n++] = c;
  if (n == 6 && (f[2] != 0x01 || f[3] != 0x07 || (f[4] | f[5] << 8) != 92)) n = 0;
  else if (n == sizeof(f)) {
    n = 0;
    uint8_t a = 0, b = 0;
    for (uint16_t i = 2; i < 98; i++) { a += f[i]; b += a; }
    if (a == f[98] && b == f[99]) gpsPvt(f + 6);
  }
}

void gpsPoll() {
  static uint32_t lastI2c = 0;
  while (gpsSerial.available()) gpsByte(gpsSerial.read());
  if (gpsI2C && millis() - lastI2c >= 50) {
    lastI2c = millis();
    Wire.beginTransmission(gpsI2C); Wire.write(0xFD); Wire.endTransmission(false);
    if (Wire.requestFrom(gpsI2C, (uint8_t)2) == 2) {
      uint16_t avail = Wire.read() << 8; avail |= Wire.read();
      if (avail == 0xFFFF) avail = 0;
      for (uint8_t chunk = 0; chunk < 3 && avail; chunk++) {   // the Wire buffer is 32 bytes
        uint8_t n = avail > 32 ? 32 : avail;
        Wire.requestFrom(gpsI2C, n);                // the register pointer now sits on the 0xFF stream
        while (Wire.available()) gpsByte(Wire.read());
        avail -= n;
      }
    }
  }
  if (millis() - gpsLastPvt > 3000 && millis() - gpsLastCfg > 5000) { gpsLastCfg = millis(); gpsConfigure(); }
}

bool gpsTalking() { return gpsLastPvt && millis() - gpsLastPvt < 3000; }

// ---------- altitude, velocity and flight phase ----------
// A two-state Kalman filter (altitude, vertical velocity). The accelerometers drive it at
// 400 Hz and the barometer corrects it. Without a barometer it is accel only and drifts, so
// velocity is pinned to zero whenever the board sits still on the pad or after landing, and
// altitude to zero while it sits on the pad.
// Vertical acceleration comes from the IMU turned into the world frame by the attitude filter,
// so turning the board, or the rocket arcing over, doesn't read as climbing or falling. Only the
// high-g accelerometer (used past the IMU's 30 g, or with no IMU) has no attitude to go by: for
// it "up" is whichever way gravity pulled while it sat still, which assumes the rocket flies
// roughly along the axis it stood on, as it does under that kind of thrust.
struct Up { float x, y, z; bool ok; };            // gravity as read at rest, in sensor axes
Up upImu = {}, upHig = {};
float kh = 0, kv = 0, P00 = 1, P01 = 0, P11 = 1;
float p0 = 0, maxAlt = 0, maxVel = 0;
Phase phase = PAD;
uint32_t stillN = 0;

void learn(Up &u, float x, float y, float z) {
  const float a = 0.002f;                         // about a 1 s average at 400 Hz
  if (!u.ok) { u = {x, y, z, true}; return; }
  u.x += a * (x - u.x); u.y += a * (y - u.y); u.z += a * (z - u.z);
}

float upAccel(const Up &u, float x, float y, float z) {   // m/s^2 along up, gravity removed
  float g = sqrtf(u.x * u.x + u.y * u.y + u.z * u.z);
  return ((x * u.x + y * u.y + z * u.z) / g - g) * G0;
}

void kfPredict(float a, float dt) {
  const float q = 4.0f;                           // accel noise, (2 m/s^2)^2
  kh += kv * dt + 0.5f * a * dt * dt;
  kv += a * dt;
  P00 += dt * (2 * P01 + dt * P11) + q * dt * dt * dt * dt / 4;
  P01 += dt * P11 + q * dt * dt * dt / 2;
  P11 += q * dt * dt;
}

void kfBaro(float h) {
  const float r = 0.25f;                          // baro noise, (0.5 m)^2
  float s = P00 + r, k0 = P00 / s, k1 = P01 / s, y = h - kh;
  kh += k0 * y; kv += k1 * y;
  P11 -= k1 * P01; P00 *= 1 - k0; P01 *= 1 - k0;
}

// aUp: the IMU's acceleration along world up, gravity included, in g (NAN without an IMU)
void estimate(float x, float y, float z, float hx, float hy, float hz, float gyro, float aUp, float dt) {
  float lsm = sqrtf(x * x + y * y + z * z), hi = sqrtf(hx * hx + hy * hy + hz * hz);
  bool useImu = imu.bus && lsm < 30, ok = imu.bus || hig.bus;
  if (!ok) return;

  // still = a steady reading of about 1 g and not rotating, for half a second. The accelerometer
  // reads gravity a few % differently depending on which way it lies, so "about 1 g" is loose
  // and steadiness against its own 0.25 s average does the work. It is checked against 1 g, not
  // the learned up vector: while that re-learns after the board is turned it averages old and new
  // directions and comes up short, which would stop it counting as still halfway through.
  const Up &ref = imu.bus ? upImu : upHig;
  float g = sqrtf(ref.x * ref.x + ref.y * ref.y + ref.z * ref.z);
  static float lsmSlow = 1;
  lsmSlow += 0.01f * (lsm - lsmSlow);
  bool calm = imu.bus ? fabsf(lsm - 1) < 0.15f && fabsf(lsm - lsmSlow) < 0.02f && gyro < 8 : fabsf(hi - g) < 0.3f;
  stillN = calm ? stillN + 1 : 0;
  bool still = stillN >= 200 || !ref.ok;
  bool grounded = phase == PAD || phase == LANDED;
  if (still && grounded) {
    if (imu.bus) learn(upImu, x, y, z);
    if (hig.bus) learn(upHig, hx, hy, hz);
  }

  // what the IMU reads for 1 g lying the way it lies now (a few % off, and not the same every way)
  static float gRef = 0;
  if (still && grounded && imu.bus) gRef = gRef ? gRef + 0.002f * (lsm - gRef) : lsm;
  float a = useImu ? (aUp - (gRef ? gRef : 1)) * G0 : upAccel(upHig, hx, hy, hz);
  kfPredict(a, dt);
  if (still && grounded) { kv = 0; P01 = 0; P11 = 0.01f; }   // zero-velocity update
  // with no barometer nothing corrects the altitude, and sitting on the pad it is zero by definition
  if (still && phase == PAD && !baro.bus) { kh = 0; P00 = 0.01f; }

  static uint16_t hot = 0, cool = 0, falling = 0;
  switch (phase) {
    case PAD:   hot = a > 3 * G0 ? hot + 1 : 0;   if (hot >= 60) phase = BOOST;       break;  // 3 g net for 0.15 s
    case BOOST: cool = a < 0 ? cool + 1 : 0;      if (cool >= 20) phase = COAST;      break;  // burnout
    case COAST: falling = kv < 0 ? falling + 1 : 0; if (falling >= 80) phase = DESCENT; break;  // apogee
    default: break;
  }
  // still for 5 s; hanging steady under the chute reads 1 g too, so with a barometer it must also
  // have stopped coming down
  if (phase != PAD && phase != LANDED && stillN >= 2000 && (!baro.bus || fabsf(kv) < 2)) phase = LANDED;
  if (phase != PAD) { maxAlt = max(maxAlt, kh); maxVel = max(maxVel, kv); }
}

// One barometer altitude into the filter. Near Mach 1 the static port reads wrong, so the filter
// coasts on the accels then, but only in flight: anywhere else a velocity that high is the filter
// gone wrong, and the barometer is what brings it back. A reading far off what the filter expects
// (a glitch on the bus) is skipped, unless they stay that far off for about a second, which means
// the filter is the one that's wrong; then it starts over from the barometer.
void baroAlt(float h) {
  static uint8_t skipped = 0;
  if ((phase == BOOST || phase == COAST) && fabsf(kv) > 200) return;
  float y = h - kh;
  bool far = fabsf(y) > 30 && y * y > 36 * (P00 + 0.25f);   // more than 30 m and 6 sigma off
  if (far && skipped < 100) { skipped++; return; }
  if (far) { kh = h; P00 = 0.25f; P01 = 0; }
  skipped = 0;
  kfBaro(h);
}

void baroPoll() {
  static uint32_t paSum = 0;
  static uint8_t zeroN = 0;
  if (!baro.bus || micros() - baroAt < 9100) return;
  uint32_t raw = baroCmd(0x00, 3);
  bool wasTemp = baroStep == 0;
  baroStep = (baroStep + 1) % 10;
  baroCmd(baroStep == 0 ? 0x58 : 0x48, 0);        // next conversion: D2 (temp) or D1 (pressure), OSR 4096
  baroAt = micros();
  if (!raw || raw >= 0xFFFFFF) return;            // nothing converted, or a bus that read all 1s
  if (wasTemp) { baroD2 = raw; return; }
  if (!baroD2) return;

  int32_t t;
  uint32_t pa = baroCompute(raw, baroD2, &t);
  if (t < -4000 || t > 8500 || pa < 1000 || pa > 120000) return;   // outside the sensor's range: garbled
  pkt.pres = pa;
  // The pad is altitude zero. The first readings after power-up run low while the sensor settles
  // (by about 1 hPa, which is 7 m), so skip half a second, then average 32. After that, follow
  // the pressure (about a 10 s average) whenever the board sits still on the pad, so being moved
  // or the weather drifting doesn't show up as height. Frozen from liftoff.
  if (zeroN < 50 + 32) {
    if (++zeroN > 50) paSum += pa;
    if (zeroN == 50 + 32) { p0 = paSum / 32.0f; kh = 0; P00 = 1; }
    return;
  }
  if (phase == PAD && stillN >= 200) p0 += 0.001f * (pa - p0);
  baroAlt(44330.0f * (1 - powf(pa / p0, 0.190295f)));
}

void shtPoll() {
  static uint32_t at = 0;
  static bool waiting = false;
  if (!shtAddr) return;
  if (!waiting && millis() - at >= 500) {
    Wire.beginTransmission(shtAddr); Wire.write(0xFD); Wire.endTransmission();   // high precision measurement
    at = millis(); waiting = true;
  } else if (waiting && millis() - at >= 10) {
    waiting = false;
    uint8_t b[6];
    if (!shtRead6(b)) return;
    pkt.airT = (int16_t)((-45 + 175 * ((b[0] << 8 | b[1]) / 65535.0f)) * 100);
    pkt.rh = (uint16_t)(constrain(-6 + 125 * ((b[3] << 8 | b[4]) / 65535.0f), 0, 100) * 100);
  }
}

// ---------- MMC5983MA magnetometer ----------
// I2C at 0x30, 18 bits, 100 readings a second, with a SET pulse every 100 readings to keep the
// sensor magnetized the same way. Two calibrations:
//  - pad, automatic: SET, measure, RESET, measure. The field flips sign between the two and the
//    sensor's own offset does not, so their average is the offset. Runs at boot and every 30 s
//    while the board sits still on the pad, which follows the offset as the sensor warms up.
//  - bench, by hand: send 'm' over USB serial and turn the payload through every direction for
//    20 s. It prints the hard-iron offsets (battery, screws, magnetized parts near the sensor) and
//    the mapping from magnetometer to IMU axes, to paste in below. The pad can't learn these
//    because the rocket can't turn on the rail.
const uint8_t MMC = 0x30;
const int8_t magAxes[3] = {1, 2, 3};              // magnetometer axis behind IMU x, y, z: 1-based, signed
const float magHard[3] = {0, 0, 0};               // gauss, magnetometer axes
const float magScale[3] = {1, 1, 1};
bool mmcFound = false;
float mmcOfs[3] = {131072, 131072, 131072};       // counts; 131072 is the nominal zero
float magImu[3];                                  // calibrated field in IMU axes, gauss
uint32_t magAt = 0;                               // millis of the last reading
uint8_t mmcStep = 0;                              // 0 running, 1-6 offset calibration
uint32_t mmcStepAt = 0, mmcLastCal = 0, mmcSet[3];

void mmcWrite(uint8_t reg, uint8_t v) { Wire.beginTransmission(MMC); Wire.write(reg); Wire.write(v); Wire.endTransmission(); }

bool mmcRead(uint32_t *c) {                       // raw 18-bit counts
  Wire.beginTransmission(MMC); Wire.write(0x00); Wire.endTransmission(false);
  if (Wire.requestFrom(MMC, (uint8_t)7) != 7) return false;
  uint8_t b[7];
  for (int i = 0; i < 7; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++) c[i] = (uint32_t)b[2 * i] << 10 | b[2 * i + 1] << 2 | (b[6] >> (6 - 2 * i) & 3);
  return true;
}

bool detectMmc() {
  if (!i2cAck(MMC)) return false;
  Wire.beginTransmission(MMC); Wire.write(0x2F); Wire.endTransmission(false);   // product ID
  if (Wire.requestFrom(MMC, (uint8_t)1) != 1 || Wire.read() != 0x30) return false;
  mmcWrite(0x0A, 0x80); delay(15);                // software reset, 10 ms power-on time
  mmcWrite(0x0A, 0x01);                           // 4 ms measurements (200 Hz bandwidth, 0.6 mG noise)
  return true;
}

void mmcRun() {
  mmcWrite(0x09, 0x20);                           // Auto_SR_en, needed for the periodic SET
  mmcWrite(0x0B, 0x80 | 0x30 | 0x08 | 0x05);      // periodic SET every 100, continuous, 100 Hz
}

void mmcCalStep() {                               // the pad offset calibration, one step per call
  uint32_t c[3], dt = millis() - mmcStepAt;
  if (mmcStep > 1 && dt > 50) { mmcRun(); mmcStep = 0; return; }   // the sensor stopped answering
  switch (mmcStep) {
    case 1: mmcWrite(0x0B, 0); mmcWrite(0x09, 0x08); break;               // stop, SET
    case 2: if (dt < 1) return; mmcWrite(0x09, 0x01); break;              // measure
    case 3: if (dt < 6 || !mmcRead(mmcSet)) return; mmcWrite(0x09, 0x10); break;   // RESET
    case 4: if (dt < 1) return; mmcWrite(0x09, 0x01); break;              // measure
    case 5:
      if (dt < 6 || !mmcRead(c)) return;
      for (int i = 0; i < 3; i++) mmcOfs[i] = (mmcSet[i] + c[i]) / 2.0f;
      mmcWrite(0x09, 0x08);                       // SET again, the polarity continuous mode runs in
      mmcLastCal = millis();
      break;
    case 6: if (dt < 1) return; mmcRun(); mmcStep = 0; return;
  }
  mmcStep++;
  mmcStepAt = millis();
}

// bench calibration: field extremes on each axis for the hard iron, and gravity/field pairs
// for the axis mapping. The angle between gravity and the field is the same whichever way the
// payload is turned, but only when the two sensors' axes are matched up right.
struct MagPair { float g[3], m[3]; };
MagPair bench[200];
uint16_t benchN = 0;
uint32_t benchUntil = 0;
float benchMin[3], benchMax[3];

void benchStart() {
  for (int i = 0; i < 3; i++) { benchMin[i] = 1e9f; benchMax[i] = -1e9f; }
  benchN = 0;
  benchUntil = millis() + 20000;
  Serial.println("\nmagnetometer calibration: turn the payload slowly through every direction for 20 s,"
                 " away from steel and laptops");
}

void benchFinish() {
  benchUntil = 0;
  float hard[3], r[3], rMax = 0;
  for (int i = 0; i < 3; i++) {
    hard[i] = (benchMax[i] + benchMin[i]) / 2;
    r[i] = (benchMax[i] - benchMin[i]) / 2;
    rMax = max(rMax, r[i]);
  }
  float rAvg = (r[0] + r[1] + r[2]) / 3, rMin = min(r[0], min(r[1], r[2]));
  Serial.println("\nmagnetometer calibration done");
  if (rMin < 0.7f * rMax || rMax < 0.15f)
    Serial.println("  WARNING: some axis barely moved, turn it through more directions and send 'm' again");
  if (rMin < 0.05f) { Serial.println("  not enough movement to calibrate, nothing changed"); return; }

  // try all 48 ways the axes could line up, keep the one where the gravity/field angle holds
  // steadiest; of a mapping and its mirror image, the field dips down in the northern hemisphere
  static const uint8_t perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  int8_t best[3] = {0, 0, 0};
  float bestSd = 1e9f, bestDip = 0, curSd = -1;
  for (int p = 0; p < 6 && benchN >= 20; p++)
    for (int sg = 0; sg < 8; sg++) {
      int8_t ax[3];
      for (int i = 0; i < 3; i++) ax[i] = (sg >> i & 1) ? -(perms[p][i] + 1) : perms[p][i] + 1;
      float sum = 0, sum2 = 0;
      for (uint16_t j = 0; j < benchN; j++) {
        float m[3], d = 0, n = 0;
        for (int i = 0; i < 3; i++) {
          int k = abs(ax[i]) - 1;
          m[i] = (bench[j].m[k] - hard[k]) * rAvg / r[k] * (ax[i] > 0 ? 1 : -1);
          n += m[i] * m[i];
        }
        for (int i = 0; i < 3; i++) d += bench[j].g[i] * m[i];
        d /= sqrtf(n);
        sum += d; sum2 += d * d;
      }
      float mean = sum / benchN, sd = sqrtf(max(0.0f, sum2 / benchN - mean * mean));
      if (ax[0] == magAxes[0] && ax[1] == magAxes[1] && ax[2] == magAxes[2]) curSd = sd;
      if (mean < 0 && sd < bestSd) { bestSd = sd; bestDip = -asinf(constrain(mean, -1.0f, 1.0f)) * RAD_TO_DEG; memcpy(best, ax, 3); }
    }
  Serial.println("  paste these over the three lines in the magnetometer section of main.cpp:");
  if (best[0]) Serial.printf("const int8_t magAxes[3] = {%d, %d, %d};\n", best[0], best[1], best[2]);
  else Serial.printf("const int8_t magAxes[3] = {%d, %d, %d};   // unchanged: not enough steady samples\n",
                     magAxes[0], magAxes[1], magAxes[2]);
  Serial.printf("const float magHard[3] = {%.4ff, %.4ff, %.4ff};\n", hard[0], hard[1], hard[2]);
  Serial.printf("const float magScale[3] = {%.3ff, %.3ff, %.3ff};\n", rAvg / r[0], rAvg / r[1], rAvg / r[2]);
  if (best[0]) Serial.printf("  field %.0f mG, dipping %.0f deg; the axis fit holds the dip to +-%.1f deg (current mapping: %.1f)\n",
                             rAvg * 1000, bestDip, asinf(min(bestSd, 1.0f)) * RAD_TO_DEG,
                             curSd < 0 ? NAN : asinf(min(curSd, 1.0f)) * RAD_TO_DEG);
}

void benchSample(const float *m) {               // m: field in magnetometer axes, before hard iron
  static uint32_t last = 0;
  for (int i = 0; i < 3; i++) { benchMin[i] = min(benchMin[i], m[i]); benchMax[i] = max(benchMax[i], m[i]); }
  float a[3], n = 0, w = 0;
  for (int i = 0; i < 3; i++) {
    a[i] = pkt.acc[i] * 0.000976f; n += a[i] * a[i];
    w += sq(pkt.gyr[i] * 0.07f);
  }
  n = sqrtf(n);
  if (imu.bus && benchN < 200 && millis() - last >= 90 && fabsf(n - 1) < 0.05f && w < 60 * 60) {
    last = millis();                              // only while turning slowly, so the accel reads gravity
    for (int i = 0; i < 3; i++) { bench[benchN].g[i] = a[i] / n; bench[benchN].m[i] = m[i]; }
    benchN++;
  }
  if ((int32_t)(millis() - benchUntil) >= 0) benchFinish();
}

void magPoll() {
  static uint32_t at = 0;
  if (!mmcFound) return;
  if (mmcStep) { mmcCalStep(); return; }
  if (phase == PAD && stillN >= 200 && !benchUntil && millis() - mmcLastCal > 30000) { mmcStep = 1; mmcCalStep(); return; }
  if (millis() - at < 10) return;
  at = millis();
  uint32_t c[3];
  if (!mmcRead(c)) return;
  float m[3];
  for (int i = 0; i < 3; i++) m[i] = (c[i] - mmcOfs[i]) / 16384.0f;   // 16384 counts per gauss
  if (benchUntil) benchSample(m);
  for (int i = 0; i < 3; i++) m[i] = (m[i] - magHard[i]) * magScale[i];
  for (int i = 0; i < 3; i++) { int k = abs(magAxes[i]) - 1; magImu[i] = magAxes[i] > 0 ? m[k] : -m[k]; }
  magAt = millis();
}

// ---------- attitude ----------
// Mahony filter: the gyro is integrated at 400 Hz, and gravity pulls pitch and yaw back
// whenever the accelerometer reads about 1 g (never during boost or coast, when thrust and
// drag swamp gravity). Gravity can't see a vertical rocket's roll, so the magnetometer holds
// that one, in every phase: it only ever turns the estimate about the vertical, so a disturbed
// field can't tilt it, and readings whose strength or dip stray from what the pad saw are
// skipped. Without a magnetometer roll comes from the gyro alone and drifts slowly.
// With the magnetometer the world frame is x up, y magnetic north, z west.
// The nose is a fixed IMU axis, set by how the board is mounted. Roll, pitch and yaw are sent
// relative to how the airframe sat when it first settled on the pad, so they read 0, 0, 0 there,
// and turning it afterwards, on the pad or in flight, shows as roll.
const int noseAxis = 0, noseSign = -1;           // -X of the LSM6DSO32 points at the nose
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;             // rocket body to world, world x is up
float gb[3] = {0, 0, 0};                          // gyro bias in rocket axes, rad/s
float magRef = 0, magRefUp = 0;                   // field strength (G) and its up share on the pad
float padTurn = 0;                                // airframe turn about the vertical on the pad, rad

void toRocket(const float *v, float *r) {         // IMU axes to rocket axes, x = nose, right-handed
  int k = noseAxis, a = (k + 1) % 3, b = (k + 2) % 3;
  if (noseSign > 0) { r[0] = v[k];  r[1] = v[a]; r[2] = v[b]; }
  else              { r[0] = -v[k]; r[1] = v[b]; r[2] = v[a]; }
}

void attitude(const float *accImu, const float *gyrImuDps, bool still, float dt) {
  float a[3], w[3];
  toRocket(accImu, a);
  toRocket(gyrImuDps, w);
  for (int i = 0; i < 3; i++) w[i] *= DEG_TO_RAD;
  if (still) for (int i = 0; i < 3; i++) gb[i] += 0.002f * (w[i] - gb[i]);
  for (int i = 0; i < 3; i++) w[i] -= gb[i];

  float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (fabsf(n - 1) < 0.1f) {
    float kp = still ? 5.0f : 0.5f;
    float vx = 1 - 2 * (q2 * q2 + q3 * q3), vy = 2 * (q1 * q2 - q0 * q3), vz = 2 * (q1 * q3 + q0 * q2);  // up, in body axes
    float ax = a[0] / n, ay = a[1] / n, az = a[2] / n;
    w[0] += kp * (ay * vz - az * vy);
    w[1] += kp * (az * vx - ax * vz);
    w[2] += kp * (ax * vy - ay * vx);
  }
  if (mmcFound && millis() - magAt < 100) {
    float m[3];
    toRocket(magImu, m);
    float mn = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    if (mn > 0.05f) {
      for (int i = 0; i < 3; i++) m[i] /= mn;
      float r0[3] = {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)};   // up, in body axes
      float r1[3] = {2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)};   // north
      float r2[3] = {2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)};   // west
      float up = r0[0] * m[0] + r0[1] * m[1] + r0[2] * m[2];   // field in the world frame
      float hy = r1[0] * m[0] + r1[1] * m[1] + r1[2] * m[2];
      float hz = r2[0] * m[0] + r2[1] * m[1] + r2[2] * m[2];
      if (still && phase == PAD) {
        if (!magRef) { magRef = mn; magRefUp = up; }
        magRef += 0.002f * (mn - magRef); magRefUp += 0.002f * (up - magRefUp);
      }
      bool sane = !magRef || (fabsf(mn / magRef - 1) < 0.15f && fabsf(up - magRefUp) < 0.15f);
      float hor = sqrtf(hy * hy + hz * hz);       // cos(dip): about 0.44 in Charlotte
      if (sane && hor > 0.15f) {
        float e[3], wv[3];
        for (int i = 0; i < 3; i++) wv[i] = up * r0[i] + hor * r1[i];   // where the field should point if north were right
        e[0] = m[1] * wv[2] - m[2] * wv[1];
        e[1] = m[2] * wv[0] - m[0] * wv[2];
        e[2] = m[0] * wv[1] - m[1] * wv[0];
        // the error about the vertical is hor^2 * sin(angle off north); divide hor^2 out so the
        // gain is the same at any dip: 2/s pulls in within a second on the pad, 0.5/s in flight
        float km = (still ? 2.0f : 0.5f) * (e[0] * r0[0] + e[1] * r0[1] + e[2] * r0[2]) / (hor * hor);
        for (int i = 0; i < 3; i++) w[i] += km * r0[i];
      }
    }
  }
  float h = 0.5f * dt, a0 = q0, a1 = q1, a2 = q2, a3 = q3;
  q0 += (-a1 * w[0] - a2 * w[1] - a3 * w[2]) * h;
  q1 += ( a0 * w[0] + a2 * w[2] - a3 * w[1]) * h;
  q2 += ( a0 * w[1] - a1 * w[2] + a3 * w[0]) * h;
  q3 += ( a0 * w[2] + a1 * w[1] - a2 * w[0]) * h;
  float r = 1 / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= r; q1 *= r; q2 *= r; q3 *= r;
}

float upwardAccel(const float *accImu) {          // IMU acceleration along world up, gravity included, g
  float a[3];
  toRocket(accImu, a);
  return (1 - 2 * (q2 * q2 + q3 * q3)) * a[0] + 2 * (q1 * q2 - q0 * q3) * a[1] + 2 * (q1 * q3 + q0 * q2) * a[2];
}

void eulerOut() {                                 // yaw, pitch, roll (z, y, x), relative to the pad
  // tilt-compensated heading: the compass bearing of the nose's shadow on the ground, so the way
  // the rocket leans on the rail, flies, or points once landed. Unknown within 3 deg of vertical.
  float nUp = 1 - 2 * (q2 * q2 + q3 * q3), nNorth = 2 * (q1 * q2 + q0 * q3), nWest = 2 * (q1 * q3 - q0 * q2);
  pkt.heading = 0xFFFF;
  pkt.magField = 0;
  if (mmcFound && millis() - magAt < 100) {
    pkt.magField = (uint16_t)(sqrtf(magImu[0] * magImu[0] + magImu[1] * magImu[1] + magImu[2] * magImu[2]) * 1000);
    if (nUp < 0.9986f) {                          // cos 3 deg
      float hd = atan2f(-nWest, nNorth) * RAD_TO_DEG;
      pkt.heading = (uint16_t)((hd < 0 ? hd + 360 : hd) * 100) % 36000;
    }
  }
  // turn the world about the vertical by however the airframe sat once it settled: still for 2 s
  // on the pad (time for the magnetometer to pull the estimate round to north), or at liftoff if
  // it never was. Taken once; re-taking it on every packet would cancel any roll on the pad.
  static bool padTurnSet = false;
  if (!padTurnSet && (phase != PAD || stillN >= 800)) {
    padTurn = atan2f(2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q3 * q3));
    padTurnSet = true;
  }
  float c = cosf(padTurn / 2), s = -sinf(padTurn / 2);
  float o0 = c * q0 - s * q1, o1 = c * q1 + s * q0, o2 = c * q2 - s * q3, o3 = c * q3 + s * q2;
  pkt.roll  = (int16_t)(atan2f(2 * (o0 * o1 + o2 * o3), 1 - 2 * (o1 * o1 + o2 * o2)) * RAD_TO_DEG * 100);
  pkt.pitch = (int16_t)(asinf(constrain(2 * (o0 * o2 - o3 * o1), -1.0f, 1.0f)) * RAD_TO_DEG * 100);
  pkt.yaw   = (int16_t)(atan2f(2 * (o0 * o3 + o1 * o2), 1 - 2 * (o2 * o2 + o3 * o3)) * RAD_TO_DEG * 100);
}

// ---------- IMU sampling ----------
float mag(float x, float y, float z) { return sqrtf(x * x + y * y + z * z); }

void sample(float dt) {
  uint8_t b[14];
  float lsm = 0, hi = 0;
  if (imu.bus) {
    readRegs(imu, 0x20, b, 14);                   // temp, gyro xyz, accel xyz
    pkt.temp = (int16_t)((int16_t)(b[0] | b[1] << 8) / 2.56f + 2500);
    for (int i = 0; i < 3; i++) {
      pkt.gyr[i] = (int16_t)(b[2 + 2 * i] | b[3 + 2 * i] << 8);
      pkt.acc[i] = (int16_t)(b[8 + 2 * i] | b[9 + 2 * i] << 8);
    }
    lsm = 0.000976f * mag(pkt.acc[0], pkt.acc[1], pkt.acc[2]);
  }
  if (hig.bus) {
    readRegs(hig, 0x28, b, 6);
    for (int i = 0; i < 3; i++) pkt.hig[i] = (int16_t)(b[2 * i] | b[2 * i + 1] << 8) >> 4;  // 12-bit, left-justified
    hi = 0.098f * mag(pkt.hig[0], pkt.hig[1], pkt.hig[2]);
  }
  float g = (imu.bus && lsm < 30) ? lsm : hi;     // IMU is finer, high-g takes over near its 32 g limit
  if (g > peakG) peakG = g;

  float aUp = NAN;                                // attitude first: altitude needs to know which way is up
  if (imu.bus) {
    float acc[3], gyr[3];
    for (int i = 0; i < 3; i++) { acc[i] = pkt.acc[i] * 0.000976f; gyr[i] = pkt.gyr[i] * 0.07f; }
    attitude(acc, gyr, stillN >= 200 && (phase == PAD || phase == LANDED), dt);
    aUp = upwardAccel(acc);
  }
  estimate(pkt.acc[0] * 0.000976f, pkt.acc[1] * 0.000976f, pkt.acc[2] * 0.000976f,
           pkt.hig[0] * 0.098f, pkt.hig[1] * 0.098f, pkt.hig[2] * 0.098f,
           0.07f * mag(pkt.gyr[0], pkt.gyr[1], pkt.gyr[2]), aUp, dt);
}

// ---------- battery ----------
// The Feather has a 100k/100k divider from BAT to A6 (PA3). On the payload PCB it is VBAT_SENSE
// on PA1 through 100k/33k, sized for a 2S pack, so switch these two lines for the PCB.
const uint32_t VBAT_PIN = PA3;
const float VBAT_DIV = 2.0f;                      // PCB: (100k + 33k) / 33k
float vbat = 0;                                   // volts, about a 1 s average

void batteryPoll() {
  static uint32_t at = 0;
  if (millis() - at < 20) return;                 // 50 readings a second
  at = millis();
  // measure against the internal reference, not the 3.3 V rail: on a low 1S cell the
  // regulator drops out and the rail sags with the battery, which would hide the drop
  uint32_t vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
  float v = __LL_ADC_CALC_DATA_TO_VOLTAGE(vdda, analogRead(VBAT_PIN), LL_ADC_RESOLUTION_12B) / 1000.0f * VBAT_DIV;
  vbat = vbat ? vbat + 0.02f * (v - vbat) : v;
}

void printLine() {
  static const char *names[] = {"PAD", "BOOST", "COAST", "DESCENT", "LANDED"};
  Serial.printf("#%u  %s  alt %.1f m  vel %.1f m/s  acc %.2f g  peak %.1f g", pkt.seq, names[pkt.state],
    pkt.alt / 100.0f, pkt.vel / 10.0f, 0.000976f * mag(pkt.acc[0], pkt.acc[1], pkt.acc[2]), pkt.peak / 10.0f);
  if (baro.bus) Serial.printf("  %.2f hPa", pkt.pres / 100.0f);
  if (shtAddr)  Serial.printf("  air %.1f C %.0f%%", pkt.airT / 100.0f, pkt.rh / 100.0f);
  if (gpsTalking()) Serial.printf("  gps fix %u, %u sats", pkt.fix, pkt.sats);
  if (imu.bus) Serial.printf("  roll %.0f pitch %.0f yaw %.0f", pkt.roll / 100.0f, pkt.pitch / 100.0f, pkt.yaw / 100.0f);
  if (mmcFound) {
    if (pkt.heading == 0xFFFF) Serial.printf("  hdg -- (%u mG)", pkt.magField);
    else Serial.printf("  hdg %.0f (%u mG)", pkt.heading / 100.0f, pkt.magField);
  }
  Serial.printf("  bat %.2f V", pkt.vbat / 1000.0f);
  Serial.println();
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  for (int cs : {imu.cs, hig.cs, baro.cs}) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  sensorSPI.begin();
  Wire.begin();
  Wire.setClock(400000);
  gpsSerial.begin(38400);                         // u-blox M10 default
  analogReadResolution(12);

  if (detect(imu)) {
    writeReg(imu, 0x12, 0x01); delay(10);         // CTRL3_C: software reset
    writeReg(imu, 0x12, 0x44);                    // CTRL3_C: BDU, auto-increment
    writeReg(imu, 0x10, 0x64);                    // CTRL1_XL: 416 Hz, +-32 g
    writeReg(imu, 0x11, 0x6C);                    // CTRL2_G: 416 Hz, +-2000 dps
  }
  if (detect(hig)) {
    writeReg(hig, 0x20, 0x37);                    // CTRL_REG1: normal mode, 400 Hz, XYZ on
    writeReg(hig, 0x23, 0x90);                    // CTRL_REG4: BDU, +-200 g
  }
  if (detectBaro()) { baroCmd(0x58, 0); baroAt = micros(); }   // first conversion is temperature
  if (detectSht()) shtAddr = 0x44;
  if ((mmcFound = detectMmc())) { mmcStep = 1; while (mmcStep) mmcCalStep(); }   // first offset calibration
  gpsConfigure();
  gpsLastCfg = millis();

  report(imu.name, imu.bus, imu.addr);
  if (imu.bus) Serial.printf("nose axis: IMU %c%c\n", noseSign > 0 ? '+' : '-', 'X' + noseAxis);
  report(hig.name, hig.bus, hig.addr);
  report(baro.name, baro.bus, baro.addr);
  report("SHT45", shtAddr ? VIA_I2C : NONE, shtAddr);
  report("MMC5983MA", mmcFound ? VIA_I2C : NONE, MMC);
  if (mmcFound) Serial.println("  send 'm' to calibrate the magnetometer on the bench");
  Serial.println(gpsI2C ? "GPS: I2C 0x42" : "GPS: listening on UART (RX/TX pins) at 38400");

  int s = radio.begin(FREQ, BW, SF, CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, TX_DBM, 8, TCXO);
  if (s != RADIOLIB_ERR_NONE) {
    Serial.print("radio failed, code "); Serial.println(s);
    while (true) { digitalWrite(LED, !digitalRead(LED)); delay(100); }
  }
  radio.setDio2AsRfSwitch(true);
  radio.setCurrentLimit(140);
  radio.setPacketSentAction(onTxDone);
  Serial.println("radio ok");
}

void loop() {
  static uint32_t lastSample = micros(), lastTx = 0;

  uint32_t us = micros();
  if (us - lastSample >= 2500) {                  // 400 Hz
    sample(min((us - lastSample) * 1e-6f, 0.01f));
    lastSample = us;
  }
  baroPoll();
  shtPoll();
  gpsPoll();
  batteryPoll();
  magPoll();
  if (Serial.available() && Serial.read() == 'm' && mmcFound && !benchUntil) benchStart();

  if (txDone) {
    txDone = false;
    radio.finishTransmit();
    txBusy = false;
    digitalWrite(LED, LOW);
  }

  if (!txBusy && millis() - lastTx >= TX_PERIOD_MS) {
    lastTx = millis();
    pkt.ms = lastTx;
    pkt.peak = (uint16_t)(peakG * 10);
    peakG = 0;
    pkt.flags = (imu.bus ? 1 : 0) | (hig.bus ? 2 : 0) | (baro.bus ? 4 : 0) | (shtAddr ? 8 : 0) | (gpsTalking() ? 16 : 0) |
                (mmcFound ? 32 : 0);
    if (!gpsTalking()) pkt.fix = 0;
    pkt.state = phase;
    pkt.alt = (int32_t)(kh * 100);
    pkt.vel = (int16_t)constrain(kv * 10, -32000, 32000);
    pkt.maxAlt = (uint16_t)constrain(maxAlt, 0, 65535);
    pkt.maxVel = (uint16_t)constrain(maxVel * 10, 0, 65535);
    eulerOut();
    pkt.vbat = (uint16_t)constrain(vbat * 1000, 0, 65535);
    int s = radio.startTransmit((uint8_t *)&pkt, sizeof(pkt));
    if (s == RADIOLIB_ERR_NONE) { txBusy = true; digitalWrite(LED, HIGH); }
    else { Serial.print("tx failed, code "); Serial.println(s); }
    printLine();
    pkt.seq++;
  }
}
