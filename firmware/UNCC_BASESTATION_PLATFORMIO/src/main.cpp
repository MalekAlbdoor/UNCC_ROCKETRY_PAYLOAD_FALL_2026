#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Wio-SX1262 on the XIAO ESP32-S3
#define LORA_SCK   7
#define LORA_MISO  8
#define LORA_MOSI  9
#define LORA_CS    41
#define LORA_RST   42
#define LORA_DIO1  39
#define LORA_BUSY  40
#define LORA_RXEN  38

// OLED on the XIAO's I2C pins
#define OLED_SDA   5     // silkscreen D4
#define OLED_SCL   6     // silkscreen D5
#define OLED_H     64    // 32 for a 128x32 screen

// 1 = live (10 Hz, SF8), 0 = max range (1 Hz, SF12). Must match the rocket.
#define LIVE 1

#define FREQ  915.0
#define BW    500.0
#define PWR   22
#define TCXO  1.8
#if LIVE
  #define SF  8
  #define CR  5
#else
  #define SF  12
  #define CR  8
#endif

#define REDRAW_MS 100

// must match the rocket's struct byte for byte
struct __attribute__((packed)) Telem {
  uint8_t  magic;      // 'V'
  uint8_t  flags;      // bit0 IMU, bit1 high-g, bit2 baro, bit3 SHT45, bit4 GPS talking
  uint16_t seq;
  uint32_t ms;
  int16_t  acc[3];     // 0.976 mg/LSB
  int16_t  gyr[3];     // 70 mdps/LSB
  int16_t  hig[3];     // 98 mg/LSB
  uint16_t peak;       // 0.1 g/LSB
  int16_t  temp;       // 0.01 C/LSB
  uint8_t  state;      // 0 pad, 1 boost, 2 coast, 3 descent, 4 landed
  int32_t  alt;        // cm above the pad
  int16_t  vel;        // 0.1 m/s, up is positive
  uint16_t maxAlt;     // m
  uint16_t maxVel;     // 0.1 m/s
  uint32_t pres;       // Pa
  int16_t  airT;       // 0.01 C
  uint16_t rh;         // 0.01 %
  uint8_t  fix;        // 0 none, 2 2D, 3 3D
  uint8_t  sats;
  int32_t  lat, lon;   // 1e-7 deg
  int16_t  gpsAlt;     // m above sea level
  int16_t  gpsVup;     // 0.1 m/s
  uint16_t gpsSpd;     // 0.1 m/s
  int16_t  roll, pitch, yaw;   // 0.01 deg
};
static_assert(sizeof(Telem) == 71, "rocket sends 71 bytes");

const char *PHASES[] = {"PAD", "BOOST", "COAST", "DESCENT", "LANDED"};
const char *phaseName(uint8_t s) { return s < 5 ? PHASES[s] : "?"; }

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI);
Adafruit_SSD1306 oled(128, OLED_H, &Wire, -1);
uint8_t oledAddr = 0;                             // 0 = not found

volatile bool rxFlag = false;
void IRAM_ATTR onRx() { rxFlag = true; }

// serial output: dashboard for a terminal, or one JSON line per packet for the laptop GUI.
// The GUI sends 'j' to switch to JSON and 'd' to switch back.
bool jsonMode = false;

// latest packet and link stats
Telem last;
bool have = false;
float lastRssi = 0, lastSnr = 0, lossPct = 0;
uint32_t lastHeard = 0, crcErrors = 0, badPackets = 0;
uint32_t badAt = 0;                               // when the last wrong-sized packet arrived
size_t badLen = 0;

float mag(float x, float y, float z) { return sqrtf(x * x + y * y + z * z); }

void initOled() {
  const uint8_t addrs[] = {0x3C, 0x3D};
  for (uint8_t a : addrs) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0 && oled.begin(SSD1306_SWITCHCAPVCC, a, true, false)) {
      oledAddr = a;
      oled.setTextColor(SSD1306_WHITE);
      return;
    }
  }
}

void drawOled() {
  if (!oledAddr) return;
  oled.clearDisplay();
  oled.setCursor(0, 0);

  if (!have) {
    oled.print("listening...");
  } else {
    uint32_t age = millis() - lastHeard;
    float acc = 0.000976f * mag(last.acc[0], last.acc[1], last.acc[2]);
    float hi  = 0.098f * mag(last.hig[0], last.hig[1], last.hig[2]);
    float gyr = 0.07f * mag(last.gyr[0], last.gyr[1], last.gyr[2]);

    if (age > 5000) oled.printf("NO SIGNAL  %lus\n", (unsigned long)(age / 1000));
    else            oled.printf("#%u  %lums ago\n", last.seq, (unsigned long)age);
    oled.printf("RSSI %.0f  SNR %.1f\n", lastRssi, lastSnr);
    oled.printf("%-7s alt %.0fm\n", phaseName(last.state), last.alt / 100.0f);
    oled.printf("vel %.0f  max %um\n", last.vel / 10.0f, last.maxAlt);
    oled.printf("peak %.1fg acc %.2fg\n", last.peak / 10.0f, acc);
    oled.printf("loss %.1f%%  gyr %.0f\n", lossPct, gyr);
    if (last.flags & 16) oled.printf("GPS %s %u sats\n", last.fix == 3 ? "3D" : last.fix == 2 ? "2D" : "no fix", last.sats);
    else                 oled.print("GPS --\n");
    oled.printf("I%c H%c B%c S%c  hi %.0fg\n", (last.flags & 1) ? '+' : '-', (last.flags & 2) ? '+' : '-',
      (last.flags & 4) ? '+' : '-', (last.flags & 8) ? '+' : '-', hi);
  }
  oled.display();
}

void row(const char *label, float x, float y, float z, int dp, const char *unit) {
  Serial.printf(" %-8s %8.*f %8.*f %8.*f %8.*f  %s\033[K\n", label, dp, x, dp, y, dp, z, dp, mag(x, y, z), unit);
}

void drawSerial() {
  Serial.print("\033[?25l\033[H");                // hide cursor, jump to top-left, redraw in place
  Serial.printf(" UNCC BASE STATION   %s  SF%d  %.1f MHz\033[K\n", LIVE ? "LIVE" : "RANGE", SF, FREQ);
  Serial.print(" ---------------------------------------------------\033[K\n");

  if (!have) {
    Serial.print(" waiting for first packet...\033[K\n");
  } else {
    uint32_t age = millis() - lastHeard;
    if (age > 5000) Serial.printf(" Link     NO SIGNAL for %lu s\033[K\n", (unsigned long)(age / 1000));
    else            Serial.printf(" Link     #%-7u %5lu ms ago\033[K\n", last.seq, (unsigned long)age);
    Serial.printf(" Signal   RSSI %4.0f dBm   SNR %5.1f dB   loss %5.1f%%\033[K\n", lastRssi, lastSnr, lossPct);
    Serial.print("\033[K\n");
    Serial.printf(" %-8s %8s %8s %8s %8s\033[K\n", "", "X", "Y", "Z", "total");
    row("Accel",  last.acc[0] * 0.000976f, last.acc[1] * 0.000976f, last.acc[2] * 0.000976f, 2, "g");
    row("Gyro",   last.gyr[0] * 0.07f,     last.gyr[1] * 0.07f,     last.gyr[2] * 0.07f,     1, "dps");
    row("High-g", last.hig[0] * 0.098f,    last.hig[1] * 0.098f,    last.hig[2] * 0.098f,    1, "g");
    Serial.print("\033[K\n");
    Serial.printf(" Peak     %5.1f g    Temp %5.1f C    Uptime %7.1f s\033[K\n",
      last.peak / 10.0f, last.temp / 100.0f, last.ms / 1000.0f);
    Serial.print("\033[K\n");
    Serial.printf(" Flight   %-8s alt %8.1f m   vel %7.1f m/s   max %u m, %.1f m/s\033[K\n", phaseName(last.state),
      last.alt / 100.0f, last.vel / 10.0f, last.maxAlt, last.maxVel / 10.0f);
    if (last.flags & 1) Serial.printf(" Attitude roll %6.1f   pitch %6.1f   yaw %6.1f deg\033[K\n",
      last.roll / 100.0f, last.pitch / 100.0f, last.yaw / 100.0f);
    if (last.flags & 4) Serial.printf(" Baro     %8.2f hPa\033[K\n", last.pres / 100.0f);
    if (last.flags & 8) Serial.printf(" Air      %6.1f C   %5.1f %%RH\033[K\n", last.airT / 100.0f, last.rh / 100.0f);
    if (last.flags & 16) {
      if (last.fix) Serial.printf(" GPS      %s fix, %u sats   %.6f, %.6f   %d m   %.1f m/s up   %.1f m/s ground\033[K\n",
        last.fix == 3 ? "3D" : "2D", last.sats, last.lat * 1e-7, last.lon * 1e-7, last.gpsAlt, last.gpsVup / 10.0f, last.gpsSpd / 10.0f);
      else          Serial.printf(" GPS      no fix yet, %u sats\033[K\n", last.sats);
    }
    auto ok = [](int bit) { return (last.flags & bit) ? "ok" : "--"; };
    Serial.printf(" Sensors  IMU %s  High-g %s  Baro %s  SHT45 %s  GPS %s\033[K\n", ok(1), ok(2), ok(4), ok(8), ok(16));
  }
  if (badLen && millis() - badAt < 5000)
    Serial.printf(" Firmware MISMATCH: got %u byte packets, expected %u. Flash the rocket and base station together.\033[K\n",
      (unsigned)badLen, (unsigned)sizeof(Telem));
  Serial.printf(" Errors   crc %lu   bad %lu\033[K\n", (unsigned long)crcErrors, (unsigned long)badPackets);
  if (oledAddr) Serial.printf(" Screen   OLED at 0x%02X\033[K\n", oledAddr);
  else          Serial.print(" Screen   OLED not found (check VCC/GND order, SDA = D4, SCL = D5)\033[K\n");
  Serial.print("\033[J");                         // clear anything left below
}

void printJson() {
  Serial.printf("{\"seq\":%u,\"t\":%.2f,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
                "\"hx\":%.2f,\"hy\":%.2f,\"hz\":%.2f,\"peak\":%.1f,\"temp\":%.2f,\"flags\":%u,",
    last.seq, last.ms / 1000.0f,
    last.acc[0] * 0.000976f, last.acc[1] * 0.000976f, last.acc[2] * 0.000976f,
    last.gyr[0] * 0.07f, last.gyr[1] * 0.07f, last.gyr[2] * 0.07f,
    last.hig[0] * 0.098f, last.hig[1] * 0.098f, last.hig[2] * 0.098f,
    last.peak / 10.0f, last.temp / 100.0f, last.flags);
  Serial.printf("\"st\":%u,\"alt\":%.2f,\"vel\":%.1f,\"malt\":%u,\"mvel\":%.1f,\"pres\":%.2f,\"airt\":%.2f,\"rh\":%.2f,"
                "\"fix\":%u,\"sats\":%u,\"lat\":%.7f,\"lon\":%.7f,\"galt\":%d,\"gvu\":%.1f,\"gspd\":%.1f,",
    last.state, last.alt / 100.0f, last.vel / 10.0f, last.maxAlt, last.maxVel / 10.0f, last.pres / 100.0f,
    last.airT / 100.0f, last.rh / 100.0f, last.fix, last.sats, last.lat * 1e-7, last.lon * 1e-7,
    last.gpsAlt, last.gpsVup / 10.0f, last.gpsSpd / 10.0f);
  Serial.printf("\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,", last.roll / 100.0f, last.pitch / 100.0f, last.yaw / 100.0f);
  Serial.printf("\"rssi\":%.1f,\"snr\":%.2f,\"loss\":%.2f}\n", lastRssi, lastSnr, lossPct);
}

void printHeartbeat() {
  Serial.printf("{\"hb\":1,\"live\":%d,\"sf\":%d,\"freq\":%.1f,\"bw\":%.0f,\"crc\":%lu,\"bad\":%lu,\"oled\":%u,\"age\":%ld,"
                "\"plen\":%u,\"badlen\":%u}\n",
    LIVE, SF, FREQ, BW, (unsigned long)crcErrors, (unsigned long)badPackets, oledAddr,
    have ? (long)(millis() - lastHeard) : -1L,
    (unsigned)sizeof(Telem), (unsigned)(badLen && millis() - badAt < 5000 ? badLen : 0));
}

void handlePacket() {
  static long lastSeq = -1;
  static uint32_t got = 0, lost = 0;

  Telem p;
  size_t len = radio.getPacketLength();
  int s = radio.readData((uint8_t *)&p, sizeof(p));

  if (s == RADIOLIB_ERR_CRC_MISMATCH) {
    crcErrors++;
  } else if (s != RADIOLIB_ERR_NONE || len != sizeof(p) || p.magic != 'V') {
    badPackets++;
    if (s == RADIOLIB_ERR_NONE && len != sizeof(p)) { badLen = len; badAt = millis(); }   // old or new firmware on the other end
  } else {
    if (lastSeq >= 0 && p.seq > lastSeq) lost += p.seq - lastSeq - 1;
    else if (p.seq <= lastSeq) { got = 0; lost = 0; }   // rocket rebooted
    lastSeq = p.seq;
    got++;

    last = p;
    have = true;
    lastHeard = millis();
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();
    lossPct = 100.0f * lost / (got + lost);
    if (jsonMode) printJson();
  }
  radio.startReceive();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin(OLED_SDA, OLED_SCL);
  initOled();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  int s = radio.begin(FREQ, BW, SF, CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, PWR, 8, TCXO);
  if (s != RADIOLIB_ERR_NONE) {
    if (oledAddr) { oled.clearDisplay(); oled.setCursor(0, 0); oled.printf("radio failed %d", s); oled.display(); }
    while (true) { Serial.printf("radio failed, code %d\n", s); delay(2000); }
  }
  radio.setDio2AsRfSwitch(true);
  radio.setRfSwitchPins(LORA_RXEN, RADIOLIB_NC);
  radio.setCurrentLimit(140);
  radio.setRxBoostedGainMode(true);
  radio.setPacketReceivedAction(onRx);
  radio.startReceive();
}

void loop() {
  static uint32_t lastDraw = 0, lastTry = 0, lastBeat = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'j') { jsonMode = true; Serial.print("\n"); }
    else if (c == 'd') { jsonMode = false; Serial.print("\033[2J"); }
  }

  if (rxFlag) {
    rxFlag = false;
    handlePacket();
  }
  if (!oledAddr && millis() - lastTry > 2000) {   // keep looking until the screen answers
    lastTry = millis();
    initOled();
  }
  if (jsonMode && millis() - lastBeat >= 1000) {  // lets the GUI see link status and errors with no packets
    lastBeat = millis();
    printHeartbeat();
  }
  if (millis() - lastDraw >= REDRAW_MS) {
    lastDraw = millis();
    if (!jsonMode) drawSerial();
    drawOled();
  }
}