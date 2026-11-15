#pragma once
#include <Arduino.h>
#include <SPI.h>

// Flight log on the W25Q512JV (64 MB NOR) on SPI3, as wired on the payload PCB:
//   CLK PC10, DO (MISO) PC11, DI (MOSI) PC12, CS PA15.
//
// Layout: 3 telemetry records per 256 byte page (3 x 77 = 231 bytes, the other 25 stay 0xFF),
// written one page at a time from the start of the chip. Every page therefore starts with a
// record whose first byte is the magic 'X', and an erased page starts with 0xFF, so at boot the
// end of the log is found with a binary search and logging carries on after it. Nothing is ever
// erased while the rocket is running; the flash is only erased by the 'E' command on the ground.
//
// Page programs are started and not waited for (0.4 to 3 ms); the next flash command waits for
// the chip to be idle first, so the 400 Hz sensor loop is not held up.

#define FL_REC       77
#define FL_PAGE      256
#define FL_PER_PAGE  3
#define FL_CAP       (64UL * 1024 * 1024)
#define FL_PAGES     (FL_CAP / FL_PAGE)
#define FL_BLOCK     65536UL

SPIClass flashSPI(PC12, PC11, PC10);                // MOSI, MISO, SCK
const int FLASH_CS = PA15;
SPISettings flashCfg(10000000, MSBFIRST, SPI_MODE0);

bool flReady = false;
uint32_t flPage = 0;                                // next page to program
uint8_t flBuf[FL_PAGE];
uint8_t flN = 0;                                    // records waiting in flBuf

static inline void flSel()   { flashSPI.beginTransaction(flashCfg); digitalWrite(FLASH_CS, LOW); }
static inline void flDesel() { digitalWrite(FLASH_CS, HIGH); flashSPI.endTransaction(); }

static void flCmd(uint8_t op) { flSel(); flashSPI.transfer(op); flDesel(); }

static void flAddr(uint8_t op, uint32_t a) {        // opcode and 4 address bytes, CS left low
  flSel();
  flashSPI.transfer(op);
  flashSPI.transfer(a >> 24); flashSPI.transfer(a >> 16); flashSPI.transfer(a >> 8); flashSPI.transfer(a);
}

static uint8_t flStatus() { flSel(); flashSPI.transfer(0x05); uint8_t s = flashSPI.transfer(0); flDesel(); return s; }

static bool flWait(uint32_t timeoutMs = 3000) {
  uint32_t t = millis();
  while (flStatus() & 1) if (millis() - t > timeoutMs) return false;
  return true;
}

static void flRead(uint32_t a, uint8_t *buf, uint16_t n) {
  flWait();
  flAddr(0x13, a);                                  // read, 4 byte address
  for (uint16_t i = 0; i < n; i++) buf[i] = flashSPI.transfer(0);
  flDesel();
}

static void flProgram(uint32_t a, const uint8_t *d, uint16_t n) {   // within one page; not waited for
  flWait();
  flCmd(0x06);                                      // write enable
  flAddr(0x12, a);                                  // page program, 4 byte address
  for (uint16_t i = 0; i < n; i++) flashSPI.transfer(d[i]);
  flDesel();
}

static bool flEraseBlock(uint32_t a) {              // 64 KB, 150 ms typical
  flWait();
  flCmd(0x06);
  flAddr(0xDC, a);
  flDesel();
  return flWait(3000);
}

// first erased page after the log; false if the chip is not there
bool flBegin() {
  pinMode(FLASH_CS, OUTPUT); digitalWrite(FLASH_CS, HIGH);
  flashSPI.begin();
  flCmd(0x66); flCmd(0x99); delay(1);               // reset
  flSel(); flashSPI.transfer(0x9F);
  uint8_t mfr = flashSPI.transfer(0), type = flashSPI.transfer(0), cap = flashSPI.transfer(0);
  flDesel();
  if (mfr != 0xEF || type != 0x40 || cap != 0x20) return false;   // Winbond W25Q512JV
  flCmd(0xB7);                                      // 4 byte addressing

  uint32_t lo = 0, hi = FL_PAGES;                   // pages before the end start with 'X', after it with 0xFF
  while (lo < hi) {
    uint32_t mid = (lo + hi) / 2;
    uint8_t b;
    flRead(mid * FL_PAGE, &b, 1);
    if (b == 0xFF) hi = mid; else lo = mid + 1;
  }
  flPage = lo;
  // a power cut in the middle of a page program leaves a page that starts blank but isn't; skip those
  for (; flPage < FL_PAGES; flPage++) {
    uint8_t p[FL_PAGE];
    flRead(flPage * FL_PAGE, p, FL_PAGE);
    bool blank = true;
    for (uint16_t i = 0; i < FL_PAGE && blank; i++) blank = p[i] == 0xFF;
    if (blank) break;
  }
  flN = 0;
  return flReady = true;
}

static void flFlush() {                             // write whatever is waiting, even a part page
  if (!flN || flPage >= FL_PAGES) return;
  flProgram(flPage * FL_PAGE, flBuf, flN * FL_REC);
  flPage++;
  flN = 0;
}

bool flAppend(const void *rec) {
  if (!flReady || flPage >= FL_PAGES) return false;
  memcpy(flBuf + flN * FL_REC, rec, FL_REC);
  if (++flN == FL_PER_PAGE) flFlush();
  return true;
}

uint32_t flUsedBytes() { return (flPage + (flN ? 1 : 0)) * FL_PAGE; }

static uint32_t crcUpdate(uint32_t c, const uint8_t *d, uint16_t n) {   // standard CRC-32, same as zlib.crc32
  static const uint32_t t[16] = {0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C, 0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C, 0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C};
  for (uint16_t i = 0; i < n; i++) {
    c ^= d[i];
    c = (c >> 4) ^ t[c & 15];
    c = (c >> 4) ^ t[c & 15];
  }
  return c;
}

// "DUMP <bytes>\n", the raw pages, then "\nCRC <hex>\n"
void flDump(Stream &out) {
  flFlush();
  uint32_t n = flPage * FL_PAGE, crc = 0xFFFFFFFF;
  out.print("DUMP "); out.print(n); out.print("\n");
  uint8_t p[FL_PAGE];
  for (uint32_t a = 0; a < n; a += FL_PAGE) {
    flRead(a, p, FL_PAGE);
    crc = crcUpdate(crc, p, FL_PAGE);
    out.write(p, FL_PAGE);
  }
  out.print("\nCRC "); out.printf("%08lx", (unsigned long)~crc); out.print("\n");
}

// erase everything that has been written, 64 KB at a time (the chip erase command takes minutes)
bool flEraseAll(Stream &out) {
  flFlush();
  uint32_t blocks = (flUsedBytes() + FL_BLOCK - 1) / FL_BLOCK;
  for (uint32_t b = 0; b < blocks; b++) {
    if (!flEraseBlock(b * FL_BLOCK)) { out.println("\nerase timed out"); return false; }
    if (b % 16 == 15) out.print('.');
  }
  flPage = 0; flN = 0;
  return true;
}
