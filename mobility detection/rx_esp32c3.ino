/*
  LoRa (SX1278) -> Serial raw display              *** RECEIVER ***

  Board   : ESP32-C3 SuperMini  -> Arduino IDE: "ESP32C3 Dev Module"
            Tools > USB CDC On Boot = Enabled   (otherwise Serial prints nothing)
  Library : "LoRa" by Sandeep Mistry   (Library Manager)

  Wiring (3.3 V logic)
    SX1278  3.3V-> 3V3             SX1278 GND -> GND
    SX1278  SCK -> GPIO4    MISO -> GPIO5    MOSI -> GPIO6
    SX1278  NSS -> GPIO7    RST  -> GPIO3    DIO0 -> GPIO10
    Antenna must be attached.

  Output, one line per packet:
    [#12  RSSI -78 dBm  SNR 9.5 dB  30 B]  AA FF 03 00 0E 03 B1 86 ...
*/
#include <SPI.h>
#include <LoRa.h>

// ---------------- pins ----------------
#define LORA_SCK     4
#define LORA_MISO    5
#define LORA_MOSI    6
#define LORA_NSS     7
#define LORA_RST     3
#define LORA_DIO0   10

// ---------------- radio (MUST match the transmitter) ----------------
#define LORA_FREQ   433E6
#define LORA_SF     7
#define LORA_BW     125E3
#define LORA_CR     5
#define LORA_SYNC   0xF3

// 0 = raw hex only.  1 = also print a decoded line per target (mm, cm/s).
#define DECODE_LD2450  0

// ---- decode begin ----
// LD2450 signed fields: bit15 = 1 -> positive, bit15 = 0 -> negative, low 15 bits = magnitude.
static int16_t ldSigned(uint8_t lo, uint8_t hi) {
  uint16_t raw = ((uint16_t)hi << 8) | lo;
  int16_t mag = (int16_t)(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : (int16_t)-mag;
}
// ---- decode end ----

#if DECODE_LD2450
static void decodeFrame(const uint8_t *f, int len) {
  if (len != 30 || f[0] != 0xAA || f[1] != 0xFF || f[2] != 0x03 || f[3] != 0x00 ||
      f[28] != 0x55 || f[29] != 0xCC) {
    Serial.println("   (not a valid LD2450 frame)");
    return;
  }
  for (int t = 0; t < 3; t++) {
    const uint8_t *p = f + 4 + t * 8;
    int16_t  x   = ldSigned(p[0], p[1]);
    int16_t  y   = ldSigned(p[2], p[3]);
    int16_t  spd = ldSigned(p[4], p[5]);
    uint16_t res = (uint16_t)(p[6] | (p[7] << 8));
    if (!x && !y && !spd && !res) {
      Serial.printf("   T%d: --\n", t + 1);
    } else {
      Serial.printf("   T%d: x=%d mm  y=%d mm  v=%d cm/s  res=%u mm\n", t + 1, x, y, spd, res);
    }
  }
}
#endif

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\nLoRa RX (raw)");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init FAILED. Check wiring and that the module is on 3.3 V.");
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  Serial.println("LoRa ready, listening");
}

void loop() {
  static uint32_t count = 0;

  int size = LoRa.parsePacket();
  if (size == 0) return;

  uint8_t buf[255];
  int n = 0;
  while (LoRa.available() && n < (int)sizeof(buf)) buf[n++] = (uint8_t)LoRa.read();

  Serial.printf("[#%lu  RSSI %d dBm  SNR %.1f dB  %d B]  ",
                (unsigned long)++count, LoRa.packetRssi(), LoRa.packetSnr(), n);
  for (int i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
  Serial.println();

#if DECODE_LD2450
  decodeFrame(buf, n);
#endif
}
