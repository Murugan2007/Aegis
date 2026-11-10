/*
  LD2450 -> LoRa (SX1278) raw forwarder            *** TRANSMITTER ***

  Board   : ESP32 DevKit V1 (CH340G)   -> Arduino IDE: "ESP32 Dev Module"
  Library : "LoRa" by Sandeep Mistry   (Library Manager)

  Wiring (all logic is 3.3 V)
    LD2450  5V  -> 5V / VIN        LD2450 GND -> GND
    LD2450  TX  -> GPIO16 (RX2)    LD2450 RX  -> GPIO17 (TX2)
    SX1278  3.3V-> 3V3             SX1278 GND -> GND   (NOT 5 V!)
    SX1278  SCK -> GPIO18   MISO -> GPIO19   MOSI -> GPIO23
    SX1278  NSS -> GPIO5    RST  -> GPIO14   DIO0 -> GPIO26
    Antenna must be attached before transmitting.

  What it does
    The LD2450 streams 30-byte frames (~10 Hz):
      AA FF 03 00 | 3 x 8 byte targets | 55 CC
    This sketch finds the frame boundaries and sends each frame byte-for-byte
    as one LoRa packet. Nothing is decoded or altered.
*/
#include <SPI.h>
#include <LoRa.h>

// ---------------- pins ----------------
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
#define LORA_NSS     5
#define LORA_RST    14
#define LORA_DIO0   26

#define LD_RX_PIN   16          // ESP32 RX2 <- LD2450 TX
#define LD_TX_PIN   17          // ESP32 TX2 -> LD2450 RX
#define LD_BAUD     256000      // LD2450 default

// ---------------- radio (MUST match the receiver) ----------------
#define LORA_FREQ   433E6       // SX1278 = 433 MHz class. Check the legal band for your country.
#define LORA_SF     7
#define LORA_BW     125E3
#define LORA_CR     5           // 4/5
#define LORA_SYNC   0xF3        // private network (not 0x34 = LoRaWAN)
#define LORA_POWER  17          // dBm

// LD2450 sends ~10 frames/s. One 30 B packet = ~72 ms airtime at SF7/125k,
// so 200 ms (5 packets/s) is ~36 % duty. Raise this if your rules limit duty cycle.
// Only the NEWEST frame is sent; older ones in between are dropped.
#define TX_INTERVAL_MS  200

// ---- parser begin ----
#define LD_FRAME_LEN 30
static const uint8_t LD_HDR[4] = {0xAA, 0xFF, 0x03, 0x00};

// Feed one byte. Returns true when a complete, valid frame has been copied to out[30].
bool ldFeed(uint8_t b, uint8_t *out) {
  static uint8_t buf[LD_FRAME_LEN];
  static uint8_t n = 0;

  if (n < 4) {                              // hunting for header AA FF 03 00
    if (b == LD_HDR[n]) {
      buf[n++] = b;
    } else {
      n = 0;
      if (b == LD_HDR[0]) buf[n++] = b;     // this byte may start a new header
    }
    return false;
  }
  buf[n++] = b;
  if (n < LD_FRAME_LEN) return false;
  n = 0;
  if (buf[28] != 0x55 || buf[29] != 0xCC) return false;   // bad tail: drop, resync
  memcpy(out, buf, LD_FRAME_LEN);
  return true;
}
// ---- parser end ----

static void printHex(const uint8_t *d, size_t len) {
  for (size_t i = 0; i < len; i++) Serial.printf("%02X ", d[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nLD2450 -> LoRa TX");

  Serial2.setRxBufferSize(1024);            // must be before begin()
  Serial2.begin(LD_BAUD, SERIAL_8N1, LD_RX_PIN, LD_TX_PIN);

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
  LoRa.setTxPower(LORA_POWER);
  LoRa.enableCrc();
  Serial.println("LoRa ready");
}

void loop() {
  static uint8_t  frame[LD_FRAME_LEN];
  static bool     pending   = false;
  static uint32_t lastTx    = 0;
  static uint32_t lastFrame = 0;
  static uint32_t sent      = 0;
  static bool     warned    = false;

  while (Serial2.available()) {
    if (ldFeed((uint8_t)Serial2.read(), frame)) {
      pending   = true;
      lastFrame = millis();
      warned    = false;
    }
  }

  if (pending && millis() - lastTx >= TX_INTERVAL_MS) {
    LoRa.beginPacket();
    LoRa.write(frame, LD_FRAME_LEN);
    LoRa.endPacket();                       // blocking, ~72 ms
    lastTx  = millis();
    pending = false;
    Serial.printf("TX #%lu  ", (unsigned long)++sent);
    printHex(frame, LD_FRAME_LEN);
  }

  if (!warned && millis() - lastFrame > 2000) {
    Serial.println("No valid LD2450 frames for 2 s. Check TX/RX wiring, 5 V power, baud 256000.");
    warned = true;
  }
}
