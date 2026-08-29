/*
 * M5Stack CoreS3 + Module GNSS (NEO-M9N)
 *   -> RaceChrono BLE DIY GPS device
 *
 * NMEA を TinyGPSPlus でパースして、RaceChrono の BLE DIY API
 * (service 0x1FF8) の GPS 機能として配信する。
 *
 * 仕様: https://github.com/aollin/racechrono-ble-diy-device
 */
#include <M5Unified.h>
#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLE2902.h>

static const int      GNSS_RX   = 18;
static const int      GNSS_TX   = 17;
static const uint32_t GNSS_BAUD = 38400;

static const char *DEVICE_NAME = "M5-RC-GPS";
#define SVC_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
#define CH_MAIN  "00000003-0000-1000-8000-00805f9b34fb"
#define CH_TIME  "00000004-0000-1000-8000-00805f9b34fb"

TinyGPSPlus       gps;
BLECharacteristic *chMain = nullptr;
BLECharacteristic *chTime = nullptr;

volatile bool bleConnected = false;
uint8_t       syncBits     = 0;
uint32_t      lastHourKey  = 0xFFFFFFFF;
uint32_t      lastTimeKey  = 0xFFFFFFFF;
uint32_t      lastDraw     = 0;
uint32_t      epochCount   = 0;   // 受信した測位エポック数（レート確認用）
uint32_t      rateWindow   = 0;
uint32_t      lastHz       = 0;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }
  void onDisconnect(BLEServer *) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

static inline void put24(uint8_t *p, uint32_t v) { p[0] = v >> 16; p[1] = v >> 8; p[2] = v; }
static inline void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8;  p[1] = v; }
static inline void put32(uint8_t *p, uint32_t v) {
  p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

// ---------------------------------------------------------------------------
// UBX フレーム送信（チェックサムはコードで計算するので手打ちミスが起きない）
// ---------------------------------------------------------------------------
void sendUBX(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t head[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < 6; i++) { ckA += head[i]; ckB += ckA; }
  for (uint16_t i = 0; i < len; i++) { ckA += payload[i]; ckB += ckA; }
  Serial1.write(head, 6);
  if (len) Serial1.write(payload, len);
  Serial1.write(ckA);
  Serial1.write(ckB);
  Serial1.flush();
}

// CFG-VALSET (0x06 0x8A) を1キーずつ送る。
// 1件ずつ送るので、キーが1つ間違っていても他は生きる。
void valSetU1(uint32_t key, uint8_t val) {
  uint8_t p[9] = {0x00, 0x01, 0x00, 0x00,                       // version, layer=RAM
                  (uint8_t)(key), (uint8_t)(key >> 8),
                  (uint8_t)(key >> 16), (uint8_t)(key >> 24),
                  val};
  sendUBX(0x06, 0x8A, p, sizeof(p));
  delay(50);
}
void valSetU2(uint32_t key, uint16_t val) {
  uint8_t p[10] = {0x00, 0x01, 0x00, 0x00,
                   (uint8_t)(key), (uint8_t)(key >> 8),
                   (uint8_t)(key >> 16), (uint8_t)(key >> 24),
                   (uint8_t)(val), (uint8_t)(val >> 8)};
  sendUBX(0x06, 0x8A, p, sizeof(p));
  delay(50);
}

void configureGnss() {
  // 使わないNMEAセンテンスを止める（38400bpsで10Hzを流すには帯域が足りないため）
  valSetU1(0x209100C5, 0);   // GSV off
  valSetU1(0x209100C0, 0);   // GSA off
  valSetU1(0x209100CA, 0);   // GLL off
  valSetU1(0x209100B1, 0);   // VTG off
  // 必要なものは残す
  valSetU1(0x209100BB, 1);   // GGA on
  valSetU1(0x209100AC, 1);   // RMC on
  // 測位レート 100ms = 10Hz
  valSetU2(0x30210001, 100);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setTextSize(2);
  M5.Display.println("booting...");

  Serial1.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  delay(500);
  configureGnss();

  BLEDevice::init(DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService *svc = server->createService(SVC_UUID);
  chMain = svc->createCharacteristic(
      CH_MAIN, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  chMain->addDescriptor(new BLE2902());
  chTime = svc->createCharacteristic(
      CH_TIME, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  chTime->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  M5.Display.clear();
}

void sendEpoch() {
  const bool hasFix = gps.location.isValid() && gps.location.age() < 2000;
  const uint8_t sats = hasFix ? (uint8_t)gps.satellites.value() : 0;

  // ---- 0x0004: 日付 + 時 ----
  if (gps.date.isValid()) {
    const uint32_t hourKey = (uint32_t)(gps.date.year() - 2000) * 8928
                           + (uint32_t)(gps.date.month() - 1) * 744
                           + (uint32_t)(gps.date.day() - 1) * 24
                           + (uint32_t)gps.time.hour();
    if (hourKey != lastHourKey) {
      lastHourKey = hourKey;
      syncBits    = (syncBits + 1) & 0x07;
      uint8_t t[3];
      put24(t, ((uint32_t)syncBits << 21) | (hourKey & 0x1FFFFF));
      chTime->setValue(t, 3);
      if (bleConnected) chTime->notify();
    }
  }

  // ---- 0x0003: 測位データ本体 ----
  uint8_t pkt[20];

  const uint32_t tms = (uint32_t)gps.time.minute() * 30000
                     + (uint32_t)gps.time.second() * 500
                     + (uint32_t)gps.time.centisecond() * 10 / 2;
  put24(pkt, ((uint32_t)syncBits << 21) | (tms & 0x1FFFFF));

  pkt[3] = ((hasFix ? 1 : 0) << 6) | (hasFix ? (sats & 0x3F) : 0x3F);

  if (hasFix) {
    put32(pkt + 4, (uint32_t)(int32_t)llround(gps.location.lat() * 10000000.0));
    put32(pkt + 8, (uint32_t)(int32_t)llround(gps.location.lng() * 10000000.0));
  } else {
    put32(pkt + 4, 0x7FFFFFFF);
    put32(pkt + 8, 0x7FFFFFFF);
  }

  uint16_t alt = 0xFFFF;
  if (hasFix && gps.altitude.isValid()) {
    const double m = gps.altitude.meters();
    if (m > -500.0 && m < 6053.0) alt = (uint16_t)lround((m + 500.0) * 10.0) & 0x7FFF;
    else                          alt = ((uint16_t)lround(m + 500.0) & 0x7FFF) | 0x8000;
  }
  put16(pkt + 12, alt);

  uint16_t spd = 0xFFFF;
  if (hasFix && gps.speed.isValid()) {
    const double kmh = gps.speed.kmph();
    spd = (kmh < 655.0) ? ((uint16_t)lround(kmh * 100.0) & 0x7FFF)
                        : (((uint16_t)lround(kmh * 10.0) & 0x7FFF) | 0x8000);
  }
  put16(pkt + 14, spd);

  uint16_t brg = 0xFFFF;
  if (hasFix && gps.course.isValid()) brg = (uint16_t)lround(gps.course.deg() * 100.0);
  put16(pkt + 16, brg);

  // TinyGPSPlus の hdop.value() は 1/100 単位。仕様は 1/10 単位なので 10 で割る
  const uint32_t hv = gps.hdop.isValid() ? gps.hdop.value() : 0;
  pkt[18] = (hv == 0 || hv > 2550) ? 0xFF : (uint8_t)(hv / 10);
  pkt[19] = 0xFF;

  chMain->setValue(pkt, 20);
  if (bleConnected) chMain->notify();
}

void loop() {
  M5.update();

  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // 時刻が動いたら1エポック分として送信（RMC/GGAの二重送信を防ぐ）
  if (gps.time.isValid()) {
    const uint32_t timeKey = (uint32_t)gps.time.hour() * 360000
                           + (uint32_t)gps.time.minute() * 6000
                           + (uint32_t)gps.time.second() * 100
                           + gps.time.centisecond();
    if (timeKey != lastTimeKey) {
      lastTimeKey = timeKey;
      sendEpoch();
      epochCount++;
    }
  }

  const uint32_t now = millis();
  if (now - rateWindow >= 1000) {
    rateWindow = now;
    lastHz     = epochCount;
    epochCount = 0;
  }

  if (now - lastDraw > 500) {
    lastDraw = now;
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(bleConnected ? GREEN : YELLOW, BLACK);
    M5.Display.printf("%-16s\n", bleConnected ? "BLE connected" : "BLE waiting");
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.printf("sats:%2lu  %luHz  \n",
                      (unsigned long)gps.satellites.value(), (unsigned long)lastHz);
    if (gps.location.isValid()) {
      M5.Display.printf("%.6f      \n", gps.location.lat());
      M5.Display.printf("%.6f      \n", gps.location.lng());
      M5.Display.printf("%.1f km/h    \n", gps.speed.kmph());
    } else {
      M5.Display.printf("no fix          \n                \n                \n");
    }
  }
}