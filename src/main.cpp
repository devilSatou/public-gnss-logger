/*
 * M5Stack CoreS3 + Module GNSS (NEO-M9N)
 *   -> RaceChrono BLE DIY GPS device
 *
 * UBX-NAV-PVT / UBX-NAV-DOP を直接パースして、RaceChrono の BLE DIY API
 * (service 0x1FF8) の GPS 機能として配信する。
 *
 * NMEA を使わない理由:
 *   GGA の numSV フィールドは u-blox の仕様上 0-12 で頭打ちになるため、
 *   マルチGNSS で実際に使用している衛星数（20個超）を表現できない。
 *   NAV-PVT の numSV は 8bit で、実数がそのまま入る。
 *   ついでに緯度経度が 1e-7 度の整数で得られるので、RaceChrono の
 *   ペイロード形式にそのまま流し込める（NMEA の文字列往復が消える）。
 *
 * 仕様: https://github.com/aollin/racechrono-ble-diy-device
 */
#include <M5Unified.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <math.h>

static const int      GNSS_RX        = 18;
static const int      GNSS_TX        = 17;
static const uint32_t GNSS_BAUD      = 115200;  // 起動時に受信機ごと引き上げる
static const uint32_t GNSS_BOOT_BAUD = 38400;   // NEO-M9N の工場出荷値
static const uint16_t NAV_PERIOD_MS  = 100;     // 10Hz

static const char *DEVICE_NAME = "M5-RC-GPS";
#define SVC_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
#define CH_MAIN  "00000003-0000-1000-8000-00805f9b34fb"
#define CH_TIME  "00000004-0000-1000-8000-00805f9b34fb"

BLECharacteristic *chMain = nullptr;
BLECharacteristic *chTime = nullptr;

volatile bool bleConnected = false;
uint8_t       syncBits     = 0;
uint32_t      lastHourKey  = 0xFFFFFFFF;
uint32_t      lastITOW     = 0xFFFFFFFF;
uint32_t      lastDraw     = 0;
uint32_t      epochCount   = 0;   // 受信した測位エポック数（レート確認用）
uint32_t      rateWindow   = 0;
uint32_t      lastHz       = 0;

// ---------------------------------------------------------------------------
// UBX-NAV-PVT / UBX-NAV-DOP から拾った最新値
// ---------------------------------------------------------------------------
struct NavPvt {
  uint32_t iTOW;
  uint16_t year;
  uint8_t  month, day, hour, min, sec;
  uint8_t  valid;      // bit0 validDate / bit1 validTime / bit2 fullyResolved
  int32_t  nano;       // 秒の端数 (-1e9 .. 1e9)
  uint8_t  fixType;    // 0=no fix, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
  uint8_t  flags;      // bit0 gnssFixOK
  uint8_t  numSV;      // 測位に使用中の衛星数（頭打ちなし）
  int32_t  lon, lat;   // 1e-7 deg
  int32_t  hMSL;       // mm
  int32_t  gSpeed;     // mm/s
  int32_t  headMot;    // 1e-5 deg
};

NavPvt   pvt      = {};
bool     pvtFresh = false;   // 未送信の測位エポックが載っているか
uint16_t navHDOP  = 0;       // NAV-DOP hDOP (0.01 単位), 0 = 不明
uint16_t navVDOP  = 0;       // NAV-DOP vDOP (0.01 単位), 0 = 不明

static inline void put24(uint8_t *p, uint32_t v) { p[0] = v >> 16; p[1] = v >> 8; p[2] = v; }
static inline void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8;  p[1] = v; }
static inline void put32(uint8_t *p, uint32_t v) {
  p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

static inline uint32_t rdU32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t  rdI32(const uint8_t *p) { return (int32_t)rdU32(p); }
static inline uint16_t rdU16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

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
static void valSet(uint32_t key, const uint8_t *val, uint8_t n) {
  uint8_t p[12] = {0x00, 0x01, 0x00, 0x00,                       // version, layer=RAM
                   (uint8_t)(key), (uint8_t)(key >> 8),
                   (uint8_t)(key >> 16), (uint8_t)(key >> 24)};
  for (uint8_t i = 0; i < n; i++) p[8 + i] = val[i];
  sendUBX(0x06, 0x8A, p, (uint16_t)(8 + n));
  delay(50);
}
void valSetU1(uint32_t key, uint8_t val)  { valSet(key, &val, 1); }
void valSetU2(uint32_t key, uint16_t val) {
  uint8_t v[2] = {(uint8_t)val, (uint8_t)(val >> 8)};
  valSet(key, v, 2);
}
void valSetU4(uint32_t key, uint32_t val) {
  uint8_t v[4] = {(uint8_t)val, (uint8_t)(val >> 8), (uint8_t)(val >> 16), (uint8_t)(val >> 24)};
  valSet(key, v, 4);
}

void configureGnss() {
  // NMEA は一切使わないので UART1 から全部止める（そのぶん UBX に帯域を回す）
  valSetU1(0x209100BB, 0);   // GGA off
  valSetU1(0x209100AC, 0);   // RMC off
  valSetU1(0x209100C5, 0);   // GSV off
  valSetU1(0x209100C0, 0);   // GSA off
  valSetU1(0x209100CA, 0);   // GLL off
  valSetU1(0x209100B1, 0);   // VTG off
  // 必要な UBX を毎エポック出す
  valSetU1(0x20910007, 1);   // CFG-MSGOUT-UBX_NAV_PVT_UART1
  valSetU1(0x20910039, 1);   // CFG-MSGOUT-UBX_NAV_DOP_UART1
  // 測位レート 100ms = 10Hz、1測位ごとに1回出力
  valSetU2(0x30210001, NAV_PERIOD_MS);   // CFG-RATE-MEAS
  valSetU2(0x30210002, 1);               // CFG-RATE-NAV
}

// 受信機のボーレートを GNSS_BAUD に引き上げる。
// layer=RAM なので電源を切れば工場出荷値に戻る。ESP32 だけリセットされた場合に
// 備えて、旧ボーレートと新ボーレートの両方から同じ設定を投げておく。
void raiseGnssBaud() {
  Serial1.begin(GNSS_BOOT_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  delay(200);
  valSetU4(0x40520001, GNSS_BAUD);   // CFG-UART1-BAUDRATE
  delay(150);

  Serial1.updateBaudRate(GNSS_BAUD);
  delay(150);
  valSetU4(0x40520001, GNSS_BAUD);   // すでに GNSS_BAUD だった場合の保険（冪等）
  delay(150);
  while (Serial1.available()) Serial1.read();   // 切り替え中の化けバイトを捨てる
}

// ---------------------------------------------------------------------------
// 民間暦 <-> 2000-01-01 起点の通算日（Howard Hinnant の days_from_civil 系）
// 秒の端数 nano を足した結果が日をまたいでも破綻しないように使う。
// ---------------------------------------------------------------------------
static int32_t daysFromCivil(int32_t y, int32_t m, int32_t d) {
  y -= (m <= 2);
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const int32_t yoe = y - era * 400;                                    // 0..399
  const int32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;   // 0..365
  const int32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;            // 0..146096
  return era * 146097 + doe - 730425;
}
static void civilFromDays(int32_t z, int32_t &y, int32_t &m, int32_t &d) {
  z += 730425;
  const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  const int32_t doe = z - era * 146097;                                 // 0..146096
  const int32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int32_t mp  = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp + (mp < 10 ? 3 : -9);
  y = yoe + era * 400 + (m <= 2);
}

// ---------------------------------------------------------------------------
// UBX ストリームのバイト単位パーサ
// ---------------------------------------------------------------------------
static void handleUbx(uint8_t cls, uint8_t id, const uint8_t *b, uint16_t len) {
  if (cls == 0x01 && id == 0x07 && len >= 72) {          // NAV-PVT
    pvt.iTOW    = rdU32(b + 0);
    pvt.year    = rdU16(b + 4);
    pvt.month   = b[6];
    pvt.day     = b[7];
    pvt.hour    = b[8];
    pvt.min     = b[9];
    pvt.sec     = b[10];
    pvt.valid   = b[11];
    pvt.nano    = rdI32(b + 16);
    pvt.fixType = b[20];
    pvt.flags   = b[21];
    pvt.numSV   = b[23];
    pvt.lon     = rdI32(b + 24);
    pvt.lat     = rdI32(b + 28);
    pvt.hMSL    = rdI32(b + 36);
    pvt.gSpeed  = rdI32(b + 60);
    pvt.headMot = rdI32(b + 64);
    pvtFresh    = true;
  } else if (cls == 0x01 && id == 0x04 && len >= 18) {   // NAV-DOP
    navVDOP = rdU16(b + 10);
    navHDOP = rdU16(b + 12);
  }
}

static void feedByte(uint8_t b) {
  static uint8_t  st = 0, cls = 0, id = 0, ckA = 0, ckB = 0;
  static uint16_t len = 0, idx = 0;
  static uint8_t  buf[92];   // NAV-PVT が最大

  switch (st) {
    case 0:                                                    // sync1
      if (b == 0xB5) st = 1;
      break;
    case 1:                                                    // sync2
      if (b == 0x62)      { ckA = ckB = 0; st = 2; }
      else if (b == 0xB5) { st = 1; }   // 0xB5 が続いた場合は sync1 のまま
      else                { st = 0; }
      break;
    case 2: cls = b; ckA += b; ckB += ckA; st = 3; break;
    case 3: id  = b; ckA += b; ckB += ckA; st = 4; break;
    case 4: len = b; ckA += b; ckB += ckA; st = 5; break;
    case 5:
      len |= (uint16_t)b << 8; ckA += b; ckB += ckA;
      idx = 0;
      st  = len ? 6 : 7;
      break;
    case 6:                                                    // payload
      if (idx < sizeof(buf)) buf[idx] = b;
      ckA += b; ckB += ckA;
      if (++idx >= len) st = 7;
      break;
    case 7: st = (b == ckA) ? 8 : 0; break;                    // CK_A
    case 8:                                                    // CK_B
      if (b == ckB && len <= sizeof(buf)) handleUbx(cls, id, buf, len);
      st = 0;
      break;
  }
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }
  void onDisconnect(BLEServer *) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setTextSize(2);
  M5.Display.println("booting...");

  raiseGnssBaud();
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

static inline bool pvtHasFix() {
  return (pvt.flags & 0x01) &&
         (pvt.fixType == 2 || pvt.fixType == 3 || pvt.fixType == 4);
}

void sendEpoch() {
  const bool hasFix = pvtHasFix();

  // 使用衛星数。RaceChrono のフィールドは 6bit で、0x3F は "不明" の予約値。
  uint8_t sats = hasFix ? pvt.numSV : 0;
  if (sats > 62) sats = 62;

  // nano を足し込んだうえで「日」と「日内オフセット(ms)」に正規化する。
  // これで秒／分／時／日付のどこをまたいでも桁上がりが破綻しない。
  int32_t day     = daysFromCivil(pvt.year, pvt.month, pvt.day);
  int64_t msOfDay = (int64_t)pvt.hour * 3600000 + (int64_t)pvt.min * 60000
                  + (int64_t)pvt.sec * 1000 + pvt.nano / 1000000;
  while (msOfDay < 0)         { msOfDay += 86400000; day -= 1; }
  while (msOfDay >= 86400000) { msOfDay -= 86400000; day += 1; }

  int32_t yy, mm, dd;
  civilFromDays(day, yy, mm, dd);
  const uint32_t hour     = (uint32_t)(msOfDay / 3600000);
  const uint32_t inHourMs = (uint32_t)(msOfDay % 3600000);

  // ---- 0x0004: 日付 + 時 ----
  const uint32_t hourKey = (uint32_t)(yy - 2000) * 8928
                         + (uint32_t)(mm - 1) * 744
                         + (uint32_t)(dd - 1) * 24
                         + hour;
  if (hourKey != lastHourKey) {
    lastHourKey = hourKey;
    syncBits    = (syncBits + 1) & 0x07;
    uint8_t t[3];
    put24(t, ((uint32_t)syncBits << 21) | (hourKey & 0x1FFFFF));
    chTime->setValue(t, 3);
    if (bleConnected) chTime->notify();
  }

  // ---- 0x0003: 測位データ本体 ----
  uint8_t pkt[20];

  const uint32_t tms = inHourMs / 2;   // 1/500 秒単位（最大 1800000 < 2^21）
  put24(pkt, ((uint32_t)syncBits << 21) | (tms & 0x1FFFFF));

  pkt[3] = ((hasFix ? 1 : 0) << 6) | (hasFix ? (sats & 0x3F) : 0x3F);

  if (hasFix) {
    // NAV-PVT の lat/lon は 1e-7 度の整数。仕様と同じ単位なのでそのまま入る。
    put32(pkt + 4, (uint32_t)pvt.lat);
    put32(pkt + 8, (uint32_t)pvt.lon);
  } else {
    put32(pkt + 4, 0x7FFFFFFF);
    put32(pkt + 8, 0x7FFFFFFF);
  }

  uint16_t alt = 0xFFFF;
  if (hasFix) {
    const double m = pvt.hMSL / 1000.0;
    if (m > -500.0 && m < 6053.0) alt = (uint16_t)lround((m + 500.0) * 10.0) & 0x7FFF;
    else                          alt = ((uint16_t)lround(m + 500.0) & 0x7FFF) | 0x8000;
  }
  put16(pkt + 12, alt);

  uint16_t spd = 0xFFFF;
  if (hasFix) {
    const double kmh = pvt.gSpeed * 0.0036;   // mm/s -> km/h
    spd = (kmh < 655.0) ? ((uint16_t)lround(kmh * 100.0) & 0x7FFF)
                        : (((uint16_t)lround(kmh * 10.0) & 0x7FFF) | 0x8000);
  }
  put16(pkt + 14, spd);

  uint16_t brg = 0xFFFF;
  if (hasFix) {
    double deg = pvt.headMot / 100000.0;      // 1e-5 deg -> deg
    deg = fmod(deg, 360.0);
    if (deg < 0) deg += 360.0;
    brg = (uint16_t)lround(deg * 100.0);
    if (brg >= 36000) brg = 0;
  }
  put16(pkt + 16, brg);

  // NAV-DOP の hDOP / vDOP は 1/100 単位。仕様は 1/10 単位なので 10 で割る。
  // 値が無い間は仕様どおり 0xFF (invalid) を送る。
  pkt[18] = (navHDOP == 0 || navHDOP > 2550) ? 0xFF : (uint8_t)(navHDOP / 10);
  pkt[19] = (navVDOP == 0 || navVDOP > 2550) ? 0xFF : (uint8_t)(navVDOP / 10);

  chMain->setValue(pkt, 20);
  if (bleConnected) chMain->notify();
}

void loop() {
  M5.update();

  while (Serial1.available()) {
    feedByte((uint8_t)Serial1.read());
  }

  // NAV-PVT を1つ受けるごとに1エポック送る。
  // iTOW も見ておき、同じ測位エポックを二重送信しない。
  if (pvtFresh) {
    pvtFresh = false;
    const bool timeOk = (pvt.valid & 0x03) == 0x03;   // validDate && validTime
    if (timeOk && pvt.iTOW != lastITOW) {
      lastITOW = pvt.iTOW;
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
    M5.Display.printf("sats:%2u  %luHz  \n",
                      (unsigned)pvt.numSV, (unsigned long)lastHz);
    if (pvtHasFix()) {
      M5.Display.printf("%.6f      \n", pvt.lat / 10000000.0);
      M5.Display.printf("%.6f      \n", pvt.lon / 10000000.0);
      M5.Display.printf("%.1f km/h    \n", pvt.gSpeed * 0.0036);
    } else {
      M5.Display.printf("no fix          \n                \n                \n");
    }
  }
}
