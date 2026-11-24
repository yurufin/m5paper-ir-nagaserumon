// ir-timer: 流せるもんの洗浄信号を決まった時刻に送る(M5Paper + IR Unit U002 on PORT.B)
//
//   信号: SD の /ir/flush.txt(ir-register で採用したもの)。マーク/スペースの µs 並び
//   時刻: 内蔵 RTC(BM8563)に日本時間をそのまま入れて使う。SD に /ssid.txt があれば起動時と 24 時間ごとに NTP で合わせる
//   予定: /ir/schedule.txt に HH:MM を 1 行ずつ(無ければ SCHEDULE_DEFAULT)
//   手動: 画面の「今すぐ流す」ボタン、または押込ボタン(G38)。手動は 15 分ガードの対象外
//   記録: 累計回数・最後の実行は NVS(Preferences)、履歴は SD の /ir/log.txt
//
//   常時起動(USB 給電前提)。タッチでいつでも流せるようにするため電源は落とさない。
#include <M5Unified.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <vector>
#include <algorithm>

// ---- 設定 ----
static const char* SCHEDULE_DEFAULT[] = {"00:00", "06:00", "12:00", "18:00"};
static constexpr uint32_t IR_FREQUENCY_HZ = 38000;
static constexpr float IR_DUTY = 0.33f;
static constexpr int IR_REPEAT_COUNT = 1;           // 1 回の洗浄で送る回数(受信データ自体にリピートフレームを含む)
static constexpr uint32_t IR_REPEAT_WAIT_MS = 500;
static constexpr uint32_t AUTO_GUARD_SEC = 15 * 60; // 自動送信どうしの最小間隔(再起動の連続で何度も流さない)
static constexpr uint32_t CATCHUP_SEC = 30 * 60;    // 予定時刻をこれ以内に過ぎていれば(停電復帰など)送る
static constexpr uint32_t MANUAL_DEBOUNCE_MS = 3000; // 手動の二度押し防止
static constexpr uint32_t NTP_INTERVAL_MS = 24UL * 60 * 60 * 1000;
static const char* NTP_SERVER1 = "ntp.nict.jp";
static const char* NTP_SERVER2 = "pool.ntp.org";

// ---- ハード構成 ----
static constexpr int EXT_PWR_PIN = 5;
static constexpr int PORTB_PIN_33 = 33, PORTB_PIN_26 = 26;
static constexpr int SD_CS = 4, SD_SCK = 14, SD_MISO = 13, SD_MOSI = 12;

static const char* SIGNAL_PATH = "/ir/flush.txt";
static const char* SCHEDULE_PATH = "/ir/schedule.txt";
static const char* LOG_PATH = "/ir/log.txt";

// 画面: 縦 540x960、天地逆(USB 端子が上)。向きが逆なら DISPLAY_ROTATION を 0 にする
static constexpr int DISPLAY_ROTATION = 2;
static constexpr int BTN_X = 30, BTN_Y = 430, BTN_W = 480, BTN_H = 220;
static constexpr int MSG_Y = BTN_Y + BTN_H + 70;

// ---- 状態 ----
static int rxPin = PORTB_PIN_33, txPin = PORTB_PIN_26;
static String pinNote;
static bool sdOk = false;
static std::vector<uint16_t> signalRaw;
static String signalNote;
static std::vector<int> scheduleMin;  // 0:00 からの分。昇順
static String scheduleNote;
static Preferences prefs;
static String message;     // ボタンの下に大きく(MANUAL TEST OK など)
static String messageSub;  // その下に時刻
static String ntpNote = "NTP: 未実行";
static uint32_t lastNtpMs = 0;
static uint32_t lastManualMs = 0;
static int lastDrawnMinute = -1;

enum Kind : uint8_t { KIND_NONE = 0, KIND_AUTO = 1, KIND_MANUAL = 2 };

// ---- 時刻(RTC の日本時間を「ローカル秒」として扱う。UTC には直さない) ----
static int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

// 有効な時刻なら true。now はローカル秒(1970-01-01 00:00 JST からの秒)
static bool readNow(uint32_t& now, m5::rtc_datetime_t& dt) {
  if (!M5.Rtc.getDateTime(&dt)) return false;
  if (dt.date.year < 2025 || dt.date.year > 2099) return false;
  now = (uint32_t)(daysFromCivil(dt.date.year, dt.date.month, dt.date.date) * 86400 + dt.time.hours * 3600 +
                   dt.time.minutes * 60 + dt.time.seconds);
  return true;
}

static String fmtTime(uint32_t t, bool withDate = true) {
  time_t tt = (time_t)t;
  struct tm tmv;
  gmtime_r(&tt, &tmv);  // ローカル秒なので gmtime で分解すればそのまま日本時間
  char buf[32];
  if (withDate) {
    snprintf(buf, sizeof(buf), "%d/%02d/%02d %02d:%02d", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour,
             tmv.tm_min);
  } else {
    snprintf(buf, sizeof(buf), "%02d:%02d", tmv.tm_hour, tmv.tm_min);
  }
  return String(buf);
}

// now 以前で一番新しい予定時刻 / now より後で一番近い予定時刻
static uint32_t prevSlot(uint32_t now) {
  uint32_t day = now / 86400 * 86400;
  int curMin = (now % 86400) / 60;
  for (int i = (int)scheduleMin.size() - 1; i >= 0; i--) {
    if (scheduleMin[i] <= curMin) return day + scheduleMin[i] * 60;
  }
  return day - 86400 + scheduleMin.back() * 60;
}

static uint32_t nextSlot(uint32_t now) {
  uint32_t day = now / 86400 * 86400;
  int curMin = (now % 86400) / 60;
  for (int m : scheduleMin) {
    if (m > curMin) return day + m * 60;
  }
  return day + 86400 + scheduleMin.front() * 60;
}

// ---- 受信ピン判定(ir-register と同じ。受信モジュールは待機中 HIGH) ----
static void detectPins() {
  pinMode(PORTB_PIN_33, INPUT_PULLDOWN);
  pinMode(PORTB_PIN_26, INPUT_PULLDOWN);
  delay(50);
  int h33 = 0, h26 = 0;
  for (int i = 0; i < 20; i++) {
    h33 += digitalRead(PORTB_PIN_33);
    h26 += digitalRead(PORTB_PIN_26);
    delay(2);
  }
  if (h33 >= 15 && h26 < 15) {
    rxPin = PORTB_PIN_33; txPin = PORTB_PIN_26; pinNote = "自動判定";
  } else if (h26 >= 15 && h33 < 15) {
    rxPin = PORTB_PIN_26; txPin = PORTB_PIN_33; pinNote = "自動判定";
  } else {
    rxPin = PORTB_PIN_33; txPin = PORTB_PIN_26; pinNote = "既定値";
  }
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);
}

// ---- SD ----
static void loadSignal() {
  signalRaw.clear();
  File f = SD.open(SIGNAL_PATH);
  if (!f) { signalNote = String(SIGNAL_PATH) + " がありません"; return; }
  String num;
  bool comment = false;
  auto flushNum = [&]() {
    if (num.length()) {
      long v = num.toInt();
      if (v > 0 && v <= 65535) signalRaw.push_back((uint16_t)v);
      num = "";
    }
  };
  while (f.available()) {
    char ch = f.read();
    if (ch == '\n') { comment = false; flushNum(); continue; }
    if (comment) continue;
    if (ch == '#') { flushNum(); comment = true; continue; }
    if (ch >= '0' && ch <= '9') num += ch; else flushNum();
  }
  flushNum();
  f.close();
  if (signalRaw.size() < 10) {
    signalNote = String(SIGNAL_PATH) + " の中身が足りません";
    signalRaw.clear();
    return;
  }
  uint32_t total = 0;
  for (auto v : signalRaw) total += v;
  char buf[64];
  snprintf(buf, sizeof(buf), "長さ %u / 全長 %.1f ms", (unsigned)signalRaw.size(), total / 1000.0);
  signalNote = buf;
}

static void loadSchedule() {
  scheduleMin.clear();
  scheduleNote = "既定";
  if (sdOk) {
    File f = SD.open(SCHEDULE_PATH);
    if (f) {
      while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (!line.length() || line[0] == '#') continue;
        int colon = line.indexOf(':');
        if (colon < 1) continue;
        int h = line.substring(0, colon).toInt(), m = line.substring(colon + 1).toInt();
        if (h >= 0 && h < 24 && m >= 0 && m < 60) scheduleMin.push_back(h * 60 + m);
      }
      f.close();
      if (!scheduleMin.empty()) scheduleNote = SCHEDULE_PATH;
    }
  }
  if (scheduleMin.empty()) {
    for (auto s : SCHEDULE_DEFAULT) scheduleMin.push_back(atoi(s) * 60 + atoi(s + 3));
  }
  std::sort(scheduleMin.begin(), scheduleMin.end());
  scheduleMin.erase(std::unique(scheduleMin.begin(), scheduleMin.end()), scheduleMin.end());
}

static void appendLog(const String& line) {
  Serial.println(line);
  if (!sdOk) return;
  M5.Display.waitDisplay();
  File f = SD.open(LOG_PATH, FILE_APPEND);
  if (!f) return;
  f.println(line);
  f.close();
}

// ---- NTP(SD の /ssid.txt: 1 行 "SSID,PASSWORD"。Clock225 と同じ形式) ----
static void syncNtp() {
  lastNtpMs = millis();
  if (!sdOk) { ntpNote = "NTP: SD なし"; return; }
  File f = SD.open("/ssid.txt");
  if (!f) { ntpNote = "NTP: /ssid.txt なし"; return; }
  std::vector<std::pair<String, String>> creds;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line[0] == '#') continue;
    int c = line.indexOf(',');
    if (c < 1) continue;
    creds.push_back({line.substring(0, c), line.substring(c + 1)});
  }
  f.close();

  WiFi.mode(WIFI_STA);
  bool connected = false;
  for (auto& cr : creds) {
    WiFi.begin(cr.first.c_str(), cr.second.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
    if (WiFi.status() == WL_CONNECTED) { connected = true; break; }
    WiFi.disconnect();
  }
  if (!connected) {
    ntpNote = "NTP: Wi-Fi 接続失敗";
  } else {
    // M5.begin が RTC から入れたシステム時刻が残っていると getLocalTime が同期前に返るので、いったん 0 にする
    struct timeval zero = {0, 0};
    settimeofday(&zero, nullptr);
    configTzTime("JST-9", NTP_SERVER1, NTP_SERVER2);
    struct tm tmv;
    if (getLocalTime(&tmv, 10000)) {
      M5.Rtc.setDateTime(&tmv);  // RTC には日本時間をそのまま入れる
      uint32_t now; m5::rtc_datetime_t dt;
      ntpNote = readNow(now, dt) ? "NTP: " + fmtTime(now) + " に合わせました" : String("NTP: RTC 書き込み失敗");
    } else {
      ntpNote = "NTP: 時刻を取れませんでした";
    }
  }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(ntpNote);
}

// ---- 赤外線送信(RMT、1 tick = 1µs、38kHz キャリア) ----
static bool rmtReady = false;

static bool sendIrOnce() {
  if (signalRaw.empty()) return false;
  if (!rmtReady) {
    rmtReady = rmtInit(txPin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000) &&
               rmtSetCarrier(txPin, true, false, IR_FREQUENCY_HZ, IR_DUTY) && rmtSetEOT(txPin, LOW);  // 第3引数は polarity_active_low。true だと待機中にキャリアが出っぱなしになる
    if (!rmtReady) return false;
  }
  // RMT の 1 区間は最大 32767 tick なので長いスペースは分ける
  std::vector<std::pair<uint16_t, bool>> halves;
  for (size_t i = 0; i < signalRaw.size(); i++) {
    bool mark = (i % 2) == 0;
    uint32_t d = signalRaw[i];
    while (d > 0) {
      uint16_t part = d > 32767 ? 32767 : (uint16_t)d;
      halves.push_back({part, mark});
      d -= part;
    }
  }
  if (halves.size() % 2) halves.push_back({100, false});
  std::vector<rmt_data_t> syms(halves.size() / 2);
  for (size_t i = 0; i < syms.size(); i++) {
    syms[i].duration0 = halves[i * 2].first;
    syms[i].level0 = halves[i * 2].second;
    syms[i].duration1 = halves[i * 2 + 1].first;
    syms[i].level1 = halves[i * 2 + 1].second;
  }
  return rmtWrite(txPin, syms.data(), syms.size(), 3000);
}

static bool sendFlush() {
  bool ok = true;
  for (int i = 0; i < IR_REPEAT_COUNT; i++) {
    if (i) delay(IR_REPEAT_WAIT_MS);
    ok = sendIrOnce() && ok;
  }
  return ok;
}

// ---- 画面 ----
static void drawClockLine() {
  uint32_t now; m5::rtc_datetime_t dt;
  auto& d = M5.Display;
  d.startWrite();
  d.fillRect(30, 80, 480, 40, TFT_WHITE);
  d.setFont(&fonts::lgfxJapanGothic_36);
  d.drawString(readNow(now, dt) ? fmtTime(now) : String("時刻未設定"), 30, 80);
  d.endWrite();
  d.display();
}

static void drawButton(bool pressed) {
  auto& d = M5.Display;
  d.startWrite();
  d.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 24, pressed ? TFT_BLACK : TFT_WHITE);
  d.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 24, TFT_BLACK);
  d.drawRoundRect(BTN_X + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, 23, TFT_BLACK);
  d.drawRoundRect(BTN_X + 2, BTN_Y + 2, BTN_W - 4, BTN_H - 4, 22, TFT_BLACK);
  d.setFont(&fonts::lgfxJapanGothic_40);
  d.setTextColor(pressed ? TFT_WHITE : TFT_BLACK);
  d.setTextDatum(middle_center);
  d.drawString(pressed ? "送信中…" : "今すぐ流す", BTN_X + BTN_W / 2, BTN_Y + BTN_H / 2);
  d.setTextDatum(top_left);
  d.setTextColor(TFT_BLACK, TFT_WHITE);
  d.endWrite();
  d.display();
}

static void drawScreen() {
  auto& d = M5.Display;
  uint32_t now; m5::rtc_datetime_t dt;
  bool valid = readNow(now, dt);

  d.startWrite();
  d.fillScreen(TFT_WHITE);
  d.setTextColor(TFT_BLACK, TFT_WHITE);
  d.setFont(&fonts::lgfxJapanGothic_40);
  d.drawString("AUTO TOILET FLUSHER", 30, 20);

  d.setFont(&fonts::lgfxJapanGothic_24);
  int y = 140;
  String sched = "予定:";
  for (int m : scheduleMin) {
    char b[8];
    snprintf(b, sizeof(b), " %02d:%02d", m / 60, m % 60);
    sched += b;
  }
  d.drawString(sched, 30, y); y += 34;
  d.drawString(valid ? "次回: " + fmtTime(nextSlot(now)) : String("次回: 時刻未設定のため自動送信しません"), 30, y); y += 46;

  uint32_t lastT = prefs.getUInt("lastTime", 0);
  uint8_t lastK = prefs.getUChar("lastKind", KIND_NONE);
  String lastS = "最終: ";
  if (lastK == KIND_NONE) lastS += "なし";
  else lastS += (lastT ? fmtTime(lastT) : String("時刻不明")) + (lastK == KIND_AUTO ? "(自動)" : "(手動)");
  d.drawString(lastS, 30, y); y += 34;
  d.drawString(String("累計: ") + prefs.getUInt("total", 0) + " 回", 30, y); y += 46;

  d.drawString("信号: " + signalNote, 30, y); y += 34;
  int mv = M5.Power.getBatteryVoltage();
  d.drawString(String("電池: ") + (mv > 0 ? String(mv / 1000.0, 2) + " V" : String("-")) + "   送信 G" + txPin +
                   "(" + pinNote + ")",
               30, y); y += 34;
  d.drawString(ntpNote, 30, y); y += 34;

  d.setFont(&fonts::lgfxJapanGothic_20);
  d.drawString("押込ボタン(G38)でも流せます", BTN_X + 10, BTN_Y + BTN_H + 14);

  if (message.length()) {
    d.setFont(&fonts::lgfxJapanGothic_40);
    d.drawString(message, 30, MSG_Y);
    d.setFont(&fonts::lgfxJapanGothic_24);
    d.drawString(messageSub, 30, MSG_Y + 56);
  }
  d.endWrite();

  drawButton(false);
  drawClockLine();
  lastDrawnMinute = valid ? (int)(now / 60) : -1;
}

// ---- 実行 ----
static void doFlush(Kind kind, uint32_t now, bool timeValid) {
  if (kind == KIND_MANUAL) drawButton(true);
  bool ok = sendFlush();
  String when = timeValid ? fmtTime(now) : String("時刻不明");
  if (ok) {
    prefs.putUInt("total", prefs.getUInt("total", 0) + 1);
    prefs.putUInt("lastTime", timeValid ? now : 0);
    prefs.putUChar("lastKind", kind);
    if (kind == KIND_AUTO) prefs.putUInt("lastAuto", now);
    message = kind == KIND_AUTO ? "AUTO FLUSH OK" : "MANUAL TEST OK";
    messageSub = when;
  } else {
    message = kind == KIND_AUTO ? "AUTO FLUSH 失敗" : "MANUAL TEST 失敗";
    messageSub = when + (signalRaw.empty() ? " 信号なし" : "");
  }
  appendLog(when + "," + (kind == KIND_AUTO ? "auto" : "manual") + "," + (ok ? "ok" : "ng"));
  drawScreen();
}

static void checkSchedule() {
  uint32_t now; m5::rtc_datetime_t dt;
  if (!readNow(now, dt)) return;
  uint32_t slot = prevSlot(now);

  if (!prefs.isKey("lastSlot")) prefs.putUInt("lastSlot", slot);  // 初回起動ではすぐには流さない
  uint32_t lastSlot = prefs.getUInt("lastSlot", 0);
  if (lastSlot > now + 3600) lastSlot = 0;  // 時計が大きく戻されたとき
  if (slot <= lastSlot) return;

  prefs.putUInt("lastSlot", slot);
  uint32_t lastAuto = prefs.getUInt("lastAuto", 0);
  if (lastAuto > now) lastAuto = 0;
  if (now - slot > CATCHUP_SEC) {
    appendLog(fmtTime(now) + ",auto,skip(" + fmtTime(slot, false) + " から時間が経ちすぎ)");
    return;
  }
  if (lastAuto && now - lastAuto < AUTO_GUARD_SEC) {
    appendLog(fmtTime(now) + ",auto,skip(前回の自動送信から15分以内)");
    return;
  }
  doFlush(KIND_AUTO, now, true);
}

static void manualFlush() {
  if (millis() - lastManualMs < MANUAL_DEBOUNCE_MS) return;
  uint32_t now; m5::rtc_datetime_t dt;
  bool valid = readNow(now, dt);
  doFlush(KIND_MANUAL, now, valid);
  lastManualMs = millis();
}

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);

  pinMode(EXT_PWR_PIN, OUTPUT);
  digitalWrite(EXT_PWR_PIN, HIGH);
  delay(200);  // U002 の電源安定待ち

  M5.Display.setRotation(DISPLAY_ROTATION);
  M5.Display.setEpdMode(epd_mode_t::epd_fast);
  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);

  detectPins();
  prefs.begin("flush", false);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, SPI, 20000000);
  if (sdOk) loadSignal(); else signalNote = "SD が読めません";
  loadSchedule();

  M5.Display.fillScreen(TFT_WHITE);
  M5.Display.setFont(&fonts::lgfxJapanGothic_24);
  M5.Display.drawString("起動中: 時刻合わせ…", 30, 30);
  M5.Display.display();
  syncNtp();

  Serial.printf("ir-timer: tx=G%d (%s) signal=%s schedule=%s\n", txPin, pinNote.c_str(), signalNote.c_str(),
                scheduleNote.c_str());
  drawScreen();
}

void loop() {
  M5.update();

  if (M5.BtnB.wasPressed()) manualFlush();
  auto t = M5.Touch.getDetail();
  if (t.wasPressed() && t.x >= BTN_X && t.x < BTN_X + BTN_W && t.y >= BTN_Y && t.y < BTN_Y + BTN_H) manualFlush();

  static uint32_t lastTick = 0;
  if (millis() - lastTick >= 1000) {
    lastTick = millis();
    checkSchedule();
    uint32_t now; m5::rtc_datetime_t dt;
    if (readNow(now, dt) && (int)(now / 60) != lastDrawnMinute) {
      lastDrawnMinute = now / 60;
      drawClockLine();
    }
    if (millis() - lastNtpMs >= NTP_INTERVAL_MS) {
      syncNtp();
      drawScreen();
    }
  }
  delay(10);
}
