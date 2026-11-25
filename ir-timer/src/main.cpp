// ir-timer: 流せるもんの洗浄信号を決まった時刻に送る(M5Paper + IR Unit U002 on PORT.B)
//
//   信号: SD の /ir/flush.txt(ir-register で採用したもの)。マーク/スペースの µs 並び
//   時刻: 内蔵 RTC(BM8563)に日本時間をそのまま入れて使う。SD に /ssid.txt があれば起動時と 24 時間ごとに NTP で合わせる
//   予定: /ir/schedule.txt に HH:MM を 1 行ずつ(無ければ SCHEDULE_DEFAULT)
//   手動: 画面の「今すぐ流す」ボタン、または押込ボタン(G38)。手動は 15 分ガードの対象外
//   記録: 累計回数・最後の実行は NVS(Preferences)、履歴は SD の /ir/log.txt
//   画面: 文字は tools/gen_fonts.ps1 で作る 1 ビットのフォント(src/ui_fonts.h)。シリアルで 'S' を送ると画面を PBM で出す
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
#include <climits>

#include "ui_fonts.h"  // tools/gen_fonts.ps1 が作る

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
static constexpr int W = 540, H = 960;
static constexpr int BAR_H = 44;                                    // 上の黒帯(電池・Wi-Fi)
static constexpr int DATE_TOP = 180, TIME_BOTTOM = 376;             // 毎分書き換える範囲
static constexpr int BTN_X = 26, BTN_Y = 388, BTN_W = 488, BTN_H = 112;
static constexpr int BOX_Y = 548, BOX_BOTTOM = 948;                 // 自動洗浄時刻・最終洗浄時刻の枠

// ---- 状態 ----
static int rxPin = PORTB_PIN_33, txPin = PORTB_PIN_26;
static String pinNote;
static bool sdOk = false;
static std::vector<uint16_t> signalRaw;
static String signalNote;
static std::vector<int> scheduleMin;  // 0:00 からの分。昇順
static String scheduleNote;
static Preferences prefs;
static String message;     // 枠の下の黒帯(MANUAL FLUSH OK など)
static String ntpNote = "NTP: 未実行";  // シリアルにだけ出す
static bool ntpOk = false;              // 最後の時刻合わせが成功したか(Wi-Fi アイコン)
static uint32_t lastNtpMs = 0;
static uint32_t lastManualMs = 0;
static int drawnMinute = -1, drawnHour = -1;
static uint32_t drawnNext = 0;

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

// ---- NTP(SD の /ssid.txt: 1 行 "SSID,PASSWORD"。複数行なら上から順に試す) ----
static void syncNtp() {
  lastNtpMs = millis();
  ntpOk = false;
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
      ntpOk = readNow(now, dt);
      ntpNote = ntpOk ? "NTP: " + fmtTime(now) + " に合わせました" : String("NTP: RTC 書き込み失敗");
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
//   1 ビットのキャンバスに全部描き、変わった範囲だけ電子ペーパーへ送る
static M5Canvas canvas(&M5.Display);

static const UiGlyph* findGlyph(const UiFont& f, uint32_t code) {
  int lo = 0, hi = f.count - 1;
  while (lo <= hi) {
    int mid = (lo + hi) / 2;
    if (f.glyphs[mid].code == code) return &f.glyphs[mid];
    if (f.glyphs[mid].code < code) lo = mid + 1; else hi = mid - 1;
  }
  return nullptr;  // フォントに無い文字は描かない(gen_fonts.cs に足す)
}

static std::vector<uint32_t> decodeUtf8(const String& s) {
  std::vector<uint32_t> out;
  const uint8_t* p = (const uint8_t*)s.c_str();
  while (*p) {
    uint32_t c = *p;
    int n = c < 0x80 ? 0 : c < 0xE0 ? 1 : c < 0xF0 ? 2 : 3;
    if (n) c &= (0x3F >> n);
    p++;
    for (int i = 0; i < n && (*p & 0xC0) == 0x80; i++, p++) c = (c << 6) | (*p & 0x3F);
    out.push_back(c);
  }
  return out;
}

struct Ink { int left = INT_MAX, right = INT_MIN, top = INT_MAX, bottom = INT_MIN; };

// 描画原点(行の左上)を 0,0 としたときのインクの範囲
static Ink measure(const UiFont& f, const String& s, int spacing) {
  Ink k;
  int pen = 0;
  for (uint32_t c : decodeUtf8(s)) {
    const UiGlyph* g = findGlyph(f, c);
    if (!g) continue;
    if (g->w) {
      k.left = std::min(k.left, pen + g->x);
      k.right = std::max(k.right, pen + g->x + g->w);
      k.top = std::min(k.top, (int)g->y);
      k.bottom = std::max(k.bottom, g->y + g->h);
    }
    pen += g->adv + spacing;
  }
  if (k.left == INT_MAX) k = Ink{0, 0, 0, 0};
  return k;
}

static void drawAt(const UiFont& f, const String& s, int ox, int oy, uint16_t color, int spacing) {
  int pen = ox;
  for (uint32_t c : decodeUtf8(s)) {
    const UiGlyph* g = findGlyph(f, c);
    if (!g) continue;
    if (g->w) canvas.drawBitmap(pen + g->x, oy + g->y, f.bits + g->off, g->w, g->h, color);
    pen += g->adv + spacing;
  }
}

// インクの中心を (cx, top〜bottom の中央) に合わせる
static void drawCentered(const UiFont& f, const String& s, int cx, int top, int bottom, uint16_t color = TFT_BLACK,
                         int spacing = 0) {
  Ink k = measure(f, s, spacing);
  drawAt(f, s, cx - (k.left + k.right) / 2, (top + bottom) / 2 - (k.top + k.bottom) / 2, color, spacing);
}

// インクの左端を x に、上下は top〜bottom の中央に
static void drawLeft(const UiFont& f, const String& s, int x, int top, int bottom, uint16_t color = TFT_BLACK) {
  Ink k = measure(f, s, 0);
  drawAt(f, s, x - k.left, (top + bottom) / 2 - (k.top + k.bottom) / 2, color, 0);
}

// 太さ t の角丸の枠
static void frameRound(int x, int y, int w, int h, int r, int t, uint16_t inner = TFT_WHITE) {
  canvas.fillRoundRect(x, y, w, h, r, TFT_BLACK);
  canvas.fillRoundRect(x + t, y + t, w - 2 * t, h - 2 * t, std::max(r - t, 0), inner);
}

static void drawStatusBar() {
  canvas.fillRect(0, 0, W, BAR_H, TFT_BLACK);
  // 電池
  int level = M5.Power.getBatteryLevel();
  const int bx = 14, by = 11, bw = 40, bh = 22;
  canvas.fillRect(bx, by, bw, bh, TFT_WHITE);
  canvas.fillRect(bx + 2, by + 2, bw - 4, bh - 4, TFT_BLACK);
  canvas.fillRect(bx + bw, by + 6, 4, bh - 12, TFT_WHITE);
  if (level > 0) canvas.fillRect(bx + 4, by + 4, (bw - 8) * std::min(level, 100) / 100, bh - 8, TFT_WHITE);
  drawLeft(FONT_STATUS, level >= 0 ? String(level) + "%" : String("--%"), bx + bw + 14, 0, BAR_H, TFT_WHITE);
  // Wi-Fi(最後の時刻合わせが成功していれば)
  if (ntpOk) {
    const int cx = W - 34, cy = 36;
    canvas.fillArc(cx, cy, 23, 27, 225, 315, TFT_WHITE);
    canvas.fillArc(cx, cy, 14, 18, 225, 315, TFT_WHITE);
    canvas.fillCircle(cx, cy - 3, 4, TFT_WHITE);
  }
}

static void drawDateTime(bool valid, uint32_t now) {
  canvas.fillRect(0, DATE_TOP, W, TIME_BOTTOM - DATE_TOP, TFT_WHITE);
  if (!valid) {
    drawCentered(FONT_DATE, "時刻未設定", W / 2, DATE_TOP, DATE_TOP + 46);
    drawCentered(FONT_TIME, "--:--", W / 2, DATE_TOP + 56, TIME_BOTTOM);
    return;
  }
  static const char* WDAY[] = {"日", "月", "火", "水", "木", "金", "土"};
  time_t tt = (time_t)now;
  struct tm tmv;
  gmtime_r(&tt, &tmv);
  char buf[40];
  snprintf(buf, sizeof(buf), "%d/%02d/%02d (%s)", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, WDAY[tmv.tm_wday]);
  drawCentered(FONT_DATE, buf, W / 2, DATE_TOP, DATE_TOP + 46);
  snprintf(buf, sizeof(buf), "%d:%02d", tmv.tm_hour, tmv.tm_min);
  drawCentered(FONT_TIME, buf, W / 2, DATE_TOP + 56, TIME_BOTTOM);
}

static void drawButton(bool pressed) {
  canvas.fillRect(BTN_X, BTN_Y, BTN_W, BTN_H, TFT_WHITE);
  frameRound(BTN_X, BTN_Y, BTN_W, BTN_H, 22, 6, pressed ? TFT_BLACK : TFT_WHITE);
  drawCentered(FONT_BUTTON, pressed ? "送信中…" : "今すぐ流す", W / 2, BTN_Y, BTN_Y + BTN_H,
               pressed ? TFT_WHITE : TFT_BLACK);
}

static String barText(bool timeValid) {
  if (signalRaw.empty()) return sdOk ? "信号ファイルがありません" : "SD が読めません";
  if (!timeValid) return "時刻未設定のため自動洗浄停止中";
  return message;
}

static void drawBox(bool valid, uint32_t now) {
  const int L = 14, R = W - 14;
  frameRound(L, BOX_Y, R - L, BOX_BOTTOM - BOX_Y, 20, 3);
  frameRound(L + 7, BOX_Y + 7, R - L - 14, BOX_BOTTOM - BOX_Y - 14, 14, 2);

  int y = BOX_Y + 18;
  drawLeft(FONT_LABEL, "◆自動洗浄時刻", 40, y, y + 36);
  y += 46;

  // 2 列。次の予定を角丸で囲む
  size_t n = std::min<size_t>(scheduleMin.size(), 8);
  int rowH = n <= 4 ? 62 : n <= 6 ? 48 : 40;
  int next = valid ? (int)((nextSlot(now) % 86400) / 60) : -1;
  const int colX[2] = {150, 390};
  for (size_t i = 0; i < n; i++) {
    int cx = colX[i % 2], top = y + (int)(i / 2) * rowH;
    char buf[8];
    snprintf(buf, sizeof(buf), "%d:%02d", scheduleMin[i] / 60, scheduleMin[i] % 60);
    if (scheduleMin[i] == next) frameRound(cx - 100, top + 3, 200, rowH - 6, 12, 3);
    drawCentered(FONT_SLOT, buf, cx, top, top + rowH);
  }
  y += (int)((n + 1) / 2) * rowH + 8;

  canvas.fillRect(40, y, W - 80, 2, TFT_BLACK);
  y += 12;
  drawLeft(FONT_LABEL, "◆最終洗浄時刻", 40, y, y + 36);
  y += 42;

  uint32_t lastT = prefs.getUInt("lastTime", 0);
  uint8_t lastK = prefs.getUChar("lastKind", KIND_NONE);
  String lastS;
  if (lastK == KIND_NONE) lastS = "なし";
  else lastS = (lastT ? fmtTime(lastT) : String("時刻不明")) + (lastK == KIND_AUTO ? " (自動)" : " (手動)");
  drawCentered(FONT_LAST, lastS, W / 2, y, y + 42);
  y += 52;

  String bar = barText(valid);
  if (bar.length()) {
    canvas.fillRoundRect(40, y, W - 80, 50, 8, TFT_BLACK);
    drawCentered(FONT_BAR, bar, W / 2, y, y + 50, TFT_WHITE);
  }
}

static void render(bool pressed = false) {
  uint32_t now; m5::rtc_datetime_t dt;
  bool valid = readNow(now, dt);
  canvas.fillScreen(TFT_WHITE);
  drawStatusBar();
  drawCentered(FONT_LOGO, "LIXIL", W / 2, 52, 84, TFT_BLACK, 8);
  drawCentered(FONT_TITLE, "流せるもん!", W / 2, 88, 176);
  drawDateTime(valid, now);
  drawButton(pressed);
  drawCentered(FONT_HINT, "★サイドの押込ボタン(G38)でも流せます", W / 2, BTN_Y + BTN_H + 6, BTN_Y + BTN_H + 40);
  drawBox(valid, now);
}

static void pushRegion(int y0, int y1, epd_mode_t mode) {
  auto& d = M5.Display;
  d.setEpdMode(mode);
  d.setClipRect(0, y0, W, y1 - y0);
  canvas.pushSprite(0, 0);
  d.clearClipRect();
  d.display();
}

// 全面をきれいに書き直す(残像も消える)
static void drawScreen() {
  render();
  pushRegion(0, H, epd_mode_t::epd_quality);
  uint32_t now; m5::rtc_datetime_t dt;
  bool valid = readNow(now, dt);
  drawnMinute = valid ? (int)(now / 60) : -1;
  drawnHour = valid ? (int)(now / 3600) : -1;
  drawnNext = valid ? nextSlot(now) : 0;
}

// 毎分: 時計と上の帯だけ。1 時間ごと・次の予定が変わったときは全面
static void updateMinute() {
  uint32_t now; m5::rtc_datetime_t dt;
  if (!readNow(now, dt)) return;
  if ((int)(now / 60) == drawnMinute) return;
  if ((int)(now / 3600) != drawnHour || nextSlot(now) != drawnNext) { drawScreen(); return; }
  render();
  pushRegion(0, BAR_H, epd_mode_t::epd_fast);
  pushRegion(DATE_TOP, TIME_BOTTOM, epd_mode_t::epd_fast);
  drawnMinute = now / 60;
}

static void showPressed() {
  render(true);
  pushRegion(BTN_Y, BTN_Y + BTN_H, epd_mode_t::epd_fast);
}

// シリアルで 'S' を受けたら画面(キャンバス)を PBM(P4)で出す。見た目の確認用
static void dumpScreen() {
  Serial.printf("P4\n%d %d\n", W, H);
  uint8_t row[(W + 7) / 8];
  for (int y = 0; y < H; y++) {
    for (int bx = 0; bx < (W + 7) / 8; bx++) {
      uint8_t b = 0;
      for (int i = 0; i < 8; i++)
        if (bx * 8 + i < W && canvas.readPixel(bx * 8 + i, y) != TFT_WHITE) b |= 0x80 >> i;
      row[bx] = b;
    }
    Serial.write(row, sizeof(row));
  }
  Serial.flush();
}

// ---- 実行 ----
static void doFlush(Kind kind, uint32_t now, bool timeValid) {
  if (kind == KIND_MANUAL) showPressed();
  bool ok = sendFlush();
  String when = timeValid ? fmtTime(now) : String("時刻不明");
  if (ok) {
    prefs.putUInt("total", prefs.getUInt("total", 0) + 1);  // 画面には出さないが記録は続ける
    prefs.putUInt("lastTime", timeValid ? now : 0);
    prefs.putUChar("lastKind", kind);
    if (kind == KIND_AUTO) prefs.putUInt("lastAuto", now);
    message = kind == KIND_AUTO ? "AUTO FLUSH OK" : "MANUAL FLUSH OK";
  } else {
    message = kind == KIND_AUTO ? "AUTO FLUSH 失敗" : "MANUAL FLUSH 失敗";
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
  canvas.setColorDepth(1);
  canvas.setPsram(true);
  canvas.createSprite(W, H);

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

  if (Serial.available() && Serial.read() == 'S') dumpScreen();

  static uint32_t lastTick = 0;
  if (millis() - lastTick >= 1000) {
    lastTick = millis();
    checkSchedule();
    updateMinute();
    if (millis() - lastNtpMs >= NTP_INTERVAL_MS) {
      syncNtp();
      drawScreen();
    }
  }
  delay(10);
}
