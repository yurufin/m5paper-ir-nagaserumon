// ir-register: IR リモコン信号を受信して記録する(M5Paper + IR Unit U002 on PORT.B)
//
//   受信は GPIO 割り込みで復調後の信号のエッジ時刻を取り、マーク/スペースの長さ(µs)の並びにする。
//   ※ IRremoteESP8266 は Arduino コア 3.x でタイマー API が合わずビルドできないため使わない。
//     流せるもんは生データをそのまま送り返す前提なので、プロトコル解析は不要。
//
//   1 回の受信 = 最初の立ち下がりから END_GAP_US 無信号になるまで(リピートフレームも全部含む)。
//   受信するたびに:
//     - シリアルに C++ 配列(uint16_t rawData[] = {...})を出す
//     - SD の /ir/capNNN.txt に保存
//     - 画面に概要と、直前の受信と一致したかを出す(3 回押して一致を確かめる運用)
//   押込ボタン(G38): 最後に受信した信号を「採用」して /ir/flush.txt に書く(ir-timer が読む)
#include <M5Unified.h>
#include <SD.h>
#include <SPI.h>
#include <esp_timer.h>
#include <vector>

// ---- ハード構成 ----
static constexpr int EXT_PWR_PIN = 5;       // Grove 5V 給電。HIGH にしないと U002 が動かない
static constexpr int PORTB_PIN_33 = 33;
static constexpr int PORTB_PIN_26 = 26;
// U002 の受信側/送信側が 26/33 のどちらに来るかは起動時に判定する(受信モジュールは待機中 HIGH を出す)

static constexpr int SD_CS = 4, SD_SCK = 14, SD_MISO = 13, SD_MOSI = 12;

// ---- 受信パラメータ ----
static constexpr size_t MAX_EDGES = 2048;           // 1 回の受信で記録できるエッジ数
static constexpr uint32_t END_GAP_US = 150 * 1000;  // これだけ無信号なら受信終了
static constexpr uint32_t FRAME_GAP_US = 10 * 1000; // これより長いスペースはフレームの区切りとみなす(表示用)
static constexpr size_t MIN_DURATIONS = 10;         // これより短いものはノイズとして捨てる
static constexpr int MATCH_TOL_PCT = 25;            // 一致判定の許容誤差
static constexpr uint32_t MATCH_TOL_MIN_US = 150;

static const char* IR_DIR = "/ir";
static const char* ADOPT_PATH = "/ir/flush.txt";

// ---- 割り込みで書くもの ----
static volatile uint32_t edgeTimes[MAX_EDGES];
static volatile uint16_t edgeCount = 0;
static volatile uint32_t lastEdgeUs = 0;
static volatile bool overflowed = false;
static volatile bool armed = false;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static int rxPin = PORTB_PIN_33;
static int txPin = PORTB_PIN_26;
static String pinNote;

static void IRAM_ATTR onEdge() {
  if (!armed) return;
  uint32_t now = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL_ISR(&mux);
  if (edgeCount < MAX_EDGES) {
    edgeTimes[edgeCount++] = now;
  } else {
    overflowed = true;
  }
  lastEdgeUs = now;
  portEXIT_CRITICAL_ISR(&mux);
}

// ---- 状態 ----
struct Capture {
  int no = 0;                     // 保存番号(SD が無ければ通し番号)
  std::vector<uint16_t> raw;      // マーク, スペース, マーク, ... (µs)
  bool overflow = false;
  uint32_t totalUs = 0;
  int frames = 0;
};

static bool sdOk = false;
static int nextNo = 1;
static Capture last, prev;
static bool hasLast = false, hasPrev = false;
static String adoptMsg;

// ---- 受信ピンの判定 ----
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
  bool hi33 = h33 >= 15, hi26 = h26 >= 15;
  if (hi33 && !hi26) {
    rxPin = PORTB_PIN_33; txPin = PORTB_PIN_26; pinNote = "自動判定";
  } else if (hi26 && !hi33) {
    rxPin = PORTB_PIN_26; txPin = PORTB_PIN_33; pinNote = "自動判定";
  } else {
    rxPin = PORTB_PIN_33; txPin = PORTB_PIN_26;
    pinNote = String("判定不能(G33=") + h33 + "/20, G26=" + h26 + "/20)、既定値";
  }
  // 送信側は LED を消した状態で固定(浮かせない)
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);
  pinMode(rxPin, INPUT);
}

// ---- SD ----
static void initSd() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, SPI, 20000000);
  if (!sdOk) return;
  if (!SD.exists(IR_DIR)) SD.mkdir(IR_DIR);
  // 既存の capNNN.txt の続きから番号を振る
  File dir = SD.open(IR_DIR);
  if (dir) {
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      String name = f.name();
      int slash = name.lastIndexOf('/');
      if (slash >= 0) name = name.substring(slash + 1);
      if (name.startsWith("cap") && name.endsWith(".txt")) {
        int n = name.substring(3, name.length() - 4).toInt();
        if (n >= nextNo) nextNo = n + 1;
      }
      f.close();
    }
    dir.close();
  }
}

static String capPath(int no) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s/cap%03d.txt", IR_DIR, no);
  return String(buf);
}

// 形式: # で始まる行はコメント。データはカンマ区切りの µs(マークから始まり交互)
static bool writeCapture(const String& path, const Capture& c) {
  M5.Display.waitDisplay();
  if (SD.exists(path)) SD.remove(path);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  f.printf("# ir-register cap%03d\n", c.no);
  f.printf("# rx_pin=%d len=%u total_us=%lu frames=%d overflow=%d\n", rxPin, (unsigned)c.raw.size(),
           (unsigned long)c.totalUs, c.frames, c.overflow ? 1 : 0);
  for (size_t i = 0; i < c.raw.size(); i++) {
    f.print(c.raw[i]);
    f.print(i + 1 < c.raw.size() ? (((i + 1) % 16) ? "," : ",\n") : "\n");
  }
  f.close();
  return true;
}

// ---- 比較 ----
static bool closeEnough(uint16_t a, uint16_t b) {
  uint32_t diff = a > b ? a - b : b - a;
  uint32_t tol = max<uint32_t>(MATCH_TOL_MIN_US, (uint32_t)max(a, b) * MATCH_TOL_PCT / 100);
  return diff <= tol;
}

static String compare(const Capture& a, const Capture& b) {
  if (a.raw.size() != b.raw.size()) {
    return String("不一致: 長さ ") + a.raw.size() + " と " + b.raw.size();
  }
  int bad = 0, firstBad = -1;
  for (size_t i = 0; i < a.raw.size(); i++) {
    if (!closeEnough(a.raw[i], b.raw[i])) {
      if (firstBad < 0) firstBad = i;
      bad++;
    }
  }
  if (bad == 0) return "一致";
  return String("不一致: ") + bad + " 箇所(最初は [" + firstBad + "] " + a.raw[firstBad] + " と " +
         b.raw[firstBad] + ")";
}

// ---- 出力 ----
static void printSerial(const Capture& c, const String& cmp) {
  Serial.printf("\n===== cap%03d  len=%u  total=%.1fms  frames=%d%s =====\n", c.no, (unsigned)c.raw.size(),
                c.totalUs / 1000.0, c.frames, c.overflow ? "  [OVERFLOW]" : "");
  if (cmp.length()) Serial.printf("前回との比較: %s\n", cmp.c_str());
  Serial.printf("uint16_t rawData[%u] = {", (unsigned)c.raw.size());
  for (size_t i = 0; i < c.raw.size(); i++) {
    if (i % 16 == 0) Serial.print("\n  ");
    Serial.print(c.raw[i]);
    if (i + 1 < c.raw.size()) Serial.print(", ");
  }
  Serial.println("\n};");
}

static void drawScreen(const String& cmp, const String& saveMsg) {
  auto& d = M5.Display;
  d.startWrite();
  d.fillScreen(TFT_WHITE);
  d.setTextColor(TFT_BLACK, TFT_WHITE);

  d.setFont(&fonts::lgfxJapanGothic_40);
  d.drawString("IR 受信・記録", 30, 20);

  d.setFont(&fonts::lgfxJapanGothic_24);
  int y = 80;
  d.drawString(String("受信 G") + rxPin + " / 送信 G" + txPin + "(" + pinNote + ")", 30, y); y += 34;
  d.drawString(sdOk ? String("SD: OK  次の番号 cap") + nextNo : String("SD: なし(シリアル出力のみ)"), 30, y); y += 44;

  if (!hasLast) {
    d.drawString("リモコンを受光部に 5〜15cm から向けてボタンを押してください", 30, y);
  } else {
    char buf[128];
    snprintf(buf, sizeof(buf), "cap%03d  長さ %u  全長 %.1f ms  フレーム %d%s", last.no, (unsigned)last.raw.size(),
             last.totalUs / 1000.0, last.frames, last.overflow ? "  ※あふれ" : "");
    d.drawString(buf, 30, y); y += 34;
    if (hasPrev) {
      d.drawString(String("前回 cap") + (prev.no < 100 ? (prev.no < 10 ? "00" : "0") : "") + prev.no + " と " + cmp, 30, y);
    } else {
      d.drawString("前回: なし", 30, y);
    }
    y += 34;
    if (saveMsg.length()) { d.drawString(saveMsg, 30, y); }
    y += 44;

    // 先頭の値を並べる
    d.setFont(&fonts::Font2);
    const int perLine = 14;
    const int maxLines = 9;
    for (int line = 0; line < maxLines; line++) {
      size_t start = (size_t)line * perLine;
      if (start >= last.raw.size()) break;
      String s;
      for (size_t i = start; i < start + perLine && i < last.raw.size(); i++) {
        char v[10];
        snprintf(v, sizeof(v), "%6u", last.raw[i]);
        s += v;
      }
      if (line == maxLines - 1 && start + perLine < last.raw.size()) s += "  ...";
      d.drawString(s, 30, y);
      y += 20;
    }
  }

  d.setFont(&fonts::lgfxJapanGothic_24);
  if (adoptMsg.length()) d.drawString(adoptMsg, 30, 460);
  d.drawString("押込ボタン: 最後の信号を採用 → /ir/flush.txt", 30, 500);
  d.endWrite();
  d.display();
}

// ---- 受信 1 回分の処理 ----
static void handleCapture(const std::vector<uint32_t>& edges, bool ovf) {
  Capture c;
  c.overflow = ovf;
  for (size_t i = 1; i < edges.size(); i++) {
    uint32_t dur = edges[i] - edges[i - 1];
    c.raw.push_back(dur > 65535 ? 65535 : (uint16_t)dur);
    c.totalUs += dur;
    if ((i - 1) % 2 == 1 && dur >= FRAME_GAP_US) c.frames++;
  }
  c.frames++;
  if (c.raw.size() < MIN_DURATIONS) {
    Serial.printf("(ノイズとして無視: %u 個)\n", (unsigned)c.raw.size());
    return;
  }

  c.no = nextNo++;
  String cmp;
  if (hasLast) {
    prev = last;
    hasPrev = true;
    cmp = compare(c, prev);
  }
  last = c;
  hasLast = true;

  String saveMsg;
  if (sdOk) {
    String path = capPath(c.no);
    saveMsg = writeCapture(path, c) ? path + " に保存" : path + " の保存に失敗";
  }
  printSerial(c, cmp);
  if (saveMsg.length()) Serial.println(saveMsg);
  adoptMsg = "";
  drawScreen(cmp, saveMsg);
}

static void adoptLast() {
  if (!hasLast) {
    adoptMsg = "採用する信号がまだありません";
  } else if (!sdOk) {
    adoptMsg = "SD が無いので採用できません";
  } else if (writeCapture(ADOPT_PATH, last)) {
    adoptMsg = String("cap") + (last.no < 100 ? (last.no < 10 ? "00" : "0") : "") + last.no + " を " + ADOPT_PATH + " に採用しました";
  } else {
    adoptMsg = String(ADOPT_PATH) + " の書き込みに失敗";
  }
  Serial.println(adoptMsg);
  String cmp = hasPrev ? compare(last, prev) : String();
  drawScreen(cmp, "");
}

// 受信モジュールが待機状態(HIGH)のときだけ受け付けを始める。最初のエッジが必ず立ち下がり(マーク開始)になる
static void rearm() {
  portENTER_CRITICAL(&mux);
  edgeCount = 0;
  overflowed = false;
  portEXIT_CRITICAL(&mux);
  uint32_t t0 = millis();
  while (digitalRead(rxPin) == LOW && millis() - t0 < 500) delay(1);
  armed = true;
}

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);

  pinMode(EXT_PWR_PIN, OUTPUT);
  digitalWrite(EXT_PWR_PIN, HIGH);
  delay(200);  // U002 の電源安定待ち

  M5.Display.setRotation(1);  // 横向き 960x540
  M5.Display.setEpdMode(epd_mode_t::epd_fast);

  detectPins();
  initSd();
  Serial.printf("\nir-register: rx=G%d tx=G%d (%s) SD=%s\n", rxPin, txPin, pinNote.c_str(), sdOk ? "OK" : "NG");

  drawScreen("", "");

  attachInterrupt(digitalPinToInterrupt(rxPin), onEdge, CHANGE);
  rearm();
}

void loop() {
  M5.update();
  if (M5.BtnB.wasPressed()) adoptLast();

  uint16_t n;
  uint32_t lastUs;
  portENTER_CRITICAL(&mux);
  n = edgeCount;
  lastUs = lastEdgeUs;
  portEXIT_CRITICAL(&mux);

  if (n > 0 && (uint32_t)esp_timer_get_time() - lastUs > END_GAP_US) {
    armed = false;
    std::vector<uint32_t> edges(n);
    portENTER_CRITICAL(&mux);
    for (uint16_t i = 0; i < n; i++) edges[i] = edgeTimes[i];
    bool ovf = overflowed;
    portEXIT_CRITICAL(&mux);
    rearm();  // 描画中も次の受信を取れるよう、先に再開する
    handleCapture(edges, ovf);
  }
  delay(5);
}
