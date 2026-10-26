/*
 * M5Stack Core2 用 DG-Lab Coyote 3.0 BLEコントローラー
 * ------------------------------------------------------------
 * 必要ライブラリ（Arduino IDE のライブラリマネージャで入れる）:
 *   - M5Unified        (m5stack/M5Unified, 0.2.x)
 *   - NimBLE-Arduino   (h2zero/NimBLE-Arduino, 2.x 系。1.x では動かない)
 * ボード: ESP32 Arduino core の「M5Core2」
 *
 * プロトコル参照: dungeonlab-open/dglab-bluetooth-protocol (coyote/v3, 非商用公開仕様)
 *
 * 機能:
 *  - BLEスキャン→自動接続 (デバイス名 "47L121000")、切断後は自動で再スキャン
 *  - 接続直後に0xBFでソフトリミットを送信し、強度を0に揃えてから操作開始
 *  - タッチスライダーでチャンネルA/B強度を操作（上げるときは段階的に上げる）
 *  - 波形プリセット(呼吸/潮/パルス低中高/タップ)をタッチで切替
 *  - 100ms周期でB0コマンドを送信。強度変更は seq 付きで送り、B1 の応答を待つ
 *  - 本体ダイヤル操作（B1 の seq=0）を画面に反映
 *  - 物理ボタンC / BLE切断時の緊急停止(強度0)
 *  - バッテリー残量表示
 * ------------------------------------------------------------
 * 免責: 出力強度には個人差があります。ソフトリミットは必ず安全な値に
 *       設定し、初回は低強度から確認してください。
 */

#include <M5Unified.h>
#include <NimBLEDevice.h>

// ===================== BLE UUID定義 =====================
static const char* TARGET_NAME   = "47L121000";
static NimBLEUUID SERVICE_MAIN  ("0000180C-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHAR_WRITE    ("0000150A-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHAR_NOTIFY   ("0000150B-0000-1000-8000-00805f9b34fb");
static NimBLEUUID SERVICE_BATT  ("0000180A-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHAR_BATTERY  ("00001500-0000-1000-8000-00805f9b34fb");

// ===================== 状態管理 =====================
enum ConnState { ST_IDLE, ST_SCANNING, ST_CONNECTING, ST_CONNECTED, ST_ERROR };
ConnState connState = ST_IDLE;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pWriteChar = nullptr;

// BLEコールバック（NimBLEのタスク）→ loop() への受け渡し用
volatile bool foundDevice = false;
NimBLEAddress foundAddr;
volatile bool disconnected = false;
volatile bool b1Received = false;
volatile uint8_t b1Seq = 0, b1A = 0, b1B = 0;
volatile uint8_t battLevel = 0;
volatile bool battChanged = false;

// ソフトリミット（接続時にBFで送る安全上限。必ず自分の許容範囲に調整すること）
uint8_t softLimitA = 60;
uint8_t softLimitB = 60;
// 周波数バランス / パルス幅バランス（Web版と同じ値。公式推奨値は未確認）
const uint8_t FREQ_BALANCE  = 160;
const uint8_t PULSE_BALANCE = 0;

// 強度を上げるときの1回あたりの上限（下げるときは一気に下げる）
const uint8_t MAX_STEP_UP = 5;
// B1 の応答を待つ時間。過ぎたら再送する
const uint32_t B1_TIMEOUT_MS = 600;
const uint8_t STRENGTH_UNKNOWN = 0xFF;

uint8_t targetA = 0, targetB = 0;   // UIで設定する目標強度（0〜softLimit）
uint8_t actualA = STRENGTH_UNKNOWN; // B1 で通知された本体の実強度
uint8_t actualB = STRENGTH_UNKNOWN;
uint8_t seqNo = 0;                  // 直近に使った seq (1〜15)
uint8_t pendingSeq = 0;             // 応答待ちの seq（0なら待ちなし）
uint32_t pendingAt = 0;

bool needRedraw = true;
bool needSliderRedraw = false;

void drawAllNow();

// ===================== 波形プリセット =====================
enum WaveShape { SHAPE_SINE, SHAPE_SQUARE };
struct PresetDef {
  const char* name;
  WaveShape shape;
  float freqMinHz, freqMaxHz;   // 周波数の変動レンジ
  uint32_t freqCycleMs;         // 周波数変動の周期
  float strMinPct, strMaxPct;   // 強度エンベロープ(0-100)のレンジ
  uint32_t strCycleMs;          // 強度変動の周期（矩形波ならON/OFF周期）
};

const PresetDef PRESETS[] = {
  {"Breath",    SHAPE_SINE,   5,  20, 4000,  20, 80, 4000},
  {"Tide",      SHAPE_SINE,   3,  15, 6000,  10, 90, 3000},
  {"PulseLow",  SHAPE_SQUARE, 2,   2,    1,  10, 45,  800},
  {"PulseMid",  SHAPE_SQUARE, 8,   8,    1,  10, 55,  500},
  {"PulseHigh", SHAPE_SQUARE, 20, 20,    1,  10, 65,  250},
  {"Tap",       SHAPE_SQUARE, 30, 30,    1,   0, 70,  120},
};
const int PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);
int currentPreset = 0;

// ===================== 周波数(ms周期)→V3生値 変換 =====================
// 公開仕様の非線形マッピング（1〜10Hzの例から逆算した区分式）
uint8_t periodMsToRaw(uint32_t periodMs) {
  if (periodMs < 10) periodMs = 10;
  if (periodMs > 1000) periodMs = 1000;
  int raw;
  if (periodMs <= 100) {
    raw = periodMs;
  } else if (periodMs <= 600) {
    raw = (periodMs - 100) / 5 + 100;
  } else {
    raw = (periodMs - 600) / 10 + 200;
  }
  if (raw < 10) raw = 10;
  if (raw > 240) raw = 240;
  return (uint8_t)raw;
}

// ===================== 波形サンプル生成 =====================
// 指定プリセット・指定時刻(ms)における (周波数raw, 強度0-100) を計算
void samplePreset(const PresetDef& p, uint32_t tMs, uint8_t& freqRaw, uint8_t& strPct) {
  // 周波数
  float freqHz;
  if (p.shape == SHAPE_SINE) {
    float phase = (float)(tMs % p.freqCycleMs) / p.freqCycleMs;
    float s = (sinf(phase * 2.0f * PI) + 1.0f) / 2.0f; // 0..1
    freqHz = p.freqMinHz + s * (p.freqMaxHz - p.freqMinHz);
  } else {
    freqHz = p.freqMaxHz; // 矩形波系は周波数固定
  }
  if (freqHz < 0.5f) freqHz = 0.5f;
  uint32_t periodMs = (uint32_t)(1000.0f / freqHz);
  freqRaw = periodMsToRaw(periodMs);

  // 強度エンベロープ
  float pct;
  if (p.shape == SHAPE_SINE) {
    float phase = (float)(tMs % p.strCycleMs) / p.strCycleMs;
    float s = (sinf(phase * 2.0f * PI) + 1.0f) / 2.0f;
    pct = p.strMinPct + s * (p.strMaxPct - p.strMinPct);
  } else {
    uint32_t half = p.strCycleMs / 2;
    bool on = (tMs % p.strCycleMs) < half;
    pct = on ? p.strMaxPct : p.strMinPct;
  }
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  strPct = (uint8_t)pct;
}

// ===================== BLE送信 =====================
void sendBF() {
  if (!pWriteChar) return;
  uint8_t buf[7];
  buf[0] = 0xBF;
  buf[1] = softLimitA;
  buf[2] = softLimitB;
  buf[3] = FREQ_BALANCE;  buf[4] = FREQ_BALANCE;
  buf[5] = PULSE_BALANCE; buf[6] = PULSE_BALANCE;
  pWriteChar->writeValue(buf, 7, false);
}

// mode: 上位2bit=A, 下位2bit=B（00変更なし / 11絶対値）
// silent=true のときは波形の強度を全て0にする（緊急停止用）
void sendB0(uint8_t seq, uint8_t mode, uint8_t valA, uint8_t valB, uint32_t tMs, bool silent) {
  if (!pWriteChar) return;
  const PresetDef& p = PRESETS[currentPreset];
  uint8_t buf[20];
  buf[0] = 0xB0;
  buf[1] = (uint8_t)((seq << 4) | (mode & 0x0F));
  buf[2] = valA;
  buf[3] = valB;
  for (int i = 0; i < 4; i++) {
    uint8_t fr, sp;
    samplePreset(p, tMs + i * 25, fr, sp);
    if (silent) sp = 0;
    // 波形強度(0-100)はプリセットのエンベロープ、実際の強度は valA/valB（本体側の強度設定）
    buf[4 + i]  = fr; buf[8 + i]  = sp;   // A: 周波数4 / 強度4
    buf[12 + i] = fr; buf[16 + i] = sp;   // B: 周波数4 / 強度4
  }
  pWriteChar->writeValue(buf, 20, false);
}

uint8_t nextSeq() {
  seqNo = seqNo % 15 + 1;
  return seqNo;
}

// 上げるときは MAX_STEP_UP ずつ、下げるときは目標値まで一気に。
// 実強度が不明なときは0とみなす（いきなり高い値を送らない）
uint8_t stepToward(uint8_t actual, uint8_t target) {
  uint8_t base = (actual == STRENGTH_UNKNOWN) ? 0 : actual;
  if (target <= base) return target;
  uint16_t v = base + MAX_STEP_UP;
  return v < target ? (uint8_t)v : target;
}

void emergencyStop() {
  targetA = 0;
  targetB = 0;
  if (connState == ST_CONNECTED && pWriteChar) {
    uint8_t seq = nextSeq();
    pendingSeq = seq;
    pendingAt = millis();
    for (int i = 0; i < 2; i++) sendB0(seq, 0b1111, 0, 0, millis(), true);
  }
  needSliderRedraw = true;
}

// ===================== BLEコールバック =====================
// ここでは重い処理をせず、フラグを立てて loop() に任せる
void onNotify(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
  if (len >= 4 && data[0] == 0xB1) {
    b1Seq = data[1];
    b1A = data[2];
    b1B = data[3];
    b1Received = true;
  }
}

void onBattery(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
  if (len >= 1) {
    battLevel = data[0];
    battChanged = true;
  }
}

class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* c, int reason) override {
    disconnected = true;
  }
};
ClientCB clientCallbacks;

class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (foundDevice) return;
    if (dev->haveName() && dev->getName() == TARGET_NAME) {
      foundAddr = dev->getAddress();
      foundDevice = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};
ScanCB scanCallbacks;

void startScan() {
  foundDevice = false;
  connState = ST_SCANNING;
  needRedraw = true;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);
  scan->start(0 /* 無期限、見つかり次第 stop */, false);
}

// 切断・失敗時の後始末。出力は安全側（強度0）に戻す
void resetConnection() {
  pWriteChar = nullptr;
  targetA = targetB = 0;
  actualA = actualB = STRENGTH_UNKNOWN;
  pendingSeq = 0;
  if (pClient) {
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
  }
}

// loop() から呼ぶ。接続処理はブロッキング（最大 約5秒）
bool connectToDevice() {
  connState = ST_CONNECTING;
  drawAllNow();

  disconnected = false;
  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(&clientCallbacks, false);
  pClient->setConnectTimeout(5000);
  if (!pClient->connect(foundAddr)) return false;

  NimBLERemoteService* svcMain = pClient->getService(SERVICE_MAIN);
  if (!svcMain) return false;
  pWriteChar = svcMain->getCharacteristic(CHAR_WRITE);
  NimBLERemoteCharacteristic* notifyChar = svcMain->getCharacteristic(CHAR_NOTIFY);
  if (!pWriteChar || !notifyChar) return false;
  if (!notifyChar->canNotify() || !notifyChar->subscribe(true, onNotify)) return false;

  NimBLERemoteService* svcBatt = pClient->getService(SERVICE_BATT);
  if (svcBatt) {
    NimBLERemoteCharacteristic* battChar = svcBatt->getCharacteristic(CHAR_BATTERY);
    if (battChar) {
      if (battChar->canRead()) {
        NimBLEAttValue v = battChar->readValue();
        if (v.length() >= 1) battLevel = v.data()[0];
      }
      if (battChar->canNotify()) battChar->subscribe(true, onBattery);
    }
  }

  // 接続直後にソフトリミットを必ず送信し、強度0から開始する
  sendBF();
  targetA = targetB = 0;
  actualA = actualB = STRENGTH_UNKNOWN;   // 不明扱いにして、最初のB0で0を明示的に送る
  pendingSeq = 0;
  return true;
}

// ===================== UI描画 =====================
#define COL_BG      TFT_BLACK
#define COL_TEXT    TFT_WHITE
#define COL_A       TFT_CYAN
#define COL_B       TFT_MAGENTA
#define COL_PRESET  TFT_ORANGE
#define COL_STOP    TFT_RED

const int SLIDER_TOP = 40;
const int SLIDER_BOTTOM = 190;
const int SLIDER_A_X = 40;
const int SLIDER_B_X = 120;
const int SLIDER_W = 40;

const int PRESET_X0 = 190;
const int PRESET_Y0 = 40;
const int PRESET_W = 120;
const int PRESET_H = 24;
const int PRESET_GAP = 4;

void drawStatusBar() {
  M5.Display.fillRect(0, 0, 320, 30, TFT_NAVY);
  M5.Display.setTextColor(COL_TEXT, TFT_NAVY);
  M5.Display.setCursor(4, 7);
  switch (connState) {
    case ST_IDLE:       M5.Display.print("待機中"); break;
    case ST_SCANNING:   M5.Display.print("スキャン中..."); break;
    case ST_CONNECTING: M5.Display.print("接続中..."); break;
    case ST_CONNECTED:  M5.Display.print("接続済み"); break;
    case ST_ERROR:      M5.Display.print("接続エラー"); break;
  }
  M5.Display.setCursor(230, 7);
  M5.Display.printf("電池:%d%%", battLevel);
}

void drawSlider(int x, uint8_t val, uint8_t actual, uint8_t limit, uint16_t color, const char* label) {
  M5.Display.fillRect(x, SLIDER_TOP, SLIDER_W, SLIDER_BOTTOM - SLIDER_TOP, COL_BG);
  M5.Display.drawRect(x, SLIDER_TOP, SLIDER_W, SLIDER_BOTTOM - SLIDER_TOP, TFT_WHITE);
  int fillH = map(val, 0, 200, 0, SLIDER_BOTTOM - SLIDER_TOP);
  M5.Display.fillRect(x + 2, SLIDER_BOTTOM - fillH, SLIDER_W - 4, fillH, color);
  // 上限ライン
  int limY = SLIDER_BOTTOM - map(limit, 0, 200, 0, SLIDER_BOTTOM - SLIDER_TOP);
  M5.Display.drawFastHLine(x, limY, SLIDER_W, TFT_YELLOW);
  // 目標値 / 実強度（不明なら "-"）
  M5.Display.fillRect(x - 20, SLIDER_BOTTOM + 2, SLIDER_W + 40, 18, COL_BG);
  M5.Display.setTextColor(COL_TEXT, COL_BG);
  M5.Display.setCursor(x - 16, SLIDER_BOTTOM + 3);
  if (actual == STRENGTH_UNKNOWN) M5.Display.printf("%s:%d/-", label, val);
  else                            M5.Display.printf("%s:%d/%d", label, val, actual);
}

void drawSliders() {
  drawSlider(SLIDER_A_X, targetA, actualA, softLimitA, COL_A, "A");
  drawSlider(SLIDER_B_X, targetB, actualB, softLimitB, COL_B, "B");
}

void drawPresetButtons() {
  for (int i = 0; i < PRESET_COUNT; i++) {
    int y = PRESET_Y0 + i * (PRESET_H + PRESET_GAP);
    uint16_t bg = (i == currentPreset) ? COL_PRESET : TFT_DARKGREY;
    M5.Display.fillRoundRect(PRESET_X0, y, PRESET_W, PRESET_H, 4, bg);
    M5.Display.setTextColor(TFT_BLACK, bg);
    M5.Display.setCursor(PRESET_X0 + 8, y + 4);
    M5.Display.print(PRESETS[i].name);
  }
}

void drawStopButtonHint() {
  M5.Display.setTextColor(COL_STOP, COL_BG);
  M5.Display.setCursor(220, 220);
  M5.Display.print("[C] 緊急停止");
}

void drawAllNow() {
  M5.Display.startWrite();
  M5.Display.fillScreen(COL_BG);
  drawStatusBar();
  drawSliders();
  drawPresetButtons();
  drawStopButtonHint();
  M5.Display.endWrite();
  needRedraw = false;
  needSliderRedraw = false;
}

// ===================== タッチ処理 =====================
void handleTouch() {
  if (M5.Touch.getCount() == 0) return;
  auto t = M5.Touch.getDetail();
  if (!t.wasPressed() && !t.isPressed()) return;

  // スライダーA
  if (t.x >= SLIDER_A_X && t.x <= SLIDER_A_X + SLIDER_W &&
      t.y >= SLIDER_TOP && t.y <= SLIDER_BOTTOM) {
    int v = map(t.y, SLIDER_BOTTOM, SLIDER_TOP, 0, 200);
    v = constrain(v, 0, softLimitA);
    if (v != targetA) { targetA = v; needSliderRedraw = true; }
    return;
  }
  // スライダーB
  if (t.x >= SLIDER_B_X && t.x <= SLIDER_B_X + SLIDER_W &&
      t.y >= SLIDER_TOP && t.y <= SLIDER_BOTTOM) {
    int v = map(t.y, SLIDER_BOTTOM, SLIDER_TOP, 0, 200);
    v = constrain(v, 0, softLimitB);
    if (v != targetB) { targetB = v; needSliderRedraw = true; }
    return;
  }
  // プリセットボタン（押した瞬間のみ反応）
  if (t.wasPressed() && t.x >= PRESET_X0 && t.x <= PRESET_X0 + PRESET_W) {
    for (int i = 0; i < PRESET_COUNT; i++) {
      int y = PRESET_Y0 + i * (PRESET_H + PRESET_GAP);
      if (t.y >= y && t.y <= y + PRESET_H) {
        currentPreset = i;
        needRedraw = true;
        return;
      }
    }
  }
}

// ===================== B1 受信処理 =====================
void handleB1() {
  if (!b1Received) return;
  b1Received = false;
  uint8_t seq = b1Seq, a = b1A, b = b1B;
  actualA = a;
  actualB = b;
  if (seq != 0 && seq == pendingSeq) pendingSeq = 0;
  if (seq == 0) {
    // 本体のダイヤル操作など：目標を実機に合わせる（ソフトリミットは超えない）
    targetA = a < softLimitA ? a : softLimitA;
    targetB = b < softLimitB ? b : softLimitB;
  }
  needSliderRedraw = true;
}

// ===================== 100ms周期の送信 =====================
uint32_t lastSendMs = 0;
uint32_t t0 = 0;

void tick(uint32_t now) {
  if (pendingSeq && now - pendingAt > B1_TIMEOUT_MS) {
    pendingSeq = 0;   // 応答が来なかった。差分が残っていれば下で再送される
  }
  uint8_t seq = 0, mode = 0, vA = 0, vB = 0;
  if (!pendingSeq && (targetA != actualA || targetB != actualB)) {
    seq = nextSeq();
    if (targetA != actualA) { mode |= 0b1100; vA = stepToward(actualA, targetA); }
    if (targetB != actualB) { mode |= 0b0011; vB = stepToward(actualB, targetB); }
    pendingSeq = seq;
    pendingAt = now;
  }
  sendB0(seq, mode, vA, vB, now - t0, false);
}

// ===================== setup / loop =====================
uint32_t retryAt = 0;

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setFont(&fonts::lgfxJapanGothic_16);
  M5.Display.setTextSize(1);
  drawAllNow();

  NimBLEDevice::init("");
  NimBLEDevice::setPower(9);  // dBm

  startScan();
}

void loop() {
  M5.update();

  // 物理ボタンCで緊急停止（接続状態に関係なく常に受け付ける）
  if (M5.BtnC.wasPressed()) {
    emergencyStop();
  }

  if (battChanged) { battChanged = false; needRedraw = true; }

  // 切断（コールバックから通知）
  if (disconnected && connState == ST_CONNECTED) {
    disconnected = false;
    resetConnection();
    connState = ST_IDLE;
    retryAt = millis() + 1000;
    needRedraw = true;
  }

  switch (connState) {
    case ST_SCANNING:
      if (foundDevice) {
        if (connectToDevice()) {
          connState = ST_CONNECTED;
          t0 = millis();
          lastSendMs = 0;
        } else {
          resetConnection();
          connState = ST_ERROR;
          retryAt = millis() + 3000;
        }
        needRedraw = true;
      }
      break;

    case ST_CONNECTED: {
      handleB1();
      handleTouch();
      uint32_t now = millis();
      if (now - lastSendMs >= 100) {
        lastSendMs = now;
        tick(now);
      }
      break;
    }

    case ST_IDLE:
    case ST_ERROR:
      if ((int32_t)(millis() - retryAt) >= 0) startScan();
      break;

    default:
      break;
  }

  if (needRedraw) drawAllNow();
  else if (needSliderRedraw) { drawSliders(); needSliderRedraw = false; }

  delay(10);
}
