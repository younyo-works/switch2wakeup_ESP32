// ============================================================================
//  Switch 2 Wakeup  -  Stand-alone 版  (ESP32-C3 / NimBLE-Arduino 2.x)
//
//  ■ 通常動作
//    ディープスリープで待機 → ホームボタンで復帰 → 起動信号を送信 → 再スリープ
//    送信は最低3秒。押している間は最大30秒まで延長され、離すと終了する
//
//  ■ 設定動作（スキャナーモード）
//    マイナスボタンを押しながら「ホームで復帰」または「電源投入/リセット」
//    → スキャナーモード(15秒) → Joy-Con 2 のボタンを押す/本体から外す
//    → Joy-Con / Switch 2 のMACとPIDをNVSへ保存 → ディープスリープ
//    ※ ホームで復帰する場合は、マイナスを先に押しておくこと
//       （復帰直後にマイナスの状態を読み取るため）
//
//  ■ 操作まとめ
//    ホームのみ                   : 起動信号を送信
//    マイナスを押しながらホーム    : スキャナーモード
//    マイナスを押しながら電源投入  : スキャナーモード
//    マイナスなしで電源投入/書込み : サービス待機(5秒)。シリアルで CFG/SHOW/CLR が使える
//
//  ■ 初期値（ダミー）
//    Joy-Con MAC = AA:BB:CC:DD:EE:FF / Switch 2 MAC = 11:22:33:44:55:66
//
//  ■ LEDパターン (GPIO8)
//    1秒に1回、短く点灯          : サービス待機中（USB/シリアルが生きている）
//    高速点滅 (50ms周期)         : 起動信号を送信中
//    「ピピッ」を繰り返す         : スキャナーモード中
//    2秒間点灯                  : スキャン成功（保存完了）
//    素早く5回点滅               : スキャン失敗（15秒タイムアウト、設定は変更なし）
//
//  ■ 配線の注意（実機での測定結果より）
//    ・プロコンのホーム/マイナス線は、GNDへ直結ではなくパルス状に落ちる信号。
//      デジタル読みでは押下を判別できないため、押下判定はADCの最小値で行う。
//        未押下               : 約1.8V付近（変動 約80mV）
//        押した線             : 最小値が0Vまで落ちる
//        押していない線        : 回り込みで最小値 約1.4V付近まで（0Vにはならない）
//
//  ■ ピン割り当て
//    ホーム   : GPIO4（ディープスリープ復帰 + ADC1）
//    マイナス : GPIO3（ADC1）
//    LED      : GPIO8
//    ※ ピンを変える場合: ホームはGPIO0-5、マイナスはGPIO0-4から選ぶ。
//       GPIO2/8/9はブート設定ピンのため避ける（GPIO5はADC2で使えない）
//
//  ■ Arduino IDE 設定
//    ボード「ESP32C3 Dev Module」 / USB CDC On Boot = Enabled
//
//  ■ シリアルモニタの注意
//    ESP32-C3 の USB シリアルは接続直後すぐには開かないため、起動直後の
//    表示は接続を短時間待ってから行い、再接続時にも表示し直す。
//    ディープスリープに入ると USB 自体が切れて COM ポートが消えるため、
//    直前の送信/スキャン結果は RTC に保持し、次回サービス待機の画面に
//    Last event として再表示する。
//    モニタは Arduino IDE 2.x 推奨（1.x はポートの消滅・復活に弱い）。
// ============================================================================
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEAdvertising.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "esp_mac.h"
#include "esp_sleep.h"

// ---------------------------- ピン設定 --------------------------------------
// ピンを変えるときは、この数値だけを書き換える（選べるピンはヘッダーの「ピン割り当て」参照）
static constexpr int  PIN_HOME  = 4;   // ホームボタン（GPIO4）
static constexpr int  PIN_MINUS = 3;   // マイナスボタン（GPIO3）
static constexpr int  PIN_LED   = 8;   // ESP32-C3 Super Mini の青LED
static constexpr bool LED_ACTIVE_LOW = true;   // 点灯/消灯が逆なら false に

// ESP-IDFのgpio_*関数はgpio_num_t型を要求するため、ホームのピン番号から自動で作る
static constexpr gpio_num_t HOME_GPIO = static_cast<gpio_num_t>(PIN_HOME);

// ---------------------------- 動作時間 --------------------------------------
static constexpr uint32_t BURST_MS          = 3000;   // 起動信号の最低送信時間（単独押し）
static constexpr uint32_t MAX_BURST_MS      = 30000;  // 押しっぱなし延長の上限
static constexpr uint32_t SCAN_MODE_MS      = 15000;  // スキャナーモードの動作時間
static constexpr uint32_t SCAN_SETTLE_MS    = 1500;   // 最初の検出後、より強い電波を待つ時間
static constexpr uint32_t SERVICE_WINDOW_MS = 5000;  // 電源投入/リセット後にUSBを生かしておく時間
static constexpr uint32_t RELEASE_TIMEOUT_MS = 10000; // ボタン解放待ちの上限
static constexpr uint32_t SERIAL_WAIT_POWERON_MS = 3000; // 電源投入時: シリアル接続を待つ上限
static constexpr uint32_t SERIAL_WAIT_SCAN_MS    = 2500; // スキャン時: 同上（開始前のみ）
static constexpr uint32_t SERIAL_WAIT_WAKE_MS    = 600;  // 復帰時: 起動が遅れないよう短く
static constexpr uint32_t SERIAL_GRACE_MS        = 500;  // スリープ前に送信を見届ける時間
static constexpr bool     WAIT_RELEASE_BEFORE_SLEEP = true;  // ホームを離すまでスリープしない

// ---------------------------- スキャン条件 ----------------------------------
static constexpr int SCAN_MIN_RSSI = -75;  // これより弱い機器は無視（他人のJoy-Con誤取得防止）

// ---------------------------- ボタン判定（ADC） -----------------------------
// 「この電圧未満に落ちたサンプル」を押下中のLOWパルスとみなす
static constexpr int      HOME_LOW_MV     = 500;  // ホーム
static constexpr int      MINUS_LOW_MV    = 500;  // マイナス
static constexpr uint32_t MINUS_SAMPLE_MS = 80;   // マイナス判定のサンプリング時間
static constexpr int      MINUS_LOW_COUNT = 3;    // LOWサンプルがこの回数以上なら押下
// 解放判定：40msの無押下窓が3回続いたら「離した」（約120ms）
static constexpr uint32_t HOME_RELEASE_WINDOW_MS = 40;
static constexpr int      HOME_RELEASE_CLEAN_N   = 3;

// ---------------------------- 初期値 ----------------------------------------
static const char* DEFAULT_JC_MAC = "AA:BB:CC:DD:EE:FF";
static const char* DEFAULT_SW_MAC = "11:22:33:44:55:66";
static const char* DEFAULT_PID    = "0x2066";

Preferences preferences;

// ディープスリープを跨いでも残る記録（電源を切ると消える）。
// ライブで見逃したログを、次回のサービス待機で確認するために使う。
RTC_DATA_ATTR static char     g_lastEvent[128] = "";
RTC_DATA_ATTR static uint32_t g_wakeCount = 0;

// ※ Arduino IDEは関数プロトタイプを自動で先頭に挿入するため、
//   関数の引数・戻り値で使う型は、関数より前に定義しておく
struct Config {
    String jc;
    String sw;
    String pid;
};

// ---------------------------- 共通ヘルパー ----------------------------------
static void led(bool on) {
    digitalWrite(PIN_LED, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

static bool parseMac(const char* s, uint8_t* out) {
    if (strlen(s) != 17) return false;
    unsigned int p[6];
    if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x",
               &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]) != 6) return false;
    for (int i = 0; i < 6; i++) out[i] = (uint8_t)p[i];
    return true;
}

static void formatMac(const uint8_t* m, char* out, size_t n) {
    snprintf(out, n, "%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
}

// シリアル接続が開くまで短時間だけ待つ。戻り値: 接続できているか
// ※ standalone（PCなし）でも固まらないよう、必ずタイムアウトで抜ける
static bool waitForSerial(uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < timeoutMs)) delay(10);
    return (bool)Serial;
}

// 接続があるときだけ、キューに残った文字を送り切る
static void flushSerialIfConnected() {
    if (Serial) { Serial.flush(); delay(50); }
}

static void printLastEvent() {
    if (g_lastEvent[0]) {
        Serial.printf("    Last event: %s (wake count=%lu)\n",
                      g_lastEvent, (unsigned long)g_wakeCount);
    }
}

// 48bit値から2を引く（桁借りあり）。BLE MAC = ベースMAC + 2 なので逆算に使う
static void macMinus2(uint8_t* mac) {
    int borrow = 2;
    for (int i = 5; i >= 0 && borrow; i--) {
        int v = (int)mac[i] - borrow;
        if (v >= 0) { mac[i] = (uint8_t)v; borrow = 0; }
        else        { mac[i] = (uint8_t)(v + 256); borrow = 1; }
    }
}

// ---------------------------- 設定の保存/読込（NVS） ------------------------
static Config loadConfig() {
    Config c;
    preferences.begin("settings", false);
    c.jc  = preferences.getString("jc_mac", DEFAULT_JC_MAC);
    c.sw  = preferences.getString("sw_mac", DEFAULT_SW_MAC);
    c.pid = preferences.getString("pid",    DEFAULT_PID);
    preferences.end();
    return c;
}

static void saveConfig(const String& jc, const String& sw, const String& pid) {
    preferences.begin("settings", false);
    preferences.putString("jc_mac", jc);
    preferences.putString("sw_mac", sw);
    preferences.putString("pid",    pid);
    preferences.end();
}

static void clearConfig() {
    preferences.begin("settings", false);
    preferences.clear();
    preferences.end();
}

// ---------------------------- ボタン判定 ------------------------------------
// マイナス押下判定：サンプリング中に0付近まで落ちるサンプルが複数あれば押下
//   （未押下 最小約1750mV / ホームだけ押下 最小約1480mV / マイナス押下 最小=0）
static bool minusHeld() {
    int lowCount = 0;
    uint32_t t = millis();
    while (millis() - t < MINUS_SAMPLE_MS) {
        if (analogReadMilliVolts(PIN_MINUS) < MINUS_LOW_MV) lowCount++;
    }
    return lowCount >= MINUS_LOW_COUNT;
}

// 指定時間の間に、ホーム線が0付近まで落ちたサンプルがあるか
static bool homeLowSeen(uint32_t ms) {
    uint32_t t = millis();
    while (millis() - t < ms) {
        if (analogReadMilliVolts(PIN_HOME) < HOME_LOW_MV) return true;
    }
    return false;
}

// ホームが離されるまで待つ（押下中のデジタル読みはパルスで不安定なためADCで判定）
static void waitHomeRelease() {
    uint32_t t0 = millis();
    int clean = 0;
    while (millis() - t0 < RELEASE_TIMEOUT_MS) {
        if (!homeLowSeen(HOME_RELEASE_WINDOW_MS)) {
            if (++clean >= HOME_RELEASE_CLEAN_N) break;
        } else {
            clean = 0;
        }
    }
    pinMode(PIN_HOME, INPUT_PULLUP);   // ADC読み取り後に元のピン設定へ戻す
    delay(50);
}

// ---------------------------- ディープスリープ ------------------------------
static void goDeepSleep() {
    // スリープに入る表示は元コードには無く、しかも flush しないと
    // USB送信完了前にスリープで切れて最後の行が消えるため、ここで出す
    Serial.println(">>> Going to DEEP SLEEP. (USB serial will disconnect)");
    flushSerialIfConnected();
    if (WAIT_RELEASE_BEFORE_SLEEP) waitHomeRelease();
    led(false);
    delay(20);

    // スリープ中もホーム端子のプルアップを維持する（LOWレベル復帰のため）
    gpio_set_direction(HOME_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(HOME_GPIO);
    gpio_pulldown_dis(HOME_GPIO);
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP
    gpio_hold_en(HOME_GPIO);
    gpio_deep_sleep_hold_en();
#endif

    esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_HOME, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
    while (true) {}   // ここには来ない
}

// ---------------------------- サービス待機（シリアル設定） ------------------
// 電源投入/リセット直後だけ一定時間起きている。USBが生きているので
//   ・ブラウザからの再書込み
//   ・シリアルモニタからの CFG: / SHOW / CLR コマンド
// が使える。設定を変更したら待機を終えてスリープへ移る。
// 戻り値: true = 設定が変更された
static bool handleSerialLine(String line) {
    line.trim();
    if (line.length() == 0) return false;

    if (line.equalsIgnoreCase("SHOW")) {
        Config c = loadConfig();
        Serial.printf("JC=%s SW=%s PID=%s\n", c.jc.c_str(), c.sw.c_str(), c.pid.c_str());
        return false;
    }
    if (line.equalsIgnoreCase("CLR")) {
        clearConfig();
        Serial.println(">>> [OK] config cleared (back to dummy MAC)");
        return true;
    }
    if (line.startsWith("CFG:")) {
        // 形式: CFG:JoyConMAC,SwitchMAC,PID
        // 例  : CFG:AA:BB:CC:DD:EE:FF,11:22:33:44:55:66,0x2066
        int c1 = line.indexOf(',', 4);
        int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
        if (c1 < 0 || c2 < 0) { Serial.println(">>> [ERROR] format"); return false; }
        String jc  = line.substring(4, c1);      jc.trim();  jc.toUpperCase();
        String sw  = line.substring(c1 + 1, c2); sw.trim();  sw.toUpperCase();
        String pid = line.substring(c2 + 1);     pid.trim();
        uint8_t tmp[6];
        if (!parseMac(jc.c_str(), tmp) || !parseMac(sw.c_str(), tmp)) {
            Serial.println(">>> [ERROR] bad MAC");
            return false;
        }
        char* endp = nullptr;
        long pv = strtol(pid.c_str(), &endp, 16);
        if (endp == pid.c_str() || pv <= 0 || pv > 0xFFFF) {
            Serial.println(">>> [ERROR] bad PID");
            return false;
        }
        char pidNorm[8];
        snprintf(pidNorm, sizeof(pidNorm), "0x%04lX", pv);
        saveConfig(jc, sw, String(pidNorm));
        Serial.println(">>> [SUCCESS] CONFIG SAVED TO NVS!");
        return true;
    }
    Serial.printf("Received: [%s]\n", line.c_str());
    return false;
}

static void printServiceBanner() {
    Serial.println();
    Serial.println(">>> [SERVICE WINDOW] standby. CFG:JC,SW,PID / SHOW / CLR available.");
    Config c = loadConfig();
    Serial.printf("    JC=%s SW=%s PID=%s\n", c.jc.c_str(), c.sw.c_str(), c.pid.c_str());
    printLastEvent();
}

static void runServiceWindow() {
    // 先に接続を待たないと、下のバナーはホスト不在のまま捨てられる
    waitForSerial(SERIAL_WAIT_POWERON_MS);
    printServiceBanner();
    Serial.setTimeout(100);

    bool wasConnected = (bool)Serial;
    uint32_t t0 = millis();
    uint32_t lastBannerMs = t0;
    while (millis() - t0 < SERVICE_WINDOW_MS) {
        led((millis() % 1000) < 60);   // 1秒に1回、短く点灯
        // モニタを開き直したタイミングでバナーを出し直す（取りこぼし対策）
        // 接続判定の効き方はコア依存なので、5秒ごとの再表示も併用する
        bool nowConnected = (bool)Serial;
        if ((nowConnected && !wasConnected) || (millis() - lastBannerMs >= 5000)) {
            printServiceBanner();
            lastBannerMs = millis();
        }
        wasConnected = nowConnected;
        if (Serial.available()) {
            if (handleSerialLine(Serial.readStringUntil('\n'))) {
                delay(200);
                return;   // 設定を変更したのでスリープへ
            }
        }
        delay(10);
    }
}

// ---------------------------- 通常動作：起動信号の送信 ----------------------
static void runTransmitter() {
    Config c = loadConfig();
    // 送信前のシリアル待ちはしない（送信タイミングに影響させないため）。
    // 表示はすべて送信後にまとめて行い、結果は RTC の Last event にも残す。
    const bool dummyCfg = (c.jc == DEFAULT_JC_MAC || c.sw == DEFAULT_SW_MAC);

    uint16_t sourcePid = (uint16_t)strtol(c.pid.c_str(), NULL, 16);

    // 送信元MAC（BLEアドレス = ベースMAC + 2 なので、-2 して設定する）
    uint8_t srcMac[6];
    if (parseMac(c.jc.c_str(), srcMac)) {
        macMinus2(srcMac);
        esp_base_mac_addr_set(srcMac);
    }

    // 宛先（Switch 2）のMACは逆順でペイロードへ入れる
    uint8_t swMac[6] = {0};
    if (!parseMac(c.sw.c_str(), swMac)) Serial.println("SW MAC parse error");

    uint8_t raw_payload[31] = {
        0x02, 0x01, 0x06,                    // Flags
        0x1B, 0xFF,                          // 製造元データ(長さ27)
        0x53, 0x05,                          // Nintendo
        0x01, 0x00, 0x03,
        0x7E, 0x05,
        0x00, 0x00,                          // [12][13] PID（下で設定）
        0x00, 0x01, 0x81,
        0, 0, 0, 0, 0, 0,                    // [17..22] 宛先MAC（下で設定、逆順）
        0x0F, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00
    };
    raw_payload[12] = sourcePid & 0xFF;
    raw_payload[13] = (sourcePid >> 8) & 0xFF;
    for (int i = 0; i < 6; i++) raw_payload[17 + i] = swMac[5 - i];

    NimBLEDevice::init("");
    // 広告の送信電力を最大にする（既定は +3dBm 程度。受信側の余裕を稼ぐ）
    NimBLEDevice::setPowerLevel(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_ADV);
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

    NimBLEAdvertisementData advData;
    advData.addData(raw_payload, sizeof(raw_payload));
    adv->setAdvertisementData(advData);

    adv->setConnectableMode(BLE_GAP_CONN_MODE_NON);
    adv->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
    adv->setMinInterval(0x20);
    adv->setMaxInterval(0x20);   // 間隔は最短の20ms固定（窓に当たる確率を上げる）
    // 【ハイブリッド仕様】最低 BURST_MS は必ず送る（単独押し）。
    // その後はホームが押されている間だけ延長し、離されたら終える
    // （上限 MAX_BURST_MS）。
    // 送信は止めずに流し続ける。1秒ごとの開始し直しは送信の
    // 空白を作るためやめ、開始が落ちた時だけ立て直す方式にした。
    bool advOk = false;
    uint32_t burstStart = millis();
    uint32_t sentMs = 0;
    bool advRunning = false;

    auto startAdv = [&]() -> bool {
        for (int attempt = 0; attempt < 10; attempt++) {
            adv->setAdvertisementData(advData);
            if (adv->start() || adv->isAdvertising()) {
                uint32_t tw = millis();
                while (!adv->isAdvertising() && (millis() - tw < 300)) delay(10);
                if (adv->isAdvertising()) return true;
            }
            delay(150);
        }
        return false;
    };

    advRunning = startAdv();
    if (advRunning) advOk = true;
    while (true) {
        if ((millis() - burstStart) >= MAX_BURST_MS) break;
        // 送信が落ちていたら立て直す
        if (!advRunning || !adv->isAdvertising()) {
            if (advRunning) { adv->stop(); advRunning = false; }
            advRunning = startAdv();
            if (advRunning) advOk = true;
        }
        // LEDを点滅させながら少し待つ（この間も送信は継続している）
        uint32_t sliceStart = millis();
        while (millis() - sliceStart < 200) {
            led(((millis() / 50) % 2) == 0);
            delay(10);
        }
        if (advRunning) sentMs += millis() - sliceStart;
        // 最低時間を過ぎたら、押されている間だけ続ける
        if ((millis() - burstStart) >= BURST_MS) {
            uint32_t probeStart = millis();
            bool held = homeLowSeen(150);
            if (advRunning) sentMs += millis() - probeStart;
            if (!held) break;
        }
    }
    if (advRunning) adv->stop();
    led(false);
    g_wakeCount++;
    if (advOk) {
        snprintf(g_lastEvent, sizeof(g_lastEvent),
                 "Wake burst sent to SW=%s (JC=%s, PID=%s, %lu ms)%s",
                 c.sw.c_str(), c.jc.c_str(), c.pid.c_str(), (unsigned long)sentMs,
                 dummyCfg ? " DUMMY!" : "");
    } else {
        snprintf(g_lastEvent, sizeof(g_lastEvent),
                 "Wake FAILED: advertising did not start (SW=%s)%s",
                 c.sw.c_str(), dummyCfg ? " DUMMY!" : "");
    }
    // 送信が終わってからまとめて表示する（送信タイミングには影響しない）
    waitForSerial(SERIAL_WAIT_WAKE_MS);
    Serial.println();
    Serial.printf(">>> [WAKE] JC=%s SW=%s PID=%s\n", c.jc.c_str(), c.sw.c_str(), c.pid.c_str());
    if (!advOk) Serial.println(">>> [WAKE] WARNING: advertising did not start");
    if (dummyCfg) Serial.println(">>> [WAKE] WARNING: DUMMY CONFIG!");
    Serial.println(">>> Wake burst complete. Going back to DEEP SLEEP.");
    // 最後の行を送り切ってからスリープへ（flushなしだとほぼ消える）
    flushSerialIfConnected();
    if (Serial) delay(SERIAL_GRACE_MS);
}

// ---------------------------- スキャナーモード ------------------------------
struct Found {
    bool     valid;
    uint8_t  jc[6];
    uint8_t  sw[6];
    uint16_t pid;
    int      rssi;
};

static Found        g_best = {};
static uint32_t     g_firstHitMs = 0;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Nintendoの起動信号と同じ形式（Flags + 製造元データ）かを確認
static bool isNintendoWakeFormat(const uint8_t* d, size_t len) {
    if (len < 23) return false;
    if (d[0] != 0x02 || d[1] != 0x01 || d[2] != 0x06) return false;
    if (d[3] != 0x1B || d[4] != 0xFF || d[5] != 0x53 || d[6] != 0x05) return false;
    if (d[7] != 0x01 || d[8] != 0x00 || d[9] != 0x03 || d[10] != 0x7E || d[11] != 0x05) return false;
    return true;
}

class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* device) override {
        const std::vector<uint8_t>& payload = device->getPayload();
        const uint8_t* d = payload.data();
        const size_t len = payload.size();

        if (!isNintendoWakeFormat(d, len)) return;
        const int rssi = device->getRSSI();
        if (rssi < SCAN_MIN_RSSI) return;

        std::string addr = device->getAddress().toString();
        uint8_t jc[6];
        if (!parseMac(addr.c_str(), jc)) return;

        Found f;
        f.valid = true;
        memcpy(f.jc, jc, 6);
        for (int i = 0; i < 6; i++) f.sw[i] = d[22 - i];   // バイト17-22は宛先MACの逆順
        f.pid  = (uint16_t)d[12] | ((uint16_t)d[13] << 8);
        f.rssi = rssi;

        portENTER_CRITICAL(&g_mux);
        if (!g_best.valid) g_firstHitMs = millis();
        if (!g_best.valid || rssi > g_best.rssi) g_best = f;   // 一番近い（強い）ものを採用
        portEXIT_CRITICAL(&g_mux);
    }
};
static ScanCallbacks g_scanCallbacks;

static void runScannerMode() {
    // 15秒カウントを始める前に接続を待つ（説明文が捨てられないように）
    waitForSerial(SERIAL_WAIT_SCAN_MS);
    Serial.println();
    Serial.println(">>> [SCANNER MODE] 15s. Press a button on the Joy-Con 2 (or detach it).");
    Serial.println("    Found Joy-Con 2 will be saved to NVS automatically.");

    g_best = {};
    g_firstHitMs = 0;

    NimBLEDevice::init("");
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&g_scanCallbacks, false);
    scan->setActiveScan(true);
    scan->setInterval(96);
    scan->setWindow(96);
    scan->setMaxResults(0);
    if (!scan->start(0, false, true)) {
        Serial.println("ERROR: Failed to start BLE scan");
    }

    bool committed = false;
    Found result = {};
    bool wasConnected = (bool)Serial;
    uint32_t t0 = millis();
    uint32_t lastStatusMs = t0;
    while (millis() - t0 < SCAN_MODE_MS) {
        uint32_t ph = millis() % 1000;
        led(ph < 100 || (ph >= 200 && ph < 300));    // 「ピピッ」
        bool nowConnected = (bool)Serial;
        if ((nowConnected && !wasConnected) || (millis() - lastStatusMs >= 5000)) {
            Serial.printf(">>> [SCANNER MODE] still scanning... %lus left. Press a button on the Joy-Con 2.\n",
                          (unsigned long)((SCAN_MODE_MS - (millis() - t0)) / 1000));
            lastStatusMs = millis();
        }
        wasConnected = nowConnected;

        portENTER_CRITICAL(&g_mux);
        bool have = g_best.valid;
        uint32_t first = g_firstHitMs;
        Found snap = g_best;
        portEXIT_CRITICAL(&g_mux);

        if (have && (millis() - first) >= SCAN_SETTLE_MS) {
            result = snap;
            committed = true;
            break;
        }
        delay(10);
    }
    scan->stop();

    if (committed) {
        char jcs[18], sws[18], pids[8];
        formatMac(result.jc, jcs, sizeof(jcs));
        formatMac(result.sw, sws, sizeof(sws));
        snprintf(pids, sizeof(pids), "0x%04X", result.pid);
        saveConfig(String(jcs), String(sws), String(pids));
        Serial.printf(">>> [SCAN OK] JC=%s SW=%s PID=%s RSSI=%d -> saved\n",
                      jcs, sws, pids, result.rssi);
        snprintf(g_lastEvent, sizeof(g_lastEvent), "SCAN saved JC=%s SW=%s PID=%s", jcs, sws, pids);
        flushSerialIfConnected();
        led(true);
        delay(2000);
        led(false);
    } else {
        Serial.println(">>> [SCAN TIMEOUT] no Joy-Con 2 wake packet found. Config unchanged.");
        snprintf(g_lastEvent, sizeof(g_lastEvent), "SCAN timeout, config unchanged");
        flushSerialIfConnected();
        for (int i = 0; i < 5; i++) { led(true); delay(60); led(false); delay(60); }
    }
}

// ---------------------------- エントリポイント ------------------------------
void setup() {
    Serial.begin(115200);

    // 前回スリープ中に保持していたGPIO設定を解除
#if SOC_GPIO_SUPPORT_HOLD_IO_IN_DSLP
    gpio_deep_sleep_hold_dis();
#endif
    gpio_hold_dis(HOME_GPIO);

    pinMode(PIN_LED, OUTPUT);
    led(false);
    pinMode(PIN_HOME, INPUT_PULLUP);
    pinMode(PIN_MINUS, INPUT);          // プルアップなし（実機の信号条件に合わせる）
    analogReadResolution(12);           // ADC読み取り（ボタン判定用）
    delay(30);                          // 入力が安定するまで待つ

    const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    const bool scanHeld = minusHeld();  // マイナス押下中か

    if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        // ホーム押下で復帰
        if (scanHeld) runScannerMode();   // ホーム + マイナス
        else          runTransmitter();   // 通常
    } else {
        // 電源投入 / リセット / 書込み直後
        if (scanHeld) runScannerMode();   // マイナスを押したまま電源投入/リセット
        else          runServiceWindow(); // USB/シリアルが使える猶予
    }
    goDeepSleep();
}

void loop() {}
