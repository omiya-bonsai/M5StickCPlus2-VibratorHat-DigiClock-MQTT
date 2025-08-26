# M5StickC Plus2 Digi‑Clock + MQTT センサーモニター with Vibrator HAT

[English here / README.md](README.md)

**M5StickC Plus2** 向け Arduino スケッチです：
- 本体 LCD に **CO₂** と **THI（温熱快適性指数）** を大きく交互表示
- **Digi‑Clock Unit**（4桁 7セグ）に NTP 同期した `HH:MM` を表示（分が変わった時のみ更新）
- **重力による 180° 自動回転**（メインボタン右=通常／左でも正立）
- **Vibrator HAT アラート**：THI が **> 70.0** または **< 65.0** のとき **10秒振動**し、その後 **1時間スリープ**
- Wi‑Fi & MQTT 接続済みで**まだセンサデータが未到着**の間は **“Now Loading” スピナー**を表示

> 機微情報は **`config.h`** に記述します（**コミット禁止**）。リポジトリには **`config.example.h`** を同梱するので、**`config.h` にコピーして編集**してください。

![IMG_7649](https://github.com/user-attachments/assets/17ef902f-59bf-40c1-8898-b8f0f159dbc9)
---

## 特長

- **MQTT 受信**（単一トピック）＆ JSON パース（ArduinoJson）
- タイトル・NTP 時刻・MQTT 接続状態と **CO₂ / THI** の大文字表示
- **Digi‑Clock** は分が変化した時のみ更新してチラつきを抑制
- **NTP 同期**：`pool.ntp.org`、JST（+9h）を既定（変更可）
- **重力ベースの自動回転**：ローパス、傾き閾値、ヒステリシスを設定可能
- **Vibrator HAT アラート**：非ブロッキング制御とクールダウン制御
- **ロード中スピナー**：`| / - \` を最初の有効データ受信まで表示

---

## ハードウェア

- M5Stack **M5StickC Plus2**
- M5Stack **Digi‑Clock Unit**（Grove I²C）
- M5Stack **Vibrator HAT**
- Wi‑Fi と MQTT ブローカ（例：Mosquitto）

**配線**（StickC Plus2 の Grove）  
SDA = **G32**、SCL = **G33**（コード内で `Wire.begin(32, 33)`）  
Vibrator HAT 制御ピン：**GPIO 26**（`VIBRATOR_CTRL_PIN` で変更可）

![IMG_7648](https://github.com/user-attachments/assets/ca7fc8f5-6255-45a6-a3f1-9847cd604502)
---

## 使用ライブラリ

Arduino IDE でインストール：
- **M5StickCPlus2**（M5Unified ベース）
- **PubSubClient**
- **ArduinoJson**
- **NTPClient**
- **M5UNIT_DIGI_CLOCK**
- ESP32 **Arduino Core**

---

## クイックスタート

1. **`config.example.h`** を **`config.h`** にコピー。  
2. `config.h` を開いて **Wi‑Fi／MQTT／各種オプション**（自動回転・バイブ・スピナー）を設定。  
3. Arduino IDE で `.ino` を開き、ボード/ポートを設定して **検証→書き込み**。

---

## 設定（抜粋）

```cpp
// Wi‑Fi
const char* WIFI_NETWORK_NAME     = "Your_WiFi_SSID";
const char* WIFI_NETWORK_PASSWORD = "Your_WiFi_Password";

// MQTT
const char* MQTT_BROKER_ADDRESS   = "192.168.1.100";
const int   MQTT_BROKER_PORT      = 1883;
const char* MQTT_TOPIC_NAME       = "sensor_data";
const char* MQTT_CLIENT_ID_PREFIX = "M5StickCPlus2-";

// NTP / 時刻
const char* TIME_SERVER_ADDRESS = "pool.ntp.org";
const long  JAPAN_TIME_OFFSET_SECONDS = 32400; // +9時間
const unsigned long TIME_UPDATE_INTERVAL_MILLISECONDS = 60000;

// 自動回転
const bool  ENABLE_GRAVITY_AUTO_ROTATE     = true;
const unsigned long ORIENTATION_CHECK_INTERVAL_MS = 200;
const float ORIENTATION_TILT_THRESHOLD_G   = 0.20f;
const float ORIENTATION_HYSTERESIS_G       = 0.05f;
const int   DISPLAY_ROTATION_NORMAL        = 1;
const int   DISPLAY_ROTATION_FLIPPED       = 3;
const bool  ORIENTATION_INVERT_X           = false;

// Vibrator HAT
const int   VIBRATOR_CTRL_PIN              = 26;
const bool  VIBRATOR_ACTIVE_HIGH           = true;
const bool  ENABLE_VIBRATOR_ALERTS         = true;
const float THI_HIGH_THRESHOLD             = 70.0f;
const float THI_LOW_THRESHOLD              = 65.0f;
const unsigned long VIBRATOR_ON_DURATION_MS     = 10000; // 10秒
const unsigned long VIBRATOR_SLEEP_COOLDOWN_MS  = 60UL * 60UL * 1000UL; // 1時間

// ローディングスピナー
const unsigned long LOADING_SPINNER_INTERVAL_MS = 150;
#define LOADING_LABEL_TEXT    "Now Loading"
#define LOADING_LABEL_COLOR   YELLOW
#define LOADING_SPINNER_COLOR YELLOW
```

---

## MQTT ペイロード

購読トピック：`MQTT_TOPIC_NAME`  
例：
```json
{
  "co2": 820,
  "thi": 75.2,
  "temperature": 27.3,
  "humidity": 60.0,
  "comfort_level": "Warm",
  "timestamp": 1724639200
}
```
すべてのキーは任意です。受信項目のみ表示し、LCD は **CO₂** と **THI** を数秒ごとに交互表示します。THI が `[65.0, 70.0]` の範囲外でバイブレーションを 1 回起動し、以後 1 時間はクールダウンします。

---

## トラブルシュート

- **MQTT Connection Failed, rc = -2**  
  PubSubClient の `state()` が **-2** の場合はトランスポート接続失敗です。  
  - Wi‑Fi 接続と IP 取得を確認。  
  - `MQTT_BROKER_ADDRESS` へ到達可能か（同一サブネット／FW）。  
  - ブローカが `MQTT_BROKER_PORT`（標準 1883）で待ち受けているか（本スケッチは TLS 非対応）。  
  - 認証が必要なら `connect(clientId, user, pass)` を利用。

- **Digi‑Clock が表示しない**  
  Grove ケーブル向き、I²C ピン（**G32/SDA、G33/SCL**）を確認。接続後の再起動が確実。

- **時刻が `--` のまま／不正確**  
  NTP サーバへ到達できるネットワークか確認。JST 以外ならオフセット変更。

- **JSON Error** が出る  
  JSON の整形・引用符・桁数を確認。

---

## リポジトリ運用

- **`config.example.h`** をコミットし、**`config.h`** は `.gitignore` に追加（秘密情報保護）。

---

## ライセンス

MIT（`LICENSE` を参照）。
