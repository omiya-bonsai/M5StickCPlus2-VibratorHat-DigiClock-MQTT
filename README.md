# M5StickC Plus2 Digi‑Clock + MQTT Sensor Monitor with Vibrator HAT

[日本語はこちら / README‑ja.md](README-ja.md)

An Arduino sketch for **M5StickC Plus2** that:
- Displays **CO₂** and **THI (Thermal Comfort Index)** on the built‑in LCD (alternating view).
- Drives the **M5Stack Digi‑Clock Unit** (4‑digit 7‑segment) via Grove (I²C) to show NTP‑synced time in `HH:MM`.
- **Auto‑rotates** the LCD by **180° using gravity** so the UI stays upright whether the main button is on the right (default) or left.
- **Vibrator HAT alerts**: vibrates for **10 seconds** when THI is **> 70.0** or **< 65.0**, then sleeps for **1 hour** (cooldown).
- **“Now Loading” spinner** on the LCD while Wi‑Fi & MQTT are connected but no sensor payload has arrived yet.

> Private settings live in `config.h`, which should **NOT** be committed. This repo contains **`config.example.h`** — copy it to `config.h` and edit for your environment.


![IMG_7649](https://github.com/user-attachments/assets/cf519ac9-e86e-4603-990d-4e656f7a2289)
---

## Features

- **MQTT subscribe** to a topic and parse JSON payloads (ArduinoJson).
- **CO₂ / THI** large‑font view with title, NTP clock, and MQTT status.
- **Digi‑Clock** shows minute‑accurate time without flicker (updates only when minute changes).
- **NTP time** via `pool.ntp.org`, JST offset by +9h (configurable).
- **Gravity‑based auto‑rotation** with low‑pass filtering, tilt threshold, and hysteresis — all configurable.
- **Vibrator HAT alerts** with non‑blocking control and cooldown window.
- **Loading spinner** (`| / - \`) shown until the first valid sensor packet arrives.

---

## Hardware

- M5Stack **M5StickC Plus2**
- M5Stack **Digi‑Clock Unit** (I²C via Grove)
- M5Stack **Vibrator HAT**
- Wi‑Fi network and an MQTT broker (e.g., Mosquitto)

**Wiring** (Grove on StickC Plus2):
- SDA = **G32**, SCL = **G33** (handled in code with `Wire.begin(32, 33)`)
- Vibrator HAT control: **GPIO 26** (configurable; see `VIBRATOR_CTRL_PIN`)

![IMG_7648](https://github.com/user-attachments/assets/42b7b98b-4614-417f-be57-81130a44efc4)
---

## Libraries

Install these in Arduino IDE:
- **M5StickCPlus2** (M5Unified‑based)
- **PubSubClient**
- **ArduinoJson**
- **NTPClient**
- **M5UNIT_DIGI_CLOCK**
- ESP32 **Arduino core**

---

## Quick Start

1. Copy **`config.example.h`** → **`config.h`** (same folder as the sketch).
2. Open `config.h` and set **Wi‑Fi**, **MQTT**, and **options** (auto‑rotation, vibrator, spinner).
3. Open the `.ino` in Arduino IDE, choose the M5StickC Plus2 board/port, **Verify** and **Upload**.

---

## Configuration (excerpt)

```cpp
// Wi‑Fi
const char* WIFI_NETWORK_NAME     = "Your_WiFi_SSID";
const char* WIFI_NETWORK_PASSWORD = "Your_WiFi_Password";

// MQTT
const char* MQTT_BROKER_ADDRESS   = "192.168.1.100";
const int   MQTT_BROKER_PORT      = 1883;
const char* MQTT_TOPIC_NAME       = "sensor_data";
const char* MQTT_CLIENT_ID_PREFIX = "M5StickCPlus2-";

// NTP / time
const char* TIME_SERVER_ADDRESS = "pool.ntp.org";
const long  JAPAN_TIME_OFFSET_SECONDS = 32400; // +9h JST
const unsigned long TIME_UPDATE_INTERVAL_MILLISECONDS = 60000;

// Auto-rotation
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
const unsigned long VIBRATOR_ON_DURATION_MS     = 10000; // 10s
const unsigned long VIBRATOR_SLEEP_COOLDOWN_MS  = 60UL * 60UL * 1000UL; // 1h

// Loading spinner
const unsigned long LOADING_SPINNER_INTERVAL_MS = 150;
#define LOADING_LABEL_TEXT    "Now Loading"
#define LOADING_LABEL_COLOR   YELLOW
#define LOADING_SPINNER_COLOR YELLOW
```

---

## MQTT Payload

Subscribed topic: `MQTT_TOPIC_NAME`  
Example JSON:
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
All keys are optional; present ones will be displayed. The LCD alternates **CO₂** and **THI** every few seconds. THI outside `[65.0, 70.0]` triggers the Vibrator HAT once, then a 1‑hour cooldown.

---

## Troubleshooting

- **MQTT Connection Failed, rc = -2**  
  PubSubClient `state()` of **-2** = transport connect failure. Check:
  - Wi‑Fi is connected and the StickC has an IP.
  - `MQTT_BROKER_ADDRESS` is reachable (same subnet / firewall rules).
  - Broker is listening on `MQTT_BROKER_PORT` (1883 by default), no TLS in this sketch.
  - If the broker requires auth, use `connect(clientId, user, pass)`.

- **Digi‑Clock shows nothing**  
  Verify Grove cable orientation; I²C pins are **G32/SDA** and **G33/SCL**. Power‑cycle after attaching.

- **Time is `--` or wrong**  
  Ensure NTP server reachability; change time offset if not JST.

- **JSON Error** on screen  
  Validate JSON formatting and quoting.

---

## Repository Hygiene

- Commit **`config.example.h`**.  
- Add **`config.h`** to `.gitignore` to avoid leaking secrets.

---

## License

MIT (see `LICENSE`).
