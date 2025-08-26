#ifndef CONFIG_H
#define CONFIG_H

// ================================================================================
// M5StickCPlus2 Sensor Monitor - config.example.h
// Copy this file to "config.h" and edit for your environment. Do NOT commit "config.h".
// ================================================================================

// ========== Wi-Fi ==========
const char* WIFI_NETWORK_NAME = "Your_WiFi_SSID";
const char* WIFI_NETWORK_PASSWORD = "Your_WiFi_Password";

// ========== MQTT ==========
const char* MQTT_BROKER_ADDRESS = "192.168.1.100";
const char* MQTT_TOPIC_NAME = "sensor_data";
const int MQTT_BROKER_PORT = 1883;
const char* MQTT_CLIENT_ID_PREFIX = "M5StickCPlus2-";

// ========== NTP / Time ==========
const char* TIME_SERVER_ADDRESS = "pool.ntp.org";
const long JAPAN_TIME_OFFSET_SECONDS = 32400;                   // JST (+9h)
const unsigned long TIME_UPDATE_INTERVAL_MILLISECONDS = 60000;  // 1 min

// ========== Main loop pacing ==========
const unsigned long MAIN_LOOP_DELAY_MILLISECONDS = 100;

// ========== Display coordinates ==========
const int VERTICAL_OFFSET = 5;
const int TITLE_POSITION_X = 5;
const int TITLE_POSITION_Y = 2 + VERTICAL_OFFSET;
const int TIME_DISPLAY_X = 140;
const int TIME_DISPLAY_Y = 2 + VERTICAL_OFFSET;
const int CONNECTION_STATUS_X = 190;
const int CONNECTION_STATUS_Y = 2 + VERTICAL_OFFSET;
const int LARGE_LABEL_X = 15;
const int LARGE_LABEL_Y = 30 + VERTICAL_OFFSET;
const int LARGE_VALUE_Y = 50 + VERTICAL_OFFSET;
const int DISPLAY_RIGHT_MARGIN = 15;
const int NO_DATA_MESSAGE_X = 40;
const int NO_DATA_MESSAGE_Y = 55 + VERTICAL_OFFSET;

// ========== Retries / timeouts ==========
const int MAXIMUM_NTP_RETRY_ATTEMPTS = 10;
const unsigned long MQTT_RECONNECTION_DELAY_MILLISECONDS = 5000;
const unsigned long CONNECTION_SUCCESS_DISPLAY_TIME = 2000;

// ========== JSON parsing buffer ==========
const size_t JSON_PARSING_MEMORY_SIZE = 2048;

// ========== Alternating view interval ==========
const unsigned long INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDS = 3000;

// ========== Gravity-based auto rotation ==========
const bool ENABLE_GRAVITY_AUTO_ROTATE = true;
const unsigned long ORIENTATION_CHECK_INTERVAL_MS = 200;
const float ORIENTATION_TILT_THRESHOLD_G = 0.20f;
const float ORIENTATION_HYSTERESIS_G = 0.05f;
const int DISPLAY_ROTATION_NORMAL = 1;   // main button on right
const int DISPLAY_ROTATION_FLIPPED = 3;  // 180°
const bool ORIENTATION_INVERT_X = false;

// ========== Vibrator HAT alerts (THI-based) ==========
const int VIBRATOR_CTRL_PIN = 26;        // GPIO26
const bool VIBRATOR_ACTIVE_HIGH = true;  // HIGH=on (invert if needed)
const bool ENABLE_VIBRATOR_ALERTS = true;
const float THI_HIGH_THRESHOLD = 70.0f;
const float THI_LOW_THRESHOLD = 65.0f;
const unsigned long VIBRATOR_ON_DURATION_MS = 10000;                    // 10s
const unsigned long VIBRATOR_SLEEP_COOLDOWN_MS = 60UL * 60UL * 1000UL;  // 1 hour

// ========== "Now Loading" spinner ==========
const unsigned long LOADING_SPINNER_INTERVAL_MS = 150;  // ms per frame
// Label and color settings
#define LOADING_LABEL_TEXT "Now Loading"
#define LOADING_LABEL_COLOR YELLOW
#define LOADING_SPINNER_COLOR YELLOW

#endif  // CONFIG_H
