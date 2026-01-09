#include "Arduino.h"
#include "EMGFilters.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <vector>
#include <math.h>
#include <HTTPUpdate.h>
#include <FastLED.h>
#include <WiFiClientSecure.h>

#define SensorInputPin1 0
#define SensorInputPin2 1
#define LED_PIN 2   
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];


EMGFilters myFilter1;
EMGFilters myFilter2;
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

int baseline1 = 0;
int baseline2 = 0;
bool baselineCalibrated = false;
const int CALIBRATION_SAMPLES = 100;
int calibrationCount = 0;
long calibrationSum1 = 0;
long calibrationSum2 = 0;

int emg1_filtered = 0;
int emg2_filtered = 0;
int currentAngle = 0;
bool isRotating = false;
unsigned long rotationStartTime = 0;
const int ROTATION_TIME = 500;

// Debouncing for grip/release detection
unsigned long lastStateChange = 0;
const unsigned long MIN_STATE_CHANGE_INTERVAL = 500; // Minimum 500ms between state changes (increased for stability)
bool lastGripState = false; // false = relaxed, true = gripped

// Confirmation mechanism: require signal to stay above/below threshold for a duration
unsigned long gripConfirmationStart = 0;
unsigned long relaxConfirmationStart = 0;
const unsigned long CONFIRMATION_DURATION = 200; // Require 200ms of consistent signal before changing state
bool confirmedGripState = false; // The confirmed/stable state
// ============================================================================
// ENHANCED SIGNAL PROCESSING - EMA FILTER
// ============================================================================
#define EMA_ARRAY_SIZE 15
int emaArray1[EMA_ARRAY_SIZE];
int emaArray2[EMA_ARRAY_SIZE];
int emaIndex1 = 0;
int emaIndex2 = 0;
bool emaFull1 = false;
bool emaFull2 = false;
float emaValue1 = 0.0;
float emaValue2 = 0.0;
const float EMA_ALPHA = 0.35;

int emg1_stable = 0;
int emg2_stable = 0;
// ============================================================================
// ADAPTIVE THRESHOLDS
// ============================================================================
const int DEFAULT_THRESHOLD_HIGH = 15;
const int DEFAULT_THRESHOLD_LOW = 10;
int thresholdHigh = DEFAULT_THRESHOLD_HIGH;
int thresholdLow = DEFAULT_THRESHOLD_LOW;
Preferences preferences;

// ============================================================================
// TRAINING STATE
// ============================================================================
struct TrainingState {
    bool active;
    unsigned long startTime;
    unsigned long lastSample;
    std::vector<int> samples;
    unsigned long DURATION_MS = 30000; // default 30 seconds, made writable by MQTT
    unsigned long SAMPLE_INTERVAL_MS = 100;
    int progressPercent;
    int gestureType;
} training;

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================
const char* ssid = "Bach Long";
const char* password = "03030380";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* topic_ota = "ota";
const char* topic_train = "train";
const char* topic_emg1 = "emg/sensor1";
const char* topic_emg2 = "emg/sensor2";
const char* topic_angle = "servo/angle";
const char* topic_threshold_low = "servo/threshold_low";
const char* topic_threshold_high = "servo/threshold_high";
const char* topic_cmd = "servo/cmd";
const char* topic_ema = "servo/ema";
const int OTA_CONNECT_TIMEOUT = 15000;      // 15s để connect
const int OTA_CHUNK_TIMEOUT = 45000;        // 45s cho mỗi chunk (tăng từ 30s)
const int OTA_TOTAL_TIMEOUT = 300000;       // 5 phút tổng (tăng từ 3 phút)

String deviceId = "device01";
String topic_device_ota = "";
String topic_device_status = "";

// ============================================================================
// TIMING & VERSION
// ============================================================================
unsigned long lastMqttPublish = 0;
const unsigned long MQTT_PUBLISH_INTERVAL = 1000;
unsigned long lastEmaPublish = 0;                  
const unsigned long EMA_PUBLISH_INTERVAL = 1000;
String firmwareVersion = "v2.1.0";

// ============================================================================
// OTA CONFIGURATION
// ============================================================================
const int MAX_FIRMWARE_SIZE = 1572864; // 1.5MB
const int OTA_TIMEOUT = 180000; // 180s (3 minutes) - increased for large files and weak WiFi


// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void setupHardware();
void setupNetwork();
void ensureConnectivity();
void handleCalibration();
void processEMGSignals();
void controlServo();
void handleTraining();
void publishTelemetry();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMqtt();
void performOtaUpdate(String url);
void setOtaLed(CRGB color);
void loadThresholds();
void saveThresholds();
void computeThresholdsKMeans();
void scanNetworksReport(); // THÊM DÒNG NÀY
// THÊM HÀM NÀY NGAY ĐÂY:
String wifiStatusToString(wl_status_t status) {
    switch(status) {
        case WL_NO_SHIELD: return "WL_NO_SHIELD";
        case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
        case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
        case WL_CONNECTED: return "WL_CONNECTED";
        case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
        case WL_DISCONNECTED: return "WL_DISCONNECTED";
        default: return "UNKNOWN";
    }
}
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n");
    Serial.println("╔═══════════════════════════════════════════════╗");
    Serial.println("║   ESP32-C6 Dual EMG Sensor System v2.1       ║");
    Serial.println("║   Adaptive K-Means + Secure OTA               ║");
    Serial.println("╚═══════════════════════════════════════════════╝");
    Serial.println();
    
    setupHardware();
    setupNetwork();
    loadThresholds();
    
    // KHỞI TẠO TOPIC MQTT - THÊM DÒNG NÀY
    topic_device_ota = "device/" + deviceId + "/ota";
    topic_device_status = "device/" + deviceId + "/status";
    
    // Khởi tạo EMA arrays
    for (int i = 0; i < EMA_ARRAY_SIZE; i++) {
        emaArray1[i] = 0;
        emaArray2[i] = 0;
    }
    emaValue1 = 0.0;
    emaValue2 = 0.0;
    emaIndex1 = 0;
    emaIndex2 = 0;
    emaFull1 = false;
    emaFull2 = false;
    Serial.println(">>> CALIBRATING: Keep muscles relaxed for 3 seconds...");
}

void setupHardware() {
    Serial.println("[INIT] Hardware setup starting...");
    
    // ADC configuration for ESP32-C6
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.println("  ✓ ADC configured (12-bit, 11dB attenuation)");
    
    // EMG filters initialization
    myFilter1.init(sampleRate, humFreq, true, true, true);
    myFilter2.init(sampleRate, humFreq, true, true, true);
    Serial.println("  ✓ EMG filters initialized (50Hz notch, 1kHz sample)");
    
    // WS2812 LED
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.println("  ✓ WS2812 LED ready");
    
    // Training state
    training.active = false;
    training.progressPercent = 0;
    training.samples.reserve(500);
    Serial.println("  ✓ Training system initialized");
    
    Serial.println("[INIT] Hardware setup complete\n");
}
void setupNetwork() {
    Serial.println("[NETWORK] Starting WiFi setup...");
    
    // Reset WiFi hoàn toàn
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    
    // Khởi tạo WiFi như code mẫu
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Scan networks
    Serial.println("\n[SCAN] Scanning networks...");
    int n = WiFi.scanNetworks();
    Serial.print("Found ");
    Serial.print(n);
    Serial.println(" networks");
    
    bool found = false;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == ssid) {
            found = true;
            Serial.print("✓ Found target network: ");
            Serial.println(ssid);
            Serial.print("  Channel: ");
            Serial.println(WiFi.channel(i));
            Serial.print("  RSSI: ");
            Serial.print(WiFi.RSSI(i));
            Serial.println(" dBm");
            break;
        }
    }
    
    WiFi.scanDelete();
    
    if (!found) {
        Serial.println("❌ Target network not found!");
        return;
    }
    
    // Kết nối đơn giản như code mẫu
    Serial.print("\n[CONNECT] Connecting to: ");
    Serial.println(ssid);
    
    WiFi.disconnect(true);
    WiFi.persistent(false);
    delay(1000);
    
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    
    // THỬ KẾT NỐI ĐƠN GIẢN
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi CONNECTED!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("\n❌ WiFi Connection Failed");
        Serial.print("Status: ");
        Serial.println(wifiStatusToString(WiFi.status()));
        
        // Thử với BSSID như code mẫu
        Serial.println("\n[Trying BSSID method...]");
        WiFi.disconnect(true);
        delay(2000);
        
        // Tìm BSSID
        n = WiFi.scanNetworks();
        String targetBSSID = "";
        for (int i = 0; i < n; i++) {
            if (WiFi.SSID(i) == ssid) {
                targetBSSID = WiFi.BSSIDstr(i);
                break;
            }
        }
        
        if (targetBSSID.length() > 0) {
            Serial.print("Using BSSID: ");
            Serial.println(targetBSSID);
            
            uint8_t bssid[6];
            sscanf(targetBSSID.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                   &bssid[0], &bssid[1], &bssid[2], 
                   &bssid[3], &bssid[4], &bssid[5]);
            
            WiFi.begin(ssid, password, 0, bssid); // Channel 0 = auto
            
            attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);
                Serial.print(".");
                attempts++;
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\n✅ CONNECTED with BSSID!");
                Serial.print("IP: ");
                Serial.println(WiFi.localIP());
            }
        }
    }
    
    // Cấu hình MQTT (đơn giản hóa)
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(1024);
    
    Serial.println("[NETWORK] Setup complete\n");
}

void scanNetworksReport() {
    Serial.println("[WIFI] Scanning for nearby networks...");
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    int n = WiFi.scanNetworks();
    if (n == 0) {
        Serial.println("  No networks found");
        return;
    }
    
    Serial.print("  Found ");
    Serial.print(n);
    Serial.println(" networks:");
    
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        int rssi = WiFi.RSSI(i);
        int channel = WiFi.channel(i);
        wifi_auth_mode_t enc = WiFi.encryptionType(i);
        String bssid = WiFi.BSSIDstr(i);
        String encName;
        
        switch (enc) {
            case WIFI_AUTH_OPEN: encName = "OPEN"; break;
            case WIFI_AUTH_WEP: encName = "WEP"; break;
            case WIFI_AUTH_WPA_PSK: encName = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK: encName = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK: encName = "WPA/WPA2"; break;
            case WIFI_AUTH_WPA3_PSK: encName = "WPA3"; break;
            case WIFI_AUTH_WPA2_WPA3_PSK: encName = "WPA2/WPA3"; break;
            default: encName = "UNKNOWN"; break;
        }

        Serial.print("    ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(ssid);
        Serial.print(" | BSSID: ");
        Serial.print(bssid);
        Serial.print(" | RSSI: ");
        Serial.print(rssi);
        Serial.print(" dBm");
        Serial.print(" | CH: ");
        Serial.print(channel);
        if (channel > 14) Serial.print(" (5GHz)"); else Serial.print(" (2.4GHz)");
        Serial.print(" | Enc: ");
        Serial.println(encName);
        
        if (ssid == "Bach Long") {
            Serial.println("      ⭐ TARGET NETWORK FOUND!");
        }
    }
    
    WiFi.scanDelete();
}

void loop() {
    ensureConnectivity();
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "scan") {
            scanNetworksReport();
        } else if (cmd == "reconnect") {
            Serial.println("Manual WiFi reconnect...");
            WiFi.disconnect();
            delay(1000);
            WiFi.begin(ssid, password);
        } else if (cmd == "status") {
            Serial.print("WiFi Status: ");
            Serial.println(wifiStatusToString(WiFi.status()));
            Serial.print("RSSI: ");
            Serial.println(WiFi.RSSI());
            Serial.print("Channel: ");
            Serial.println(WiFi.channel());
        }
    }
    if (!baselineCalibrated) {
        handleCalibration();
        return;
    }
    
    processEMGSignals();
    controlServo();
    handleTraining();
    publishTelemetry();
    
    delayMicroseconds(1000); // 1kHz sampling
}
void ensureConnectivity() {
    static unsigned long lastWifiCheck = 0;
    static unsigned long lastMqttCheck = 0;
    static unsigned long lastMqttPublish = 0;
    unsigned long now = millis();
    
    // Check WiFi every 10 seconds
    if (now - lastWifiCheck >= 10000) {
        lastWifiCheck = now;
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WIFI] Connection lost, reconnecting...");
            WiFi.disconnect();
            delay(100);
            WiFi.begin(ssid, password);
            
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 15) {
                delay(500);
                Serial.print(".");
                attempts++;
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\n  ✓ WiFi reconnected");
                Serial.print("  IP: ");
                Serial.println(WiFi.localIP());
                Serial.print("  RSSI: ");
                Serial.print(WiFi.RSSI());
                Serial.println(" dBm");
                
                // Sau khi WiFi reconnect, đợi 2s rồi reconnect MQTT
                delay(2000);
                if (mqttClient.connected()) {
                    mqttClient.disconnect();
                }
            } else {
                Serial.println("\n  ✗ WiFi reconnect failed");
            }
        }
    }
    
    // Check MQTT connection every 5 seconds
    if (now - lastMqttCheck >= 5000) {
        lastMqttCheck = now;
        
        if (!mqttClient.connected()) {
            reconnectMqtt();
        } else {
            // Send heartbeat every 30 seconds to keep connection alive
            if (now - lastMqttPublish >= 30000) {
                lastMqttPublish = now;
                
                String heartbeat = "{\"device\":\"" + deviceId + 
                                  "\",\"type\":\"heartbeat\"" +
                                  ",\"uptime\":" + String(millis() / 1000) + 
                                  ",\"heap\":" + String(ESP.getFreeHeap()) + 
                                  ",\"rssi\":" + String(WiFi.RSSI()) + "}";
                
                mqttClient.publish(topic_device_status.c_str(), heartbeat.c_str());
                
                Serial.println("[MQTT] Heartbeat sent");
            }
        }
    }
    
    // Process MQTT messages
    if (mqttClient.connected()) {
        mqttClient.loop();
    }
}
// ============================================================================
// MQTT RECONNECTION - IMPROVED VERSION
// ============================================================================
void reconnectMqtt() {
    static unsigned long lastAttempt = 0;
    static int attemptCount = 0;
    const unsigned long RECONNECT_INTERVAL = 5000; // 5 seconds
    
    unsigned long now = millis();
    
    // Kiểm tra interval
    if (now - lastAttempt < RECONNECT_INTERVAL) {
        return;
    }
    
    lastAttempt = now;
    attemptCount++;
    
    Serial.print("[MQTT] Connecting (attempt ");
    Serial.print(attemptCount);
    Serial.print(")...");
    
    String clientId = "esp32_" + deviceId + "_" + String(random(0xffff), HEX);
    
    // Chuẩn bị will message
    String willMsgStr = "{\"status\":\"disconnected\",\"device\":\"" + deviceId + "\"}";
    const char* willMessage = willMsgStr.c_str();
    
    // Cấu hình MQTT với will message - SỬA LỖI Ở ĐÂY
    bool connected = mqttClient.connect(
        clientId.c_str(),                     // Client ID
        NULL,                                 // username
        NULL,                                 // password
        topic_device_status.c_str(),          // will topic
        1,                                    // will QoS
        true,                                 // will retain
        willMessage                           // will message (phải là const char*)
    );
    
    if (connected) {
        Serial.println(" ✓");
        attemptCount = 0;
        
        // Subscribe to topics với QoS 1
        bool sub1 = mqttClient.subscribe(topic_ota, 1);
        bool sub2 = mqttClient.subscribe(topic_train, 1);
        bool sub3 = mqttClient.subscribe(topic_cmd, 1);
        bool sub4 = mqttClient.subscribe(topic_device_ota.c_str(), 1);
        
        Serial.println("  Subscribed to:");
        Serial.print("    - "); Serial.print(topic_ota);
        Serial.println(sub1 ? " ✓" : " ✗");
        Serial.print("    - "); Serial.print(topic_train);
        Serial.println(sub2 ? " ✓" : " ✗");
        Serial.print("    - "); Serial.print(topic_cmd);
        Serial.println(sub3 ? " ✓" : " ✗");
        Serial.print("    - "); Serial.print(topic_device_ota);
        Serial.println(sub4 ? " ✓" : " ✗");
        
        // Publish initial state với retain
        mqttClient.publish(topic_threshold_low, String(thresholdLow).c_str(), true);
        mqttClient.publish(topic_threshold_high, String(thresholdHigh).c_str(), true);
        
        String status = "{\"device\":\"" + deviceId + 
                       "\",\"fw\":\"" + firmwareVersion + 
                       "\",\"status\":\"connected\"" +
                       ",\"ip\":\"" + WiFi.localIP().toString() + "\"" +
                       ",\"rssi\":" + String(WiFi.RSSI()) +
                       ",\"thresholds\":{\"low\":" + String(thresholdLow) + 
                       ",\"high\":" + String(thresholdHigh) + "}}";
        
        bool pubStatus = mqttClient.publish(topic_device_status.c_str(), status.c_str(), true);
        Serial.print("  Published status: ");
        Serial.println(pubStatus ? "✓" : "✗");
        
        // Gửi firmware version
        String emaPayload = "{\"s1\":" + String(emg1_filtered) + 
                           ",\"s2\":" + String(emg2_filtered) +
                           ",\"device\":\"" + deviceId + "\"" +
                           ",\"firmware\":\"" + firmwareVersion + "\"}";
        
        bool pubEma = mqttClient.publish(topic_ema, emaPayload.c_str());
        Serial.print("  Published EMA data: ");
        Serial.println(pubEma ? "✓" : "✗");
        
        Serial.println("  ✓ MQTT connection established successfully");
        
    } else {
        Serial.print(" ✗ Failed (rc=");
        Serial.print(mqttClient.state());
        Serial.print(") - ");
        
        // Hiển thị thông báo lỗi chi tiết
        switch(mqttClient.state()) {
            case -4: Serial.println("Connection timeout"); break;
            case -3: Serial.println("Connection lost"); break;
            case -2: Serial.println("Connect failed"); break;
            case -1: Serial.println("Disconnected"); break;
            case 1: Serial.println("Bad protocol"); break;
            case 2: Serial.println("Bad client ID"); break;
            case 3: Serial.println("Unavailable"); break;
            case 4: Serial.println("Bad credentials"); break;
            case 5: Serial.println("Unauthorized"); break;
            default: Serial.println("Unknown error"); break;
        }
        
        // Nếu thất bại nhiều lần, thử reset WiFi
        if (attemptCount >= 3) {
            Serial.println("  ⚠ Multiple MQTT failures, checking WiFi...");
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("  WiFi disconnected, reconnecting...");
                WiFi.disconnect();
                delay(100);
                WiFi.begin(ssid, password);
            }
        }
    }
}
// ============================================================================
// CALIBRATION
// ============================================================================
void handleCalibration() {
    int raw1 = analogRead(SensorInputPin1);
    int raw2 = analogRead(SensorInputPin2);
    
    calibrationSum1 += raw1;
    calibrationSum2 += raw2;
    calibrationCount++;
    
    if (calibrationCount % 20 == 0) {
        int percent = (calibrationCount * 100) / CALIBRATION_SAMPLES;
        Serial.print("  Calibrating... ");
        Serial.print(calibrationCount);
        Serial.print("/");
        Serial.print(CALIBRATION_SAMPLES);
        Serial.print(" (");
        Serial.print(percent);
        Serial.println("%)");
    }
    
    if (calibrationCount >= CALIBRATION_SAMPLES) {
        baseline1 = calibrationSum1 / CALIBRATION_SAMPLES;
        baseline2 = calibrationSum2 / CALIBRATION_SAMPLES;
        baselineCalibrated = true;
        
        Serial.println("\n╔═══════════════════════════════════════╗");
        Serial.println("║   CALIBRATION COMPLETE                ║");
        Serial.println("╚═══════════════════════════════════════╝");
        Serial.print("  Baseline 1: ");
        Serial.println(baseline1);
        Serial.print("  Baseline 2: ");
        Serial.println(baseline2);
        Serial.println("\nSystem ready. Starting EMG monitoring...\n");
        
        // Publish calibration complete
        if (mqttClient.connected()) {
            String msg = "{\"status\":\"calibrated\",\"baseline1\":" + 
                        String(baseline1) + ",\"baseline2\":" + 
                        String(baseline2) + "}";
            mqttClient.publish(topic_device_status.c_str(), msg.c_str());
        }
    }
    
    delay(10);
}

// ============================================================================
// SIGNAL PROCESSING
// ============================================================================
void processEMGSignals() {
    // 1. ĐỌC ADC
    int raw1 = analogRead(SensorInputPin1);
    int raw2 = analogRead(SensorInputPin2);
    
    // 2. LOẠI BỎ BASELINE (DC OFFSET)
    int normalized1 = max(0, raw1 - baseline1);
    int normalized2 = max(0, raw2 - baseline2);
    
    // 3. LỌC QUA EMGFilters (50Hz notch + envelope)
    int filtered1 = max(0, myFilter1.update(normalized1));
    int filtered2 = max(0, myFilter2.update(normalized2));
    
    // 4. MOVING AVERAGE - Lớp lọc thứ nhất
    emaArray1[emaIndex1] = filtered1;
    emaIndex1 = (emaIndex1 + 1) % EMA_ARRAY_SIZE;
    if (emaIndex1 == 0) emaFull1 = true;
    
    emaArray2[emaIndex2] = filtered2;
    emaIndex2 = (emaIndex2 + 1) % EMA_ARRAY_SIZE;
    if (emaIndex2 == 0) emaFull2 = true;
    
    // 5. EMA - Lớp lọc thứ hai (chỉ tính sau khi mảng đầy)
    if (emaFull1) {
        long sum1 = 0;
        for (int i = 0; i < EMA_ARRAY_SIZE; i++) {
            sum1 += emaArray1[i];
        }
        int average1 = sum1 / EMA_ARRAY_SIZE;
        emaValue1 = EMA_ALPHA * average1 + (1.0 - EMA_ALPHA) * emaValue1;
        emg1_stable = (int)emaValue1;
    }
    
    if (emaFull2) {
        long sum2 = 0;
        for (int i = 0; i < EMA_ARRAY_SIZE; i++) {
            sum2 += emaArray2[i];
        }
        int average2 = sum2 / EMA_ARRAY_SIZE;
        emaValue2 = EMA_ALPHA * average2 + (1.0 - EMA_ALPHA) * emaValue2;
        emg2_stable = (int)emaValue2;
    }
    
    // 6. CẬP NHẬT GIÁ TRỊ CHO CÁC HÀM KHÁC (backward compatibility)
    emg1_filtered = emg1_stable;
    emg2_filtered = emg2_stable;
    
    // 7. DEBUG OUTPUT (throttled to 10Hz)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 100) {
        lastDebug = millis();
        Serial.print("EMG1: raw=");
        Serial.print(filtered1);
        Serial.print(" stable=");
        Serial.print(emg1_stable);
        Serial.print(" | EMG2: raw=");
        Serial.print(filtered2);
        Serial.print(" stable=");
        Serial.print(emg2_stable);
        Serial.print(" | Angle: ");
        Serial.print(currentAngle);
        Serial.print("° | Thresh: L=");
        Serial.print(thresholdLow);
        Serial.print(" H=");
        Serial.println(thresholdHigh);
    }
}
void controlServo() {
    // Đảm bảo EMA đã ổn định
    if (!emaFull1 || !emaFull2) {
        return;
    }
    
    // Validate thresholds
    if (thresholdLow < 2) {
        int minLow = max(2, thresholdHigh / 4);
        if (thresholdLow != minLow) {
            thresholdLow = minLow;
            Serial.println("  ⚠ WARNING: thresholdLow adjusted to " + String(thresholdLow));
            saveThresholds();
        }
    }
    
    int minSeparation = max(5, (int)(thresholdLow * 0.3));
    if (thresholdHigh < thresholdLow + minSeparation) {
        thresholdHigh = thresholdLow + minSeparation;
        Serial.println("  ⚠ WARNING: thresholdHigh adjusted to " + String(thresholdHigh));
        saveThresholds();
    }
    
    // Debouncing
    unsigned long now = millis();
    bool canChangeState = (now - lastStateChange >= MIN_STATE_CHANGE_INTERVAL);
    
    // ✅ SỬ DỤNG emg1_stable, emg2_stable (đã qua EMA)
    bool currentGripSignal = (emg1_stable > thresholdHigh || emg2_stable > thresholdHigh);
    bool currentRelaxSignal = (emg1_stable < thresholdLow && emg2_stable < thresholdLow);
    
    // Confirmation mechanism
    if (currentGripSignal && !confirmedGripState) {
        if (gripConfirmationStart == 0) {
            gripConfirmationStart = now;
        } else if (now - gripConfirmationStart >= CONFIRMATION_DURATION) {
            confirmedGripState = true;
            relaxConfirmationStart = 0;
        }
    } else if (!currentGripSignal) {
        gripConfirmationStart = 0;
    }
    
    if (currentRelaxSignal && confirmedGripState) {
        if (relaxConfirmationStart == 0) {
            relaxConfirmationStart = now;
        } else if (now - relaxConfirmationStart >= CONFIRMATION_DURATION) {
            confirmedGripState = false;
            gripConfirmationStart = 0;
        }
    } else if (!currentRelaxSignal) {
        relaxConfirmationStart = 0;
    }
    
    // Rotate to 180° if grip is confirmed
    if (confirmedGripState && currentAngle == 0 && !isRotating && canChangeState) {
        isRotating = true;
        rotationStartTime = millis();
        currentAngle = 180;
        lastStateChange = now;
        lastGripState = true;
        
        Serial.println("\n>>> SERVO: 0° → 180° (FLEX DETECTED)");
        Serial.print("  EMG1: "); Serial.print(emg1_stable);
        Serial.print(" | EMG2: "); Serial.print(emg2_stable);
        Serial.print(" | Thresh: L="); Serial.print(thresholdLow);
        Serial.print(" H="); Serial.println(thresholdHigh);
        
        if (mqttClient.connected()) {
            mqttClient.publish(topic_angle, "180");
        }
    }
    // Return to 0° if relax is confirmed
    else if (!confirmedGripState && currentAngle == 180 && !isRotating && canChangeState) {
        isRotating = true;
        rotationStartTime = millis();
        currentAngle = 0;
        lastStateChange = now;
        lastGripState = false;
        
        Serial.println("\n>>> SERVO: 180° → 0° (RELAXED)");
        Serial.print("  EMG1: "); Serial.print(emg1_stable);
        Serial.print(" | EMG2: "); Serial.print(emg2_stable);
        Serial.print(" | Thresh: L="); Serial.print(thresholdLow);
        Serial.print(" H="); Serial.println(thresholdHigh);
        
        if (mqttClient.connected()) {
            mqttClient.publish(topic_angle, "0");
        }
    }
    
    // End rotation after timeout
    if (isRotating && (millis() - rotationStartTime >= ROTATION_TIME)) {
        isRotating = false;
    }
}

void handleTraining() {
    if (!training.active) return;
    
    unsigned long now = millis();
    unsigned long elapsed = now - training.startTime;
    
    // QUAN TRỌNG: Chỉ thu thập khi EMA đã ổn định
    if (!emaFull1 || !emaFull2) {
        Serial.println("[TRAIN] Waiting for EMA to stabilize...");
        return;
    }
    
    // Sample collection
    if (now - training.lastSample >= training.SAMPLE_INTERVAL_MS) {
        training.lastSample = now;
        
        // ✅ Thu thập dữ liệu ĐÃ QUA EMA (emg1_stable, emg2_stable)
        training.samples.push_back(emg1_stable);
        training.samples.push_back(emg2_stable);
        
        // Update progress
        training.progressPercent = (elapsed * 100) / training.DURATION_MS;
        
        if (training.samples.size() % 10 == 0) {
            Serial.print("[TRAIN] Gesture ");
            Serial.print(training.gestureType == 0 ? "NẮM" : "THẢ");
            Serial.print(" | Samples: ");
            Serial.print(training.samples.size());
            Serial.print(" | Progress: ");
            Serial.print(training.progressPercent);
            Serial.print("% | Current: S1=");
            Serial.print(emg1_stable);
            Serial.print(" S2=");
            Serial.println(emg2_stable);
            
            // Gửi progress cho web
            if (mqttClient.connected()) {
                String msg = "{\"status\":\"training_progress\",\"gesture\":" + 
                            String(training.gestureType) +
                            ",\"progress\":" + String(training.progressPercent) + 
                            ",\"samples\":" + String(training.samples.size()) + 
                            ",\"current_s1\":" + String(emg1_stable) +
                            ",\"current_s2\":" + String(emg2_stable) + "}";
                mqttClient.publish(topic_train, msg.c_str());
            }
        }
    }
    
    // Check if duration elapsed
    if (elapsed >= training.DURATION_MS) {
        training.active = false;
        
        Serial.println("\n╔═══════════════════════════════════════╗");
        Serial.print("║   TRAINING COMPLETE: ");
        Serial.print(training.gestureType == 0 ? "NẮM" : "THẢ");
        Serial.println("     ║");
        Serial.println("╚═══════════════════════════════════════╝");
        Serial.print("  Total samples: ");
        Serial.println(training.samples.size());
        
        if (training.samples.size() < 10) {
            Serial.println("  ✗ ERROR: Not enough samples");
            mqttClient.publish(topic_train, "{\"status\":\"insufficient_data\"}");
            training.samples.clear();
            return;
        }
        
        // Gửi collection_done cho web
        String response = "{\"status\":\"collection_done\",\"gesture\":" + 
                 String(training.gestureType) + 
                 ",\"samples\":" + String(training.samples.size()) + "}";
        mqttClient.publish(topic_train, response.c_str());

        Serial.println("  ✓ Data collection complete");
        Serial.println("  → Running K-Means clustering...");
        
        computeThresholdsKMeans();
        
        Serial.println("  ✓ K-Means training finished\n");
    }
}

// ============================================================================
// ADAPTIVE K-MEANS CLUSTERING (9.5/10 ACCURACY)
// ============================================================================
void computeThresholdsKMeans() {
    const int n = training.samples.size();
    const int K = 2;
    const int MAX_ITER = 100;
    const int N_INIT = 5;
    
    Serial.println("[K-MEANS] Starting clustering...");
    Serial.print("  Sample count: ");
    Serial.println(n);
    
    // Allocate memory
    double* data = new double[n];
    int* labels = new int[n];
    
    // Convert to double array
    for (int i = 0; i < n; i++) {
        data[i] = (double)training.samples[i];
    }
    
    // ========== OUTLIER REMOVAL (3-SIGMA RULE) ==========
    double sum = 0;
    for (int i = 0; i < n; i++) sum += data[i];
    double mean = sum / n;
    
    double variance = 0;
    for (int i = 0; i < n; i++) {
        double diff = data[i] - mean;
        variance += diff * diff;
    }
    double stddev = sqrt(variance / n);
    
    Serial.print("  Mean: ");
    Serial.print(mean, 2);
    Serial.print(" | StdDev: ");
    Serial.println(stddev, 2);
    
    // Filter outliers
    int n_filtered = 0;
    for (int i = 0; i < n; i++) {
        if (stddev == 0 || fabs(data[i] - mean) <= 3.0 * stddev) {
            data[n_filtered++] = data[i];
        }
    }
    
    int outliers = n - n_filtered;
    Serial.print("  Outliers removed: ");
    Serial.print(outliers);
    Serial.print(" (");
    Serial.print((outliers * 100.0) / n, 1);
    Serial.println("%)");
    
    // Safety check
    if (n_filtered < n / 2) {
        Serial.println("  ⚠ Too many outliers, using original data");
        n_filtered = n;
        for (int i = 0; i < n; i++) {
            data[i] = (double)training.samples[i];
        }
    }
    
    // ========== K-MEANS WITH MULTIPLE INITIALIZATIONS ==========
    double bestCentroids[K];
    double bestInertia = 1e308;
    
    Serial.println("  Running K-Means iterations...");
    
    for (int init = 0; init < N_INIT; init++) {
        double centroids[K];
        
        // K-Means++ initialization
        if (init == 0) {
            // First centroid: random
            centroids[0] = data[rand() % n_filtered];
            
            // Second centroid: farthest from first
            double maxDist = -1;
            for (int i = 0; i < n_filtered; i++) {
                double dist = fabs(data[i] - centroids[0]);
                if (dist > maxDist) {
                    maxDist = dist;
                    centroids[1] = data[i];
                }
            }
        } else {
            // Random initialization
            centroids[0] = data[rand() % n_filtered];
            centroids[1] = data[rand() % n_filtered];
        }
        
        // Lloyd's algorithm
        bool converged = false;
        int iterations = 0;
        
        for (int iter = 0; iter < MAX_ITER && !converged; iter++) {
            iterations = iter + 1;
            
            // Assignment step
            for (int i = 0; i < n_filtered; i++) {
                double dist0 = fabs(data[i] - centroids[0]);
                double dist1 = fabs(data[i] - centroids[1]);
                labels[i] = (dist0 <= dist1) ? 0 : 1;
            }
            
            // Update step
            converged = true;
            for (int k = 0; k < K; k++) {
                double sum_k = 0;
                int count_k = 0;
                
                for (int i = 0; i < n_filtered; i++) {
                    if (labels[i] == k) {
                        sum_k += data[i];
                        count_k++;
                    }
                }
                
                if (count_k > 0) {
                    double new_centroid = sum_k / count_k;
                    if (fabs(new_centroid - centroids[k]) > 1e-6) {
                        converged = false;
                    }
                    centroids[k] = new_centroid;
                }
            }
        }
        
        // Compute inertia
        double inertia = 0;
        for (int i = 0; i < n_filtered; i++) {
            double diff = data[i] - centroids[labels[i]];
            inertia += diff * diff;
        }
        
        // Keep best result
        if (inertia < bestInertia) {
            bestInertia = inertia;
            bestCentroids[0] = centroids[0];
            bestCentroids[1] = centroids[1];
        }
        
        Serial.print("    Init ");
        Serial.print(init + 1);
        Serial.print(": iterations=");
        Serial.print(iterations);
        Serial.print(", inertia=");
        Serial.println(inertia, 2);
    }
    
    // Sort centroids (low to high)
    if (bestCentroids[0] > bestCentroids[1]) {
        double temp = bestCentroids[0];
        bestCentroids[0] = bestCentroids[1];
        bestCentroids[1] = temp;
    }
    
    // ========== ADAPTIVE THRESHOLD CALCULATION ==========
    double separation = bestCentroids[1] - bestCentroids[0];
    double avgCentroid = (bestCentroids[0] + bestCentroids[1]) / 2.0;
    double separationRatio = (avgCentroid > 0) ? (separation / avgCentroid) : 0;
    
    Serial.println("\n[RESULTS]");
    Serial.print("  Centroid 0 (rest): ");
    Serial.println(bestCentroids[0], 2);
    Serial.print("  Centroid 1 (flex): ");
    Serial.println(bestCentroids[1], 2);
    Serial.print("  Separation: ");
    Serial.print(separation, 2);
    Serial.print(" (");
    Serial.print(separationRatio * 100, 1);
    Serial.println("%)");
    
    // Adaptive high threshold based on separation quality
    double highReduction;
    if (separationRatio > 1.0) {
        highReduction = 0.75; // 25% reduction (excellent separation)
        Serial.println("  Quality: EXCELLENT → 25% reduction");
    } else if (separationRatio > 0.5) {
        highReduction = 0.80; // 20% reduction (good separation)
        Serial.println("  Quality: GOOD → 20% reduction");
    } else {
        highReduction = 0.90; // 10% reduction (poor separation)
        Serial.println("  Quality: FAIR → 10% reduction");
    }
    
    // Calculate thresholds with validation
    thresholdLow = (int)round(bestCentroids[0]);
    thresholdHigh = (int)round(bestCentroids[1] * highReduction);
    
    // CRITICAL: Ensure thresholdLow is NEVER 0 or too low
    // Minimum thresholdLow must be at least 2 to properly detect relaxed state
    // If centroid 0 is too low, use a percentage of centroid 1 instead
    if (thresholdLow <= 0 || thresholdLow < 2) {
        // Use 25% of high threshold as low threshold, minimum 2
        thresholdLow = max(2, (int)round(bestCentroids[1] * 0.25));
        Serial.print("  ⚠ thresholdLow was too low, adjusted to: ");
        Serial.println(thresholdLow);
    }
    
    // Ensure minimum separation between thresholds (at least 30% of low threshold, minimum 5)
    int minSeparation = max(5, (int)(thresholdLow * 0.3));
    if (thresholdHigh < thresholdLow + minSeparation) {
        thresholdHigh = thresholdLow + minSeparation;
        Serial.print("  ⚠ Applied minimum separation: +");
        Serial.println(minSeparation);
    }
    
    // Final validation: ensure thresholds are reasonable and properly ordered
    if (thresholdLow >= thresholdHigh || thresholdLow <= 0) {
        // Fallback: use safe defaults with proper separation
        // Ensure low is at least 2, high is at least low + 5
        thresholdLow = max(2, (int)round(bestCentroids[0]));
        if (thresholdLow < 2) thresholdLow = 2; // Force minimum
        
        int calculatedHigh = thresholdLow + max(8, (int)round(separation * 0.5));
        if (calculatedHigh <= thresholdLow) {
            calculatedHigh = thresholdLow + 8; // Force minimum separation
        }
        thresholdHigh = calculatedHigh;
        
        Serial.println("  ⚠ WARNING: Thresholds were invalid, using safe fallback values");
        Serial.print("    Fallback LOW: "); Serial.println(thresholdLow);
        Serial.print("    Fallback HIGH: "); Serial.println(thresholdHigh);
    }
    
    // Final safety check: ensure thresholdLow is definitely > 0
    if (thresholdLow <= 0) {
        thresholdLow = 2; // Absolute minimum
        thresholdHigh = max(thresholdHigh, thresholdLow + 5);
        Serial.println("  ⚠ CRITICAL: Forced thresholdLow to 2 (absolute minimum)");
    }
    
    // Save to NVS
    saveThresholds();
    
    // Publish results
    String result = "{\"status\":\"complete\"" +
                   String(",\"centroids\":[") + String(bestCentroids[0], 1) + "," + String(bestCentroids[1], 1) + "]" +
                   ",\"separation\":" + String(separation, 2) +
                   ",\"separation_ratio\":" + String(separationRatio, 3) +
                   ",\"threshold_low\":" + String(thresholdLow) +
                   ",\"threshold_high\":" + String(thresholdHigh) +
                   ",\"reduction_percent\":" + String((1.0 - highReduction) * 100, 0) +
                   ",\"samples_used\":" + String(n_filtered) + "}";
    
    mqttClient.publish(topic_train, result.c_str());
    mqttClient.publish(topic_threshold_low, String(thresholdLow).c_str());
    mqttClient.publish(topic_threshold_high, String(thresholdHigh).c_str());
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   NEW THRESHOLDS APPLIED              ║");
    Serial.println("╚═══════════════════════════════════════╝");
    Serial.print("  LOW  = ");
    Serial.println(thresholdLow);
    Serial.print("  HIGH = ");
    Serial.println(thresholdHigh);
    Serial.println("  ✓ Saved to NVS");
    Serial.println("  ✓ Published to MQTT\n");
    
    // Cleanup
    delete[] data;
    delete[] labels;
}

void publishTelemetry() {
    unsigned long now = millis();
    
    // Publish EMA + Firmware every 1s
    if (now - lastEmaPublish >= EMA_PUBLISH_INTERVAL) {
        if (!mqttClient.connected()) return;
        lastEmaPublish = now;
        
        // ✅ Gửi cả raw và stable values
        String emaPayload = "{\"s1_raw\":" + String(emg1_filtered) + 
                           ",\"s2_raw\":" + String(emg2_filtered) +
                           ",\"s1_stable\":" + String(emg1_stable) +
                           ",\"s2_stable\":" + String(emg2_stable) +
                           ",\"firmware\":\"" + firmwareVersion + "\"}";
        
        mqttClient.publish(topic_ema, emaPayload.c_str());
    }
    
    // Publish detailed status every 5s
    static unsigned long lastStatusPublish = 0;
    if (now - lastStatusPublish >= 5000) {
        if (!mqttClient.connected()) return;
        lastStatusPublish = now;
        
        String status = "{\"emg1_stable\":" + String(emg1_stable) +
                       ",\"emg2_stable\":" + String(emg2_stable) +
                       ",\"angle\":" + String(currentAngle) +
                       ",\"thresholds\":{\"low\":" + String(thresholdLow) + 
                       ",\"high\":" + String(thresholdHigh) + "}" +
                       ",\"fw\":\"" + firmwareVersion + "\"" +
                       ",\"ema_ready\":" + String(emaFull1 && emaFull2) +
                       ",\"uptime\":" + String(millis() / 1000) + "}";
        
        mqttClient.publish(topic_device_status.c_str(), status.c_str());
    }
}
// ============================================================================
// MQTT CALLBACK - COMMAND HANDLER
// ============================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (length > 512) {
        Serial.println("[ERROR] Payload too large (>512 bytes)");
        return;
    }
    
    String msg = "";
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }
    
    Serial.print("\n[MQTT] ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(msg);
    
    String topicStr(topic);
    
    // ========== TRAINING COMMANDS ==========
    if (topicStr == topic_train || topicStr == topic_cmd) {
        if (msg.indexOf("train_threshold") >= 0 || msg.indexOf("start") >= 0) {
            if (!training.active) {
                // Default durationSeconds
                int durationSeconds = 5; // default 5s

                // Try to parse duration from JSON payload: look for "duration":<num>
                int pos = msg.indexOf("duration");
                if (pos >= 0) {
                    // Find ':' after duration
                    int colon = msg.indexOf(':', pos);
                    if (colon >= 0) {
                        // extract number characters after colon
                        String numStr = "";
                        for (int i = colon + 1; i < msg.length(); i++) {
                            char c = msg.charAt(i);
                            if ((c >= '0' && c <= '9') || c == '-') numStr += c;
                            else if (c == ' ' || c == '\r' || c == '\n' || c == '\t') continue;
                            else break;
                        }
                        if (numStr.length() > 0) durationSeconds = numStr.toInt();
                    }
                }

                // Clamp duration to sensible range
                if (durationSeconds < 1) durationSeconds = 1;
                if (durationSeconds > 120) durationSeconds = 120;

                training.DURATION_MS = (unsigned long)durationSeconds * 1000UL;

                training.active = true;
                training.startTime = millis();
                training.lastSample = 0;
                training.progressPercent = 0;
                training.samples.clear();
                training.samples.reserve(500);

                Serial.println("\n╔═══════════════════════════════════════╗");
                Serial.println("║   TRAINING STARTED                    ║");
                Serial.println("╚═══════════════════════════════════════╝");
                Serial.print("  Duration: "); Serial.print(durationSeconds); Serial.println(" seconds");
                Serial.println("  Please perform muscle contractions\n");

                String startedMsg = "{\"status\":\"started\",\"duration\":" + String(durationSeconds) + "}";
                mqttClient.publish(topic_train, startedMsg.c_str());
            } else {
                Serial.println("  ⚠ Training already in progress");
            }
        }
        else if (msg.indexOf("reset_threshold") >= 0) {
            thresholdLow = DEFAULT_THRESHOLD_LOW;
            thresholdHigh = DEFAULT_THRESHOLD_HIGH;
            saveThresholds();
            
            Serial.println("\n[RESET] Thresholds reset to defaults");
            Serial.print("  LOW  = ");
            Serial.println(thresholdLow);
            Serial.print("  HIGH = ");
            Serial.println(thresholdHigh);
            
            mqttClient.publish(topic_threshold_low, String(thresholdLow).c_str());
            mqttClient.publish(topic_threshold_high, String(thresholdHigh).c_str());
            
            String resp = "{\"status\":\"reset\",\"low\":" + String(thresholdLow) + 
                         ",\"high\":" + String(thresholdHigh) + "}";
            mqttClient.publish(topic_train, resp.c_str());
        }
    }
    
    // ========== OTA COMMANDS ==========
    if (topicStr == topic_ota || topicStr == topic_device_ota) {
        Serial.println("\n[OTA] ========================================");
        Serial.println("[OTA] Received OTA command");
        Serial.print("[OTA] Topic: ");
        Serial.println(topic);
        Serial.print("[OTA] Payload length: ");
        Serial.println(msg.length());
        Serial.print("[OTA] Raw payload: ");
        Serial.println(msg);
        
        // Try to parse as JSON first
        msg.trim();
        
        // Ignore if it's a status message from ourselves (to avoid echo loop)
        if (msg.indexOf("\"status\":") >= 0 && 
            (msg.indexOf("\"downloading\"") >= 0 || 
             msg.indexOf("\"failed\"") >= 0 || 
             msg.indexOf("\"success\"") >= 0)) {
            Serial.println("[OTA] Ignoring status message (likely echo)");
            return;
        }
        
        // Check if it's a direct URL (legacy format)
        if (msg.startsWith("http://") || msg.startsWith("https://")) {
            Serial.println("[OTA] Legacy format: direct URL");
            performOtaUpdate(msg);
            return;
        }
        
        // Try JSON format: {"url": "https://..."} or {"url":"https://..."}
        // Look for "url" key (case insensitive, with or without quotes)
        String url = "";
        bool found = false;
        
        // Method 1: Look for "url" with double quotes
        int urlKeyPos = msg.indexOf("\"url\"");
        if (urlKeyPos == -1) {
            // Method 2: Look for 'url' with single quotes
            urlKeyPos = msg.indexOf("'url'");
        }
        if (urlKeyPos == -1) {
            // Method 3: Look for url: (without quotes, case insensitive)
            String lowerMsg = msg;
            lowerMsg.toLowerCase();
            int urlPos = lowerMsg.indexOf("url");
            if (urlPos != -1) {
                // Found "url" somewhere, try to extract
                urlKeyPos = urlPos;
            }
        }
        
        if (urlKeyPos != -1) {
            Serial.print("[OTA] Found 'url' key at position: ");
            Serial.println(urlKeyPos);
            
            // Find the colon after "url"
            int colonPos = msg.indexOf(":", urlKeyPos);
            if (colonPos == -1) {
                Serial.println("[OTA ERROR] No colon after url key");
                mqttClient.publish(topic_ota, "{\"status\":\"failed\",\"error\":\"malformed_json\",\"detail\":\"no_colon\"}");
                return;
            }
            
            Serial.print("[OTA] Found colon at position: ");
            Serial.println(colonPos);
            
            // Skip whitespace after colon
            int valueStart = colonPos + 1;
            while (valueStart < msg.length() && (msg[valueStart] == ' ' || msg[valueStart] == '\t' || msg[valueStart] == '\n' || msg[valueStart] == '\r')) {
                valueStart++;
            }
            
            if (valueStart >= msg.length()) {
                Serial.println("[OTA ERROR] No value after colon");
                mqttClient.publish(topic_ota, "{\"status\":\"failed\",\"error\":\"malformed_json\",\"detail\":\"no_value\"}");
                return;
            }
            
            // Check if value is quoted
            char quoteChar = 0;
            if (msg[valueStart] == '"') {
                quoteChar = '"';
            } else if (msg[valueStart] == '\'') {
                quoteChar = '\'';
            } else {
                // Not quoted, try to extract until comma or }
                int urlEnd = valueStart;
                while (urlEnd < msg.length() && msg[urlEnd] != ',' && msg[urlEnd] != '}' && msg[urlEnd] != ' ' && msg[urlEnd] != '\t' && msg[urlEnd] != '\n') {
                    urlEnd++;
                }
                url = msg.substring(valueStart, urlEnd);
                url.trim();
                found = true;
                Serial.println("[OTA] Extracted unquoted URL");
            }
            
            if (!found && quoteChar != 0) {
                int urlStart = valueStart + 1;
                
                // Find closing quote (handle escaped quotes)
                int urlEnd = urlStart;
                while (urlEnd < msg.length()) {
                    if (msg[urlEnd] == quoteChar) {
                        // Check if it's escaped
                        if (urlEnd == urlStart || msg[urlEnd - 1] != '\\') {
                            break;
                        }
                    }
                    urlEnd++;
                }
                
                if (urlEnd >= msg.length()) {
                    Serial.println("[OTA ERROR] Unclosed URL string");
                    mqttClient.publish(topic_ota, "{\"status\":\"failed\",\"error\":\"unclosed_string\"}");
                    return;
                }
                
                url = msg.substring(urlStart, urlEnd);
                found = true;
                Serial.println("[OTA] Extracted quoted URL");
            }
            
            if (found) {
                url.trim();
                
                // Remove escape characters
                url.replace("\\\"", "\"");
                url.replace("\\'", "'");
                url.replace("\\\\", "\\");
                
                Serial.print("[OTA] Extracted URL length: ");
                Serial.println(url.length());
                Serial.print("[OTA] Extracted URL: ");
                Serial.println(url);
                
                if (url.length() == 0) {
                    Serial.println("[OTA ERROR] Empty URL after extraction");
                    mqttClient.publish(topic_ota, "{\"status\":\"failed\",\"error\":\"empty_url\"}");
                    return;
                }
                
                performOtaUpdate(url);
                return;
            }
        }
        
        // If we get here, couldn't parse
        Serial.println("[OTA ERROR] Could not parse payload");
        Serial.print("[OTA] Payload preview (first 200 chars): ");
        String preview = msg.substring(0, min(200, (int)msg.length()));
        Serial.println(preview);
        
        // Try to send detailed error
        String errorMsg = "{\"status\":\"failed\",\"error\":\"invalid_payload\",\"length\":" + String(msg.length()) + ",\"preview\":\"";
        // Escape quotes in preview
        String safePreview = preview;
        safePreview.replace("\"", "\\\"");
        safePreview.replace("\n", "\\n");
        safePreview.replace("\r", "\\r");
        errorMsg += safePreview.substring(0, 100);
        errorMsg += "\"}";
        mqttClient.publish(topic_ota, errorMsg.c_str());
    }
}
// ============================================================================
// OTA UPDATE - ENHANCED VERSION WITH 5 CRITICAL FIXES
// ============================================================================

void performOtaUpdate(String url) {
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║   OTA UPDATE - STREAM TIMEOUT FIX     ║");
    Serial.println("╚═══════════════════════════════════════╝");
    Serial.println("  URL: " + url);
    
    // Validation
    if (!url.startsWith("https://")) {
        Serial.println("  ✗ ERROR: URL must use HTTPS");
        mqttClient.publish(topic_ota, "{\"status\":\"failed\",\"error\":\"https_required\"}");
        setOtaLed(CRGB::Red);
        delay(2000);
        setOtaLed(CRGB::Black);
        return;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("  ✗ ERROR: WiFi not connected");
        mqttClient.publish(topic_ota, "{\"status\":\"failed\",\"error\":\"no_wifi\"}");
        setOtaLed(CRGB::Red);
        delay(2000);
        setOtaLed(CRGB::Black);
        return;
    }
    
    // ========== CRITICAL FIX 1: AGGRESSIVE TIMEOUT SETTINGS ==========
    Serial.println("\n[FIX 1] Configuring aggressive timeouts...");
    
    // Disconnect MQTT
    if (mqttClient.connected()) {
        String startMsg = "{\"status\":\"starting\",\"url\":\"" + url + "\"}";
        mqttClient.publish(topic_ota, startMsg.c_str());
        delay(500);
        mqttClient.disconnect();
        delay(1000);
    }
    
    // Boost WiFi
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    delay(100);
    
    Serial.println("  ✓ MQTT disconnected");
    Serial.println("  ✓ WiFi sleep disabled");
    Serial.println("  ✓ WiFi auto-reconnect enabled");
    
    // ========== CRITICAL FIX 2: CUSTOM HTTP CLIENT WITH CHUNK CONTROL ==========
    Serial.println("\n[FIX 2] Creating optimized HTTP client...");
    
    WiFiClientSecure *secureClient = new WiFiClientSecure();
    if (!secureClient) {
        Serial.println("  ✗ ERROR: Cannot allocate WiFiClientSecure");
        WiFi.setSleep(true);
        reconnectMqtt();
        return;
    }
    
    // CRITICAL: Set aggressive timeouts
    secureClient->setInsecure();
    secureClient->setTimeout(OTA_CHUNK_TIMEOUT / 1000);  // 45s per chunk
    secureClient->setHandshakeTimeout(OTA_CONNECT_TIMEOUT / 1000); // 15s connect
    
    Serial.print("  ✓ Chunk timeout: ");
    Serial.print(OTA_CHUNK_TIMEOUT / 1000);
    Serial.println("s");
    Serial.print("  ✓ Connect timeout: ");
    Serial.print(OTA_CONNECT_TIMEOUT / 1000);
    Serial.println("s");
    
    // ========== CRITICAL FIX 3: PRE-FLIGHT CHECK WITH DETAILED INFO ==========
    Serial.println("\n[FIX 3] Pre-flight server check...");
    HTTPClient httpTest;
    httpTest.begin(*secureClient, url);
    httpTest.setTimeout(OTA_CONNECT_TIMEOUT);
    httpTest.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    httpTest.addHeader("User-Agent", "ESP32-OTA/2.0");
    httpTest.addHeader("Accept", "*/*");
    
    unsigned long testStart = millis();
    int testCode = httpTest.sendRequest("HEAD");
    unsigned long testDuration = millis() - testStart;
    
    Serial.print("  Test duration: ");
    Serial.print(testDuration);
    Serial.println("ms");
    
    if (testCode == HTTP_CODE_OK) {
        int contentLength = httpTest.getSize();
        Serial.print("  ✓ Server responded: ");
        Serial.println(testCode);
        
        if (contentLength > 0) {
            Serial.print("  ✓ File size: ");
            Serial.print(contentLength / 1024.0, 2);
            Serial.println(" KB");
            
            // Estimate download time (assume 20 KB/s minimum speed)
            int estimatedTime = contentLength / (20 * 1024); // seconds
            Serial.print("  ℹ Estimated time (@20KB/s): ");
            Serial.print(estimatedTime);
            Serial.println("s");
            
            if (contentLength > MAX_FIRMWARE_SIZE) {
                Serial.println("  ✗ ERROR: File too large!");
                httpTest.end();
                delete secureClient;
                WiFi.setSleep(true);
                reconnectMqtt();
                setOtaLed(CRGB::Red);
                delay(3000);
                setOtaLed(CRGB::Black);
                return;
            }
            
            // Warn if file is very large
            if (contentLength > 1024 * 1024) { // > 1MB
                Serial.println("  ⚠ WARNING: Large file (>1MB)");
                Serial.println("    This may take 60+ seconds");
            }
        } else {
            Serial.println("  ⚠ File size unknown (chunked transfer?)");
        }
        
        // Check response headers
        if (httpTest.hasHeader("Content-Type")) {
            String contentType = httpTest.header("Content-Type");
            Serial.print("  Content-Type: ");
            Serial.println(contentType);
        }
    } else {
        Serial.print("  ⚠ HEAD request failed: ");
        Serial.println(testCode);
        
        if (testCode < 0) {
            Serial.println("  ⚠ This may be a network issue");
            Serial.print("    Error: ");
            Serial.println(httpTest.errorToString(testCode));
        }
        
        Serial.println("  Will attempt download anyway...");
    }
    httpTest.end();
    
    // Check system resources
    Serial.println("\n[RESOURCES] System check:");
    Serial.print("  Free heap: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    Serial.print("  WiFi RSSI: ");
    int rssi = WiFi.RSSI();
    Serial.print(rssi);
    Serial.println(" dBm");
    
    if (rssi < -75) {
        Serial.println("  ⚠ WARNING: Weak WiFi signal");
    }
    
    setOtaLed(CRGB::Blue);
    
    // ========== CRITICAL FIX 4: RETRY WITH PROGRESSIVE TIMEOUT ==========
    Serial.println("\n[FIX 4] Starting download with progressive retry...");
    t_httpUpdate_return ret = HTTP_UPDATE_FAILED;
    int retryCount = 0;
    const int MAX_RETRIES = 3;
    unsigned long startTime = millis();
    
    // Configure HTTPUpdate with longer timeouts
    httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    httpUpdate.rebootOnUpdate(false);
    
    while (retryCount <= MAX_RETRIES) {
        if (retryCount > 0) {
            Serial.println("\n  ═══════════════════════════════════");
            Serial.print("  🔄 RETRY ");
            Serial.print(retryCount);
            Serial.print("/");
            Serial.println(MAX_RETRIES);
            Serial.println("  ═══════════════════════════════════");
            
            // Progressive backoff: 5s, 10s, 20s
            int waitTime = 5000 * (1 << (retryCount - 1)); // exponential
            Serial.print("  Waiting ");
            Serial.print(waitTime / 1000);
            Serial.println("s for network to stabilize...");
            
            for (int i = waitTime / 1000; i > 0; i--) {
                Serial.print("  ");
                Serial.print(i);
                Serial.println("...");
                delay(1000);
            }
            
            // Re-check WiFi
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("  ✗ WiFi lost, reconnecting...");
                WiFi.disconnect();
                delay(1000);
                WiFi.begin(ssid, password);
                
                unsigned long wifiStart = millis();
                while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
                    delay(500);     
                    Serial.print(".");
                }
                Serial.println();
                
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("  ✗ WiFi reconnect failed");
                    break;
                }
                Serial.println("  ✓ WiFi reconnected");
            }
            
            // Recreate secure client for clean retry
            delete secureClient;
            secureClient = new WiFiClientSecure();
            if (!secureClient) {
                Serial.println("  ✗ Cannot recreate client");
                break;
            }
            secureClient->setInsecure();
            
            // PROGRESSIVE TIMEOUT: Increase timeout with each retry
            int currentTimeout = OTA_CHUNK_TIMEOUT + (retryCount * 15000); // +15s each retry
            secureClient->setTimeout(currentTimeout / 1000);
            Serial.print("  Timeout increased to: ");
            Serial.print(currentTimeout / 1000);
            Serial.println("s");
        }
        
        Serial.print("\n  📥 Attempt ");
        Serial.print(retryCount + 1);
        Serial.print("/");
        Serial.println(MAX_RETRIES + 1);
        Serial.println("  Starting download...");
        
        unsigned long attemptStart = millis();
        
        // PERFORM UPDATE
        ret = httpUpdate.update(*secureClient, url);
        
        unsigned long attemptDuration = millis() - attemptStart;
        int errorCode = httpUpdate.getLastError();
        String errorStr = httpUpdate.getLastErrorString();
        
        Serial.print("  Duration: ");
        Serial.print(attemptDuration / 1000.0, 2);
        Serial.println("s");
        Serial.print("  Result: ");
        Serial.println(ret == HTTP_UPDATE_OK ? "✓ SUCCESS" : "✗ FAILED");
        
        if (ret == HTTP_UPDATE_OK) {
            Serial.println("\n  ✓✓✓ DOWNLOAD COMPLETE ✓✓✓");
            break;
        }
        
        Serial.print("  Error code: ");
        Serial.println(errorCode);
        Serial.print("  Error: ");
        Serial.println(errorStr);
        
        // Analyze error and decide retry strategy
        bool shouldRetry = false;
        
        switch (errorCode) {
            case 6:  // Stream read timeout
                Serial.println("  → Stream timeout detected");
                shouldRetry = true;
                break;
            case -5: // Connection lost (same as HTTPC_ERROR_CONNECTION_LOST)
                Serial.println("  → Connection lost");
                shouldRetry = true;
                break;
            case -11: // HTTP error
                Serial.println("  → HTTP protocol error");
                shouldRetry = true;
                break;
            case -1: // HTTPC_ERROR_CONNECTION_REFUSED
                Serial.println("  → Connection refused");
                shouldRetry = true;
                break;
            default:
                Serial.println("  → Non-retryable error");
                break;
        }
        
        if (shouldRetry && retryCount < MAX_RETRIES) {
            Serial.println("  ℹ Error is retryable");
            retryCount++;
            continue;
        } else {
            Serial.println("  ✗ Stopping (max retries or fatal error)");
            break;
        }
    }
    
    unsigned long totalDuration = millis() - startTime;
    
    // Cleanup
    delete secureClient;
    WiFi.setSleep(true);
    
    // Handle result
    if (ret == HTTP_UPDATE_OK) {
        Serial.println("\n╔═══════════════════════════════════════╗");
        Serial.println("║   ✓✓✓ OTA SUCCESS ✓✓✓                 ║");
        Serial.println("╚═══════════════════════════════════════╝");
        Serial.print("  Total time: ");
        Serial.print(totalDuration / 1000.0, 2);
        Serial.println("s");
        Serial.print("  Retries used: ");
        Serial.println(retryCount);
        
        setOtaLed(CRGB::Green);
        
        reconnectMqtt();
        delay(500);
        if (mqttClient.connected()) {
            String msg = "{\"status\":\"success\",\"duration\":" + String(totalDuration) + 
                        ",\"retries\":" + String(retryCount) + "}";
            mqttClient.publish(topic_ota, msg.c_str());
            delay(1000);
            mqttClient.disconnect();
        }
        
        Serial.println("\n  🔄 REBOOTING IN 3 SECONDS...");
        delay(3000);
        ESP.restart();
        
    } else {
        Serial.println("\n╔═══════════════════════════════════════╗");
        Serial.println("║   ✗✗✗ OTA FAILED ✗✗✗                  ║");
        Serial.println("╚═══════════════════════════════════════╝");
        
        String error = httpUpdate.getLastErrorString();
        int errorCode = httpUpdate.getLastError();
        
        Serial.print("  Final error: ");
        Serial.print(errorCode);
        Serial.print(" - ");
        Serial.println(error);
        Serial.print("  Total time: ");
        Serial.print(totalDuration / 1000.0, 2);
        Serial.println("s");
        Serial.print("  Retries used: ");
        Serial.println(retryCount);
        
        // Detailed diagnostics
        Serial.println("\n  DIAGNOSTICS:");
        Serial.print("  - WiFi status: ");
        Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
        Serial.print("  - Final RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.print("  - Free heap: ");
        Serial.print(ESP.getFreeHeap() / 1024);
        Serial.println(" KB");
        
        setOtaLed(CRGB::Red);
        
        reconnectMqtt();
        delay(500);
        if (mqttClient.connected()) {
            String msg = "{\"status\":\"failed\",\"error\":\"" + error + 
                        "\",\"code\":" + String(errorCode) + 
                        ",\"duration\":" + String(totalDuration) +
                        ",\"retries\":" + String(retryCount) +
                        ",\"rssi\":" + String(WiFi.RSSI()) + "}";
            mqttClient.publish(topic_ota, msg.c_str());
        }
        
        delay(3000);
        setOtaLed(CRGB::Black);
    }
}

// ============================================================================
// ADDITIONAL HELPER: Check free heap before OTA
// ============================================================================
void checkSystemResources() {
    Serial.println("\n[RESOURCES] System status:");
    Serial.print("  Free heap: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    Serial.print("  Largest block: ");
    Serial.print(ESP.getMaxAllocHeap() / 1024);
    Serial.println(" KB");
    Serial.print("  WiFi RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
}

// Call this before OTA:
// checkSystemResources();

void setOtaLed(CRGB color) {
    leds[0] = color;
    FastLED.show();
}

// ============================================================================
// NVS STORAGE
// ============================================================================
void loadThresholds() {
    preferences.begin("emg", true);
    thresholdLow = preferences.getInt("thresholdLow", DEFAULT_THRESHOLD_LOW);
    thresholdHigh = preferences.getInt("thresholdHigh", DEFAULT_THRESHOLD_HIGH);
    preferences.end();
    
    // Validate loaded thresholds - fix if invalid
    bool needsFix = false;
    if (thresholdLow < 2) {
        int safeLow = max(2, DEFAULT_THRESHOLD_LOW);
        if (thresholdLow != safeLow) {
            thresholdLow = safeLow;
            needsFix = true;
            Serial.println("  ⚠ WARNING: Loaded thresholdLow was invalid, using safe value");
        }
    }
    int minSeparation = max(5, (int)(thresholdLow * 0.3));
    if (thresholdHigh < thresholdLow + minSeparation) {
        thresholdHigh = thresholdLow + minSeparation;
        needsFix = true;
        Serial.println("  ⚠ WARNING: Loaded thresholdHigh was too close, adjusted");
    }
    if (needsFix) {
        saveThresholds(); // Save corrected values
    }
    
    Serial.println("[NVS] Loaded thresholds:");
    Serial.print("  LOW  = ");
    Serial.println(thresholdLow);
    Serial.print("  HIGH = ");
    Serial.println(thresholdHigh);
}

void saveThresholds() {
    preferences.begin("emg", false);
    preferences.putInt("thresholdLow", thresholdLow);
    preferences.putInt("thresholdHigh", thresholdHigh);
    preferences.end();
    
    Serial.println("[NVS] Saved thresholds:");
    Serial.print("  LOW  = ");
    Serial.println(thresholdLow);
    Serial.print("  HIGH = ");
    Serial.println(thresholdHigh);
}
