#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

const char* ssid = "Bach Long";
const char* password = "03030380";

const char* mqtt_server = "broker.hivemq.com"; 
const int mqtt_port = 1883;
 
const char* topic_mode = "hand/mode";           // Topic để chuyển chế độ
const char* topic_emg = "servo/angle";             // Topic cho điều khiển EMG
const char* topic_manual = "hand/manual";       // Topic cho điều khiển Manual
const char* topic_status = "hand/status";       // Topic để gửi trạng thái

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  120 
#define SERVOMAX  520

#define CH_THUMB     0  // Ngón cái
#define CH_INDEX     1  // Ngón trỏ
#define CH_MIDDLE    2  // Ngón giữa
#define CH_RING      3  // Ngón áp út
#define CH_PINKY     4  // Ngón út

WiFiClient espClient;
PubSubClient client(espClient);

QueueHandle_t queueThumb, queueIndex, queueMiddle, queueRing, queuePinky;
SemaphoreHandle_t i2cMutex;

enum ControlMode {
  MODE_EMG,      // Chế độ EMG
  MODE_MANUAL    // Chế độ Manual
};

ControlMode currentMode = MODE_EMG; 
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void setAngle(uint8_t channel, int angle);
void controlHand(int t, int i, int m, int r, int p);
void publishStatus();
void processEMGCommand(String message);
void processManualCommand(String message);

void taskThumb(void *pvParameters);
void taskIndex(void *pvParameters);
void taskMiddle(void *pvParameters);
void taskRing(void *pvParameters);
void taskPinky(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  
  queueThumb = xQueueCreate(5, sizeof(int));
  queueIndex = xQueueCreate(5, sizeof(int));
  queueMiddle = xQueueCreate(5, sizeof(int));
  queueRing = xQueueCreate(5, sizeof(int));
  queuePinky = xQueueCreate(5, sizeof(int));
  i2cMutex = xSemaphoreCreateMutex();
  
  if (queueThumb == NULL || queueIndex == NULL || queueMiddle == NULL || 
      queueRing == NULL || queuePinky == NULL || i2cMutex == NULL) {
    Serial.println("LỖI: Không thể tạo queues/semaphores!");
    while(1) delay(10);
  }
  
  if (!pwm.begin()) {
    Serial.println("LỖI: Không thể khởi tạo PCA9685!");
    while(1) delay(10);
  }
  pwm.setPWMFreq(50);
  
  xTaskCreate(taskThumb, "TaskThumb", 2048, NULL, 2, NULL);
  xTaskCreate(taskIndex, "TaskIndex", 2048, NULL, 2, NULL);
  xTaskCreate(taskMiddle, "TaskMiddle", 2048, NULL, 2, NULL);
  xTaskCreate(taskRing, "TaskRing", 2048, NULL, 2, NULL);
  xTaskCreate(taskPinky, "TaskPinky", 2048, NULL, 2, NULL);
  
  Serial.println("Đã tạo 5 RTOS tasks cho 5 ngón tay!");
  delay(1000);
  
  Serial.println("Đang test tất cả các ngón tay...");
  controlHand(90, 90, 90, 90, 90);
  delay(2000);
  Serial.println("Test hoàn tất!");
  
  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("Hệ thống đã sẵn sàng!");
  Serial.print("Chế độ mặc định: ");
  Serial.println(currentMode == MODE_EMG ? "EMG" : "MANUAL");
}

void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  Serial.print("Nhận từ topic: ");
  Serial.print(topicStr);
  Serial.print(" - Nội dung: ");
  Serial.println(message);

  if (topicStr == topic_mode) {
    if (message == "emg") {
      currentMode = MODE_EMG;
      Serial.println("Chuyển sang chế độ EMG");
      publishStatus();
    } 
    else if (message == "manual") {
      currentMode = MODE_MANUAL;
      Serial.println("Chuyển sang chế độ MANUAL");
      publishStatus();
    }
  } 
  else if (topicStr == topic_emg && currentMode == MODE_EMG) {

    processEMGCommand(message);
  } 
  else if (topicStr == topic_manual && currentMode == MODE_MANUAL) {
   
    processManualCommand(message);
  }
}


void processEMGCommand(String message) {
  Serial.print("EMG Command: ");
  Serial.println(message);

  if (message == "0") {
    Serial.println("--> EMG: MỞ bàn tay");
    delay(100);
    controlHand(180, 180, 180, 0, 0);
    delay(150);
    controlHand(30, 80, 70, 110, 110);
    delay(150);
    controlHand(180, 180, 180, 0, 0);
  } 
  else {
    Serial.println("--> EMG: NẮM bàn tay");
    delay(100);
    controlHand(30, 80, 70, 110, 110);
  }
}

void processManualCommand(String message) {
  Serial.print("Manual Command: ");
  Serial.println(message);
  
  // Ví dụ: "90:90:90:90:90"
  
  int thumb = 90, index = 90, middle = 90, ring = 90, pinky = 90;
  
  int firstColon = message.indexOf(':');
  int secondColon = message.indexOf(':', firstColon + 1);
  int thirdColon = message.indexOf(':', secondColon + 1);
  int fourthColon = message.indexOf(':', thirdColon + 1);
  
  if (firstColon != -1 && secondColon != -1 && 
      thirdColon != -1 && fourthColon != -1) {
    
    thumb = message.substring(0, firstColon).toInt();
    index = message.substring(firstColon + 1, secondColon).toInt();
    middle = message.substring(secondColon + 1, thirdColon).toInt();
    ring = message.substring(thirdColon + 1, fourthColon).toInt();
    pinky = message.substring(fourthColon + 1).toInt();
    
    Serial.println("Điều khiển từng ngón:");
    Serial.print("  Thumb: "); Serial.println(thumb);
    Serial.print("  Index: "); Serial.println(index);
    Serial.print("  Middle: "); Serial.println(middle);
    Serial.print("  Ring: "); Serial.println(ring);
    Serial.print("  Pinky: "); Serial.println(pinky);
    
    controlHand(thumb, index, middle, ring, pinky);
  } else {
    Serial.println("Lỗi: Format không đúng. Sử dụng 'T:I:M:R:P'");
  }
}

void publishStatus() {
  String status = "{\"mode\":\"";
  status += (currentMode == MODE_EMG ? "EMG" : "MANUAL");
  status += "\",\"connected\":true}";
  
  client.publish(topic_status, status.c_str());
  Serial.print("Đã gửi trạng thái: ");
  Serial.println(status);
}


void controlHand(int t, int i, int m, int r, int p) {
  t = constrain(t, 0, 180);
  i = constrain(i, 0, 180);
  m = constrain(m, 0, 180);
  r = constrain(r, 0, 180);
  p = constrain(p, 0, 180);
  
  
  xQueueSend(queueThumb, &t, 0);
  xQueueSend(queueIndex, &i, 0);
  xQueueSend(queueMiddle, &m, 0);
  xQueueSend(queueRing, &r, 0);
  xQueueSend(queuePinky, &p, 0);
}

void setAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Hand-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      
    
      client.subscribe(topic_mode);
      client.subscribe(topic_emg);
      client.subscribe(topic_manual);
      

      publishStatus();
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  vTaskDelay(15 / portTICK_PERIOD_MS);
}


void taskThumb(void *pvParameters) {
  int targetAngle = 90;
  int currentAngle = 90;
  
  while(1) {
    if (xQueueReceive(queueThumb, &targetAngle, 0) == pdTRUE) {
     
    }
    
    if (currentAngle != targetAngle) {
      if (currentAngle < targetAngle) {
        currentAngle++;
      } else {
        currentAngle--;
      }
      
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        setAngle(CH_THUMB, currentAngle);
        xSemaphoreGive(i2cMutex);
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void taskIndex(void *pvParameters) {
  int targetAngle = 90;
  int currentAngle = 90;
  
  while(1) {
    if (xQueueReceive(queueIndex, &targetAngle, 0) == pdTRUE) {
      // Có lệnh mới
    }
    
    if (currentAngle != targetAngle) {
      if (currentAngle < targetAngle) {
        currentAngle++;
      } else {
        currentAngle--;
      }
      
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        setAngle(CH_INDEX, currentAngle);
        xSemaphoreGive(i2cMutex);
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void taskMiddle(void *pvParameters) {
  int targetAngle = 90;
  int currentAngle = 90;
  
  while(1) {
    if (xQueueReceive(queueMiddle, &targetAngle, 0) == pdTRUE) {
      // Có lệnh mới
    }
    
    if (currentAngle != targetAngle) {
      if (currentAngle < targetAngle) {
        currentAngle++;
      } else {
        currentAngle--;
      }
      
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        setAngle(CH_MIDDLE, currentAngle);
        xSemaphoreGive(i2cMutex);
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void taskRing(void *pvParameters) {
  int targetAngle = 90;
  int currentAngle = 90;
  
  while(1) {
    if (xQueueReceive(queueRing, &targetAngle, 0) == pdTRUE) {
     
    }
    
    if (currentAngle != targetAngle) {
      if (currentAngle < targetAngle) {
        currentAngle++;
      } else {
        currentAngle--;
      }
      
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        setAngle(CH_RING, currentAngle);
        xSemaphoreGive(i2cMutex);
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void taskPinky(void *pvParameters) {
  int targetAngle = 90;
  int currentAngle = 90;
  
  while(1) {
    if (xQueueReceive(queuePinky, &targetAngle, 0) == pdTRUE) {
    
    }
    
    if (currentAngle != targetAngle) {
      if (currentAngle < targetAngle) {
        currentAngle++;
      } else {
        currentAngle--;
      }
      
      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        setAngle(CH_PINKY, currentAngle);
        xSemaphoreGive(i2cMutex);
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}
