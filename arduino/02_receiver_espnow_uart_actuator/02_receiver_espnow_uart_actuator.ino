// [강현준/박중선] V2X Smart Cane Receiver
// ESP-NOW 수신 + 진동모터/부저 제어 + Jetson UART 전달

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// =====================
// 기능 ON/OFF
// =====================
#define USE_ACTUATOR 1
#define USE_JETSON_UART 1

// =====================
// 핀 설정
// =====================
#define LED_PIN 2

// 네가 연결한 액추에이터 핀
#define BUZZER_PIN 25
#define MOTOR_PIN 26

// Jetson UART 핀
#define JETSON_RX 16   // ESP32 RX2 <- Jetson TX
#define JETSON_TX 17   // ESP32 TX2 -> Jetson RX
#define JETSON_BAUD 115200

// =====================
// 액추에이터 동작 방식
// =====================
// 네 부저 모듈은 Active LOW
#define BUZZER_ON  LOW
#define BUZZER_OFF HIGH

// 진동모터 모듈은 일단 Active HIGH로 가정
#define MOTOR_ON  HIGH
#define MOTOR_OFF LOW

// =====================
// risk level 정의
// =====================
#define NODE_CANE 0x01

#define RISK_SAFE    0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER  3

// 송신기 MAC 주소
uint8_t senderMAC[] = {0x30, 0x76, 0xF5, 0xE7, 0x51, 0x28};

// =====================
// 송신기/수신기 공통 구조체
// 송신기 코드와 100% 똑같아야 함
// =====================
typedef struct __attribute__((packed)) v2x_message {
  uint8_t node_type;
  uint8_t risk_level;
  uint8_t gps_valid;

  float latitude;
  float longitude;
  float speed_mps;
  float heading_deg;

  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  uint32_t timestamp_ms;
  uint16_t seq_num;
} v2x_message_t;

// =====================
// 전역 변수
// =====================
HardwareSerial jetsonSerial(2);

v2x_message_t rx;
v2x_message_t replyPacket;

uint32_t recvCount = 0;
uint32_t lostCount = 0;
uint16_t prevSeq = 0;
uint32_t lastRxMs = 0;

uint8_t currentRisk = 255;

// 부저를 callback 안에서 delay로 막지 않기 위한 변수
bool beepActive = false;
uint32_t beepStartMs = 0;
const uint32_t BEEP_DURATION_MS = 30;

// =====================
// MAC 주소 출력
// =====================
void printReceiverMac() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);

  Serial.print("[RX] STA MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

// =====================
// 부저 짧게 울리기 시작
// =====================
void startShortBeep() {
#if USE_ACTUATOR
  digitalWrite(BUZZER_PIN, BUZZER_ON);
  beepActive = true;
  beepStartMs = millis();
#endif
}

// =====================
// 부저 자동으로 끄기
// loop에서 계속 호출
// =====================
void updateBeep() {
#if USE_ACTUATOR
  if (beepActive && millis() - beepStartMs >= BEEP_DURATION_MS) {
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }
#endif
}

// =====================
// 액추에이터 초기화
// =====================
void setupActuator() {
#if USE_ACTUATOR
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  // 전원 들어오자마자 부저 안 울리게
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
  digitalWrite(MOTOR_PIN, MOTOR_OFF);

  Serial.println("[ACT] Buzzer/Motor ready");
#endif
}

// =====================
// risk_level에 따른 동작
// =====================
void applyRisk(uint8_t risk) {
#if USE_ACTUATOR
  if (risk == RISK_SAFE) {
    // 안전: 전부 OFF
    digitalWrite(MOTOR_PIN, MOTOR_OFF);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }

  else if (risk == RISK_CAUTION) {
    // 주의: 진동만
    digitalWrite(MOTOR_PIN, MOTOR_ON);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }

  else if (risk == RISK_WARNING) {
    // 경고: 진동 + 짧은 삑
    digitalWrite(MOTOR_PIN, MOTOR_ON);

    // risk가 바뀌었을 때만 삑
    if (risk != currentRisk) {
      startShortBeep();
    }
  }

  else {
    // 위험: 진동 + 짧은 삑
    digitalWrite(MOTOR_PIN, MOTOR_ON);

    // risk가 바뀌었을 때만 삑
    if (risk != currentRisk) {
      startShortBeep();
    }
  }

  currentRisk = risk;
#endif
}

// =====================
// Jetson으로 JSON 한 줄 보내기
// =====================
void sendJsonToJetson(const v2x_message_t &m) {
#if USE_JETSON_UART
  jetsonSerial.printf(
    "{\"seq\":%u,\"node\":%u,\"risk\":%u,\"gps_valid\":%u,"
    "\"lat\":%.6f,\"lng\":%.6f,\"speed\":%.3f,\"heading\":%.2f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
    "\"tx_ms\":%lu,\"rx_ms\":%lu}\n",
    m.seq_num,
    m.node_type,
    m.risk_level,
    m.gps_valid,
    m.latitude,
    m.longitude,
    m.speed_mps,
    m.heading_deg,
    m.accel_x,
    m.accel_y,
    m.accel_z,
    m.gyro_x,
    m.gyro_y,
    m.gyro_z,
    (unsigned long)m.timestamp_ms,
    (unsigned long)millis()
  );
#endif
}
void sendReplyToSender(const v2x_message_t &m) {
  replyPacket = m;
  replyPacket.node_type = 0x02;  // receiver 표시용
  replyPacket.timestamp_ms = millis();

  esp_err_t result = esp_now_send(senderMAC, (uint8_t *)&replyPacket, sizeof(replyPacket));

  if (result == ESP_OK) {
    Serial.printf("[TX BACK] seq=%u risk=%u\n", replyPacket.seq_num, replyPacket.risk_level);
  } else {
    Serial.println("[TX BACK] send function error");
  }
}

// =====================
// ESP-NOW 수신 콜백
// ESP32 Core 3.x 기준
// =====================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(v2x_message_t)) {
    Serial.printf("[RX] size mismatch. len=%d expected=%d\n", len, sizeof(v2x_message_t));
    return;
  }

  memcpy(&rx, data, sizeof(rx));

  recvCount++;

  // 패킷 유실 계산
  if (recvCount > 1) {
    uint16_t expected = prevSeq + 1;

    if (rx.seq_num != expected) {
      lostCount += (uint16_t)(rx.seq_num - expected);
    }
  }

  prevSeq = rx.seq_num;
  lastRxMs = millis();

  digitalWrite(LED_PIN, HIGH);

  Serial.printf(
    "[RX] seq=%u node=%u risk=%u gps=%u lat=%.6f lng=%.6f spd=%.2f "
    "ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f recv=%lu lost=%lu size=%d\n",
    rx.seq_num,
    rx.node_type,
    rx.risk_level,
    rx.gps_valid,
    rx.latitude,
    rx.longitude,
    rx.speed_mps,
    rx.accel_x,
    rx.accel_y,
    rx.accel_z,
    rx.gyro_x,
    rx.gyro_y,
    rx.gyro_z,
    (unsigned long)recvCount,
    (unsigned long)lostCount,
    sizeof(rx)
  );

  sendJsonToJetson(rx);
  applyRisk(rx.risk_level);
  sendReplyToSender(rx);

  digitalWrite(LED_PIN, LOW);
}

// =====================
// ESP-NOW 수신기 초기화
// =====================
void setupEspNowReceiver() {
  WiFi.mode(WIFI_STA);
  delay(300);

  Serial.println("[ESP-NOW] Receiver Start");
  printReceiverMac();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed. restart.");
    ESP.restart();
  }

esp_now_register_recv_cb(onDataRecv);

esp_now_peer_info_t peer = {};
memcpy(peer.peer_addr, senderMAC, 6);
peer.channel = 0;
peer.encrypt = false;

if (esp_now_add_peer(&peer) == ESP_OK) {
  Serial.println("[ESP-NOW] sender peer added.");
} else {
  Serial.println("[ESP-NOW] sender peer add failed.");
}

Serial.println("[ESP-NOW] Receiver Ready");
}

// =====================
// setup
// =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== V2X Smart Cane Receiver ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if USE_JETSON_UART
  jetsonSerial.begin(JETSON_BAUD, SERIAL_8N1, JETSON_RX, JETSON_TX);
  Serial.println("[Jetson UART] ready");
  Serial.println("[Jetson UART] ESP32 GPIO17 TX2 -> Jetson RX");
  Serial.println("[Jetson UART] ESP32 GPIO16 RX2 <- Jetson TX");
#endif

  setupActuator();
  setupEspNowReceiver();

  Serial.println("[RX] System Ready");
}

// =====================
// loop
// =====================
void loop() {
  updateBeep();

  // 1초 이상 수신 없으면 내장 LED 깜빡임
  if (millis() - lastRxMs > 1000) {
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  }

  delay(5);
}
