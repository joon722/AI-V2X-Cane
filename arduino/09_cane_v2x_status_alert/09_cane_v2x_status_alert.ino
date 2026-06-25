// Cane V2X endpoint: broadcast cane status and react to Jetson/RSU risk alerts.

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// =====================
// Feature switches
// =====================
#define USE_ACTUATOR 1
#define USE_DFPLAYER 1
#define USE_FIXED_GPS 1

// =====================
// Pins
// =====================
#define LED_PIN 2
#define BUZZER_PIN 25
#define MOTOR_PIN 26
#define MP3_RX 32
#define MP3_TX 33
#define MP3_BAUD 9600

// Active LOW buzzer, Active HIGH vibration motor.
#define BUZZER_ON LOW
#define BUZZER_OFF HIGH
#define MOTOR_ON HIGH
#define MOTOR_OFF LOW

// =====================
// V2X protocol constants
// =====================
#define V2X_MAGIC 0x56325831UL  // "V2X1"
#define V2X_VERSION 1

#define MSG_VEHICLE_STATUS 1
#define MSG_RSU_REPLY 2
#define MSG_CANE_STATUS 3
#define MSG_RISK_ALERT 4

#define NODE_VEHICLE 0x10
#define NODE_CANE 0x20
#define NODE_RSU 0x30

#define RISK_SAFE 0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER 3

#define SEND_INTERVAL_MS 100
#define BEEP_DURATION_MS 50
#define MP3_MIN_INTERVAL_MS 1500

// Demo cane position. Change to the test location before field tests.
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000

typedef struct __attribute__((packed)) v2x_status_message {
  uint32_t magic;
  uint8_t version;
  uint8_t msg_type;
  uint8_t node_type;
  uint8_t risk_level;
  uint8_t gps_valid;

  uint32_t node_id;
  float latitude;
  float longitude;
  float speed_mps;
  float heading_deg;

  uint32_t timestamp_ms;
  uint16_t seq_num;
} v2x_status_message_t;

typedef struct __attribute__((packed)) v2x_risk_message {
  uint32_t magic;
  uint8_t version;
  uint8_t msg_type;
  uint8_t node_type;
  uint8_t risk_level;
  uint8_t reserved;

  uint32_t target_id;
  uint32_t src_id;
  uint32_t timestamp_ms;
  uint16_t seq_num;
} v2x_risk_message_t;

HardwareSerial mp3Serial(1);

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
v2x_status_message_t txStatus;
v2x_risk_message_t rxRisk;

uint32_t caneId = 0;
uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t lastSendMs = 0;

uint8_t currentRisk = 255;
uint32_t lastRiskMs = 0;
uint16_t lastRiskSeq = 0;

bool beepActive = false;
uint32_t beepStartMs = 0;
uint32_t lastMp3PlayMs = 0;

uint32_t macToNodeId(const uint8_t *mac) {
  return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
}

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
}

void setupCaneId() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  caneId = macToNodeId(mac);
  Serial.print("[CANE] STA MAC Address: ");
  printMac(mac);
  Serial.println();
  Serial.printf("[CANE] node_id=%lu\n", (unsigned long)caneId);
}

void forceOutputsOff() {
#if USE_ACTUATOR
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
  digitalWrite(MOTOR_PIN, MOTOR_OFF);
#endif
}

void dfSendCommand(uint8_t cmd, uint16_t param) {
#if USE_DFPLAYER
  uint8_t packet[10];
  packet[0] = 0x7E;
  packet[1] = 0xFF;
  packet[2] = 0x06;
  packet[3] = cmd;
  packet[4] = 0x00;
  packet[5] = (param >> 8) & 0xFF;
  packet[6] = param & 0xFF;
  uint16_t checksum = 0 - (packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);
  packet[7] = (checksum >> 8) & 0xFF;
  packet[8] = checksum & 0xFF;
  packet[9] = 0xEF;
  mp3Serial.write(packet, 10);
#endif
}

void dfSetVolume(uint8_t volume) {
#if USE_DFPLAYER
  if (volume > 30) volume = 30;
  dfSendCommand(0x06, volume);
#endif
}

void dfPlayTrack(uint16_t trackNum) {
#if USE_DFPLAYER
  dfSendCommand(0x03, trackNum);
#endif
}

void startBeep() {
#if USE_ACTUATOR
  digitalWrite(BUZZER_PIN, BUZZER_ON);
  beepActive = true;
  beepStartMs = millis();
#endif
}

void updateBeep() {
#if USE_ACTUATOR
  if (beepActive && millis() - beepStartMs >= BEEP_DURATION_MS) {
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }
#endif
}

void applyRisk(uint8_t risk) {
  if (risk == currentRisk) return;

  Serial.printf("[CANE OUT] risk %u -> %u\n", currentRisk, risk);

#if USE_ACTUATOR
  if (risk == RISK_SAFE) {
    digitalWrite(MOTOR_PIN, MOTOR_OFF);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  } else {
    digitalWrite(MOTOR_PIN, MOTOR_ON);
    startBeep();
  }
#endif

#if USE_DFPLAYER
  if (risk > RISK_SAFE && millis() - lastMp3PlayMs > MP3_MIN_INTERVAL_MS) {
    dfPlayTrack(risk);  // 0001.mp3, 0002.mp3, 0003.mp3
    lastMp3PlayMs = millis();
  }
#endif

  currentRisk = risk;
}

void buildStatusPacket() {
  memset(&txStatus, 0, sizeof(txStatus));
  txStatus.magic = V2X_MAGIC;
  txStatus.version = V2X_VERSION;
  txStatus.msg_type = MSG_CANE_STATUS;
  txStatus.node_type = NODE_CANE;
  txStatus.risk_level = currentRisk == 255 ? RISK_SAFE : currentRisk;
  txStatus.gps_valid = USE_FIXED_GPS ? 1 : 0;
  txStatus.node_id = caneId;
  txStatus.latitude = USE_FIXED_GPS ? CANE_FIXED_LAT : 0.0f;
  txStatus.longitude = USE_FIXED_GPS ? CANE_FIXED_LNG : 0.0f;
  txStatus.speed_mps = 0.0f;
  txStatus.heading_deg = 0.0f;
  txStatus.timestamp_ms = millis();
  txStatus.seq_num = seq++;
}

void sendCaneStatus() {
  buildStatusPacket();
  esp_err_t result = esp_now_send(broadcastMAC, (uint8_t *)&txStatus, sizeof(txStatus));
  sendCount++;

  Serial.printf(
    "[CANE SEND] id=%lu seq=%u lat=%.6f lng=%.6f risk=%u send=%lu result=%s\n",
    (unsigned long)txStatus.node_id,
    txStatus.seq_num,
    txStatus.latitude,
    txStatus.longitude,
    txStatus.risk_level,
    (unsigned long)sendCount,
    result == ESP_OK ? "OK" : "ERR"
  );
}

bool isForThisCane(uint32_t targetId) {
  return targetId == 0 || targetId == caneId || targetId == 0xFFFFFFFFUL;
}

void handleRiskMessage(const v2x_risk_message_t &riskMsg) {
  if (!isForThisCane(riskMsg.target_id)) {
    Serial.printf("[CANE RX] risk for other target=%lu\n", (unsigned long)riskMsg.target_id);
    return;
  }

  lastRiskSeq = riskMsg.seq_num;
  lastRiskMs = millis();
  Serial.printf(
    "[CANE RX] risk=%u target=%lu src=%lu seq=%u\n",
    riskMsg.risk_level,
    (unsigned long)riskMsg.target_id,
    (unsigned long)riskMsg.src_id,
    riskMsg.seq_num
  );
  applyRisk(riskMsg.risk_level);
}

void handleLegacyReply(const v2x_status_message_t &replyMsg) {
  if (replyMsg.msg_type != MSG_RSU_REPLY) return;
  lastRiskSeq = replyMsg.seq_num;
  lastRiskMs = millis();
  Serial.printf("[CANE RX LEGACY] risk=%u seq=%u\n", replyMsg.risk_level, replyMsg.seq_num);
  applyRisk(replyMsg.risk_level);
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(v2x_risk_message_t)) {
    memcpy(&rxRisk, data, sizeof(rxRisk));
    if (rxRisk.magic == V2X_MAGIC && rxRisk.version == V2X_VERSION && rxRisk.msg_type == MSG_RISK_ALERT) {
      handleRiskMessage(rxRisk);
      return;
    }
  }

  if (len == sizeof(v2x_status_message_t)) {
    v2x_status_message_t reply;
    memcpy(&reply, data, sizeof(reply));
    if (reply.magic == V2X_MAGIC && reply.version == V2X_VERSION) {
      handleLegacyReply(reply);
      return;
    }
  }

  Serial.printf("[CANE RX] drop len=%d\n", len);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);
  setupCaneId();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed, restart");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.println("[ESP-NOW] broadcast peer added");
  } else {
    Serial.println("[ESP-NOW] broadcast peer add failed");
  }
}

void setup() {
  forceOutputsOff();
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== V2X Cane Status + Risk Alert ===");
  Serial.printf("[CONFIG] cane fixed lat=%.6f lng=%.6f\n", (float)CANE_FIXED_LAT, (float)CANE_FIXED_LNG);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if USE_DFPLAYER
  mp3Serial.begin(MP3_BAUD, SERIAL_8N1, MP3_RX, MP3_TX);
  delay(500);
  dfSetVolume(22);
  Serial.println("[DFPlayer] ready");
#endif

  setupEspNow();
  applyRisk(RISK_SAFE);
  Serial.println("[CANE] System ready");
}

void loop() {
  updateBeep();

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    digitalWrite(LED_PIN, HIGH);
    sendCaneStatus();
    digitalWrite(LED_PIN, LOW);
  }

  if (lastRiskMs > 0 && millis() - lastRiskMs < 1000) {
    digitalWrite(LED_PIN, HIGH);
  }

  delay(5);
}
