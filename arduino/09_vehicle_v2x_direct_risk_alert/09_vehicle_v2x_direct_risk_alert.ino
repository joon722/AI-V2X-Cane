// Vehicle V2X endpoint
// Broadcast vehicle GPS status, receive cane status,
// calculate simple distance/TTC risk, and send MSG_RISK_ALERT to cane.
//
// Compatible with Cane V2X endpoint code:
// - Cane sends MSG_CANE_STATUS
// - Cane receives MSG_RISK_ALERT and triggers vibration/buzzer

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <TinyGPSPlus.h>
#include <math.h>

// =====================
// Feature switches
// =====================
#define USE_GPS 1

// GPS가 안 잡힐 때도 책상 위 테스트가 되도록 차량 위치를 가상 이동시킴.
// 실제 GPS만 쓸 거면 0으로 바꾸면 됨.
#define USE_DEMO_MOVING_FALLBACK 1

// 차량이 직접 risk를 계산해서 지팡이에 보내는 모드.
// Jetson 없이 ESP 2개만 쓸 거면 1 유지.
#define VEHICLE_CALCULATES_RISK 1

// =====================
// Pins
// =====================
#define LED_PIN 2

#define GPS_RX 16   // ESP32 RX2 <- GPS TX
#define GPS_TX 17   // ESP32 TX2 -> GPS RX
#define GPS_BAUD 9600

// =====================
// V2X protocol constants
// Cane code와 반드시 동일해야 함
// =====================
#define V2X_MAGIC 0x56325831UL  // "V2X1"
#define V2X_VERSION 1

#define MSG_VEHICLE_STATUS 1
#define MSG_RSU_REPLY      2
#define MSG_CANE_STATUS    3
#define MSG_RISK_ALERT     4

#define NODE_VEHICLE 0x10
#define NODE_CANE    0x20
#define NODE_RSU     0x30

#define RISK_SAFE    0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER  3

#define SEND_INTERVAL_MS 100

// 지팡이 코드의 fallback 위치와 맞춘 데모 기준점.
// 지팡이 GPS가 안 잡히면 지팡이는 이 위치를 보냄.
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

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

v2x_status_message_t txVehicleStatus;
v2x_status_message_t latestCaneStatus;
v2x_risk_message_t txRiskAlert;

uint8_t latestCaneMAC[6] = {0};
bool hasCaneMAC = false;
bool hasLatestCane = false;
bool newCanePacket = false;

uint32_t vehicleId = 0;
uint16_t vehicleSeq = 0;
uint16_t riskSeq = 0;

uint32_t sendCount = 0;
uint32_t caneRxCount = 0;
uint32_t riskSendCount = 0;

uint32_t lastSendMs = 0;
uint32_t lastCaneRxMs = 0;

float vehicleLat = 0.0f;
float vehicleLng = 0.0f;
float vehicleSpeed = 0.0f;
float vehicleHeading = 0.0f;
uint8_t vehicleGpsValid = 0;

uint8_t lastRiskLevel = RISK_SAFE;

float prevDistanceM = -1.0f;
uint32_t prevRiskCalcMs = 0;

// =====================
// Utility
// =====================
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

void setupVehicleId() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  vehicleId = macToNodeId(mac);

  Serial.print("[VEHICLE] STA MAC Address: ");
  printMac(mac);
  Serial.println();

  Serial.printf("[VEHICLE] node_id=%lu\n", (unsigned long)vehicleId);
}

void addPeerIfNeeded(const uint8_t *mac, const char *name) {
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.printf("[ESP-NOW] %s peer added: ", name);
    printMac(mac);
    Serial.println();
  } else {
    Serial.printf("[ESP-NOW] %s peer add failed\n", name);
  }
}

// =====================
// GPS / fallback
// =====================
void setupGps() {
#if USE_GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] ready");
  Serial.println("[GPS] GPS TX -> ESP32 GPIO16 RX2");
  Serial.println("[GPS] GPS RX -> ESP32 GPIO17 TX2");
#endif
}

// GPS가 안 잡힐 때 데모용으로 차량이 지팡이 쪽으로 접근하는 것처럼 위치 생성.
// 거리 16m 근처에서 시작해서 0m 방향으로 접근 후 다시 반복.
void updateDemoMovingFallback() {
#if USE_DEMO_MOVING_FALLBACK
  const float baseLat = CANE_FIXED_LAT;
  const float baseLng = CANE_FIXED_LNG;
  const float speedMps = 1.2f;

  uint32_t cycleMs = millis() % 16000UL;
  float t = cycleMs / 1000.0f;

  float offsetM = 16.0f - speedMps * t;
  if (offsetM < 0.5f) offsetM = 16.0f;

  float latRad = baseLat * 3.14159265f / 180.0f;
  float metersPerDegLng = 111320.0f * cosf(latRad);

  vehicleLat = baseLat;
  vehicleLng = baseLng + (offsetM / metersPerDegLng);
  vehicleSpeed = speedMps;
  vehicleHeading = 270.0f;  // 서쪽, 즉 lng 감소 방향
  vehicleGpsValid = 1;
#else
  vehicleLat = 0.0f;
  vehicleLng = 0.0f;
  vehicleSpeed = 0.0f;
  vehicleHeading = 0.0f;
  vehicleGpsValid = 0;
#endif
}

void readGps() {
#if USE_GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  bool gpsOk = gps.location.isValid() && gps.location.age() < 3000;

  if (gpsOk) {
    vehicleGpsValid = 1;
    vehicleLat = gps.location.lat();
    vehicleLng = gps.location.lng();
    vehicleSpeed = gps.speed.isValid() ? gps.speed.mps() : 0.0f;
    vehicleHeading = gps.course.isValid() ? gps.course.deg() : 0.0f;
  } else {
    updateDemoMovingFallback();
  }
#else
  updateDemoMovingFallback();
#endif
}

// =====================
// Distance / risk logic
// =====================
float degToRad(float deg) {
  return deg * 3.14159265f / 180.0f;
}

float distanceMeters(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371000.0f;

  float p1 = degToRad(lat1);
  float p2 = degToRad(lat2);
  float dp = degToRad(lat2 - lat1);
  float dl = degToRad(lng2 - lng1);

  float a = sinf(dp / 2.0f) * sinf(dp / 2.0f)
          + cosf(p1) * cosf(p2) * sinf(dl / 2.0f) * sinf(dl / 2.0f);

  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return R * c;
}

uint8_t calculateRiskFromCane(
  const v2x_status_message_t &cane,
  float *outDistance,
  float *outClosingSpeed,
  float *outTtc
) {
  float d = distanceMeters(vehicleLat, vehicleLng, cane.latitude, cane.longitude);

  uint32_t now = millis();
  float closingSpeed = 0.0f;
  float ttc = 999.0f;

  if (prevDistanceM >= 0.0f && prevRiskCalcMs > 0 && now > prevRiskCalcMs) {
    float dt = (now - prevRiskCalcMs) / 1000.0f;

    // 거리가 줄어들면 접근 중
    closingSpeed = (prevDistanceM - d) / dt;

    if (closingSpeed > 0.1f) {
      ttc = d / closingSpeed;
    }
  }

  prevDistanceM = d;
  prevRiskCalcMs = now;

  *outDistance = d;
  *outClosingSpeed = closingSpeed;
  *outTtc = ttc;

  // 간단한 rule 기반 위험도
  // distance 또는 TTC 중 하나라도 위험하면 상위 risk로 올림.
  if (d < 3.0f || ttc < 1.5f) {
    return RISK_DANGER;
  } else if (d < 6.0f || ttc < 3.0f) {
    return RISK_WARNING;
  } else if (d < 10.0f || ttc < 5.0f) {
    return RISK_CAUTION;
  } else {
    return RISK_SAFE;
  }
}

// =====================
// Packet build / send
// =====================
void buildVehicleStatusPacket() {
  memset(&txVehicleStatus, 0, sizeof(txVehicleStatus));

  txVehicleStatus.magic = V2X_MAGIC;
  txVehicleStatus.version = V2X_VERSION;
  txVehicleStatus.msg_type = MSG_VEHICLE_STATUS;
  txVehicleStatus.node_type = NODE_VEHICLE;
  txVehicleStatus.risk_level = lastRiskLevel;
  txVehicleStatus.gps_valid = vehicleGpsValid;

  txVehicleStatus.node_id = vehicleId;
  txVehicleStatus.latitude = vehicleLat;
  txVehicleStatus.longitude = vehicleLng;
  txVehicleStatus.speed_mps = vehicleSpeed;
  txVehicleStatus.heading_deg = vehicleHeading;

  txVehicleStatus.timestamp_ms = millis();
  txVehicleStatus.seq_num = vehicleSeq++;
}

void sendVehicleStatus() {
  buildVehicleStatusPacket();

  esp_err_t result = esp_now_send(
    broadcastMAC,
    (uint8_t *)&txVehicleStatus,
    sizeof(txVehicleStatus)
  );

  sendCount++;

  Serial.printf(
    "[VEHICLE SEND] id=%lu seq=%u gps=%u lat=%.6f lng=%.6f spd=%.2f heading=%.1f risk=%u send=%lu result=%s\n",
    (unsigned long)txVehicleStatus.node_id,
    txVehicleStatus.seq_num,
    txVehicleStatus.gps_valid,
    txVehicleStatus.latitude,
    txVehicleStatus.longitude,
    txVehicleStatus.speed_mps,
    txVehicleStatus.heading_deg,
    txVehicleStatus.risk_level,
    (unsigned long)sendCount,
    result == ESP_OK ? "OK" : "ERR"
  );
}

void sendRiskAlertToCane(uint8_t risk, uint32_t targetCaneId) {
#if VEHICLE_CALCULATES_RISK
  if (!hasCaneMAC) return;

  memset(&txRiskAlert, 0, sizeof(txRiskAlert));

  txRiskAlert.magic = V2X_MAGIC;
  txRiskAlert.version = V2X_VERSION;
  txRiskAlert.msg_type = MSG_RISK_ALERT;
  txRiskAlert.node_type = NODE_VEHICLE;
  txRiskAlert.risk_level = risk;
  txRiskAlert.reserved = 0;

  // 특정 지팡이에게 보내기.
  // 지팡이 코드의 isForThisCane()에서 target_id == caneId이면 수신함.
  txRiskAlert.target_id = targetCaneId;
  txRiskAlert.src_id = vehicleId;
  txRiskAlert.timestamp_ms = millis();
  txRiskAlert.seq_num = riskSeq++;

  addPeerIfNeeded(latestCaneMAC, "cane");

  esp_err_t result = esp_now_send(
    latestCaneMAC,
    (uint8_t *)&txRiskAlert,
    sizeof(txRiskAlert)
  );

  riskSendCount++;

  Serial.printf(
    "[RISK TX] to_cane=%lu risk=%u seq=%u count=%lu result=%s\n",
    (unsigned long)targetCaneId,
    risk,
    txRiskAlert.seq_num,
    (unsigned long)riskSendCount,
    result == ESP_OK ? "OK" : "ERR"
  );
#endif
}

// =====================
// Receive callback
// =====================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(v2x_status_message_t)) {
    Serial.printf("[RX] drop len=%d expected=%d\n", len, sizeof(v2x_status_message_t));
    return;
  }

  v2x_status_message_t msg;
  memcpy(&msg, data, sizeof(msg));

  if (msg.magic != V2X_MAGIC || msg.version != V2X_VERSION) {
    Serial.println("[RX] invalid magic/version");
    return;
  }

  if (msg.msg_type != MSG_CANE_STATUS || msg.node_type != NODE_CANE) {
    // 차량 status나 다른 메시지는 여기서는 무시
    return;
  }

  memcpy(&latestCaneStatus, &msg, sizeof(latestCaneStatus));
  memcpy(latestCaneMAC, info->src_addr, 6);

  hasCaneMAC = true;
  hasLatestCane = true;
  newCanePacket = true;

  caneRxCount++;
  lastCaneRxMs = millis();

  Serial.print("[CANE RX] mac=");
  printMac(latestCaneMAC);
  Serial.printf(
    " id=%lu seq=%u gps=%u lat=%.6f lng=%.6f spd=%.2f cane_risk=%u rx_count=%lu\n",
    (unsigned long)latestCaneStatus.node_id,
    latestCaneStatus.seq_num,
    latestCaneStatus.gps_valid,
    latestCaneStatus.latitude,
    latestCaneStatus.longitude,
    latestCaneStatus.speed_mps,
    latestCaneStatus.risk_level,
    (unsigned long)caneRxCount
  );
}

// =====================
// Setup ESP-NOW
// =====================
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);

  Serial.println("[ESP-NOW] Vehicle start");
  setupVehicleId();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed, restart");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);

  addPeerIfNeeded(broadcastMAC, "broadcast");

  Serial.println("[ESP-NOW] Vehicle ready");
}

// =====================
// Arduino setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== V2X Vehicle Status + Cane Risk Alert ===");
  Serial.println("[MODE] Vehicle broadcasts status, receives cane status, sends MSG_RISK_ALERT");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setupGps();
  setupEspNow();

  Serial.println("[VEHICLE] System ready");
}

void loop() {
  readGps();

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();

    digitalWrite(LED_PIN, HIGH);
    sendVehicleStatus();
    digitalWrite(LED_PIN, LOW);
  }

  if (newCanePacket && hasLatestCane) {
    newCanePacket = false;

    if (vehicleGpsValid && latestCaneStatus.gps_valid) {
      float distanceM = 0.0f;
      float closingSpeed = 0.0f;
      float ttc = 999.0f;

      uint8_t risk = calculateRiskFromCane(
        latestCaneStatus,
        &distanceM,
        &closingSpeed,
        &ttc
      );

      lastRiskLevel = risk;

      Serial.printf(
        "[RISK CALC] cane_id=%lu distance=%.2fm closing=%.2fm/s ttc=%.2fs risk=%u\n",
        (unsigned long)latestCaneStatus.node_id,
        distanceM,
        closingSpeed,
        ttc,
        risk
      );

      sendRiskAlertToCane(risk, latestCaneStatus.node_id);
    } else {
      lastRiskLevel = RISK_SAFE;
      Serial.println("[RISK CALC] GPS invalid -> risk safe");
      sendRiskAlertToCane(RISK_SAFE, latestCaneStatus.node_id);
    }
  }

  // 지팡이 신호가 최근 1초 내에 들어오면 LED 유지
  if (lastCaneRxMs > 0 && millis() - lastCaneRxMs < 1000) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  }

  delay(5);
}