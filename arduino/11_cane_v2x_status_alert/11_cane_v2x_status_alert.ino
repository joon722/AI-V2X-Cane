// Cane V2X endpoint
// Broadcast cane status, calculate cane-side fallback risk from vehicle status,
// receive risk alerts from vehicle ESP or RSU/Jetson,
// and output final_risk = max(cane_local_risk, jetson_risk).

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <TinyGPSPlus.h>
#include <math.h>

#define USE_ACTUATOR 1
#define USE_CANE_GPS 0
#define USE_FIXED_CANE_POS 1

#define LED_PIN 2
#define BUZZER_PIN 25
#define MOTOR_PIN 26

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

#define BUZZER_ON LOW
#define BUZZER_OFF HIGH
#define MOTOR_ON HIGH
#define MOTOR_OFF LOW

#define V2X_MAGIC 0x56325831UL
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
#define VEHICLE_STATUS_TIMEOUT_MS 1200
#define JETSON_RISK_TIMEOUT_MS 1500
#define BEEP_DURATION_MS 50
#define MAX_VEHICLES 6

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

typedef struct vehicle_track {
  bool active;
  uint8_t mac[6];
  uint32_t node_id;
  uint16_t last_seq;
  uint32_t recv_count;
  uint32_t lost_count;
  uint32_t last_rx_ms;
  float distance_m;
  float prev_distance_m;
  uint32_t prev_calc_ms;
  float closing_speed_mps;
  float ttc_s;
  uint8_t local_risk;
} vehicle_track_t;

HardwareSerial gpsSerial(2);
TinyGPSPlus caneGps;

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

vehicle_track_t vehicles[MAX_VEHICLES];
v2x_status_message_t txCaneStatus;
v2x_status_message_t rxStatus;
v2x_status_message_t rxLegacyReply;
v2x_risk_message_t rxRiskAlert;

uint32_t caneId = 0;
uint16_t caneSeq = 0;
uint32_t sendCount = 0;
uint32_t lastSendMs = 0;

float caneLat = 0.0f;
float caneLng = 0.0f;
float caneSpeed = 0.0f;
float caneHeading = 0.0f;
uint8_t caneGpsValid = 0;

uint8_t caneLocalRisk = RISK_SAFE;
uint8_t jetsonRisk = RISK_SAFE;
uint8_t finalRisk = RISK_SAFE;
uint32_t lastJetsonRiskMs = 0;
uint16_t lastJetsonRiskSeq = 0;

bool beepActive = false;
uint32_t beepStartMs = 0;

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

uint8_t clampRisk(uint8_t risk) {
  return risk > RISK_DANGER ? RISK_DANGER : risk;
}

uint8_t maxRisk(uint8_t a, uint8_t b) {
  return a > b ? a : b;
}

bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
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

int findOrCreateVehicle(const uint8_t *mac, uint32_t nodeId) {
  int freeIndex = -1;
  int oldestIndex = 0;
  uint32_t oldestMs = UINT32_MAX;

  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (vehicles[i].active && sameMac(vehicles[i].mac, mac)) return i;
    if (!vehicles[i].active && freeIndex < 0) freeIndex = i;
    if (vehicles[i].last_rx_ms < oldestMs) {
      oldestMs = vehicles[i].last_rx_ms;
      oldestIndex = i;
    }
  }

  int index = freeIndex >= 0 ? freeIndex : oldestIndex;
  memset(&vehicles[index], 0, sizeof(vehicles[index]));
  vehicles[index].active = true;
  memcpy(vehicles[index].mac, mac, 6);
  vehicles[index].node_id = nodeId;
  vehicles[index].distance_m = NAN;
  vehicles[index].prev_distance_m = -1.0f;
  vehicles[index].ttc_s = 999.0f;
  vehicles[index].local_risk = RISK_SAFE;
  return index;
}

float degToRad(float deg) {
  return deg * 0.017453292519943295f;
}

float distanceMeters(float lat1, float lng1, float lat2, float lng2) {
  const float earthRadiusM = 6371000.0f;
  float dLat = degToRad(lat2 - lat1);
  float dLng = degToRad(lng2 - lng1);
  float rLat1 = degToRad(lat1);
  float rLat2 = degToRad(lat2);

  float a = sinf(dLat / 2.0f) * sinf(dLat / 2.0f) +
            cosf(rLat1) * cosf(rLat2) * sinf(dLng / 2.0f) * sinf(dLng / 2.0f);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return earthRadiusM * c;
}

uint8_t riskFromDistanceAndTtc(float distanceM, float ttcS) {
  if (isnan(distanceM)) return RISK_SAFE;
  if (distanceM < 3.0f || ttcS < 1.5f) return RISK_DANGER;
  if (distanceM < 6.0f || ttcS < 3.0f) return RISK_WARNING;
  if (distanceM < 10.0f || ttcS < 5.0f) return RISK_CAUTION;
  return RISK_SAFE;
}

void updateCaneGps() {
#if USE_CANE_GPS
  while (gpsSerial.available()) {
    caneGps.encode(gpsSerial.read());
  }

  bool gpsOk = caneGps.location.isValid() && caneGps.location.age() < 3000;
  if (gpsOk) {
    caneGpsValid = 1;
    caneLat = caneGps.location.lat();
    caneLng = caneGps.location.lng();
    caneSpeed = caneGps.speed.isValid() ? caneGps.speed.mps() : 0.0f;
    caneHeading = caneGps.course.isValid() ? caneGps.course.deg() : 0.0f;
    return;
  }
#endif

#if USE_FIXED_CANE_POS
  caneGpsValid = 1;
  caneLat = CANE_FIXED_LAT;
  caneLng = CANE_FIXED_LNG;
  caneSpeed = 0.0f;
  caneHeading = 0.0f;
#else
  caneGpsValid = 0;
  caneLat = 0.0f;
  caneLng = 0.0f;
  caneSpeed = 0.0f;
  caneHeading = 0.0f;
#endif
}

void recalculateCaneLocalRisk() {
  uint8_t risk = RISK_SAFE;
  uint32_t now = millis();

  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (!vehicles[i].active) continue;
    if (now - vehicles[i].last_rx_ms > VEHICLE_STATUS_TIMEOUT_MS) continue;
    risk = maxRisk(risk, vehicles[i].local_risk);
  }

  caneLocalRisk = risk;
}

void updateFinalRisk() {
  uint8_t freshJetsonRisk = RISK_SAFE;

  if (lastJetsonRiskMs > 0 && millis() - lastJetsonRiskMs <= JETSON_RISK_TIMEOUT_MS) {
    freshJetsonRisk = jetsonRisk;
  }

  uint8_t nextFinal = maxRisk(caneLocalRisk, freshJetsonRisk);
  if (nextFinal == finalRisk) return;

  Serial.printf("[FINAL] cane_local=%u jetson=%u final %u -> %u\n",
                caneLocalRisk, freshJetsonRisk, finalRisk, nextFinal);
  finalRisk = nextFinal;

#if USE_ACTUATOR
  if (finalRisk == RISK_SAFE) {
    digitalWrite(MOTOR_PIN, MOTOR_OFF);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  } else if (finalRisk == RISK_CAUTION) {
    digitalWrite(MOTOR_PIN, MOTOR_OFF);
  } else {
    digitalWrite(MOTOR_PIN, MOTOR_ON);
    digitalWrite(BUZZER_PIN, BUZZER_ON);
    beepActive = true;
    beepStartMs = millis();
  }
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

void buildCaneStatus() {
  memset(&txCaneStatus, 0, sizeof(txCaneStatus));
  txCaneStatus.magic = V2X_MAGIC;
  txCaneStatus.version = V2X_VERSION;
  txCaneStatus.msg_type = MSG_CANE_STATUS;
  txCaneStatus.node_type = NODE_CANE;
  txCaneStatus.risk_level = finalRisk;
  txCaneStatus.gps_valid = caneGpsValid;
  txCaneStatus.node_id = caneId;
  txCaneStatus.latitude = caneLat;
  txCaneStatus.longitude = caneLng;
  txCaneStatus.speed_mps = caneSpeed;
  txCaneStatus.heading_deg = caneHeading;
  txCaneStatus.timestamp_ms = millis();
  txCaneStatus.seq_num = caneSeq++;
}

void sendCaneStatus() {
  buildCaneStatus();
  esp_err_t result = esp_now_send(broadcastMAC, (uint8_t *)&txCaneStatus, sizeof(txCaneStatus));
  sendCount++;
  Serial.printf(
    "[CANE SEND] id=%lu seq=%u gps=%u lat=%.6f lng=%.6f final_risk=%u cane_local=%u jetson=%u send=%lu result=%s\n",
    (unsigned long)txCaneStatus.node_id,
    txCaneStatus.seq_num,
    txCaneStatus.gps_valid,
    txCaneStatus.latitude,
    txCaneStatus.longitude,
    finalRisk,
    caneLocalRisk,
    jetsonRisk,
    (unsigned long)sendCount,
    result == ESP_OK ? "OK" : "ERR"
  );
}

bool isForThisCane(uint32_t targetId) {
  return targetId == 0 || targetId == caneId || targetId == 0xFFFFFFFFUL;
}

void handleRiskAlert(uint8_t sourceNodeType, uint8_t risk, uint32_t targetId, uint32_t srcId, uint16_t seqNum) {
  if (!isForThisCane(targetId)) {
    Serial.printf("[RISK RX] for other target=%lu risk=%u\n", (unsigned long)targetId, risk);
    return;
  }

  if (sourceNodeType == NODE_VEHICLE) {
    Serial.printf("[RISK RX] source=vehicle risk=%u target=%lu src=%lu seq=%u ignored_for_final\n",
                  clampRisk(risk), (unsigned long)targetId, (unsigned long)srcId, seqNum);
    return;
  }

  jetsonRisk = clampRisk(risk);
  lastJetsonRiskMs = millis();
  lastJetsonRiskSeq = seqNum;
  Serial.printf("[RISK RX] source=jetson_or_rsu risk=%u target=%lu src=%lu seq=%u\n",
                jetsonRisk, (unsigned long)targetId, (unsigned long)srcId, lastJetsonRiskSeq);

  updateFinalRisk();
}

void handleVehicleStatus(const esp_now_recv_info_t *info, const v2x_status_message_t &msg) {
  const uint8_t *srcMac = info->src_addr;
  addPeerIfNeeded(srcMac, "vehicle");

  int index = findOrCreateVehicle(srcMac, msg.node_id);
  vehicle_track_t &track = vehicles[index];

  if (track.recv_count > 0) {
    uint16_t expected = track.last_seq + 1;
    if (msg.seq_num != expected) {
      track.lost_count += (uint16_t)(msg.seq_num - expected);
    }
  }

  track.node_id = msg.node_id;
  track.recv_count++;
  track.last_seq = msg.seq_num;
  track.last_rx_ms = millis();

  if (msg.gps_valid && caneGpsValid) {
    track.distance_m = distanceMeters(caneLat, caneLng, msg.latitude, msg.longitude);
    track.closing_speed_mps = 0.0f;
    track.ttc_s = 999.0f;

    if (track.prev_distance_m >= 0.0f && track.prev_calc_ms > 0 && track.last_rx_ms > track.prev_calc_ms) {
      float dt = (track.last_rx_ms - track.prev_calc_ms) / 1000.0f;
      track.closing_speed_mps = (track.prev_distance_m - track.distance_m) / dt;
      if (track.closing_speed_mps > 0.1f) {
        track.ttc_s = track.distance_m / track.closing_speed_mps;
      }
    }

    track.prev_distance_m = track.distance_m;
    track.prev_calc_ms = track.last_rx_ms;
    track.local_risk = riskFromDistanceAndTtc(track.distance_m, track.ttc_s);
  } else {
    track.distance_m = NAN;
    track.closing_speed_mps = 0.0f;
    track.ttc_s = 999.0f;
    track.local_risk = RISK_SAFE;
  }

  Serial.printf(
    "[VEHICLE STATUS RX] id=%lu seq=%u gps=%u distance=%.2f closing=%.2f ttc=%.2f cane_local=%u vehicle_node_risk=%u recv=%lu lost=%lu\n",
    (unsigned long)track.node_id,
    msg.seq_num,
    msg.gps_valid,
    track.distance_m,
    track.closing_speed_mps,
    track.ttc_s,
    track.local_risk,
    msg.risk_level,
    (unsigned long)track.recv_count,
    (unsigned long)track.lost_count
  );

  recalculateCaneLocalRisk();
  updateFinalRisk();
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(v2x_risk_message_t)) {
    memcpy(&rxRiskAlert, data, sizeof(rxRiskAlert));
    if (rxRiskAlert.magic == V2X_MAGIC &&
        rxRiskAlert.version == V2X_VERSION &&
        rxRiskAlert.msg_type == MSG_RISK_ALERT) {
      handleRiskAlert(rxRiskAlert.node_type, rxRiskAlert.risk_level, rxRiskAlert.target_id, rxRiskAlert.src_id, rxRiskAlert.seq_num);
      return;
    }
  }

  if (len == sizeof(v2x_status_message_t)) {
    memcpy(&rxStatus, data, sizeof(rxStatus));
    if (rxStatus.magic != V2X_MAGIC || rxStatus.version != V2X_VERSION) {
      Serial.println("[RX] invalid magic/version");
      return;
    }

    if (rxStatus.msg_type == MSG_VEHICLE_STATUS && rxStatus.node_type == NODE_VEHICLE) {
      handleVehicleStatus(info, rxStatus);
      return;
    }

    if (rxStatus.msg_type == MSG_RSU_REPLY) {
      handleRiskAlert(NODE_RSU, rxStatus.risk_level, caneId, rxStatus.node_id, rxStatus.seq_num);
      return;
    }
  }

  Serial.printf("[RX] drop len=%d\n", len);
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
  addPeerIfNeeded(broadcastMAC, "broadcast");
  Serial.println("[ESP-NOW] Cane ready");
}

void setup() {
#if USE_ACTUATOR
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
  digitalWrite(MOTOR_PIN, MOTOR_OFF);
#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("=== V2X Cane Status + Final Risk Alert ===");
  Serial.printf("[CONFIG] fallback lat=%.6f lng=%.6f\n", (float)CANE_FIXED_LAT, (float)CANE_FIXED_LNG);

#if USE_CANE_GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] cane GPS ready");
#endif

  updateCaneGps();
  setupEspNow();
  updateFinalRisk();
}

void loop() {
  updateCaneGps();
  updateBeep();
  recalculateCaneLocalRisk();
  updateFinalRisk();

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    digitalWrite(LED_PIN, HIGH);
    sendCaneStatus();
    digitalWrite(LED_PIN, finalRisk > RISK_SAFE ? HIGH : LOW);
  }

  if (finalRisk == RISK_SAFE && sendCount == 0) {
    digitalWrite(LED_PIN, (millis() / 500) % 2);
  }

  delay(5);
}
