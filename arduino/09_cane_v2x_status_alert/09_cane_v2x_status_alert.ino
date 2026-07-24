// Cane V2X endpoint: broadcast cane GPS status and alert with vibration motor + beep buzzer.

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <ICM_20948.h>
#include <math.h>

// =====================
// Feature switches
// =====================
#define USE_ACTUATOR 1
#define USE_GPS 1
#define USE_IMU 1
#define USE_FIXED_GPS_FALLBACK 0

// =====================
// Pins
// =====================
#define LED_PIN 2
#define BUZZER_PIN 25
#define MOTOR_PIN 26

// Same GPS wiring as 01_sender_gps_imu_espnow.
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// Same I2C wiring as 01_sender_gps_imu_espnow.
#define I2C_SDA 21
#define I2C_SCL 22
#define AD0_VAL 1

// Active LOW buzzer, Active HIGH vibration motor.
#define BUZZER_ON LOW
#define BUZZER_OFF HIGH
#define MOTOR_ON HIGH
#define MOTOR_OFF LOW

// =====================
// V2X protocol constants
// 차량 노드와 버전 및 구조체가 반드시 같아야 한다.
// =====================
#define V2X_MAGIC 0x56325831UL  // "V2X1"
#define V2X_VERSION 2

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

#define SEND_INTERVAL_MS 100UL
#define BEEP_DURATION_MS 50UL

// GPS 데이터가 이 시간보다 오래되면 유효하지 않은 것으로 처리.
#define GPS_FIX_MAX_AGE_MS 3000UL

// 차량에서 직접 보내는 위험 패킷의 유효시간.
// 이 시간 동안에는 지팡이 자체 계산보다 차량 계산 결과를 우선한다.
#define DIRECT_RISK_TIMEOUT_MS 1500UL

// 차량 상태 패킷이 끊겼다고 판단할 시간.
#define VEHICLE_STATUS_TIMEOUT_MS 2000UL

// Fallback demo position. Used only while cane GPS is not fixed.
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

  // 차량에서 계산해서 전달한 위험 관련 값.
  float distance_m;
  float closing_speed_mps;
  float ttc_s;

  uint32_t timestamp_ms;
  uint16_t seq_num;
} v2x_risk_message_t;

TinyGPSPlus gps;
ICM_20948_I2C imu;
HardwareSerial gpsSerial(2);

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
v2x_status_message_t txStatus;
v2x_risk_message_t rxRisk;

uint32_t caneId = 0;
uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t lastSendMs = 0;
uint32_t vehicleRxCount = 0;
uint32_t lastVehicleRxMs = 0;

uint8_t currentRisk = 255;
uint32_t lastRiskMs = 0;
uint16_t lastRiskSeq = 0;

float lastLat = 0.0f;
float lastLng = 0.0f;
float lastSpeed = 0.0f;
float lastHeading = 0.0f;
uint8_t lastGpsValid = 0;

float lastAccelX = 0.0f;
float lastAccelY = 0.0f;
float lastAccelZ = 0.0f;

bool beepActive = false;
uint32_t beepStartMs = 0;

float prevVehicleDistanceM = -1.0f;
uint32_t prevVehicleRiskCalcMs = 0;

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

void setupGps() {
#if USE_GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] ready");
  Serial.println("[GPS] GPS TX -> ESP32 GPIO16 RX2");
  Serial.println("[GPS] GPS RX -> ESP32 GPIO17 TX2");
#endif
}

void setupImu() {
#if USE_IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  imu.begin(Wire, AD0_VAL);

  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.println("[IMU] connected");
  } else {
    Serial.print("[IMU] failed. status=");
    Serial.println(imu.statusString());
  }
#endif
}

void readGps() {
#if USE_GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  bool gpsFresh =
    gps.location.isValid() &&
    gps.location.age() < GPS_FIX_MAX_AGE_MS;

  if (gpsFresh) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    lastGpsValid = 1;

    if (gps.speed.isValid()) {
      lastSpeed = gps.speed.mps();
    } else {
      lastSpeed = 0.0f;
    }

    if (gps.course.isValid()) {
      lastHeading = gps.course.deg();
    }
  } else {
    // 예전에 한 번 잡힌 GPS 좌표를 무한정 사용하는 것을 방지.
    lastGpsValid = 0;
    lastSpeed = 0.0f;
  }
#endif
}

void readImu() {
#if USE_IMU
  if (imu.dataReady()) {
    imu.getAGMT();
    lastAccelX = (imu.accX() / 1000.0f) * 9.80665f;
    lastAccelY = (imu.accY() / 1000.0f) * 9.80665f;
    lastAccelZ = (imu.accZ() / 1000.0f) * 9.80665f;
  }
#endif
}

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

uint8_t calculateRiskFromVehicle(
  const v2x_status_message_t &vehicle,
  float *outDistance,
  float *outClosingSpeed,
  float *outTtc
) {
  float caneLat = lastGpsValid ? lastLat : (USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LAT : 0.0f);
  float caneLng = lastGpsValid ? lastLng : (USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LNG : 0.0f);
  float d = distanceMeters(caneLat, caneLng, vehicle.latitude, vehicle.longitude);

  uint32_t now = millis();
  float closingSpeed = 0.0f;
  float ttc = 999.0f;

  if (prevVehicleDistanceM >= 0.0f && prevVehicleRiskCalcMs > 0 && now > prevVehicleRiskCalcMs) {
    float dt = (now - prevVehicleRiskCalcMs) / 1000.0f;
    closingSpeed = (prevVehicleDistanceM - d) / dt;

    if (closingSpeed > 0.1f) {
      ttc = d / closingSpeed;
    }
  }

  prevVehicleDistanceM = d;
  prevVehicleRiskCalcMs = now;

  *outDistance = d;
  *outClosingSpeed = closingSpeed;
  *outTtc = ttc;

  if (d < 3.0f) {
  return RISK_DANGER;
} else if (d < 5.0f) {
  return RISK_WARNING;
} else if (d < 8.0f) {
  return RISK_CAUTION;
} else {
  return RISK_SAFE;
}
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

  currentRisk = risk;
}

void buildStatusPacket() {
  memset(&txStatus, 0, sizeof(txStatus));
  txStatus.magic = V2X_MAGIC;
  txStatus.version = V2X_VERSION;
  txStatus.msg_type = MSG_CANE_STATUS;
  txStatus.node_type = NODE_CANE;
  txStatus.risk_level = currentRisk == 255 ? RISK_SAFE : currentRisk;
  txStatus.node_id = caneId;

  if (lastGpsValid) {
    txStatus.gps_valid = 1;
    txStatus.latitude = lastLat;
    txStatus.longitude = lastLng;
    txStatus.speed_mps = lastSpeed;
    txStatus.heading_deg = lastHeading;
  } else {
    txStatus.gps_valid = USE_FIXED_GPS_FALLBACK ? 1 : 0;
    txStatus.latitude = USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LAT : 0.0f;
    txStatus.longitude = USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LNG : 0.0f;
    txStatus.speed_mps = 0.0f;
    txStatus.heading_deg = 0.0f;
  }

  txStatus.timestamp_ms = millis();
  txStatus.seq_num = seq++;
}

void sendCaneStatus() {
  buildStatusPacket();
  esp_err_t result = esp_now_send(broadcastMAC, (uint8_t *)&txStatus, sizeof(txStatus));
  sendCount++;

  Serial.printf(
    "[CANE SEND] id=%lu seq=%u gps=%u lat=%.6f lng=%.6f spd=%.2f risk=%u ax=%.2f ay=%.2f az=%.2f send=%lu result=%s\n",
    (unsigned long)txStatus.node_id,
    txStatus.seq_num,
    txStatus.gps_valid,
    txStatus.latitude,
    txStatus.longitude,
    txStatus.speed_mps,
    txStatus.risk_level,
    lastAccelX,
    lastAccelY,
    lastAccelZ,
    (unsigned long)sendCount,
    result == ESP_OK ? "OK" : "ERR"
  );
}

bool isForThisCane(uint32_t targetId) {
  return targetId == 0 || targetId == caneId || targetId == 0xFFFFFFFFUL;
}

void handleRiskMessage(const v2x_risk_message_t &riskMsg) {
  if (!isForThisCane(riskMsg.target_id)) {
    Serial.printf(
      "[CANE RX] risk for other target=%lu\n",
      (unsigned long)riskMsg.target_id
    );
    return;
  }

  if (riskMsg.risk_level > RISK_DANGER) {
    Serial.printf(
      "[CANE RX] invalid risk level=%u\n",
      riskMsg.risk_level
    );
    return;
  }

  lastRiskSeq = riskMsg.seq_num;
  lastRiskMs = millis();

  // 차량이 직접 계산한 위험정보가 들어왔으므로
  // 지팡이의 이전 자체 계산 기록은 초기화한다.
  prevVehicleDistanceM = -1.0f;
  prevVehicleRiskCalcMs = 0;

  Serial.printf(
    "[CANE RX] risk=%u distance=%.2fm closing=%.2fm/s "
    "ttc=%.2fs target=%lu src=%lu seq=%u\n",
    riskMsg.risk_level,
    riskMsg.distance_m,
    riskMsg.closing_speed_mps,
    riskMsg.ttc_s,
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

void handleVehicleStatus(const v2x_status_message_t &vehicleMsg) {
  if (vehicleMsg.msg_type != MSG_VEHICLE_STATUS ||
      vehicleMsg.node_type != NODE_VEHICLE) {
    return;
  }

  uint32_t now = millis();

  vehicleRxCount++;
  lastVehicleRxMs = now;

  // 차량이 계산한 직접 위험 패킷이 최근에 도착했다면
  // 지팡이 자체 계산으로 그 결과를 덮어쓰지 않는다.
  bool directRiskFresh =
    lastRiskMs > 0 &&
    now - lastRiskMs <= DIRECT_RISK_TIMEOUT_MS;

  if (directRiskFresh) {
    Serial.printf(
      "[CANE VEHICLE STATUS] vehicle=%lu seq=%u "
      "direct risk active -> local calculation skipped\n",
      (unsigned long)vehicleMsg.node_id,
      vehicleMsg.seq_num
    );
    return;
  }

  // 직접 위험 패킷이 없을 때만 지팡이 자체 계산을 예비용으로 수행.
  if (!vehicleMsg.gps_valid ||
      !(lastGpsValid || USE_FIXED_GPS_FALLBACK)) {
    prevVehicleDistanceM = -1.0f;
    prevVehicleRiskCalcMs = 0;

    Serial.println("[CANE RISK CALC] GPS invalid -> risk safe");
    applyRisk(RISK_SAFE);
    return;
  }

  float distanceM = 0.0f;
  float closingSpeed = 0.0f;
  float ttc = 999.0f;

  uint8_t risk = calculateRiskFromVehicle(
    vehicleMsg,
    &distanceM,
    &closingSpeed,
    &ttc
  );

  Serial.printf(
    "[CANE RISK FALLBACK] vehicle_id=%lu distance=%.2fm "
    "closing=%.2fm/s ttc=%.2fs risk=%u rx_count=%lu\n",
    (unsigned long)vehicleMsg.node_id,
    distanceM,
    closingSpeed,
    ttc,
    risk,
    (unsigned long)vehicleRxCount
  );

  applyRisk(risk);
}

void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len) {
  // magic 4바이트 + version + msg_type + node_type까지 필요.
  if (len < 7) {
    Serial.printf("[CANE RX] packet too short len=%d\n", len);
    return;
  }

  uint32_t receivedMagic = 0;
  memcpy(&receivedMagic, data, sizeof(receivedMagic));

  // packed 구조체 기준:
  // data[4] = version, data[5] = msg_type, data[6] = node_type
  uint8_t receivedVersion = data[4];
  uint8_t receivedMsgType = data[5];

  if (receivedMagic != V2X_MAGIC ||
      receivedVersion != V2X_VERSION) {
    Serial.printf(
      "[CANE RX] invalid magic/version len=%d version=%u\n",
      len,
      receivedVersion
    );
    return;
  }

  if (receivedMsgType == MSG_RISK_ALERT) {
    if (len != sizeof(v2x_risk_message_t)) {
      Serial.printf(
        "[CANE RX] invalid risk packet size len=%d expected=%u\n",
        len,
        (unsigned int)sizeof(v2x_risk_message_t)
      );
      return;
    }

    memcpy(&rxRisk, data, sizeof(rxRisk));
    handleRiskMessage(rxRisk);
    return;
  }

  if (receivedMsgType == MSG_VEHICLE_STATUS ||
      receivedMsgType == MSG_RSU_REPLY) {
    if (len != sizeof(v2x_status_message_t)) {
      Serial.printf(
        "[CANE RX] invalid status packet size len=%d expected=%u\n",
        len,
        (unsigned int)sizeof(v2x_status_message_t)
      );
      return;
    }

    v2x_status_message_t statusMsg;
    memcpy(&statusMsg, data, sizeof(statusMsg));

    if (statusMsg.msg_type == MSG_VEHICLE_STATUS &&
        statusMsg.node_type == NODE_VEHICLE) {
      handleVehicleStatus(statusMsg);
      return;
    }

    if (statusMsg.msg_type == MSG_RSU_REPLY) {
      handleLegacyReply(statusMsg);
      return;
    }
  }

  Serial.printf(
    "[CANE RX] unsupported msg_type=%u len=%d\n",
    receivedMsgType,
    len
  );
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
  Serial.println("[ACT] output = vibration motor + beep buzzer only, no DFPlayer");
  Serial.printf("[CONFIG] fallback lat=%.6f lng=%.6f\n", (float)CANE_FIXED_LAT, (float)CANE_FIXED_LNG);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  setupGps();
  setupImu();

  setupEspNow();
  applyRisk(RISK_SAFE);
  Serial.println("[CANE] System ready");
}

void loop() {
  updateBeep();
  readGps();
  readImu();

  uint32_t now = millis();

  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    digitalWrite(LED_PIN, HIGH);
    sendCaneStatus();
    digitalWrite(LED_PIN, LOW);
  }

  bool directRiskFresh =
    lastRiskMs > 0 &&
    now - lastRiskMs <= DIRECT_RISK_TIMEOUT_MS;

  bool vehicleStatusFresh =
    lastVehicleRxMs > 0 &&
    now - lastVehicleRxMs <= VEHICLE_STATUS_TIMEOUT_MS;

  // 차량의 직접 위험 패킷이 일정 시간 동안 오지 않으면
  // 지팡이 자체 거리 계산 모드로 돌아갈 수 있도록 기록 초기화.
  if (lastRiskMs > 0 && !directRiskFresh) {
    lastRiskMs = 0;
    lastRiskSeq = 0;
    prevVehicleDistanceM = -1.0f;
    prevVehicleRiskCalcMs = 0;

    Serial.println(
      "[CANE] direct risk timeout -> fallback mode"
    );

    // 차량 상태마저 끊겼다면 위험 출력을 안전 상태로 해제.
    if (!vehicleStatusFresh) {
      applyRisk(RISK_SAFE);
    }
  }

  // 직접 위험 패킷과 차량 상태 패킷이 모두 끊기면
  // 진동 모터가 계속 켜져 있지 않도록 SAFE 처리.
  if (lastVehicleRxMs > 0 && !vehicleStatusFresh) {
    lastVehicleRxMs = 0;
    prevVehicleDistanceM = -1.0f;
    prevVehicleRiskCalcMs = 0;

    if (lastRiskMs == 0) {
      Serial.println(
        "[CANE] vehicle communication timeout -> SAFE"
      );
      applyRisk(RISK_SAFE);
    }
  }

  if (directRiskFresh) {
    digitalWrite(LED_PIN, HIGH);
  }

  delay(5);
}
