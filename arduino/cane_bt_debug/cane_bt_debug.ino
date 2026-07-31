// Cane V2X endpoint: broadcast cane GPS status and alert with vibration motor + beep buzzer.

#include <WiFi.h>
#include <WiFiUdp.h>
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
#define USE_BT_DEBUG 0  // 블루투스 디버그 (뷰어/폰용). 시연/실전 때는 0

#if USE_BT_DEBUG
#include "BluetoothSerial.h"
#endif

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

#define V2X_WIFI_SSID "V2X-LOG"
#define V2X_WIFI_PASSWORD "12345678"
#define CANE_UDP_PORT 4210

// =====================
// 위험 단계별 진동/부저 패턴
// =====================

// 주의: 0.5초 진동 후 1.5초 정지.
#define CAUTION_CYCLE_MS 2000UL
#define CAUTION_MOTOR_ON_MS 500UL

// 경고: 0.4초 진동 후 0.4초 정지.
#define WARNING_CYCLE_MS 800UL
#define WARNING_MOTOR_ON_MS 400UL

// 위험: 0.15초 간격의 빠른 진동.
// 부저는 각 주기의 처음 0.1초 동안 울린다.
#define DANGER_CYCLE_MS 300UL
#define DANGER_MOTOR_ON_MS 150UL
#define DANGER_BUZZER_ON_MS 100UL

// GPS 데이터가 이 시간보다 오래되면 유효하지 않은 것으로 처리.
#define GPS_FIX_MAX_AGE_MS 3000UL

// 차량에서 직접 보내는 위험 패킷의 유효시간.
// 이 시간 동안에는 지팡이 자체 계산보다 차량 계산 결과를 우선한다.
#define DIRECT_RISK_TIMEOUT_MS 1500UL

// RSU(Jetson)에서 오는 위험 값의 유효시간.
// Jetson 전송 주기(1초 heartbeat)의 3배.
#define RSU_RISK_TIMEOUT_MS 3000UL

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

#if USE_BT_DEBUG
BluetoothSerial SerialBT;
#endif

WiFiUDP logUdp;
IPAddress udpBroadcastAddress(192, 168, 4, 255);
uint32_t lastUdpTelemetryMs = 0;

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
v2x_status_message_t txStatus;
v2x_risk_message_t rxRisk;

uint32_t caneId = 0;
uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t lastSendMs = 0;

uint32_t lastBtTelemetryMs = 0;
uint32_t lastBtCheckMs = 0;

uint32_t vehicleRxCount = 0;
uint32_t lastVehicleRxMs = 0;

uint8_t currentRisk = 255;
uint32_t lastRiskMs = 0;
uint16_t lastRiskSeq = 0;

// 출처별 위험 값. 마지막에 도착한 값이 무조건 덮어쓰지 않도록
// 차량 직접 계산과 RSU(Jetson) 값을 따로 저장했다가 max로 병합한다.
uint8_t vehicleRiskLevel = RISK_SAFE;
uint32_t vehicleRiskMs = 0;
uint8_t rsuRiskLevel = RISK_SAFE;
uint32_t rsuRiskMs = 0;

float lastLat = 0.0f;
float lastLng = 0.0f;
float lastSpeed = 0.0f;
float lastHeading = 0.0f;
uint8_t lastGpsValid = 0;

float lastAccelX = 0.0f;
float lastAccelY = 0.0f;
float lastAccelZ = 0.0f;

// 현재 위험 단계의 진동 패턴이 시작된 시간.
uint32_t riskPatternStartMs = 0;

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

void writeActuatorOutputs(bool motorOn, bool buzzerOn) {
#if USE_ACTUATOR
  digitalWrite(
    MOTOR_PIN,
    motorOn ? MOTOR_ON : MOTOR_OFF
  );

  digitalWrite(
    BUZZER_PIN,
    buzzerOn ? BUZZER_ON : BUZZER_OFF
  );
#endif
}

void forceOutputsOff() {
#if USE_ACTUATOR
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
#endif

  writeActuatorOutputs(false, false);
}

void updateActuators() {
#if USE_ACTUATOR
  uint32_t elapsedMs = millis() - riskPatternStartMs;
  bool motorOn = false;
  bool buzzerOn = false;

  if (currentRisk == RISK_CAUTION) {
    // 느린 진동: 0.5초 ON, 1.5초 OFF.
    uint32_t phaseMs = elapsedMs % CAUTION_CYCLE_MS;
    motorOn = phaseMs < CAUTION_MOTOR_ON_MS;
  }
  else if (currentRisk == RISK_WARNING) {
    // 반복 진동: 0.4초 ON, 0.4초 OFF.
    uint32_t phaseMs = elapsedMs % WARNING_CYCLE_MS;
    motorOn = phaseMs < WARNING_MOTOR_ON_MS;
  }
  else if (currentRisk == RISK_DANGER) {
    // 빠른 진동과 반복 부저.
    uint32_t phaseMs = elapsedMs % DANGER_CYCLE_MS;

    motorOn = phaseMs < DANGER_MOTOR_ON_MS;
    buzzerOn = phaseMs < DANGER_BUZZER_ON_MS;
  }

  // SAFE이거나 잘못된 위험 단계면 둘 다 false 상태로 유지.
  writeActuatorOutputs(motorOn, buzzerOn);
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
  // 알 수 없는 위험 단계는 안전 상태로 처리.
  if (risk > RISK_DANGER) {
    risk = RISK_SAFE;
  }

  if (risk == currentRisk) return;

  Serial.printf(
    "[CANE OUT] risk %u -> %u\n",
    currentRisk,
    risk
  );

  currentRisk = risk;

  // 위험 단계가 바뀌면 새 패턴을 처음부터 시작.
  riskPatternStartMs = millis();

  // 다음 loop까지 기다리지 않고 즉시 출력에 반영.
  updateActuators();
}

// 유효시간이 지나지 않은 출처들의 최댓값을 적용한다.
// 한쪽의 "안전(0)" 주장이 다른 쪽의 경고를 지우지 못하게 하는 병합 규칙.
void applyMergedRisk() {
  uint32_t now = millis();

  bool vehicleFresh =
    vehicleRiskMs > 0 &&
    now - vehicleRiskMs <= DIRECT_RISK_TIMEOUT_MS;

  bool rsuFresh =
    rsuRiskMs > 0 &&
    now - rsuRiskMs <= RSU_RISK_TIMEOUT_MS;

  // 둘 다 끊겼으면 기존 타임아웃/자체 계산 로직이 처리한다.
  if (!vehicleFresh && !rsuFresh) return;

  uint8_t merged = RISK_SAFE;
  if (vehicleFresh && vehicleRiskLevel > merged) merged = vehicleRiskLevel;
  if (rsuFresh && rsuRiskLevel > merged) merged = rsuRiskLevel;

  applyRisk(merged);
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

  // 보낸 노드 종류에 따라 출처별 슬롯에 기록한다.
  if (riskMsg.node_type == NODE_RSU) {
    rsuRiskLevel = riskMsg.risk_level;
    rsuRiskMs = millis();
  } else {
    vehicleRiskLevel = riskMsg.risk_level;
    vehicleRiskMs = millis();
  }

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

#if USE_BT_DEBUG
  // 차량이 계산해서 보낸 거리/접근속도/TTC를 뷰어 값 표에 반영.
  if (SerialBT.hasClient()) {
    SerialBT.printf("거리:%.2f\n", riskMsg.distance_m);
    SerialBT.printf("접근속도:%.2f\n", riskMsg.closing_speed_mps);
    SerialBT.printf("TTC:%.2f\n", riskMsg.ttc_s);
  }
#endif

  applyMergedRisk();
}

void handleLegacyReply(const v2x_status_message_t &replyMsg) {
  if (replyMsg.msg_type != MSG_RSU_REPLY) return;
  lastRiskSeq = replyMsg.seq_num;
  lastRiskMs = millis();

  // RSU(Jetson) 출처 슬롯에 기록한다.
  rsuRiskLevel = replyMsg.risk_level;
  rsuRiskMs = millis();

  Serial.printf("[CANE RX LEGACY] risk=%u seq=%u\n", replyMsg.risk_level, replyMsg.seq_num);
  applyMergedRisk();
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

#if USE_BT_DEBUG
  // 지팡이 자체 계산(예비) 결과도 뷰어 값 표에 반영.
  if (SerialBT.hasClient()) {
    SerialBT.printf("거리:%.2f\n", distanceM);
    SerialBT.printf("접근속도:%.2f\n", closingSpeed);
    SerialBT.printf("TTC:%.2f\n", ttc);
  }
#endif

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
  // 지팡이를 Wi-Fi 접속 모드로 설정
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  delay(100);

  Serial.printf(
    "[WIFI] connecting to %s",
    V2X_WIFI_SSID
  );

  // 차량 ESP32가 만든 V2X-LOG Wi-Fi에 접속
  WiFi.begin(
    V2X_WIFI_SSID,
    V2X_WIFI_PASSWORD
  );

  uint32_t connectStartedMs = millis();

  // 최대 20초 동안 연결 대기
  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - connectStartedMs < 20000UL
  ) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();

  // 20초 안에 연결하지 못하면 재부팅 후 다시 시도
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(
      "[WIFI] vehicle Wi-Fi connection failed"
    );
    Serial.println(
      "[WIFI] restarting ESP32"
    );

    delay(1000);
    ESP.restart();
    return;
  }

  Serial.println("[WIFI] connected to vehicle");

  Serial.print("[WIFI] IP=");
  Serial.println(WiFi.localIP());

  Serial.printf(
    "[WIFI] channel=%d\n",
    WiFi.channel()
  );

  // Wi-Fi 연결 후 지팡이 ID 생성
  setupCaneId();

  // 같은 Wi-Fi 채널에서 ESP-NOW 시작
  if (esp_now_init() != ESP_OK) {
    Serial.println(
      "[ESP-NOW] init failed, restarting"
    );

    delay(1000);
    ESP.restart();
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(
    peer.peer_addr,
    broadcastMAC,
    6
  );

  // 0으로 설정하면 현재 Wi-Fi 채널을 사용함
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.println(
      "[ESP-NOW] broadcast peer added"
    );
  } else {
    Serial.println(
      "[ESP-NOW] broadcast peer add failed"
    );
  }

  Serial.println("[ESP-NOW] ready");
}

#if USE_BT_DEBUG
// 뷰어의 "현재 값" 표에 뜨도록 "이름:값" 형식으로 상태를 전송.
// 송신 주기(100ms)에 맞춰 호출된다.
void sendBtTelemetry() {
  if (!SerialBT.hasClient()) {
    return;
  }

  char btBuffer[384];

  int written = snprintf(
    btBuffer,
    sizeof(btBuffer),
    "위험:%u\n"
    "GPS유효:%u\n"
    "위도:%.6f\n"
    "경도:%.6f\n"
    "속도:%.2f\n"
    "방향:%.1f\n"
    "가속도:%.2f,%.2f,%.2f\n"
    "송신:%lu\n"
    "차량수신:%lu\n",
    currentRisk == 255 ? 0 : currentRisk,
    lastGpsValid,
    lastLat,
    lastLng,
    lastSpeed,
    lastHeading,
    lastAccelX,
    lastAccelY,
    lastAccelZ,
    (unsigned long)sendCount,
    (unsigned long)vehicleRxCount
  );

  if (written > 0) {
    size_t sendLength =
      written < (int)sizeof(btBuffer)
        ? (size_t)written
        : sizeof(btBuffer) - 1;

    SerialBT.write(
      (const uint8_t *)btBuffer,
      sendLength
    );
  }
}
#endif

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

#if USE_BT_DEBUG
  SerialBT.begin("ESP32-Cane");
  Serial.println("[BT] debug started: ESP32-Cane");
#endif

  applyRisk(RISK_SAFE);
  Serial.println("[CANE] System ready");
}

void loop() {
  // 현재 위험 단계에 맞춰 진동과 부저 패턴을 계속 갱신.
  updateActuators();

  readGps();
  readImu();

  uint32_t now = millis();

if (now - lastSendMs >= SEND_INTERVAL_MS) {
  lastSendMs = now;

  digitalWrite(LED_PIN, HIGH);
  sendCaneStatus();
  digitalWrite(LED_PIN, LOW);
}

#if USE_BT_DEBUG
// Bluetooth 로그는 1초마다 전송
if (now - lastBtTelemetryMs >= 1000UL) {
  lastBtTelemetryMs = now;
  sendBtTelemetry();
}

// USB 시리얼 모니터에 실제 Bluetooth 연결 상태 출력
if (now - lastBtCheckMs >= 1000UL) {
  lastBtCheckMs = now;
  Serial.printf(
    "[BT CHECK] ESP32-Cane client=%d\n",
    SerialBT.hasClient() ? 1 : 0
  );
}
#endif

  // 수신 콜백이 loop 도중 타임스탬프를 갱신하면 미리 재둔 now보다
  // 미래 값이 되어 부호 없는 뺄셈이 거대한 값으로 오버플로우하고,
  // 방금 온 패킷을 타임아웃으로 오판한다. 최신 시각으로 다시 읽고
  // 부호 있는 비교로 판정한다.
  bool directRiskFresh =
    lastRiskMs > 0 &&
    (int32_t)(millis() - lastRiskMs) <= (int32_t)DIRECT_RISK_TIMEOUT_MS;

  bool vehicleStatusFresh =
    lastVehicleRxMs > 0 &&
    (int32_t)(millis() - lastVehicleRxMs) <= (int32_t)VEHICLE_STATUS_TIMEOUT_MS;

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
