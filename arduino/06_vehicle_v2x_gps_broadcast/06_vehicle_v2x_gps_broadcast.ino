// ESP32 V2X 차량 노드 통합 코드
// - GPS(TinyGPSPlus): 차량 위치/속도/방향 수집
// - ICM-20948: 가속도/자이로 수집 및 큰 충격 감지
// - Fermion DFPlayer Pro DFR0768 + 스피커: 위험 단계/충격 음성 안내
// - ESP-NOW: 차량 상태 송신, 지팡이 상태 수신, 거리/TTC 위험도 송신
//
// 필요한 Arduino 라이브러리
//   TinyGPSPlus
//   SparkFun ICM-20948 Arduino Library
// DFPlayer Pro는 별도 라이브러리 없이 AT 명령으로 제어함.

// USB-C로 DFPlayer Pro 내장 128MB 저장소의 루트에 넣을 음원 이름
//   /0001.mp3 : 주의
//   /0002.mp3 : 경고
//   /0003.mp3 : 위험
//   /0004.mp3 : 충격 감지

// ESP32 기본 배선
//   GPS TX  -> GPIO16 (ESP32 RX2)
//   GPS RX  -> GPIO17 (ESP32 TX2, 없어도 수신 가능)
//   IMU SDA -> GPIO21
//   IMU SCL -> GPIO22
//   DF Pro TX -> GPIO26 (ESP32 RX1)
//   DF Pro RX <- GPIO27 (ESP32 TX1)
//   스피커 한 개: DF Pro L+/L- 또는 R+/R-, 모든 모듈 GND 공통


#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <ICM_20948.h>
#include <math.h>

// =====================
// 기능 설정
// =====================
#define USE_GPS 1
#define USE_IMU 1
#define USE_DFPLAYER 1

// GPS가 안 잡히는 실내에서도 테스트할 때 1.
// 실제 도로 주행에서는 반드시 0 권장.
#define USE_DEMO_MOVING_FALLBACK 1

// Jetson/RSU 없이 차량 ESP32가 직접 위험도를 계산할 때 1.
#define VEHICLE_CALCULATES_RISK 1

// 1: 실내 책상 테스트 거리, 0: 실제 도로용 거리
#define USE_INDOOR_RISK_DISTANCE 0

// =====================
// 핀/통신 설정
// =====================
#define LED_PIN 2

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

#define IMU_SDA 21
#define IMU_SCL 22
// ICM-20948 ADR/AD0가 HIGH면 1, LOW면 0
#define IMU_AD0_VAL 1

#define DFPLAYER_RX 26  // ESP32 RX1 <- DFPlayer TX
#define DFPLAYER_TX 27  // ESP32 TX1 -> DFPlayer RX
#define DFPLAYER_BAUD 115200
#define DFPLAYER_VOLUME 25  // 0~30

// =====================
// 동작 설정
// =====================
#define SEND_INTERVAL_MS 100UL
#define CANE_TIMEOUT_MS 2000UL
#define SENSOR_LOG_INTERVAL_MS 1000UL
#define GPS_FIX_MAX_AGE_MS 3000UL

// 중력 성분을 제거한 선형가속도 크기가 이 값을 넘으면 충격으로 판단.
// 너무 민감하면 올리고, 둔하면 낮추면 됨.
#define IMPACT_THRESHOLD_MPS2 7.0f
#define IMPACT_COOLDOWN_MS 5000UL

#define TRACK_CAUTION_FILE "/0001.mp3"
#define TRACK_WARNING_FILE "/0002.mp3"
#define TRACK_DANGER_FILE "/0003.mp3"
#define TRACK_IMPACT_FILE "/0004.mp3"

// =====================
// V2X 프로토콜
// 지팡이 노드와 구조체/상수가 반드시 같아야 함.
// =====================
#define V2X_MAGIC 0x56325831UL
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

#if USE_INDOOR_RISK_DISTANCE
#define RISK_DANGER_DISTANCE_M 0.4f
#define RISK_WARNING_DISTANCE_M 0.9f
#define RISK_CAUTION_DISTANCE_M 1.8f
#define RISK_DANGER_TTC_S 0.7f
#define RISK_WARNING_TTC_S 1.5f
#define RISK_CAUTION_TTC_S 3.0f
#else
#define RISK_DANGER_DISTANCE_M 3.0f
#define RISK_WARNING_DISTANCE_M 6.0f
#define RISK_CAUTION_DISTANCE_M 12.0f
#define RISK_DANGER_TTC_S 2.0f
#define RISK_WARNING_TTC_S 4.0f
#define RISK_CAUTION_TTC_S 6.0f
#endif

// 지팡이 노드의 GPS fallback 좌표와 같아야 함.
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
HardwareSerial dfSerial(1);
TinyGPSPlus gps;
ICM_20948_I2C imu;

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

v2x_status_message_t txVehicleStatus;
v2x_status_message_t latestCaneStatus;
v2x_risk_message_t txRiskAlert;

uint8_t latestCaneMAC[6] = {0};
bool hasCaneMAC = false;
volatile bool hasLatestCane = false;
volatile bool newCanePacket = false;

uint32_t vehicleId = 0;
uint16_t vehicleSeq = 0;
uint16_t riskSeq = 0;
uint32_t sendCount = 0;
uint32_t caneRxCount = 0;
uint32_t riskSendCount = 0;
uint32_t lastSendMs = 0;
uint32_t lastCaneRxMs = 0;
uint32_t lastSensorLogMs = 0;

double vehicleLat = 0.0;
double vehicleLng = 0.0;
float vehicleSpeed = 0.0f;
float vehicleHeading = 0.0f;
uint8_t vehicleGpsValid = 0;
bool usingDemoGps = false;

bool imuReady = false;
bool imuHasSample = false;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 9.80665f;
float linearAccelMagnitude = 0.0f;
uint32_t lastImpactMs = 0;

bool dfPlayerReady = false;
uint8_t lastAnnouncedRisk = RISK_SAFE;

uint8_t lastRiskLevel = RISK_SAFE;
float prevDistanceM = -1.0f;
uint32_t prevRiskCalcMs = 0;

portMUX_TYPE caneMux = portMUX_INITIALIZER_UNLOCKED;

// =====================
// 공통 유틸리티
// =====================
uint32_t macToNodeId(const uint8_t *mac) {
  return ((uint32_t)mac[2] << 24) |
         ((uint32_t)mac[3] << 16) |
         ((uint32_t)mac[4] << 8) |
         mac[5];
}

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(':');
  }
}

void setupVehicleId() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  vehicleId = macToNodeId(mac);
  Serial.print("[VEHICLE] STA MAC=");
  printMac(mac);
  Serial.printf(" node_id=%lu\n", (unsigned long)vehicleId);
}

void addPeerIfNeeded(const uint8_t *mac, const char *name) {
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_err_t result = esp_now_add_peer(&peer);
  Serial.printf("[ESP-NOW] add %s peer: %s\n",
                name,
                result == ESP_OK ? "OK" : "FAIL");
}

// =====================
// DFPlayer Mini
// =====================
void sendDfPlayerAtCommand(const char *command) {
#if USE_DFPLAYER
  if (!dfPlayerReady) return;
  dfSerial.print(command);
  dfSerial.print("\r\n");
  dfSerial.flush();
  Serial.printf("[DFPLAYER PRO TX] %s\n", command);
#endif
}

void setupDfPlayer() {
#if USE_DFPLAYER
  dfSerial.begin(DFPLAYER_BAUD, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  delay(1000);  // DFPlayer Pro 전원 안정화
  dfPlayerReady = true;
  sendDfPlayerAtCommand("AT");
  Serial.printf("[DFPLAYER PRO] UART ready, baud=%lu\n",
                (unsigned long)DFPLAYER_BAUD);
#endif
}

void initializeDfPlayer() {
#if USE_DFPLAYER
  if (!dfPlayerReady) return;
  char volumeCommand[20];
  snprintf(volumeCommand,
           sizeof(volumeCommand),
           "AT+VOL=%u",
           (unsigned int)constrain(DFPLAYER_VOLUME, 0, 30));
  sendDfPlayerAtCommand(volumeCommand);
  sendDfPlayerAtCommand("AT+PLAYMODE=3");  // 한 곡 재생 후 정지
  sendDfPlayerAtCommand("AT+AMP=ON");      // 내장 스피커 앰프 켜기
#endif
}

void playAudioFile(const char *filePath) {
#if USE_DFPLAYER
  if (!dfPlayerReady) return;
  char playCommand[96];
  snprintf(playCommand,
           sizeof(playCommand),
           "AT+PLAYFILE=%s",
           filePath);
  sendDfPlayerAtCommand(playCommand);
#endif
}

void announceRisk(uint8_t risk) {
  if (risk == lastAnnouncedRisk) return;

  // 안전 복귀 시에는 음원을 재생하지 않고 상태만 초기화.
  if (risk == RISK_CAUTION) playAudioFile(TRACK_CAUTION_FILE);
  if (risk == RISK_WARNING) playAudioFile(TRACK_WARNING_FILE);
  if (risk == RISK_DANGER) playAudioFile(TRACK_DANGER_FILE);
  lastAnnouncedRisk = risk;
}

void updateDfPlayer() {
#if USE_DFPLAYER
  static char response[96];
  static size_t responseLength = 0;

  while (dfPlayerReady && dfSerial.available() > 0) {
    char c = (char)dfSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (responseLength > 0) {
        response[responseLength] = '\0';
        Serial.printf("[DFPLAYER PRO RX] %s\n", response);
        responseLength = 0;
      }
      continue;
    }

    if (responseLength < sizeof(response) - 1) {
      response[responseLength++] = c;
    } else {
      responseLength = 0;
    }
  }
#endif
}

// =====================
// GPS
// =====================
void setupGps() {
#if USE_GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] ready: TX->GPIO16, RX<-GPIO17");
#endif
}

void updateDemoMovingFallback() {
#if USE_DEMO_MOVING_FALLBACK
  const float speedMps = 1.2f;
  uint32_t cycleMs = millis() % 16000UL;
  float offsetM = 16.0f - speedMps * (cycleMs / 1000.0f);
  if (offsetM < 0.5f) offsetM = 16.0f;

  float latRad = CANE_FIXED_LAT * DEG_TO_RAD;
  float metersPerDegLng = 111320.0f * cosf(latRad);

  vehicleLat = CANE_FIXED_LAT;
  vehicleLng = CANE_FIXED_LNG + offsetM / metersPerDegLng;
  vehicleSpeed = speedMps;
  vehicleHeading = 270.0f;
  vehicleGpsValid = 1;
  usingDemoGps = true;
#else
  vehicleLat = 0.0;
  vehicleLng = 0.0;
  vehicleSpeed = 0.0f;
  vehicleHeading = 0.0f;
  vehicleGpsValid = 0;
  usingDemoGps = false;
#endif
}

void readGps() {
#if USE_GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  bool gpsOk = gps.location.isValid() &&
               gps.location.age() < GPS_FIX_MAX_AGE_MS;

  if (gpsOk) {
    vehicleLat = gps.location.lat();
    vehicleLng = gps.location.lng();
    vehicleSpeed = gps.speed.isValid() ? gps.speed.mps() : 0.0f;
    vehicleHeading = gps.course.isValid() ? gps.course.deg() : 0.0f;
    vehicleGpsValid = 1;
    usingDemoGps = false;
  } else {
    updateDemoMovingFallback();
  }
#else
  updateDemoMovingFallback();
#endif
}

// =====================
// ICM-20948 IMU
// =====================
void setupImu() {
#if USE_IMU
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  imu.begin(Wire, IMU_AD0_VAL);

  if (imu.status == ICM_20948_Stat_Ok) {
    imuReady = true;
    Serial.println("[IMU] ICM-20948 connected");
  } else {
    Serial.print("[IMU] 연결 실패: ");
    Serial.println(imu.statusString());
    Serial.println("[IMU] IMU_AD0_VAL을 1/0으로 바꾸고 SDA/SCL 확인");
  }
#endif
}

void readImu() {
#if USE_IMU
  if (!imuReady || !imu.dataReady()) return;

  imu.getAGMT();

  // SparkFun 라이브러리 acc 단위는 mg, gyr 단위는 dps.
  accelX = imu.accX() * 0.00980665f;
  accelY = imu.accY() * 0.00980665f;
  accelZ = imu.accZ() * 0.00980665f;
  gyroX = imu.gyrX();
  gyroY = imu.gyrY();
  gyroZ = imu.gyrZ();

  if (!imuHasSample) {
    gravityX = accelX;
    gravityY = accelY;
    gravityZ = accelZ;
    imuHasSample = true;
    return;
  }

  // 저역통과 필터로 중력 벡터를 추정하고 제거.
  const float alpha = 0.98f;
  gravityX = alpha * gravityX + (1.0f - alpha) * accelX;
  gravityY = alpha * gravityY + (1.0f - alpha) * accelY;
  gravityZ = alpha * gravityZ + (1.0f - alpha) * accelZ;

  float linearX = accelX - gravityX;
  float linearY = accelY - gravityY;
  float linearZ = accelZ - gravityZ;
  linearAccelMagnitude = sqrtf(linearX * linearX +
                               linearY * linearY +
                               linearZ * linearZ);

  uint32_t now = millis();
  bool cooldownDone = lastImpactMs == 0 || now - lastImpactMs >= IMPACT_COOLDOWN_MS;
  if (cooldownDone && linearAccelMagnitude >= IMPACT_THRESHOLD_MPS2) {
    lastImpactMs = now;
    Serial.printf("[IMU IMPACT] linear=%.2f m/s^2\n", linearAccelMagnitude);
    playAudioFile(TRACK_IMPACT_FILE);
  }
#endif
}

// =====================
// 거리/TTC 위험도
// =====================
float degToRad(float degree) {
  return degree * DEG_TO_RAD;
}

float distanceMeters(double lat1, double lng1, double lat2, double lng2) {
  const float earthRadiusM = 6371000.0f;
  float p1 = degToRad(lat1);
  float p2 = degToRad(lat2);
  float dp = degToRad(lat2 - lat1);
  float dl = degToRad(lng2 - lng1);

  float a = sinf(dp * 0.5f) * sinf(dp * 0.5f) +
            cosf(p1) * cosf(p2) * sinf(dl * 0.5f) * sinf(dl * 0.5f);
  a = constrain(a, 0.0f, 1.0f);
  return earthRadiusM * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

uint8_t calculateRiskFromCane(const v2x_status_message_t &cane,
                              float *outDistance,
                              float *outClosingSpeed,
                              float *outTtc) {
  float distanceM = distanceMeters(vehicleLat,
                                   vehicleLng,
                                   cane.latitude,
                                   cane.longitude);
  uint32_t now = millis();
  float closingSpeed = 0.0f;
  float ttc = 999.0f;

  if (prevDistanceM >= 0.0f && prevRiskCalcMs > 0 && now > prevRiskCalcMs) {
    float dt = (now - prevRiskCalcMs) / 1000.0f;
    // 비정상적으로 짧은 간격의 GPS 흔들림은 TTC에 사용하지 않음.
    if (dt >= 0.05f) {
      closingSpeed = (prevDistanceM - distanceM) / dt;
      if (closingSpeed > 0.1f) ttc = distanceM / closingSpeed;
    }
  }

  prevDistanceM = distanceM;
  prevRiskCalcMs = now;
  *outDistance = distanceM;
  *outClosingSpeed = closingSpeed;
  *outTtc = ttc;

  if (distanceM < RISK_DANGER_DISTANCE_M || ttc < RISK_DANGER_TTC_S) {
    return RISK_DANGER;
  }
  if (distanceM < RISK_WARNING_DISTANCE_M || ttc < RISK_WARNING_TTC_S) {
    return RISK_WARNING;
  }
  if (distanceM < RISK_CAUTION_DISTANCE_M || ttc < RISK_CAUTION_TTC_S) {
    return RISK_CAUTION;
  }
  return RISK_SAFE;
}

// =====================
// ESP-NOW 송수신
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
  txVehicleStatus.latitude = (float)vehicleLat;
  txVehicleStatus.longitude = (float)vehicleLng;
  txVehicleStatus.speed_mps = vehicleSpeed;
  txVehicleStatus.heading_deg = vehicleHeading;
  txVehicleStatus.timestamp_ms = millis();
  txVehicleStatus.seq_num = vehicleSeq++;
}

void sendVehicleStatus() {
  buildVehicleStatusPacket();
  esp_err_t result = esp_now_send(broadcastMAC,
                                  (uint8_t *)&txVehicleStatus,
                                  sizeof(txVehicleStatus));
  sendCount++;

  Serial.printf(
    "[VEHICLE TX] seq=%u gps=%u%s lat=%.6f lng=%.6f speed=%.2f risk=%u result=%s\n",
    txVehicleStatus.seq_num,
    txVehicleStatus.gps_valid,
    usingDemoGps ? "(DEMO)" : "",
    txVehicleStatus.latitude,
    txVehicleStatus.longitude,
    txVehicleStatus.speed_mps,
    txVehicleStatus.risk_level,
    result == ESP_OK ? "OK" : "ERR"
  );
}

void sendRiskAlertToCane(uint8_t risk,
                         uint32_t targetCaneId,
                         const uint8_t *targetMac) {
#if VEHICLE_CALCULATES_RISK
  addPeerIfNeeded(targetMac, "cane");

  memset(&txRiskAlert, 0, sizeof(txRiskAlert));
  txRiskAlert.magic = V2X_MAGIC;
  txRiskAlert.version = V2X_VERSION;
  txRiskAlert.msg_type = MSG_RISK_ALERT;
  txRiskAlert.node_type = NODE_VEHICLE;
  txRiskAlert.risk_level = risk;
  txRiskAlert.target_id = targetCaneId;
  txRiskAlert.src_id = vehicleId;
  txRiskAlert.timestamp_ms = millis();
  txRiskAlert.seq_num = riskSeq++;

  esp_err_t result = esp_now_send(targetMac,
                                  (uint8_t *)&txRiskAlert,
                                  sizeof(txRiskAlert));
  riskSendCount++;
  Serial.printf("[RISK TX] cane=%lu risk=%u seq=%u result=%s\n",
                (unsigned long)targetCaneId,
                risk,
                txRiskAlert.seq_num,
                result == ESP_OK ? "OK" : "ERR");
#endif
}

void handleRsuReply(const v2x_status_message_t &message) {
  lastRiskLevel = message.risk_level;
  announceRisk(lastRiskLevel);
  Serial.printf("[RSU RX] risk=%u seq=%u\n",
                message.risk_level,
                message.seq_num);
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(v2x_status_message_t)) return;

  v2x_status_message_t message;
  memcpy(&message, data, sizeof(message));
  if (message.magic != V2X_MAGIC || message.version != V2X_VERSION) return;

  if (message.msg_type == MSG_RSU_REPLY && message.node_type == NODE_RSU) {
    // 콜백에서는 공유값만 갱신하는 것이 가장 안전하지만, 기존 RSU 호환을 위해 처리.
    handleRsuReply(message);
    return;
  }

  if (message.msg_type != MSG_CANE_STATUS || message.node_type != NODE_CANE) return;

  portENTER_CRITICAL(&caneMux);
  memcpy(&latestCaneStatus, &message, sizeof(latestCaneStatus));
  memcpy(latestCaneMAC, info->src_addr, 6);
  hasCaneMAC = true;
  hasLatestCane = true;
  newCanePacket = true;
  lastCaneRxMs = millis();
  caneRxCount++;
  portEXIT_CRITICAL(&caneMux);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);
  setupVehicleId();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed, restarting");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  addPeerIfNeeded(broadcastMAC, "broadcast");
  Serial.println("[ESP-NOW] ready");
}

void processLatestCanePacket() {
  if (!newCanePacket || !hasLatestCane) return;

  v2x_status_message_t cane;
  uint8_t caneMac[6];

  portENTER_CRITICAL(&caneMux);
  memcpy(&cane, &latestCaneStatus, sizeof(cane));
  memcpy(caneMac, latestCaneMAC, sizeof(caneMac));
  newCanePacket = false;
  portEXIT_CRITICAL(&caneMux);

  if (!vehicleGpsValid || !cane.gps_valid) {
    lastRiskLevel = RISK_SAFE;
    announceRisk(RISK_SAFE);
    sendRiskAlertToCane(RISK_SAFE, cane.node_id, caneMac);
    Serial.println("[RISK] GPS invalid -> SAFE");
    return;
  }

  float distanceM = 0.0f;
  float closingSpeed = 0.0f;
  float ttc = 999.0f;
  uint8_t risk = calculateRiskFromCane(cane,
                                       &distanceM,
                                       &closingSpeed,
                                       &ttc);

  lastRiskLevel = risk;
  announceRisk(risk);
  sendRiskAlertToCane(risk, cane.node_id, caneMac);

  Serial.printf("[RISK] cane=%lu distance=%.2fm closing=%.2fm/s ttc=%.2fs risk=%u\n",
                (unsigned long)cane.node_id,
                distanceM,
                closingSpeed,
                ttc,
                risk);
}

void resetStaleCaneRisk() {
  if (!hasLatestCane || lastCaneRxMs == 0) return;
  if (millis() - lastCaneRxMs <= CANE_TIMEOUT_MS) return;

  portENTER_CRITICAL(&caneMux);
  hasLatestCane = false;
  newCanePacket = false;
  portEXIT_CRITICAL(&caneMux);

  lastRiskLevel = RISK_SAFE;
  announceRisk(RISK_SAFE);
  prevDistanceM = -1.0f;
  prevRiskCalcMs = 0;
  Serial.println("[CANE] timeout -> risk reset");
}

void logSensors() {
  if (millis() - lastSensorLogMs < SENSOR_LOG_INTERVAL_MS) return;
  lastSensorLogMs = millis();

  Serial.printf(
    "[SENSOR] GPS=%s lat=%.6f lng=%.6f | IMU=%s acc=(%.2f,%.2f,%.2f) gyro=(%.1f,%.1f,%.1f) linear=%.2f\n",
    vehicleGpsValid ? (usingDemoGps ? "DEMO" : "FIX") : "NO_FIX",
    vehicleLat,
    vehicleLng,
    imuHasSample ? "OK" : "WAIT",
    accelX,
    accelY,
    accelZ,
    gyroX,
    gyroY,
    gyroZ,
    linearAccelMagnitude
  );
}

// =====================
// Arduino setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=== V2X Vehicle + GPS + ICM-20948 + DFPlayer Pro ===");
  Serial.printf("[CONFIG] risk distance: %.1f / %.1f / %.1f m\n",
                (float)RISK_CAUTION_DISTANCE_M,
                (float)RISK_WARNING_DISTANCE_M,
                (float)RISK_DANGER_DISTANCE_M);

  setupGps();
  setupImu();
  setupDfPlayer();
  initializeDfPlayer();
  setupEspNow();
  Serial.println("[VEHICLE] system ready");
}

void loop() {
  readGps();
  readImu();
  updateDfPlayer();
  processLatestCanePacket();
  resetStaleCaneRisk();
  logSensors();

  uint32_t now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    sendVehicleStatus();
  }

  // 최근 지팡이 패킷이 있으면 점등, 없으면 천천히 점멸.
  if (lastCaneRxMs > 0 && now - lastCaneRxMs < 1000UL) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, (now / 500UL) % 2);
  }

  delay(5);
}
