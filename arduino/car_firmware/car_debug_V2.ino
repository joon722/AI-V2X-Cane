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
#include <WiFiUdp.h>
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
#define USE_DEMO_MOVING_FALLBACK 0

// Jetson/RSU 없이 차량 ESP32가 직접 위험도를 계산할 때 1.
#define VEHICLE_CALCULATES_RISK 1

// 1: 실내 책상 테스트 거리, 0: 실제 도로용 거리
#define USE_INDOOR_RISK_DISTANCE 0

// 블루투스 디버그 (뷰어/폰용). 시연/실전 때는 0.
#define USE_BT_DEBUG 0

#if USE_BT_DEBUG
#include "BluetoothSerial.h"
#endif

WiFiUDP logUdp;
IPAddress udpBroadcastAddress(192, 168, 4, 255);
uint32_t lastUdpTelemetryMs = 0;

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
#define DFPLAYER_VOLUME 30  // 0~30

// =====================
// 동작 설정
// =====================

// =====================
// 차량 ESP32 자체 Wi-Fi 및 UDP 로그
// =====================
#define V2X_WIFI_CHANNEL 6
#define VEHICLE_UDP_PORT 4211

const char *V2X_AP_SSID = "V2X-LOG";
const char *V2X_AP_PASSWORD = "12345678";

#define SEND_INTERVAL_MS 100UL
// IMU 장착/스윙 분석 로그용 20Hz. 보정 완료 후 1000UL로 되돌릴 것.
#define UDP_TELEMETRY_INTERVAL_MS 20UL
#define CANE_TIMEOUT_MS 2000UL
#define SENSOR_LOG_INTERVAL_MS 1000UL
#define GPS_FIX_MAX_AGE_MS 3000UL

// ESP-NOW RSSI 평활 계수. 작을수록 순간 흔들림을 더 강하게 줄인다.
#define CANE_RSSI_FILTER_ALPHA 0.20f

// GPS 품질/이상치 필터 설정: RC카 기준.
#define GPS_MIN_SATELLITES 4
#define GPS_MAX_HDOP 3.0f
#define GPS_NODE_MAX_SPEED_MPS 6.0f
#define GPS_OUTLIER_BASE_M 4.0f
#define GPS_FILTER_ALPHA 0.50f
#define GPS_FILTER_BETA 0.08f
#define GPS_RELOCALIZE_AFTER_REJECTS 3
#define GPS_FILTER_RESET_GAP_MS 5000UL
#define GPS_PREDICTION_MAX_MS 700UL

// 정지 상태용 GPS 필터.
#define GPS_STATIONARY_SPEED_MPS 0.45f
#define GPS_STATIONARY_ALPHA 0.08f
#define GPS_STATIONARY_BETA 0.01f

// GPS 진행 방향을 신뢰하기 위한 최소 차량 속도.
#define MIN_VALID_HEADING_SPEED_MPS 0.5f

// 차량 진행 방향을 기준으로 좌우 몇 도까지 전방으로 볼 것인지 설정.
#define FORWARD_CONE_HALF_ANGLE_DEG 45.0f

// TTC 계산에 사용할 최소 접근속도와 차량 속도.
#define MIN_TTC_CLOSING_SPEED_MPS 0.3f
#define MIN_TTC_VEHICLE_SPEED_MPS 0.70f

// 위험 상승은 빠르게, 위험 해제는 천천히 확정.
#define RISK_ESCALATE_CONFIRM_COUNT 3
#define RISK_CLEAR_CONFIRM_COUNT 12

// GPS 거리 변화가 너무 짧은 주기로 계산되지 않도록 제한.
#define TTC_SAMPLE_INTERVAL_MS 500UL

// 접근속도 저역통과 필터. 클수록 부드럽지만 반응은 느려진다.
#define TTC_FILTER_ALPHA 0.70f

// 동시에 추적할 지팡이 노드 개수.
#define MAX_TRACKED_CANES 4

// 중력 성분을 제거한 선형가속도 크기가 이 값을 넘으면 충격으로 판단.
// 너무 민감하면 올리고, 둔하면 낮추면 됨.
#define IMPACT_THRESHOLD_MPS2 30.0f
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

#if USE_INDOOR_RISK_DISTANCE
#define RISK_DANGER_DISTANCE_M 0.4f
#define RISK_WARNING_DISTANCE_M 0.9f
#define RISK_CAUTION_DISTANCE_M 1.8f
#define RISK_DANGER_TTC_S 0.7f
#define RISK_WARNING_TTC_S 1.5f
#define RISK_CAUTION_TTC_S 3.0f
#else
#define RISK_DANGER_DISTANCE_M 3.0f
#define RISK_WARNING_DISTANCE_M 5.0f
#define RISK_CAUTION_DISTANCE_M 8.0f
#define RISK_DANGER_TTC_S 1.5f
#define RISK_WARNING_TTC_S 3.0f
#define RISK_CAUTION_TTC_S 5.0f
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

  float distance_m;
  float closing_speed_mps;
  float ttc_s;

  uint32_t timestamp_ms;
  uint16_t seq_num;
} v2x_risk_message_t;

HardwareSerial gpsSerial(2);
HardwareSerial dfSerial(1);
TinyGPSPlus gps;
ICM_20948_I2C imu;

#if USE_BT_DEBUG
BluetoothSerial SerialBT;
#endif

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

uint32_t lastBtTelemetryMs = 0;
uint32_t lastBtCheckMs = 0;

uint32_t lastCaneRxMs = 0;
uint32_t lastSensorLogMs = 0;

// 지팡이 -> 차량 ESP-NOW 수신 신호 세기 진단값.
// 실제 거리 보정 전에는 거리값으로 단정하지 않고 dBm 원시/평활값을 모두 기록한다.
int8_t latestCaneRssiDbm = -127;
float filteredCaneRssiDbm = -127.0f;
bool hasCaneRssi = false;
uint32_t caneRssiSampleCount = 0;
uint32_t lastCaneRssiMs = 0;

double vehicleLat = 0.0;
double vehicleLng = 0.0;
float vehicleSpeed = 0.0f;
float vehicleHeading = 0.0f;
bool vehicleHeadingValid = false;
uint8_t vehicleGpsValid = 0;
bool usingDemoGps = false;

// 원시 GPS 값과 필터 진단값. UDP 로그에서 원시/보정 좌표를 비교한다.
double rawGpsLat = 0.0;
double rawGpsLng = 0.0;
float rawGpsHdop = 99.0f;
float rawGpsSpeedMps = 0.0f;
float rawGpsCourseDeg = 0.0f;
bool rawGpsCourseValid = false;
uint32_t rawGpsSatellites = 0;
uint32_t gpsAcceptedCount = 0;
uint32_t gpsRejectedCount = 0;
uint32_t gpsQualityRejectedCount = 0;
uint32_t gpsRelocalizedCount = 0;

typedef struct {
  bool initialized;
  double originLat;
  double originLng;
  float xM;       // 동쪽 방향 위치(m)
  float yM;       // 북쪽 방향 위치(m)
  float vxMps;    // 동쪽 방향 속도(m/s)
  float vyMps;    // 북쪽 방향 속도(m/s)
  uint32_t lastFixMs;
  uint8_t consecutiveOutliers;
  bool hasRelocationCandidate;
  float relocationX;
  float relocationY;
} GpsFilterState;

GpsFilterState gpsFilter = {};

float normalizeHeading(float headingDeg) {
  while (headingDeg < 0.0f) headingDeg += 360.0f;
  while (headingDeg >= 360.0f) headingDeg -= 360.0f;
  return headingDeg;
}

float blendHeading(float previousDeg, float measuredDeg, float alpha) {
  float difference =
    fmodf(measuredDeg - previousDeg + 540.0f, 360.0f) - 180.0f;
  return normalizeHeading(previousDeg + alpha * difference);
}

void initializeGpsFilter(double lat,
                         double lng,
                         float speedMps,
                         float courseDeg,
                         bool velocityValid,
                         uint32_t now) {
  memset(&gpsFilter, 0, sizeof(gpsFilter));
  gpsFilter.initialized = true;
  gpsFilter.originLat = lat;
  gpsFilter.originLng = lng;
  gpsFilter.lastFixMs = now;

  if (velocityValid) {
    float courseRad = courseDeg * DEG_TO_RAD;
    gpsFilter.vxMps = speedMps * sinf(courseRad);
    gpsFilter.vyMps = speedMps * cosf(courseRad);
  }
}

bool updateGpsFilter(double lat,
                     double lng,
                     float speedMps,
                     float courseDeg,
                     bool velocityValid,
                     uint32_t now) {
  if (!gpsFilter.initialized ||
      now - gpsFilter.lastFixMs > GPS_FILTER_RESET_GAP_MS) {
    initializeGpsFilter(
      lat, lng, speedMps, courseDeg, velocityValid, now
    );
    gpsAcceptedCount++;
    return true;
  }

  float dt = (now - gpsFilter.lastFixMs) / 1000.0f;
  if (dt < 0.05f) {
    return false;
  }

  float metersPerDegLat = 111132.0f;
  float metersPerDegLng =
    111320.0f * cosf((float)gpsFilter.originLat * DEG_TO_RAD);
  if (fabsf(metersPerDegLng) < 1.0f) {
    metersPerDegLng = 1.0f;
  }

  float measuredX =
    (float)((lng - gpsFilter.originLng) * metersPerDegLng);
  float measuredY =
    (float)((lat - gpsFilter.originLat) * metersPerDegLat);

  float predictedX = gpsFilter.xM + gpsFilter.vxMps * dt;
  float predictedY = gpsFilter.yM + gpsFilter.vyMps * dt;
  float residualX = measuredX - predictedX;
  float residualY = measuredY - predictedY;
  float residualM = sqrtf(
    residualX * residualX + residualY * residualY
  );

  float estimatedSpeed = sqrtf(
    gpsFilter.vxMps * gpsFilter.vxMps +
    gpsFilter.vyMps * gpsFilter.vyMps
  );
  float expectedSpeed = max(
    estimatedSpeed,
    velocityValid ? speedMps : 0.0f
  );
  expectedSpeed = constrain(
    expectedSpeed, 0.0f, GPS_NODE_MAX_SPEED_MPS
  );

  float outlierGateM =
    GPS_OUTLIER_BASE_M + expectedSpeed * dt * 1.5f;

  if (residualM > outlierGateM) {
    gpsRejectedCount++;

    // 랜덤한 이상치 3개가 아니라 서로 가까운 새 좌표가 반복될 때만
    // 실제 이동/재수신 위치로 인정한다.
    float relocationDifferenceM = 0.0f;
    if (gpsFilter.hasRelocationCandidate) {
      float dx = measuredX - gpsFilter.relocationX;
      float dy = measuredY - gpsFilter.relocationY;
      relocationDifferenceM = sqrtf(dx * dx + dy * dy);
    }

    if (!gpsFilter.hasRelocationCandidate ||
        relocationDifferenceM > GPS_OUTLIER_BASE_M) {
      gpsFilter.hasRelocationCandidate = true;
      gpsFilter.relocationX = measuredX;
      gpsFilter.relocationY = measuredY;
      gpsFilter.consecutiveOutliers = 1;
    } else {
      gpsFilter.relocationX =
        0.5f * gpsFilter.relocationX + 0.5f * measuredX;
      gpsFilter.relocationY =
        0.5f * gpsFilter.relocationY + 0.5f * measuredY;
      gpsFilter.consecutiveOutliers++;
    }

    Serial.printf(
      "[GPS FILTER] jump rejected residual=%.1fm gate=%.1fm count=%u\n",
      residualM,
      outlierGateM,
      gpsFilter.consecutiveOutliers
    );

    // 같은 새 위치가 연속해서 들어오면 실제 이동 또는 GPS 재수신으로 판단.
    if (gpsFilter.consecutiveOutliers <
        GPS_RELOCALIZE_AFTER_REJECTS) {
      return false;
    }

    initializeGpsFilter(
      lat, lng, speedMps, courseDeg, velocityValid, now
    );
    gpsRelocalizedCount++;
    gpsAcceptedCount++;
    Serial.println("[GPS FILTER] relocalized after repeated jumps");
    return true;
  }

  gpsFilter.consecutiveOutliers = 0;
  gpsFilter.hasRelocationCandidate = false;

  bool likelyStationary =
    velocityValid && speedMps < GPS_STATIONARY_SPEED_MPS;

  float positionAlpha =
    likelyStationary ? GPS_STATIONARY_ALPHA : GPS_FILTER_ALPHA;

  float velocityBeta =
    likelyStationary ? GPS_STATIONARY_BETA : GPS_FILTER_BETA;

  gpsFilter.xM = predictedX + positionAlpha * residualX;
  gpsFilter.yM = predictedY + positionAlpha * residualY;
  gpsFilter.vxMps += velocityBeta * residualX / dt;
  gpsFilter.vyMps += velocityBeta * residualY / dt;

  // GPS가 제공한 speed/course도 약하게 섞어 이동 시 지연을 줄인다.
  if (velocityValid) {
    float courseRad = courseDeg * DEG_TO_RAD;
    float measuredVx = speedMps * sinf(courseRad);
    float measuredVy = speedMps * cosf(courseRad);
    gpsFilter.vxMps =
      0.75f * gpsFilter.vxMps + 0.25f * measuredVx;
    gpsFilter.vyMps =
      0.75f * gpsFilter.vyMps + 0.25f * measuredVy;
  }

  float filteredSpeed = sqrtf(
    gpsFilter.vxMps * gpsFilter.vxMps +
    gpsFilter.vyMps * gpsFilter.vyMps
  );
  if (filteredSpeed > GPS_NODE_MAX_SPEED_MPS) {
    float scale = GPS_NODE_MAX_SPEED_MPS / filteredSpeed;
    gpsFilter.vxMps *= scale;
    gpsFilter.vyMps *= scale;
  }

  // 정지 상태에서 작은 속도 오차로 좌표가 계속 흘러가는 것을 억제.
  if (likelyStationary) {
    gpsFilter.vxMps *= 0.15f;
    gpsFilter.vyMps *= 0.15f;
  }

  gpsFilter.lastFixMs = now;
  gpsAcceptedCount++;
  return true;
}

void projectGpsPosition(uint32_t now,
                        double *outLat,
                        double *outLng) {
  float predictionMs =
    min((float)(now - gpsFilter.lastFixMs),
        (float)GPS_PREDICTION_MAX_MS);
  float dt = predictionMs / 1000.0f;

  float projectedX = gpsFilter.xM + gpsFilter.vxMps * dt;
  float projectedY = gpsFilter.yM + gpsFilter.vyMps * dt;
  float metersPerDegLng =
    111320.0f * cosf((float)gpsFilter.originLat * DEG_TO_RAD);
  if (fabsf(metersPerDegLng) < 1.0f) {
    metersPerDegLng = 1.0f;
  }

  *outLat = gpsFilter.originLat + projectedY / 111132.0f;
  *outLng = gpsFilter.originLng + projectedX / metersPerDegLng;
}

bool gpsQualityIsGood() {
  if (!gps.satellites.isValid() || !gps.hdop.isValid()) {
    return false;
  }

  return gps.satellites.value() >= GPS_MIN_SATELLITES &&
         gps.hdop.hdop() <= GPS_MAX_HDOP;
}

bool imuReady = false;
bool imuHasSample = false;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float magX = 0.0f;
float magY = 0.0f;
float magZ = 0.0f;
uint32_t lastImuSampleMs = 0;
float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 9.80665f;
float linearAccelMagnitude = 0.0f;
float impactPeakSinceTelemetry = 0.0f;
float lastImpactTriggerMagnitude = 0.0f;
uint32_t lastImpactTriggerAtMs = 0;
uint32_t lastImpactMs = 0;

bool dfPlayerReady = false;
uint8_t lastAnnouncedRisk = RISK_SAFE;

uint8_t lastRiskLevel = RISK_SAFE;

// 위험 판정 안정화 상태.
uint8_t rawRiskLevel = RISK_SAFE;
uint8_t stableRiskLevel = RISK_SAFE;
uint8_t candidateRiskLevel = RISK_SAFE;
uint8_t candidateRiskCount = 0;

// 마지막 위험 계산 결과: UDP 1초 로그에 보존.
float lastCalculatedDistanceM = -1.0f;
float lastCalculatedClosingSpeedMps = 0.0f;
float lastCalculatedTtcS = 999.0f;
float lastCalculatedHeadingErrorDeg = 0.0f;
bool lastCalculatedInPath = false;

// 실제로 재생된 음성 원인을 구분하기 위한 카운터.
uint32_t riskAudioPlayCount = 0;
uint32_t impactAudioPlayCount = 0;
uint8_t lastPlayedRiskAudio = RISK_SAFE;

// 지팡이 노드마다 이전 거리와 접근속도를 따로 저장.
// 여러 지팡이의 거리 기록이 섞이는 문제를 막는다.
typedef struct {
  bool used;
  bool hasClosingSpeed;
  uint32_t nodeId;
  float prevDistanceM;
  float filteredClosingSpeed;
  uint32_t prevCalcMs;
  uint32_t lastUpdateMs;
} CaneRiskState;

// Arduino 전처리기가 이 함수의 자동 원형을 typedef보다 위에 만드는 것을 방지한다.
CaneRiskState *getCaneRiskState(uint32_t caneNodeId);

CaneRiskState caneRiskStates[MAX_TRACKED_CANES];

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
// 재생 모드 설정
sendDfPlayerAtCommand("AT+PLAYMODE=3");
delay(200);

// 내장 스피커 앰프 켜기
sendDfPlayerAtCommand("AT+AMP=ON");
delay(200);

// 앰프를 켠 뒤 볼륨 적용
sendDfPlayerAtCommand(volumeCommand);
delay(200);

// 실제 적용된 볼륨 조회
sendDfPlayerAtCommand("AT+VOL=?");
delay(200);
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

  bool playedRiskAudio = false;

  // 안전 복귀 시에는 음원을 재생하지 않고 상태만 초기화.
  if (risk == RISK_CAUTION) {
    playAudioFile(TRACK_CAUTION_FILE);
    playedRiskAudio = true;
  }
  else if (risk == RISK_WARNING) {
    playAudioFile(TRACK_WARNING_FILE);
    playedRiskAudio = true;
  }
  else if (risk == RISK_DANGER) {
    playAudioFile(TRACK_DANGER_FILE);
    playedRiskAudio = true;
  }

  if (playedRiskAudio) {
    riskAudioPlayCount++;
    lastPlayedRiskAudio = risk;
  }

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

  // 데모 차량은 서쪽으로 이동한다고 가정.
  vehicleHeading = 270.0f;
  vehicleHeadingValid = true;

  vehicleGpsValid = 1;
  usingDemoGps = true;
#else
  vehicleLat = 0.0;
  vehicleLng = 0.0;
  vehicleSpeed = 0.0f;
  vehicleHeading = 0.0f;
  vehicleHeadingValid = false;
  vehicleGpsValid = 0;
  usingDemoGps = false;
#endif
}

void readGps() {
#if USE_GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  uint32_t now = millis();

  if (gps.location.isUpdated()) {
    rawGpsLat = gps.location.lat();
    rawGpsLng = gps.location.lng();
    rawGpsSatellites =
      gps.satellites.isValid() ? gps.satellites.value() : 0;
    rawGpsHdop =
      gps.hdop.isValid() ? gps.hdop.hdop() : 99.0f;

    bool locationOk =
      gps.location.isValid() &&
      fabs(rawGpsLat) <= 90.0 &&
      fabs(rawGpsLng) <= 180.0 &&
      !(rawGpsLat == 0.0 && rawGpsLng == 0.0);

    rawGpsSpeedMps =
      gps.speed.isValid() ? gps.speed.mps() : 0.0f;
    rawGpsCourseValid = gps.course.isValid();
    rawGpsCourseDeg =
      rawGpsCourseValid ? gps.course.deg() : 0.0f;

    float rawSpeed = rawGpsSpeedMps;
    bool speedOk =
      gps.speed.isValid() &&
      rawSpeed >= 0.0f &&
      rawSpeed <= GPS_NODE_MAX_SPEED_MPS;
    bool courseOk = rawGpsCourseValid;
    bool velocityOk = speedOk && courseOk;

    if (!locationOk || !gpsQualityIsGood()) {
      gpsQualityRejectedCount++;
      Serial.printf(
        "[GPS FILTER] quality rejected sats=%lu hdop=%.2f\n",
        (unsigned long)rawGpsSatellites,
        rawGpsHdop
      );
    } else {
      bool accepted = updateGpsFilter(
        rawGpsLat,
        rawGpsLng,
        rawSpeed,
        courseOk ? rawGpsCourseDeg : 0.0f,
        velocityOk,
        now
      );

      if (accepted) {
        if (speedOk) {
          vehicleSpeed =
            gpsAcceptedCount <= 1
              ? rawSpeed
              : 0.50f * vehicleSpeed + 0.50f * rawSpeed;
        }

        // 359도와 1도를 단순 평균해 180도가 되는 문제를 피하는 원형 평균.
        if (courseOk &&
            rawSpeed >= MIN_VALID_HEADING_SPEED_MPS) {
          float rawHeading = rawGpsCourseDeg;
          vehicleHeading =
            vehicleHeadingValid
              ? blendHeading(vehicleHeading, rawHeading, 0.35f)
              : rawHeading;
          vehicleHeadingValid = true;
        }
      }
    }
  }

  if (gpsFilter.initialized &&
      now - gpsFilter.lastFixMs <= GPS_FIX_MAX_AGE_MS) {
    projectGpsPosition(now, &vehicleLat, &vehicleLng);
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
    // 기본 ±2g에서는 평범한 노면 충격에도 센서가 포화되므로
    // 차량 충격 측정 범위를 ±8g로 확장한다.
    ICM_20948_fss_t imuFullScale;
    imuFullScale.a = gpm8;

    imu.setFullScale(
      ICM_20948_Internal_Acc,
      imuFullScale
    );

    if (imu.status != ICM_20948_Stat_Ok) {
      Serial.print("[IMU] accel ±8g 설정 실패: ");
      Serial.println(imu.statusString());
    }

    imuReady = true;
    Serial.println("[IMU] ICM-20948 connected, accel=±8g");
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
  magX = imu.magX();
  magY = imu.magY();
  magZ = imu.magZ();
  lastImuSampleMs = millis();

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

  // 직전 UDP 전송 이후 발생한 가장 큰 충격값을 보존한다.
  if (linearAccelMagnitude > impactPeakSinceTelemetry) {
    impactPeakSinceTelemetry = linearAccelMagnitude;
  }

  uint32_t now = millis();
  bool cooldownDone =
    lastImpactMs == 0 ||
    now - lastImpactMs >= IMPACT_COOLDOWN_MS;

  if (cooldownDone &&
      linearAccelMagnitude >= IMPACT_THRESHOLD_MPS2) {
    lastImpactMs = now;

    // 충격음이 실제로 발생한 순간의 값과 시각을 보존한다.
    lastImpactTriggerMagnitude = linearAccelMagnitude;
    lastImpactTriggerAtMs = now;

    Serial.printf(
      "[IMU IMPACT] linear=%.2f m/s^2\n",
      linearAccelMagnitude
    );

    impactAudioPlayCount++;
    playAudioFile(TRACK_IMPACT_FILE);
  }
#endif
}

// =====================
// 거리/TTC/진행 방향 위험도
// =====================
float degToRad(float degree) {
  return degree * DEG_TO_RAD;
}

float distanceMeters(double lat1,
                     double lng1,
                     double lat2,
                     double lng2) {
  const float earthRadiusM = 6371000.0f;

  float p1 = degToRad(lat1);
  float p2 = degToRad(lat2);
  float dp = degToRad(lat2 - lat1);
  float dl = degToRad(lng2 - lng1);

  float a = sinf(dp * 0.5f) * sinf(dp * 0.5f) +
            cosf(p1) * cosf(p2) *
            sinf(dl * 0.5f) * sinf(dl * 0.5f);

  a = constrain(a, 0.0f, 1.0f);

  return earthRadiusM * 2.0f *
         atan2f(sqrtf(a), sqrtf(1.0f - a));
}

// 차량 위치에서 지팡이 위치까지의 방위각을 계산.
// 반환값: 북쪽 0도, 동쪽 90도, 남쪽 180도, 서쪽 270도.
float bearingDegrees(double fromLat,
                     double fromLng,
                     double toLat,
                     double toLng) {
  float lat1 = degToRad(fromLat);
  float lat2 = degToRad(toLat);
  float deltaLng = degToRad(toLng - fromLng);

  float y = sinf(deltaLng) * cosf(lat2);
  float x = cosf(lat1) * sinf(lat2) -
            sinf(lat1) * cosf(lat2) * cosf(deltaLng);

  float bearing = atan2f(y, x) * RAD_TO_DEG;

  if (bearing < 0.0f) {
    bearing += 360.0f;
  }

  return bearing;
}

// 두 방향 사이의 가장 짧은 각도 차이를 -180~180도로 반환.
float angleDifferenceDegrees(float headingDeg,
                             float targetBearingDeg) {
  float difference =
    fmodf(targetBearingDeg - headingDeg + 540.0f, 360.0f) - 180.0f;

  return difference;
}

bool isCaneInVehiclePath(float headingErrorDeg) {
  if (!vehicleHeadingValid) return false;
  if (vehicleSpeed < MIN_VALID_HEADING_SPEED_MPS) return false;

  return fabsf(headingErrorDeg) <= FORWARD_CONE_HALF_ANGLE_DEG;
}

void resetAllCaneRiskStates() {
  memset(caneRiskStates, 0, sizeof(caneRiskStates));
}

CaneRiskState *getCaneRiskState(uint32_t caneNodeId) {
  uint32_t now = millis();

  // 기존에 추적 중인 지팡이를 먼저 찾는다.
  for (int i = 0; i < MAX_TRACKED_CANES; i++) {
    if (caneRiskStates[i].used &&
        caneRiskStates[i].nodeId == caneNodeId) {
      caneRiskStates[i].lastUpdateMs = now;
      return &caneRiskStates[i];
    }
  }

  // 비어 있는 슬롯을 찾는다.
  for (int i = 0; i < MAX_TRACKED_CANES; i++) {
    if (!caneRiskStates[i].used) {
      memset(&caneRiskStates[i], 0, sizeof(CaneRiskState));

      caneRiskStates[i].used = true;
      caneRiskStates[i].nodeId = caneNodeId;
      caneRiskStates[i].prevDistanceM = -1.0f;
      caneRiskStates[i].lastUpdateMs = now;

      return &caneRiskStates[i];
    }
  }

  // 슬롯이 꽉 찬 경우 가장 오래 갱신되지 않은 슬롯을 재사용한다.
  int oldestIndex = 0;
  uint32_t oldestAge = 0;

  for (int i = 0; i < MAX_TRACKED_CANES; i++) {
    uint32_t age = now - caneRiskStates[i].lastUpdateMs;

    if (age > oldestAge) {
      oldestAge = age;
      oldestIndex = i;
    }
  }

  memset(&caneRiskStates[oldestIndex], 0, sizeof(CaneRiskState));

  caneRiskStates[oldestIndex].used = true;
  caneRiskStates[oldestIndex].nodeId = caneNodeId;
  caneRiskStates[oldestIndex].prevDistanceM = -1.0f;
  caneRiskStates[oldestIndex].lastUpdateMs = now;

  return &caneRiskStates[oldestIndex];
}

uint8_t stabilizeRiskLevel(uint8_t rawRisk) {
  if (rawRisk == stableRiskLevel) {
    candidateRiskLevel = rawRisk;
    candidateRiskCount = 0;
    return stableRiskLevel;
  }

  if (rawRisk != candidateRiskLevel) {
    candidateRiskLevel = rawRisk;
    candidateRiskCount = 1;
  } else if (candidateRiskCount < 255) {
    candidateRiskCount++;
  }

  uint8_t requiredCount =
    rawRisk > stableRiskLevel
      ? RISK_ESCALATE_CONFIRM_COUNT
      : RISK_CLEAR_CONFIRM_COUNT;

  if (candidateRiskCount >= requiredCount) {
    stableRiskLevel = rawRisk;
    candidateRiskCount = 0;
  }

  return stableRiskLevel;
}

uint8_t calculateRiskFromCane(const v2x_status_message_t &cane,
                              float *outDistance,
                              float *outClosingSpeed,
                              float *outTtc,
                              float *outBearing,
                              float *outHeadingError,
                              bool *outInVehiclePath) {
  uint32_t now = millis();

  float distanceM = distanceMeters(vehicleLat,
                                   vehicleLng,
                                   cane.latitude,
                                   cane.longitude);

  float bearing = bearingDegrees(vehicleLat,
                                 vehicleLng,
                                 cane.latitude,
                                 cane.longitude);

  float headingError =
    angleDifferenceDegrees(vehicleHeading, bearing);

  bool inVehiclePath = isCaneInVehiclePath(headingError);

  CaneRiskState *state = getCaneRiskState(cane.node_id);

  float closingSpeed = state->hasClosingSpeed
                         ? state->filteredClosingSpeed
                         : 0.0f;

  float ttc = 999.0f;

  if (state->prevDistanceM < 0.0f || state->prevCalcMs == 0) {
    // 첫 데이터는 비교 대상이 없으므로 저장만 한다.
    state->prevDistanceM = distanceM;
    state->prevCalcMs = now;
  } else {
    uint32_t elapsedMs = now - state->prevCalcMs;

    if (elapsedMs >= TTC_SAMPLE_INTERVAL_MS) {
      float dt = elapsedMs / 1000.0f;

      float rawClosingSpeed =
        (state->prevDistanceM - distanceM) / dt;

      if (!state->hasClosingSpeed) {
        state->filteredClosingSpeed = rawClosingSpeed;
        state->hasClosingSpeed = true;
      } else {
        state->filteredClosingSpeed =
          TTC_FILTER_ALPHA * state->filteredClosingSpeed +
          (1.0f - TTC_FILTER_ALPHA) * rawClosingSpeed;
      }

      state->prevDistanceM = distanceM;
      state->prevCalcMs = now;
      closingSpeed = state->filteredClosingSpeed;
    }
  }

  // 차량 전방에 있고 실제로 가까워질 때만 TTC를 유효하게 계산한다.
  if (inVehiclePath &&
      vehicleSpeed >= MIN_TTC_VEHICLE_SPEED_MPS &&
      closingSpeed >= MIN_TTC_CLOSING_SPEED_MPS) {
    ttc = distanceM / closingSpeed;
  }

  *outDistance = distanceM;
  *outClosingSpeed = closingSpeed;
  *outTtc = ttc;
  *outBearing = bearing;
  *outHeadingError = headingError;
  *outInVehiclePath = inVehiclePath;

  // 아주 가까운 물체는 방향과 관계없이 거리 기준으로 경고한다.
  // TTC 기준은 차량 진행 경로 안에 있을 때만 유효해진다.
  if (distanceM < RISK_DANGER_DISTANCE_M ||
      ttc < RISK_DANGER_TTC_S) {
    return RISK_DANGER;
  }

  if (distanceM < RISK_WARNING_DISTANCE_M ||
      ttc < RISK_WARNING_TTC_S) {
    return RISK_WARNING;
  }

  if (distanceM < RISK_CAUTION_DISTANCE_M ||
      ttc < RISK_CAUTION_TTC_S) {
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
                         float distanceM,
                         float closingSpeed,
                         float ttc,
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

  txRiskAlert.distance_m = distanceM;
  txRiskAlert.closing_speed_mps = closingSpeed;
  txRiskAlert.ttc_s = ttc;

  txRiskAlert.timestamp_ms = millis();
  txRiskAlert.seq_num = riskSeq++;

  esp_err_t result = esp_now_send(
    targetMac,
    (uint8_t *)&txRiskAlert,
    sizeof(txRiskAlert)
  );

  riskSendCount++;

  Serial.printf(
    "[RISK TX] cane=%lu risk=%u distance=%.2f "
    "closing=%.2f ttc=%.2f seq=%u result=%s\n",
    (unsigned long)targetCaneId,
    risk,
    distanceM,
    closingSpeed,
    ttc,
    txRiskAlert.seq_num,
    result == ESP_OK ? "OK" : "ERR"
  );
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

  bool rssiValid = info != nullptr && info->rx_ctrl != nullptr;
  int8_t receivedRssiDbm = rssiValid ? info->rx_ctrl->rssi : -127;
  uint32_t receivedAtMs = millis();

  portENTER_CRITICAL(&caneMux);
  memcpy(&latestCaneStatus, &message, sizeof(latestCaneStatus));
  memcpy(latestCaneMAC, info->src_addr, 6);
  hasCaneMAC = true;
  hasLatestCane = true;
  newCanePacket = true;
  lastCaneRxMs = receivedAtMs;
  caneRxCount++;

  if (rssiValid) {
    latestCaneRssiDbm = receivedRssiDbm;

    if (!hasCaneRssi) {
      filteredCaneRssiDbm = (float)receivedRssiDbm;
      hasCaneRssi = true;
    } else {
      filteredCaneRssiDbm =
        CANE_RSSI_FILTER_ALPHA * (float)receivedRssiDbm +
        (1.0f - CANE_RSSI_FILTER_ALPHA) * filteredCaneRssiDbm;
    }

    caneRssiSampleCount++;
    lastCaneRssiMs = receivedAtMs;
  }
  portEXIT_CRITICAL(&caneMux);
}

void setupEspNow() {
  // 차량은 Wi-Fi 공유기 역할과 ESP-NOW를 동시에 사용한다.
  WiFi.mode(WIFI_AP_STA);
  delay(100);

  bool apStarted = WiFi.softAP(
    V2X_AP_SSID,
    V2X_AP_PASSWORD,
    V2X_WIFI_CHANNEL
  );

  if (!apStarted) {
    Serial.println("[WIFI AP] start failed, restarting");
    delay(1000);
    ESP.restart();
  }

  delay(300);

  Serial.println("[WIFI AP] started");
  Serial.printf("[WIFI AP] SSID=%s\n", V2X_AP_SSID);
  Serial.printf("[WIFI AP] PASSWORD=%s\n", V2X_AP_PASSWORD);
  Serial.printf("[WIFI AP] CHANNEL=%d\n", V2X_WIFI_CHANNEL);
  Serial.print("[WIFI AP] IP=");
  Serial.println(WiFi.softAPIP());

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
    rawRiskLevel = RISK_SAFE;
    stableRiskLevel = RISK_SAFE;
    candidateRiskLevel = RISK_SAFE;
    candidateRiskCount = 0;

    lastCalculatedDistanceM = -1.0f;
    lastCalculatedClosingSpeedMps = 0.0f;
    lastCalculatedTtcS = 999.0f;
    lastCalculatedHeadingErrorDeg = 0.0f;
    lastCalculatedInPath = false;

    lastRiskLevel = RISK_SAFE;
    announceRisk(RISK_SAFE);

    // GPS가 끊겼다가 다시 잡혔을 때 오래된 거리 기록으로
    // 잘못된 TTC가 계산되지 않도록 초기화한다.
    resetAllCaneRiskStates();

    sendRiskAlertToCane(
      RISK_SAFE,
      0.0f,
      0.0f,
      999.0f,
      cane.node_id,
      caneMac
    );

    Serial.println("[RISK] GPS invalid -> SAFE");
    return;
  }

  float distanceM = 0.0f;
  float closingSpeed = 0.0f;
  float ttc = 999.0f;
  float bearing = 0.0f;
  float headingError = 0.0f;
  bool inVehiclePath = false;

  uint8_t calculatedRawRisk = calculateRiskFromCane(
    cane,
    &distanceM,
    &closingSpeed,
    &ttc,
    &bearing,
    &headingError,
    &inVehiclePath
  );

  rawRiskLevel = calculatedRawRisk;
  uint8_t risk = stabilizeRiskLevel(calculatedRawRisk);

  lastCalculatedDistanceM = distanceM;
  lastCalculatedClosingSpeedMps = closingSpeed;
  lastCalculatedTtcS = ttc;
  lastCalculatedHeadingErrorDeg = headingError;
  lastCalculatedInPath = inVehiclePath;

  lastRiskLevel = risk;
  announceRisk(risk);

  sendRiskAlertToCane(
    risk,
    distanceM,
    closingSpeed,
    ttc,
    cane.node_id,
    caneMac
  );

#if USE_BT_DEBUG
  // 위험 계산 결과를 뷰어 값 표에 반영.
  if (SerialBT.hasClient()) {
    SerialBT.printf("거리:%.2f\n", distanceM);
    SerialBT.printf("접근속도:%.2f\n", closingSpeed);
    SerialBT.printf("TTC:%.2f\n", ttc);
    SerialBT.printf("방위각오차:%.1f\n", headingError);
    SerialBT.printf("전방여부:%u\n", inVehiclePath ? 1 : 0);
  }
#endif

  Serial.printf(
    "[RISK] cane=%lu distance=%.2fm closing=%.2fm/s "
    "ttc=%.2fs heading=%.1f bearing=%.1f "
    "diff=%.1f inPath=%u risk=%u\n",
    (unsigned long)cane.node_id,
    distanceM,
    closingSpeed,
    ttc,
    vehicleHeading,
    bearing,
    headingError,
    inVehiclePath ? 1 : 0,
    risk
  );
}

void resetStaleCaneRisk() {
  if (!hasLatestCane || lastCaneRxMs == 0) return;
  if (millis() - lastCaneRxMs <= CANE_TIMEOUT_MS) return;

  portENTER_CRITICAL(&caneMux);
  hasLatestCane = false;
  newCanePacket = false;
  portEXIT_CRITICAL(&caneMux);

  rawRiskLevel = RISK_SAFE;
  stableRiskLevel = RISK_SAFE;
  candidateRiskLevel = RISK_SAFE;
  candidateRiskCount = 0;

  lastRiskLevel = RISK_SAFE;
  announceRisk(RISK_SAFE);

  // 기존 거리 및 접근속도 기록을 모두 초기화.
  resetAllCaneRiskStates();

  Serial.println("[CANE] timeout -> risk reset");
}

void logSensors() {
  if (millis() - lastSensorLogMs < SENSOR_LOG_INTERVAL_MS) return;
  lastSensorLogMs = millis();

  Serial.printf(
    "[SENSOR] GPS=%s lat=%.6f lng=%.6f "
    "speed=%.2fm/s heading=%.1f headingValid=%u | "
    "IMU=%s acc=(%.2f,%.2f,%.2f) "
    "gyro=(%.1f,%.1f,%.1f) mag=(%.1f,%.1f,%.1f) linear=%.2f\n",
    vehicleGpsValid ? (usingDemoGps ? "DEMO" : "FIX") : "NO_FIX",
    vehicleLat,
    vehicleLng,
    vehicleSpeed,
    vehicleHeading,
    vehicleHeadingValid ? 1 : 0,
    imuHasSample ? "OK" : "WAIT",
    accelX,
    accelY,
    accelZ,
    gyroX,
    gyroY,
    gyroZ,
    magX,
    magY,
    magZ,
    linearAccelMagnitude
  );
}

// 차량 상태를 UDP 4211로 전송한다.
void sendUdpTelemetry() {
  char udpBuffer[1280];
  
  float impactPeakSnapshot = impactPeakSinceTelemetry;
  impactPeakSinceTelemetry = 0.0f;

  int8_t rssiRawSnapshot;
  float rssiFilteredSnapshot;
  bool rssiValidSnapshot;
  uint32_t rssiSampleCountSnapshot;
  uint32_t rssiLastMsSnapshot;

  portENTER_CRITICAL(&caneMux);
  rssiRawSnapshot = latestCaneRssiDbm;
  rssiFilteredSnapshot = filteredCaneRssiDbm;
  rssiValidSnapshot = hasCaneRssi;
  rssiSampleCountSnapshot = caneRssiSampleCount;
  rssiLastMsSnapshot = lastCaneRssiMs;
  portEXIT_CRITICAL(&caneMux);

  uint32_t telemetryMs = millis();
  
  long rssiAgeMs = rssiValidSnapshot
    ? (long)(telemetryMs - rssiLastMsSnapshot)
    : -1L;

  int written = snprintf(
    udpBuffer,
    sizeof(udpBuffer),
    "시각ms:%lu\n"
    "IMU시각ms:%lu\n"
    "위험:%u\n"
    "원시위험:%u\n"
    "GPS유효:%u\n"
    "위도:%.6f\n"
    "경도:%.6f\n"
    "속도:%.2f\n"
    "방향:%.1f\n"
    "GPS원시속도:%.3f\n"
    "GPS원시방향유효:%u\n"
    "GPS원시방향:%.1f\n"
    "가속도:%.2f,%.2f,%.2f\n"
    "자이로:%.1f,%.1f,%.1f\n"
    "자력계:%.3f,%.3f,%.3f\n"
    "충격값:%.2f\n"
    "충격피크:%.2f\n"
    "마지막충격발생값:%.2f\n"
    "마지막충격발생ms:%lu\n"
    "송신:%lu\n"
    "지팡이수신:%lu\n"
    "GPS위성:%lu\n"
    "GPS_HDOP:%.2f\n"
    "원시위도:%.6f\n"
    "원시경도:%.6f\n"
    "GPS채택:%lu\n"
    "GPS이상치거부:%lu\n"
    "GPS품질거부:%lu\n"
    "GPS재기준:%lu\n"
    "계산거리:%.2f\n"
    "접근속도:%.2f\n"
    "TTC:%.2f\n"
    "방향오차:%.1f\n"
    "전방여부:%u\n"
    "위험음성횟수:%lu\n"
    "충격음성횟수:%lu\n"
    "마지막위험음성:%u\n"
    "RSSI원시:%d\n"
    "RSSI평활:%.1f\n"
    "RSSI샘플:%lu\n"
    "RSSI경과ms:%ld\n"
    "위험송신:%lu\n",
    (unsigned long)telemetryMs,
    (unsigned long)lastImuSampleMs,
    lastRiskLevel,
    rawRiskLevel,
    vehicleGpsValid,
    vehicleLat,
    vehicleLng,
    vehicleSpeed,
    vehicleHeading,
    rawGpsSpeedMps,
    rawGpsCourseValid ? 1u : 0u,
    rawGpsCourseDeg,
    accelX,
    accelY,
    accelZ,
    gyroX,
    gyroY,
    gyroZ,
    magX,
    magY,
    magZ,
    linearAccelMagnitude,
    impactPeakSnapshot,
    lastImpactTriggerMagnitude,
    (unsigned long)lastImpactTriggerAtMs,
    (unsigned long)sendCount,
    (unsigned long)caneRxCount,
    (unsigned long)rawGpsSatellites,
    rawGpsHdop,
    rawGpsLat,
    rawGpsLng,
    (unsigned long)gpsAcceptedCount,
    (unsigned long)gpsRejectedCount,
    (unsigned long)gpsQualityRejectedCount,
    (unsigned long)gpsRelocalizedCount,
    lastCalculatedDistanceM,
    lastCalculatedClosingSpeedMps,
    lastCalculatedTtcS,
    lastCalculatedHeadingErrorDeg,
    lastCalculatedInPath ? 1 : 0,
    (unsigned long)riskAudioPlayCount,
    (unsigned long)impactAudioPlayCount,
    lastPlayedRiskAudio,
    (int)rssiRawSnapshot,
    rssiFilteredSnapshot,
    (unsigned long)rssiSampleCountSnapshot,
    rssiAgeMs,
    (unsigned long)riskSendCount
  );

  if (written <= 0) {
    return;
  }

  size_t sendLength =
    written < (int)sizeof(udpBuffer)
      ? (size_t)written
      : sizeof(udpBuffer) - 1;

  if (!logUdp.beginPacket(
        udpBroadcastAddress,
        VEHICLE_UDP_PORT
      )) {
    Serial.println("[UDP] beginPacket failed");
    return;
  }

  logUdp.write(
    (const uint8_t *)udpBuffer,
    sendLength
  );

  if (logUdp.endPacket() != 1) {
    Serial.println("[UDP] send failed");
  }
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

#if USE_BT_DEBUG
  SerialBT.begin("ESP32-Car");
  Serial.println("[BT] debug started: ESP32-Car");
#endif

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

  // 차량 로그는 1초마다 UDP 4211로 전송
if (now - lastUdpTelemetryMs >= UDP_TELEMETRY_INTERVAL_MS) {
  lastUdpTelemetryMs = now;
  sendUdpTelemetry();
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
    "[BT CHECK] ESP32-Car client=%d\n",
    SerialBT.hasClient() ? 1 : 0
  );
}
#endif

  // 최근 지팡이 패킷이 있으면 점등, 없으면 천천히 점멸.
  if (lastCaneRxMs > 0 && now - lastCaneRxMs < 1000UL) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, (now / 500UL) % 2);
  }

  delay(5);
}
