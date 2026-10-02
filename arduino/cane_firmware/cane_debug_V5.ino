// Cane V2X endpoint V5: u-blox M9N GPS(KoA, MG-F10도 동작) + BNO055, 기존 V4 V2X/경고 동작 호환.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
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
#define BUTTON_PIN 33

// Same GPS wiring as 01_sender_gps_imu_espnow.
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 115200

// Same I2C wiring as 01_sender_gps_imu_espnow.
#define I2C_SDA 21
#define I2C_SCL 22
#define BNO055_I2C_ADDRESS 0x28
// CJMCU-055 보드에 32.768kHz 외부 크리스털을 납땜했을 때만 1.
// (크리스털은 보드와 따로 들어 있다. 안 달았는데 1이면 동작이 불안정할 수 있다)
#define BNO_USE_EXTERNAL_CRYSTAL 0
#define IMU_SAMPLE_INTERVAL_MS 10UL
// 가속도·자력계·보정상태는 이 주기로만 읽어 I2C(100kHz) 점유를 줄인다.
#define IMU_SLOW_INTERVAL_MS 100UL

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
#define V2X_VERSION 3

#define MSG_VEHICLE_STATUS 1
#define MSG_RSU_REPLY 2
#define MSG_CANE_STATUS 3
#define MSG_RISK_ALERT 4
#define MSG_UWB_RANGE 5

#define NODE_VEHICLE 0x10
#define NODE_CANE 0x20
#define NODE_RSU 0x30

#define RISK_SAFE 0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER 3

#define SEND_INTERVAL_MS 100UL

// V2X 송신은 10Hz 유지. 사람이 보는 UDP 로그만 2Hz로 낮춰 제어 부하와 혼잡을 줄인다.
#define UDP_TELEMETRY_INTERVAL_MS 500UL

#define V2X_WIFI_SSID "V2X-LOG"
#define V2X_ESPNOW_CHANNEL 6   // 8/18 패치: 차량 softAP 채널(car V2X_WIFI_CHANNEL)과 동일
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

// GPS 품질/이상치 필터 설정: 지팡이(보행 속도) 기준.
#define GPS_MIN_SATELLITES 4
#define GPS_MAX_HDOP 3.0f
#define GPS_NODE_MAX_SPEED_MPS 4.0f
#define GPS_OUTLIER_BASE_M 10.0f   // 8/18 패치: 5→10 (거부=좌표 동결; 젯슨에 자체 게이트 있음)
#define GPS_FILTER_ALPHA 0.40f
#define GPS_FILTER_BETA 0.08f
// GPS가 실제 5Hz면 15회, 1Hz로 떨어지면 3회를 사용해 어느 경우든
// 약 3초 동안 같은 새 위치가 반복될 때 필터 기준 위치를 재설정한다.
#define GPS_RELOCALIZE_AFTER_REJECTS_5HZ 5   // 8/18 패치: 15(3 s)→5(1 s)
#define GPS_RELOCALIZE_AFTER_REJECTS_1HZ 3
#define GPS_FILTER_RESET_GAP_MS 5000UL
#define GPS_PREDICTION_MAX_MS 1200UL
#define GPS_MIN_COURSE_SPEED_MPS 0.5f
#define GPS_5HZ_RECOVERY_COOLDOWN_MS 60000UL
#define GPS_5HZ_RECOVERY_MIN_UPTIME_MS 15000UL

// ===== 2026-08-03 지팡이 실제 장착 상태 전용 방향 보정 =====
#define CANE_MAG_CENTER_X_UT (-48.675f)
#define CANE_MAG_CENTER_Y_UT ( 69.750f)
#define CANE_MAG_CENTER_Z_UT ( -8.025f)

#define CANE_MAG_AXIS_A_X (  9.750f)
#define CANE_MAG_AXIS_A_Y (-16.950f)
#define CANE_MAG_AXIS_A_Z ( 21.300f)

#define CANE_MAG_AXIS_90_X (-22.050f)
#define CANE_MAG_AXIS_90_Y (-20.850f)
#define CANE_MAG_AXIS_90_Z ( -3.900f)

#define CANE_NEUTRAL_AX 5.545f
#define CANE_NEUTRAL_AY 4.845f
#define CANE_NEUTRAL_AZ 6.045f
#define CANE_REFERENCE_HEADING_DEG 181.0f

// 땅 접촉·스윙 순간을 버리고 기준 자세를 통과하는 샘플만 채택한다.
#define CANE_HEADING_ACCEL_MIN_MPS2 7.0f
#define CANE_HEADING_ACCEL_MAX_MPS2 13.0f
#define CANE_HEADING_GYRO_MAX_DPS 45.0f
#define CANE_HEADING_POSE_COS_MIN 0.98480775f  // cos(10도)
#define CANE_MAG_MODEL_RESIDUAL_MAX_UT 12.0f
#define CANE_MAG_MODEL_RADIUS_MIN 0.45f
#define CANE_MAG_MODEL_RADIUS_MAX 1.70f
#define CANE_HEADING_FILTER_TAU_S 0.80f
#define CANE_HEADING_VALID_TIMEOUT_MS 1500UL

// 지팡이 버튼(GPIO33) 2초 = 걸어서 방향 정렬.
// 보행자가 앞을 보고 버튼을 누른 뒤 그대로 똑바로 걸으면, GPS 진행방향(북쪽 기준)과
// IMU 방위를 GPS 표본마다 짝지어 평균내고 그 차이를 장착 보정값(head_off)으로 저장한다.
// 그 뒤 IMU방향 = 보행자가 향한 실제 방위. 좌우 스윙은 1초 저역통과와 여러 걸음 평균으로 상쇄.
#define WALK_ALIGN_HOLD_MS 2000UL
#define WALK_ALIGN_MIN_SPEED_MPS 0.60f        // GPS 정지 잡음(최대 0.38 m/s)보다 크게
#define WALK_ALIGN_MIN_SAMPLES 20             // 5 Hz면 약 4초 걷기
#define WALK_ALIGN_MAX_SAMPLES 60             // 이만큼 모아도 퍼짐이 크면 처음부터 다시
#define WALK_ALIGN_MIN_RESULTANT 0.95f        // 짝 차이의 퍼짐 약 18도 이내
#define WALK_ALIGN_MAX_COURSE_STEP_DEG 25.0f  // 한 표본 사이에 이만큼 꺾이면 그 표본은 버림
#define WALK_ALIGN_TIMEOUT_MS 60000UL
#define WALK_ALIGN_RAW_LP_TAU_S 1.0f

// ===== 차량 상태만 받을 때 사용하는 예비 CPA 위험판정 =====
// 차량이 RSSI+상대GPS로 직접 판정하므로 지팡이의 GPS 단독 fallback은 끈다.
// 이 값을 1로 바꾸면 예전 GPS 단독 예비판정을 다시 사용할 수 있다.
#define ENABLE_CANE_LOCAL_GPS_RISK_FALLBACK 0
// 야외 RSU/Jetson 시연: RSU가 보낸 MSG_RISK_ALERT를 최종 출력에 적용한다.
// 차량 직접 위험 전송은 차량 코드에서 꺼서 두 판정이 섞이지 않게 한다.
#define ENABLE_RSU_RISK_INPUT 1
#define MIN_PREDICT_VEHICLE_SPEED_MPS 0.50f
// 2026-08-12 정지 평행 실측에서 지팡이 GPS 속도 노이즈가 최대
// 0.38m/s까지 올라갔다. 0.50m/s 미만은 정지로 처리한다.
#define MIN_PREDICT_CANE_SPEED_MPS 0.50f
#define MIN_RADIAL_CLOSING_SPEED_MPS 0.15f
#define CPA_DANGER_DISTANCE_M 0.5f
#define CPA_WARNING_DISTANCE_M 0.8f
#define CPA_CAUTION_DISTANCE_M 1.2f
#define CPA_PREDICTION_HORIZON_S 5.0f
#define EMERGENCY_DISTANCE_M 0.2f

// 차량에서 직접 보내는 위험 패킷의 유효시간.
// 이 시간 동안에는 지팡이 자체 계산보다 차량 계산 결과를 우선한다.
#define DIRECT_RISK_TIMEOUT_MS 1500UL

// RSU(Jetson)에서 오는 위험 값의 유효시간.
// Jetson 전송 주기(1초 heartbeat)의 3배.
#define RSU_RISK_TIMEOUT_MS 3000UL

// 차량 상태 패킷이 끊겼다고 판단할 시간.
#define VEHICLE_STATUS_TIMEOUT_MS 2000UL

// ESP-NOW RSSI 평활 계수. 작을수록 순간 흔들림을 더 강하게 줄인다.
#define VEHICLE_RSSI_FILTER_ALPHA 0.20f

// Fallback demo position. Used only while cane GPS is not fixed.
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000

// =====================
// 실시간 튜닝 파라미터 (뷰어 입력창 / USB 시리얼에서 변경 가능)
// 초기값은 위의 #define을 그대로 사용한다.
// =====================
float cfgGpsFilterAlpha = GPS_FILTER_ALPHA;
float cfgGpsFilterBeta = GPS_FILTER_BETA;
float cfgGpsOutlierBaseM = GPS_OUTLIER_BASE_M;
float cfgGpsMaxHdop = GPS_MAX_HDOP;
uint32_t cfgGpsMinSatellites = GPS_MIN_SATELLITES;
uint32_t cfgGpsPredictionMaxMs = GPS_PREDICTION_MAX_MS;
uint32_t cfgGpsFixMaxAgeMs = GPS_FIX_MAX_AGE_MS;
float cfgGpsMinCourseSpeed = GPS_MIN_COURSE_SPEED_MPS;
float cfgGpsMaxSpeed = GPS_NODE_MAX_SPEED_MPS;
float cfgRssiAlpha = VEHICLE_RSSI_FILTER_ALPHA;
uint32_t cfgDirectRiskTimeoutMs = DIRECT_RISK_TIMEOUT_MS;
uint32_t cfgTelemetryIntervalMs = UDP_TELEMETRY_INTERVAL_MS;
uint32_t cfgLogMode = 0;  // 0=필수값만, 1=V4 전체 진단값

typedef struct __attribute__((packed)) v2x_status_message {
  uint32_t magic;
  uint8_t version;
  uint8_t msg_type;
  uint8_t node_type;
  uint8_t risk_level;
  uint8_t gps_valid;
  uint8_t heading_valid;

  uint32_t node_id;
  float latitude;
  float longitude;
  float speed_mps;
  float heading_deg;

  uint32_t timestamp_ms;
  uint16_t seq_num;
  // 차량은 보정된 최신 UWB가 있으면 UWB 거리, 아니면 RSSI 추정거리.
  // 지팡이 송신값은 -1.
  float rssi_distance_m;
} v2x_status_message_t;

static_assert(sizeof(v2x_status_message_t) == 40,
              "vehicle/cane status packet must be 40 bytes (v4: +rssi_distance_m)");

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

static_assert(sizeof(v2x_risk_message_t) == 35,
              "risk alert packet must be 35 bytes");

typedef struct __attribute__((packed)) v2x_uwb_message {
  uint32_t magic;
  uint8_t version;
  uint8_t msg_type;
  uint8_t node_type;
  uint8_t flags;
  uint32_t target_id;
  uint32_t src_id;
  float raw_distance_m;
  float distance_m;
  float closing_speed_mps;
  float offset_m;
  uint32_t timestamp_ms;
  uint16_t seq_num;
} v2x_uwb_message_t;

static_assert(sizeof(v2x_uwb_message_t) == 38,
              "UWB range packet must be 38 bytes");

TinyGPSPlus gps;
SFE_UBLOX_GNSS_SERIAL gnss;
Adafruit_BNO055 bno(55, BNO055_I2C_ADDRESS, &Wire);
HardwareSerial gpsSerial(2);

#if USE_BT_DEBUG
BluetoothSerial SerialBT;
#endif

WiFiUDP logUdp;
IPAddress udpBroadcastAddress(192, 168, 4, 255);
uint32_t lastUdpTelemetryMs = 0;
uint32_t lastWifiReconnectMs = 0;

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
v2x_status_message_t txStatus;
v2x_risk_message_t rxRisk;
v2x_uwb_message_t latestUwbRange;

uint32_t caneId = 0;
uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t lastSendMs = 0;

uint32_t lastBtTelemetryMs = 0;
uint32_t lastBtCheckMs = 0;

uint32_t vehicleRxCount = 0;
uint32_t lastVehicleRxMs = 0;
bool hasUwbRange = false;
uint32_t uwbRxCount = 0;
uint32_t lastUwbRxMs = 0;
portMUX_TYPE uwbMux = portMUX_INITIALIZER_UNLOCKED;

// 차량 -> 지팡이 ESP-NOW 수신 신호 세기 진단값.
int8_t latestVehicleRssiDbm = -127;
float filteredVehicleRssiDbm = -127.0f;
bool hasVehicleRssi = false;
uint32_t vehicleRssiSampleCount = 0;
uint32_t lastVehicleRssiMs = 0;
portMUX_TYPE rssiMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t currentRisk = 255;
uint32_t lastRiskMs = 0;
uint16_t lastRiskSeq = 0;

// USB 시리얼 1/2/3 입력 시 5초 동안 진동/부저 패턴을 강제시험한다.
uint8_t manualOutputTestRisk = RISK_SAFE;
uint32_t manualOutputTestUntilMs = 0;

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
uint32_t lastGpsMeasurementMs = 0;
float gpsObservedUpdateIntervalMs = 200.0f;
uint8_t gpsRelocalizeAfterRejects =
  GPS_RELOCALIZE_AFTER_REJECTS_5HZ;
bool gps5HzRecoveryRequested = false;
uint32_t lastGps5HzRecoveryAttemptMs = 0;
uint32_t gps5HzRecoveryCount = 0;
uint32_t lastGps5HzRecoveryStartedMs = 0;

// USB 시리얼 없이 UDP 뷰어만으로 재부팅 여부와 원인을 확인하기 위한 값.
uint32_t caneBootCount = 0;
esp_reset_reason_t caneResetReason = ESP_RST_UNKNOWN;

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWER_ON";
    case ESP_RST_EXT: return "EXTERNAL";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK_WATCHDOG";
    case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_UNKNOWN:
    default: return "UNKNOWN";
  }
}

void captureBootDiagnostics() {
  caneResetReason = esp_reset_reason();

  // 전원 재인가까지 포함해 누적 부팅 횟수를 남긴다.
  Preferences bootPrefs;
  if (bootPrefs.begin("v2xdiag", false)) {
    caneBootCount = bootPrefs.getULong("bootcnt", 0) + 1;
    bootPrefs.putULong("bootcnt", caneBootCount);
    bootPrefs.end();
  } else {
    // NVS를 열지 못해도 이번 부팅이 화면에서 0으로 보이지 않게 한다.
    caneBootCount = 1;
  }
}

void updateObservedGpsRate(uint32_t now) {
  if (lastGpsMeasurementMs > 0) {
    uint32_t intervalMs = now - lastGpsMeasurementMs;
    if (intervalMs >= 100UL && intervalMs <= 2000UL) {
      gpsObservedUpdateIntervalMs =
        0.70f * gpsObservedUpdateIntervalMs + 0.30f * intervalMs;

      uint8_t newRejectLimit =
        gpsObservedUpdateIntervalMs > 500.0f
          ? GPS_RELOCALIZE_AFTER_REJECTS_1HZ
          : GPS_RELOCALIZE_AFTER_REJECTS_5HZ;
      gps5HzRecoveryRequested =
        newRejectLimit == GPS_RELOCALIZE_AFTER_REJECTS_1HZ;

      if (newRejectLimit != gpsRelocalizeAfterRejects) {
        gpsRelocalizeAfterRejects = newRejectLimit;
        Serial.printf(
          "[GPS] observed interval=%.0fms -> relocalize rejects=%u\n",
          gpsObservedUpdateIntervalMs,
          gpsRelocalizeAfterRejects
        );
      }
    }
  }
  lastGpsMeasurementMs = now;
}

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
    expectedSpeed, 0.0f, cfgGpsMaxSpeed
  );

  float outlierGateM =
    cfgGpsOutlierBaseM + expectedSpeed * dt * 1.5f;

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
        relocationDifferenceM > cfgGpsOutlierBaseM) {
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
        gpsRelocalizeAfterRejects) {
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
  gpsFilter.xM = predictedX + cfgGpsFilterAlpha * residualX;
  gpsFilter.yM = predictedY + cfgGpsFilterAlpha * residualY;
  gpsFilter.vxMps += cfgGpsFilterBeta * residualX / dt;
  gpsFilter.vyMps += cfgGpsFilterBeta * residualY / dt;

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
  if (filteredSpeed > cfgGpsMaxSpeed) {
    float scale = cfgGpsMaxSpeed / filteredSpeed;
    gpsFilter.vxMps *= scale;
    gpsFilter.vyMps *= scale;
  }

  // 정지 상태에서 작은 속도 오차로 좌표가 계속 흘러가는 것을 억제.
  if (velocityValid && speedMps < 0.25f) {
    gpsFilter.vxMps *= 0.35f;
    gpsFilter.vyMps *= 0.35f;
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
        (float)cfgGpsPredictionMaxMs);
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

  return gps.satellites.value() >= cfgGpsMinSatellites &&
         gps.hdop.hdop() <= cfgGpsMaxHdop;
}

float lastAccelX = 0.0f;
float lastAccelY = 0.0f;
float lastAccelZ = 0.0f;
float lastGyroX = 0.0f;
float lastGyroY = 0.0f;
float lastGyroZ = 0.0f;
float lastMagX = 0.0f;
float lastMagY = 0.0f;
float lastMagZ = 0.0f;
uint32_t lastImuSampleMs = 0;
uint32_t lastImuReadAttemptMs = 0;
bool imuReady = false;
bool imuOffsetsRestored = false;
bool imuHeadingAligned = false;
uint8_t imuCalSystem = 0;
uint8_t imuCalGyro = 0;
uint8_t imuCalAccel = 0;
uint8_t imuCalMag = 0;
float imuRawHeadingDeg = 0.0f;
float imuHeadingOffsetDeg = 0.0f;

bool saveImuCalibration();
void resetImuCalibration();
bool alignImuHeading(float trueHeadingDeg);
void reportImuStatus();

float caneImuHeadingDeg = 0.0f;
bool caneImuHeadingHasFix = false;
uint32_t lastCaneHeadingAcceptedMs = 0;
uint32_t caneHeadingAcceptedCount = 0;
uint32_t caneHeadingRejectedCount = 0;

// 걸어서 방향 정렬(버튼·imuwalk) 상태.
bool walkAlignActive = false;
uint32_t walkAlignStartedMs = 0;
uint32_t walkAlignLastLogMs = 0;
uint16_t walkAlignSamples = 0;
float walkAlignSumSin = 0.0f;
float walkAlignSumCos = 0.0f;
float walkAlignLastCourseDeg = -1.0f;
uint16_t walkAlignNoGps = 0;   // 표본을 못 쓴 이유별 횟수(실패 안내용)
uint16_t walkAlignSlow = 0;
uint16_t walkAlignImuLow = 0;
// IMU 원시 방위의 저역통과(sin/cos 평균). 걷는 중 좌우 스윙을 평균낸다.
float imuRawLpSin = 0.0f;
float imuRawLpCos = 1.0f;
bool imuRawLpReady = false;
uint32_t imuRawLpLastMs = 0;
// 버튼 확인 신호(진동+부저). 위험 경보가 켜져 있으면 경보가 우선한다.
uint8_t feedbackPulses = 0;
uint16_t feedbackOnMs = 0;
uint16_t feedbackOffMs = 0;
uint32_t feedbackStartMs = 0;

// UDP 진단용 마지막 방향 판정값.
float caneHeadingAccelNorm = 0.0f;
float caneHeadingGyroNorm = 0.0f;
float caneHeadingPoseCos = 0.0f;
float caneMagModelResidual = 999.0f;
float caneMagModelRadius = 0.0f;

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

  // 버튼 확인 신호. 위험 경보 중에는 내지 않는다(경보가 항상 우선).
  if (feedbackPulses > 0) {
    uint32_t periodMs = (uint32_t)feedbackOnMs + feedbackOffMs;
    uint32_t feedbackElapsedMs = millis() - feedbackStartMs;
    if (periodMs == 0 || feedbackElapsedMs >= periodMs * feedbackPulses) {
      feedbackPulses = 0;
    } else if (currentRisk != RISK_CAUTION && currentRisk != RISK_WARNING &&
               currentRisk != RISK_DANGER) {
      bool pulseOn = feedbackElapsedMs % periodMs < feedbackOnMs;
      motorOn = pulseOn;
      buzzerOn = pulseOn;
    }
  }

  // SAFE이거나 잘못된 위험 단계면 둘 다 false 상태로 유지.
  writeActuatorOutputs(motorOn, buzzerOn);
#endif
}

void sendGpsUbxCommand(uint8_t msgClass,
                       uint8_t msgId,
                       const uint8_t *payload,
                       uint16_t payloadLength) {
  // 현재 사용하는 UBX 페이로드는 최대 6바이트이다.
  uint8_t packet[32];
  if (payloadLength > sizeof(packet) - 8) return;

  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = msgClass;
  packet[3] = msgId;
  packet[4] = payloadLength & 0xFF;
  packet[5] = payloadLength >> 8;
  if (payloadLength > 0) {
    if (payload == nullptr) return;
    memcpy(&packet[6], payload, payloadLength);
  }

  uint8_t ckA = 0;
  uint8_t ckB = 0;
  for (uint16_t i = 2; i < 6 + payloadLength; i++) {
    ckA += packet[i];
    ckB += ckA;
  }

  packet[6 + payloadLength] = ckA;
  packet[7 + payloadLength] = ckB;
  gpsSerial.write(packet, payloadLength + 8);
  gpsSerial.flush();
}

void discardGpsSerialInput(uint32_t durationMs) {
  uint32_t startedMs = millis();
  while (millis() - startedMs < durationMs) {
    while (gpsSerial.available() > 0) gpsSerial.read();
    delay(1);
  }
}

bool waitForGpsUbxAck(uint8_t targetClass,
                      uint8_t targetId,
                      uint32_t timeoutMs) {
  const uint8_t expected[] = {
    0xB5, 0x62, 0x05, 0x01, 0x02, 0x00,
    targetClass, targetId
  };
  size_t matched = 0;
  uint32_t startedMs = millis();

  while (millis() - startedMs < timeoutMs) {
    while (gpsSerial.available() > 0) {
      uint8_t value = (uint8_t)gpsSerial.read();
      if (value == expected[matched]) {
        matched++;
        if (matched == sizeof(expected)) return true;
      } else {
        matched = value == expected[0] ? 1 : 0;
      }
    }
    delay(1);
  }
  return false;
}

bool pollGpsMeasurementRate(uint16_t &measurementRateMs,
                            uint32_t timeoutMs) {
  // CFG-RATE poll: 실제 모듈에 적용된 measRate를 다시 읽는다.
  const uint8_t expectedHeader[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00
  };
  uint8_t payloadAndChecksum[8];
  size_t matched = 0;
  size_t tailLength = 0;

  discardGpsSerialInput(30);
  sendGpsUbxCommand(0x06, 0x08, nullptr, 0);
  uint32_t startedMs = millis();

  while (millis() - startedMs < timeoutMs) {
    while (gpsSerial.available() > 0) {
      uint8_t value = (uint8_t)gpsSerial.read();
      if (matched < sizeof(expectedHeader)) {
        if (value == expectedHeader[matched]) {
          matched++;
        } else {
          matched = value == expectedHeader[0] ? 1 : 0;
        }
        continue;
      }

      payloadAndChecksum[tailLength++] = value;
      if (tailLength == sizeof(payloadAndChecksum)) {
        uint8_t ckA = 0;
        uint8_t ckB = 0;
        for (size_t i = 2; i < sizeof(expectedHeader); i++) {
          ckA += expectedHeader[i];
          ckB += ckA;
        }
        for (size_t i = 0; i < 6; i++) {
          ckA += payloadAndChecksum[i];
          ckB += ckA;
        }
        if (ckA != payloadAndChecksum[6] ||
            ckB != payloadAndChecksum[7]) {
          return false;
        }

        measurementRateMs =
          (uint16_t)payloadAndChecksum[0] |
          ((uint16_t)payloadAndChecksum[1] << 8);
        return true;
      }
    }
    delay(1);
  }
  return false;
}

bool configureMgF10FiveHz() {
  bool ok = gnss.newCfgValset(VAL_LAYER_RAM);
  ok &= gnss.addCfgValset(UBLOX_CFG_UART1OUTPROT_UBX, 1);
  ok &= gnss.addCfgValset(UBLOX_CFG_UART1OUTPROT_NMEA, 1);
  ok &= gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, 1);
  ok &= gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, 1);
  ok &= gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1, 0);
  ok &= gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, 0);
  ok &= gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, 0);
  ok &= gnss.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, 0);
  ok &= gnss.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1, 0);
  ok &= gnss.addCfgValset(UBLOX_CFG_RATE_MEAS, 200);
  ok &= gnss.addCfgValset(UBLOX_CFG_RATE_NAV, 1);
  ok &= gnss.sendCfgValset(1200);
  return ok;
}

// GPS L5 위성 신호는 아직 '시험 운용(unhealthy)'으로 방송돼서 그대로는 안 쓴다.
// NEO-F10N 통합 매뉴얼 예제대로 L5를 켜고 건강상태를 L1 기준으로 덮어쓴다.
// 알 수 없는 키가 섞이면 메시지 전체가 거절되므로 5Hz 설정과 따로 보낸다.
// 신호 설정을 바꾸면 GNSS가 잠깐 재시작하므로 5Hz 설정보다 먼저 보낸다.
bool configureMgF10L5() {
  bool ok = gnss.newCfgValset(VAL_LAYER_RAM);
  ok &= gnss.addCfgValset(UBLOX_CFG_SIGNAL_GPS_L5_ENA, 1);
  ok &= gnss.addCfgValset(UBLOX_CFG_SIGNAL_GPS_L5_HEALTH_OVERRIDE, 1);
  ok &= gnss.sendCfgValset(1200);
  return ok;
}

// GPS 보드레이트 찾기: 115200(KoA·Holybro M9N, MG-F10 출고값)으로 먼저 붙고, 안 되면 u-blox 공장값 38400, 그다음 9600에서
// 찾아 GPS UART1을 115200으로 바꾼다(RAM만 — 전원을 켤 때마다 다시 함). 찾은 속도를 돌려주고, 못 찾으면 0.
const uint32_t GPS_BAUD_FALLBACK[] = {38400, 9600};

uint32_t beginGnssAnyBaud() {
  if (gnss.begin(gpsSerial, 1200)) return GPS_BAUD;
  for (uint32_t baud : GPS_BAUD_FALLBACK) {
    gpsSerial.updateBaudRate(baud);
    delay(200);
    if (!gnss.begin(gpsSerial, 1200)) continue;
    gnss.setSerialRate(GPS_BAUD, COM_PORT_UART1, VAL_LAYER_RAM, 300);  // 응답은 새 속도로 오므로 결과는 보지 않음
    delay(200);
    gpsSerial.updateBaudRate(GPS_BAUD);
    delay(200);
    return gnss.begin(gpsSerial, 1200) ? baud : 0;
  }
  gpsSerial.updateBaudRate(GPS_BAUD);
  return 0;
}

void setupGps() {
#if USE_GPS
  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(300);
  uint32_t gpsFoundBaud = beginGnssAnyBaud();
  bool gnssDetected = gpsFoundBaud > 0;
  bool gpsL5Configured = gnssDetected && configureMgF10L5();
  if (gpsL5Configured) delay(600);  // GNSS 재시작 대기
  bool gps5HzConfigured = gnssDetected && configureMgF10FiveHz();
  Serial.printf("[GPS] L5 enable+health override=%s\n",
                gpsL5Configured ? "ACK" : "FAILED");
  Serial.printf(
    "[GPS] u-blox detected=%u, 5Hz config=%s, GGA+RMC, 115200bps (found %lu)\n",
    gnssDetected ? 1u : 0u,
    gps5HzConfigured ? "ACK" : "FAILED",
    (unsigned long)gpsFoundBaud
  );
  Serial.println("[GPS] GPS TX -> ESP32 GPIO16 RX2");
  Serial.println("[GPS] GPS RX -> ESP32 GPIO17 TX2");
#endif
}

void recoverGps5HzIfNeeded() {
#if USE_GPS
  // 위험 알림 중에는 GPS 재설정의 블로킹 대기로 인해
  // 진동/부저 및 RSU 위험 수신 처리가 끊기지 않도록 복구를 미룬다.
  if (currentRisk >= RISK_CAUTION) return;
  if (!gps5HzRecoveryRequested) return;

  uint32_t now = millis();
  if (now < GPS_5HZ_RECOVERY_MIN_UPTIME_MS) return;
  if (lastGps5HzRecoveryAttemptMs > 0 &&
      now - lastGps5HzRecoveryAttemptMs <
        GPS_5HZ_RECOVERY_COOLDOWN_MS) {
    return;
  }

  lastGps5HzRecoveryAttemptMs = now;
  lastGps5HzRecoveryStartedMs = now;
  gps5HzRecoveryCount++;
  Serial.printf(
    "[GPS] observed %.0fms updates -> retrying 5Hz configuration\n",
    gpsObservedUpdateIntervalMs
  );

  bool recovered = configureMgF10FiveHz();
  Serial.printf(
    "[GPS] runtime 5Hz recovery=%s\n",
    recovered ? "ACK" : "FAILED"
  );

  // 재설정 뒤에는 실제 NMEA 간격을 처음부터 다시 측정한다.
  lastGpsMeasurementMs = 0;
  gpsObservedUpdateIntervalMs = 200.0f;
  if (recovered) gps5HzRecoveryRequested = false;
#endif
}

bool saveImuCalibration() {
  if (!imuReady || !bno.isFullyCalibrated()) return false;
  uint8_t offsets[NUM_BNO055_OFFSET_REGISTERS];
  if (!bno.getSensorOffsets(offsets)) return false;
  Preferences prefs;
  if (!prefs.begin("v5imu", false)) return false;
  bool ok = prefs.putBytes("offsets", offsets, sizeof(offsets)) == sizeof(offsets);
  prefs.putBool("aligned", imuHeadingAligned);
  prefs.putFloat("head_off", imuHeadingOffsetDeg);
  prefs.end();
  if (ok) imuOffsetsRestored = true;
  return ok;
}

void resetImuCalibration() {
  Preferences prefs;
  if (prefs.begin("v5imu", false)) {
    prefs.clear();
    prefs.end();
  }
  imuOffsetsRestored = false;
  imuHeadingAligned = false;
  imuHeadingOffsetDeg = 0.0f;
  caneImuHeadingHasFix = false;
  walkAlignActive = false;
  resetWalkAlignSamples();
}

// 장착 보정값(IMU 원시 방위 → 실제 방위)을 적용하고 플래시에 저장한다.
bool applyImuHeadingOffset(float offsetDeg, float currentRawDeg) {
  imuHeadingOffsetDeg = normalizeHeading(offsetDeg);
  imuHeadingAligned = true;
  caneImuHeadingDeg = normalizeHeading(currentRawDeg + imuHeadingOffsetDeg);
  caneImuHeadingHasFix = true;
  lastCaneHeadingAcceptedMs = millis();
  Preferences prefs;
  if (!prefs.begin("v5imu", false)) return false;
  prefs.putBool("aligned", true);
  prefs.putFloat("head_off", imuHeadingOffsetDeg);
  prefs.end();
  return true;
}

bool alignImuHeading(float trueHeadingDeg) {
  if (!imuReady || lastImuSampleMs == 0 || !isfinite(trueHeadingDeg)) return false;
  return applyImuHeadingOffset(trueHeadingDeg - imuRawHeadingDeg, imuRawHeadingDeg);
}

void startFeedback(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  feedbackPulses = pulses;
  feedbackOnMs = onMs;
  feedbackOffMs = offMs;
  feedbackStartMs = millis();
}

void updateImuRawLowPass(uint32_t now) {
  float rad = imuRawHeadingDeg * DEG_TO_RAD;
  if (!imuRawLpReady) {
    imuRawLpSin = sinf(rad);
    imuRawLpCos = cosf(rad);
    imuRawLpReady = true;
  } else {
    float dt = constrain((now - imuRawLpLastMs) / 1000.0f, 0.001f, 0.5f);
    float alpha = 1.0f - expf(-dt / WALK_ALIGN_RAW_LP_TAU_S);
    imuRawLpSin += alpha * (sinf(rad) - imuRawLpSin);
    imuRawLpCos += alpha * (cosf(rad) - imuRawLpCos);
  }
  imuRawLpLastMs = now;
}

float imuRawLowPassDeg() {
  return normalizeHeading(atan2f(imuRawLpSin, imuRawLpCos) * RAD_TO_DEG);
}

void resetWalkAlignSamples() {
  walkAlignSamples = 0;
  walkAlignSumSin = 0.0f;
  walkAlignSumCos = 0.0f;
  walkAlignLastCourseDeg = -1.0f;
}

void startWalkAlignment() {
  resetWalkAlignSamples();
  walkAlignNoGps = 0;
  walkAlignSlow = 0;
  walkAlignImuLow = 0;
  walkAlignActive = true;
  walkAlignStartedMs = millis();
  walkAlignLastLogMs = walkAlignStartedMs;
  startFeedback(1, 150, 0);
  cmdReply("방향 정렬 시작. 앞을 보고 그대로 똑바로 걸을 것(약 5초)");
}

void stopWalkAlignment(const char *reason) {
  if (!walkAlignActive) return;
  walkAlignActive = false;
  resetWalkAlignSamples();
  cmdReply("방향 정렬 중지 (%s). 기존 정렬값 유지", reason);
}

float walkAlignSpreadDeg(float resultant) {
  return sqrtf(-2.0f * logf(fmaxf(resultant, 1e-6f))) * RAD_TO_DEG;
}

// GPS 표본마다 호출. GPS 진행방향과 스윙을 평균낸 IMU 방위의 차이를 모은다.
void addWalkAlignmentSample(bool gpsOk, float speedMps, bool courseOk,
                            float courseDeg) {
  if (!walkAlignActive) return;

  if (!gpsOk || !courseOk) {
    walkAlignNoGps++;
    walkAlignLastCourseDeg = -1.0f;
    return;
  }
  if (speedMps < WALK_ALIGN_MIN_SPEED_MPS) {
    walkAlignSlow++;
    walkAlignLastCourseDeg = -1.0f;
    return;
  }
  if (!imuReady || !imuRawLpReady || imuCalGyro < 2 || imuCalMag < 2 ||
      millis() - lastImuSampleMs > 200UL) {
    walkAlignImuLow++;
    walkAlignLastCourseDeg = -1.0f;
    return;
  }

  // 방향을 꺾는 중이면 저역통과한 IMU 방위가 늦게 따라오므로 그 표본은 버린다.
  float courseStep = walkAlignLastCourseDeg < 0.0f ? 0.0f :
    fabsf(fmodf(courseDeg - walkAlignLastCourseDeg + 540.0f, 360.0f) - 180.0f);
  walkAlignLastCourseDeg = courseDeg;
  if (courseStep > WALK_ALIGN_MAX_COURSE_STEP_DEG) return;

  float diffRad = (courseDeg - imuRawLowPassDeg()) * DEG_TO_RAD;
  walkAlignSumSin += sinf(diffRad);
  walkAlignSumCos += cosf(diffRad);
  walkAlignSamples++;

  float resultant = sqrtf(walkAlignSumSin * walkAlignSumSin +
                          walkAlignSumCos * walkAlignSumCos) / walkAlignSamples;
  if (walkAlignSamples >= WALK_ALIGN_MIN_SAMPLES &&
      resultant >= WALK_ALIGN_MIN_RESULTANT) {
    float offsetDeg = atan2f(walkAlignSumSin, walkAlignSumCos) * RAD_TO_DEG;
    uint16_t used = walkAlignSamples;
    walkAlignActive = false;
    resetWalkAlignSamples();
    bool saved = applyImuHeadingOffset(offsetDeg, imuRawLowPassDeg());
    startFeedback(2, 150, 150);
    cmdReply("방향 정렬 완료: 보정 %.1f도, 퍼짐 %.1f도, 표본 %u개%s",
             normalizeHeading(offsetDeg), walkAlignSpreadDeg(resultant),
             (unsigned)used, saved ? "" : " (플래시 저장 실패)");
    return;
  }
  if (walkAlignSamples >= WALK_ALIGN_MAX_SAMPLES) {
    cmdReply("방향 정렬: 걸음이 곧지 않아 다시 모음 (퍼짐 %.0f도)",
             walkAlignSpreadDeg(resultant));
    resetWalkAlignSamples();
  }
}

// loop마다 호출: 버튼 2초 누름 → 걸어서 방향 정렬, 시간 초과 처리.
void handleCaneButton() {
  // 실수로 스쳐 눌러 정렬이 바뀌지 않게, 2초 넘게 누를 때만 한 번 시작한다.
  // 정렬 중에 다시 2초 누르면 처음부터 다시 모은다.
  static uint32_t buttonDownSinceMs = 0;
  static bool buttonFired = false;
  uint32_t now = millis();

  bool buttonPressed = digitalRead(BUTTON_PIN) == LOW;
  if (!buttonPressed) {
    buttonDownSinceMs = 0;
    buttonFired = false;
  } else if (buttonDownSinceMs == 0) {
    buttonDownSinceMs = now;
  } else if (!buttonFired && now - buttonDownSinceMs >= WALK_ALIGN_HOLD_MS) {
    buttonFired = true;
    Serial.println("[BUTTON] 2초 누름 → 걸어서 방향 정렬 시작");
    startWalkAlignment();
  }

  if (!walkAlignActive) return;
  if (now - walkAlignStartedMs > WALK_ALIGN_TIMEOUT_MS) {
    walkAlignActive = false;
    resetWalkAlignSamples();
    startFeedback(1, 1000, 0);
    const char *reason = "걸음이 곧지 않음";
    uint16_t most = 0;
    if (walkAlignNoGps > most) { most = walkAlignNoGps; reason = "GPS 없음(실외에서 할 것)"; }
    if (walkAlignSlow > most) { most = walkAlignSlow; reason = "걷는 속도 부족(0.6 m/s 이상)"; }
    if (walkAlignImuLow > most) { most = walkAlignImuLow; reason = "IMU 자이로·자력계 보정 부족(G·M 2 이상)"; }
    cmdReply("방향 정렬 실패: %s. 기존 정렬값 유지", reason);
    return;
  }
  if (now - walkAlignLastLogMs >= 2000UL) {
    walkAlignLastLogMs = now;
    Serial.printf("[ALIGN] 표본 %u/%u, GPS없음 %u, 느림 %u, IMU부족 %u\n",
                  (unsigned)walkAlignSamples, (unsigned)WALK_ALIGN_MIN_SAMPLES,
                  (unsigned)walkAlignNoGps, (unsigned)walkAlignSlow,
                  (unsigned)walkAlignImuLow);
  }
}

void reportImuStatus() {
  cmdReply("IMU ready=%u cal S/G/A/M=%u/%u/%u/%u offsets=%u aligned=%u raw=%.1f heading=%.1f offset=%.1f walk=%u n=%u",
           imuReady ? 1u : 0u, imuCalSystem, imuCalGyro, imuCalAccel, imuCalMag,
           imuOffsetsRestored ? 1u : 0u, imuHeadingAligned ? 1u : 0u,
           imuRawHeadingDeg, caneImuHeadingDeg, imuHeadingOffsetDeg,
           walkAlignActive ? 1u : 0u, (unsigned)walkAlignSamples);
}

void setupImu() {
#if USE_IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (bno.begin(OPERATION_MODE_NDOF)) {
    imuReady = true;
#if BNO_USE_EXTERNAL_CRYSTAL
    bno.setExtCrystalUse(true);
#endif
    Preferences prefs;
    if (prefs.begin("v5imu", true)) {
      uint8_t offsets[NUM_BNO055_OFFSET_REGISTERS];
      if (prefs.getBytesLength("offsets") == sizeof(offsets) &&
          prefs.getBytes("offsets", offsets, sizeof(offsets)) == sizeof(offsets)) {
        bno.setSensorOffsets(offsets);
        imuOffsetsRestored = true;
      }
      imuHeadingAligned = prefs.getBool("aligned", false);
      imuHeadingOffsetDeg = prefs.getFloat("head_off", 0.0f);
      prefs.end();
    }
    Serial.printf("[IMU] BNO055 connected 0x%02X, mode=NDOF, offsets=%u aligned=%u\n",
                  BNO055_I2C_ADDRESS, imuOffsetsRestored ? 1u : 0u,
                  imuHeadingAligned ? 1u : 0u);
  } else {
    Serial.println("[IMU] BNO055 연결 실패: 3.3V, SDA21/SCL22, COM3=GND, 주소 0x28 확인");
  }
#endif
}

void readGps() {
#if USE_GPS
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  uint32_t now = millis();

  // GGA와 RMC에 둘 다 위치가 있으므로 RMC에서 속도와 함께
  // 갱신된 위치만 처리해 1회의 GPS fix를 두 번 카운트하지 않는다.
  if (gps.location.isUpdated() && gps.speed.isUpdated()) {
    updateObservedGpsRate(now);
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
      rawSpeed <= cfgGpsMaxSpeed;
    bool courseOk = rawGpsCourseValid;
    bool velocityOk = speedOk && courseOk;
    bool qualityOk = locationOk && gpsQualityIsGood();

    addWalkAlignmentSample(qualityOk && speedOk, rawSpeed, courseOk,
                           rawGpsCourseDeg);

    if (!qualityOk) {
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
          lastSpeed =
            gpsAcceptedCount <= 1
              ? rawSpeed
              : 0.65f * lastSpeed + 0.35f * rawSpeed;
        }

        if (courseOk &&
            rawSpeed >= cfgGpsMinCourseSpeed) {
          float rawHeading = rawGpsCourseDeg;
          lastHeading =
            gpsAcceptedCount <= 1
              ? rawHeading
              : blendHeading(lastHeading, rawHeading, 0.30f);
        }
      }
    }
  }

  // 품질이 나쁜 새 fix가 와도 마지막 정상 fix가 3초 이내면 짧게 예측.
  if (gpsFilter.initialized &&
      now - gpsFilter.lastFixMs <= cfgGpsFixMaxAgeMs) {
    double filteredLat;
    double filteredLng;
    projectGpsPosition(now, &filteredLat, &filteredLng);
    lastLat = (float)filteredLat;
    lastLng = (float)filteredLng;
    lastGpsValid = 1;
  } else {
    lastGpsValid = 0;
    lastSpeed = 0.0f;
  }
#endif
}

float caneDot3(float ax, float ay, float az,
               float bx, float by, float bz) {
  return ax * bx + ay * by + az * bz;
}

float caneNorm3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

bool estimateCaneHeadingFromMeasuredModel(float *outHeadingDeg) {
  caneHeadingAccelNorm = caneNorm3(lastAccelX, lastAccelY, lastAccelZ);
  caneHeadingGyroNorm = caneNorm3(lastGyroX, lastGyroY, lastGyroZ);
  caneHeadingPoseCos = 1.0f;
  caneMagModelResidual = imuCalMag < 2 ? 999.0f : 0.0f;
  caneMagModelRadius = imuCalMag / 3.0f;

  // 새 하드웨어는 BNO055 내부 센서융합 결과를 사용한다. 장착 방향까지
  // 정렬해야 위험 예측에 방향값을 내보낸다. 방위는 자이로·자력계 보정도로
  // 판단한다. 가속도 보정도(A)는 6방향 정지를 다시 해야 오르는데, 저장한
  // 오프셋을 불러와도 재부팅 직후엔 낮게 보일 수 있어 조건에서 뺀다.
  if (!imuHeadingAligned || imuCalGyro < 2 || imuCalMag < 2) {
    return false;
  }
  *outHeadingDeg = normalizeHeading(imuRawHeadingDeg + imuHeadingOffsetDeg);
  return isfinite(*outHeadingDeg);
}

bool caneHeadingIsFresh() {
  return caneImuHeadingHasFix &&
         lastCaneHeadingAcceptedMs > 0 &&
         millis() - lastCaneHeadingAcceptedMs <=
           CANE_HEADING_VALID_TIMEOUT_MS;
}

void updateCaneHeading() {
  uint32_t now = millis();
  float measuredHeadingDeg = 0.0f;

  if (estimateCaneHeadingFromMeasuredModel(&measuredHeadingDeg)) {
    bool previousFresh = caneHeadingIsFresh();

    if (!previousFresh) {
      caneImuHeadingDeg = measuredHeadingDeg;
    } else {
      float dt =
        (now - lastCaneHeadingAcceptedMs) / 1000.0f;
      dt = constrain(dt, 0.001f, 0.50f);
      float alpha =
        1.0f - expf(-dt / CANE_HEADING_FILTER_TAU_S);

      caneImuHeadingDeg = blendHeading(
        caneImuHeadingDeg,
        measuredHeadingDeg,
        alpha
      );
    }

    caneImuHeadingHasFix = true;
    lastCaneHeadingAcceptedMs = now;
    caneHeadingAcceptedCount++;
  } else {
    caneHeadingRejectedCount++;

    if (lastCaneHeadingAcceptedMs == 0 ||
        now - lastCaneHeadingAcceptedMs >
          CANE_HEADING_VALID_TIMEOUT_MS) {
      caneImuHeadingHasFix = false;
    }
  }
}

void readImu() {
#if USE_IMU
  uint32_t now = millis();
  if (!imuReady || now - lastImuReadAttemptMs < IMU_SAMPLE_INTERVAL_MS) return;
  lastImuReadAttemptMs = now;

  // 100kHz I2C에서 벡터 하나 읽는 데 약 1ms가 든다. 매 10ms에는 방위와
  // 자이로만 읽고, 가속도·자력계·보정상태는 IMU_SLOW_INTERVAL_MS마다 읽는다.
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  lastGyroX = gyro.x(); lastGyroY = gyro.y(); lastGyroZ = gyro.z();
  imuRawHeadingDeg = normalizeHeading(euler.x());
  updateImuRawLowPass(now);

  static uint32_t lastSlowImuReadMs = 0;
  if (lastSlowImuReadMs == 0 || now - lastSlowImuReadMs >= IMU_SLOW_INTERVAL_MS) {
    lastSlowImuReadMs = now;
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getCalibration(&imuCalSystem, &imuCalGyro, &imuCalAccel, &imuCalMag);
    lastAccelX = accel.x(); lastAccelY = accel.y(); lastAccelZ = accel.z();
    lastMagX = mag.x(); lastMagY = mag.y(); lastMagZ = mag.z();
  }
  lastImuSampleMs = now;

  updateCaneHeading();
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
  float caneLat = lastGpsValid
                    ? lastLat
                    : (USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LAT : 0.0f);
  float caneLng = lastGpsValid
                    ? lastLng
                    : (USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LNG : 0.0f);

  float avgLatRad =
    0.5f * (caneLat + vehicle.latitude) * DEG_TO_RAD;
  float rx = (vehicle.longitude - caneLng) *
             111320.0f * cosf(avgLatRad);
  float ry = (vehicle.latitude - caneLat) * 111132.0f;
  float distanceM = sqrtf(rx * rx + ry * ry);

  *outDistance = distanceM;
  *outClosingSpeed = 0.0f;
  *outTtc = 999.0f;

  if (distanceM < EMERGENCY_DISTANCE_M) return RISK_DANGER;

  bool vehicleMoving =
    vehicle.heading_valid &&
    vehicle.speed_mps >= MIN_PREDICT_VEHICLE_SPEED_MPS;
  if (!vehicleMoving) return RISK_SAFE;

  float vehicleRad = vehicle.heading_deg * DEG_TO_RAD;
  float vehicleVx = vehicle.speed_mps * sinf(vehicleRad);
  float vehicleVy = vehicle.speed_mps * cosf(vehicleRad);

  bool caneMoving =
    caneHeadingIsFresh() &&
    lastSpeed >= MIN_PREDICT_CANE_SPEED_MPS;

  float caneVx = 0.0f;
  float caneVy = 0.0f;
  if (caneMoving) {
    float caneRad = caneImuHeadingDeg * DEG_TO_RAD;
    caneVx = lastSpeed * sinf(caneRad);
    caneVy = lastSpeed * cosf(caneRad);
  }

  // 차량-지팡이 상대속도.
  float rvx = vehicleVx - caneVx;
  float rvy = vehicleVy - caneVy;
  float relativeSpeedSq = rvx * rvx + rvy * rvy;
  if (relativeSpeedSq < 0.01f) return RISK_SAFE;

  float dot = rx * rvx + ry * rvy;
  float closingSpeed = -dot / fmaxf(distanceM, 0.1f);
  float tCpa = -dot / relativeSpeedSq;
  *outClosingSpeed = closingSpeed;

  if (closingSpeed < MIN_RADIAL_CLOSING_SPEED_MPS ||
      tCpa <= 0.0f ||
      tCpa > CPA_PREDICTION_HORIZON_S) {
    return RISK_SAFE;
  }

  float closestX = rx + rvx * tCpa;
  float closestY = ry + rvy * tCpa;
  float dCpa = sqrtf(closestX * closestX + closestY * closestY);
  *outTtc = tCpa;

  if (dCpa <= CPA_DANGER_DISTANCE_M &&
      tCpa <= 1.5f) return RISK_DANGER;
  if (dCpa <= CPA_WARNING_DISTANCE_M &&
      tCpa <= 3.0f) return RISK_WARNING;
  if (dCpa <= CPA_CAUTION_DISTANCE_M &&
      tCpa <= 5.0f) return RISK_CAUTION;
  return RISK_SAFE;
}

void applyRisk(uint8_t risk) {
  bool manualTestActive =
    manualOutputTestUntilMs > 0 &&
    (int32_t)(manualOutputTestUntilMs - millis()) > 0;

  // 자체시험 중에는 수신 패킷이 시험 출력을 SAFE로 덮지 못하게 한다.
  if (manualTestActive && risk != manualOutputTestRisk) {
    return;
  }

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
bool applyMergedRisk() {
  uint32_t now = millis();

  bool vehicleFresh =
    vehicleRiskMs > 0 &&
    now - vehicleRiskMs <= cfgDirectRiskTimeoutMs;

  bool rsuFresh = false;
#if ENABLE_RSU_RISK_INPUT
  rsuFresh =
    rsuRiskMs > 0 &&
    now - rsuRiskMs <= RSU_RISK_TIMEOUT_MS;
#endif

  // 둘 다 끊겼으면 호출자가 자체 계산 또는 SAFE 처리를 선택한다.
  if (!vehicleFresh && !rsuFresh) return false;

  uint8_t merged = RISK_SAFE;
  if (vehicleFresh && vehicleRiskLevel > merged) merged = vehicleRiskLevel;
  if (rsuFresh && rsuRiskLevel > merged) merged = rsuRiskLevel;

  applyRisk(merged);
  return true;
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
  } else {
    txStatus.gps_valid = USE_FIXED_GPS_FALLBACK ? 1 : 0;
    txStatus.latitude = USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LAT : 0.0f;
    txStatus.longitude = USE_FIXED_GPS_FALLBACK ? CANE_FIXED_LNG : 0.0f;
    txStatus.speed_mps = 0.0f;
  }

  // 저속 보행 GPS course는 실측에서 100도 이상 어긋난 경우가 있어
  // 지팡이 진행방향 fallback으로 사용하지 않는다.
  bool headingFresh = caneHeadingIsFresh();
  txStatus.heading_valid = headingFresh ? 1 : 0;
  txStatus.heading_deg = headingFresh ? caneImuHeadingDeg : 0.0f;

  txStatus.timestamp_ms = millis();
  txStatus.seq_num = seq++;
  txStatus.rssi_distance_m = -1.0f;   // v4: 지팡이는 미측정
}

void sendCaneStatus() {
  buildStatusPacket();
  esp_err_t result = esp_now_send(broadcastMAC, (uint8_t *)&txStatus, sizeof(txStatus));
  sendCount++;

  Serial.printf(
    "[CANE SEND] id=%lu seq=%u gps=%u headingOk=%u heading=%.1f "
    "lat=%.6f lng=%.6f spd=%.2f risk=%u "
    "ax=%.2f ay=%.2f az=%.2f send=%lu result=%s\n",
    (unsigned long)txStatus.node_id,
    txStatus.seq_num,
    txStatus.gps_valid,
    txStatus.heading_valid,
    txStatus.heading_deg,
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

#if !ENABLE_RSU_RISK_INPUT
  if (riskMsg.node_type == NODE_RSU) {
    Serial.println("[CANE RX] RSU risk ignored (direct vehicle mode)");
    return;
  }
#endif

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
#if !ENABLE_RSU_RISK_INPUT
  Serial.println("[CANE RX LEGACY] RSU reply ignored (direct vehicle mode)");
  return;
#endif
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

  // 차량 직접판정과 RSU 판정을 출처별 유효시간으로 병합한다.
  // 차량 상태 패킷의 SAFE가 살아 있는 RSU 경고를 지우지 못하게 한다.
  if (applyMergedRisk()) {
    Serial.printf(
      "[CANE VEHICLE STATUS] vehicle=%lu seq=%u "
      "direct risk active -> local calculation skipped\n",
      (unsigned long)vehicleMsg.node_id,
      vehicleMsg.seq_num
    );
    return;
  }

#if !ENABLE_CANE_LOCAL_GPS_RISK_FALLBACK
  // 독립 GPS 두 개의 상대 바이어스로 인한 오경보를 막는다.
  // 직접 위험 패킷이 없을 때는 SAFE로 두고 차량/RSU 판정을 기다린다.
  applyRisk(RISK_SAFE);
  return;
#endif

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

void handleUwbRange(const v2x_uwb_message_t &message) {
  if (message.node_type != NODE_VEHICLE) return;
  if (message.target_id != 0 &&
      message.target_id != caneId &&
      message.target_id != 0xFFFFFFFFUL) {
    return;
  }

  portENTER_CRITICAL(&uwbMux);
  latestUwbRange = message;
  hasUwbRange = true;
  uwbRxCount++;
  lastUwbRxMs = millis();
  portEXIT_CRITICAL(&uwbMux);
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
  uint8_t receivedNodeType = data[6];

  if (receivedMagic != V2X_MAGIC ||
      receivedVersion != V2X_VERSION) {
    Serial.printf(
      "[CANE RX] invalid magic/version len=%d version=%u\n",
      len,
      receivedVersion
    );
    return;
  }

  // 차량에서 온 정상 V2X 패킷이면 종류와 관계없이 RSSI를 기록한다.
  if (receivedNodeType == NODE_VEHICLE &&
      info != nullptr &&
      info->rx_ctrl != nullptr) {
    int8_t receivedRssiDbm = info->rx_ctrl->rssi;
    uint32_t receivedAtMs = millis();

    portENTER_CRITICAL(&rssiMux);
    latestVehicleRssiDbm = receivedRssiDbm;

    if (!hasVehicleRssi) {
      filteredVehicleRssiDbm = (float)receivedRssiDbm;
      hasVehicleRssi = true;
    } else {
      filteredVehicleRssiDbm =
        cfgRssiAlpha * (float)receivedRssiDbm +
        (1.0f - cfgRssiAlpha) * filteredVehicleRssiDbm;
    }

    vehicleRssiSampleCount++;
    lastVehicleRssiMs = receivedAtMs;
    portEXIT_CRITICAL(&rssiMux);
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

  if (receivedMsgType == MSG_UWB_RANGE) {
    if (len != sizeof(v2x_uwb_message_t)) {
      Serial.printf(
        "[CANE RX] invalid UWB packet size len=%d expected=%u\n",
        len,
        (unsigned int)sizeof(v2x_uwb_message_t)
      );
      return;
    }

    v2x_uwb_message_t uwbMessage;
    memcpy(&uwbMessage, data, sizeof(uwbMessage));
    handleUwbRange(uwbMessage);
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
  // 8/18 패치: 송신 전력 20→11 dBm — 브라운아웃(Wi-Fi 기동/송신 피크 전류)을 낮춘다.
  // 하드웨어(케이블·1000 µF 커패시터·전원)와 함께 쓰는 완화책. 링크 수십 m엔 충분.
  WiFi.setTxPower(WIFI_POWER_11dBm);
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

  // 8/18 패치: 20초 안에 연결하지 못해도 재부팅하지 않는다. (차량이 꺼져 있으면 21 s마다
  // 무한 재부팅 → RSU 무음.) 차량 AP 채널(6)에 맞춰 ESP-NOW 를 먼저 살리고, Wi-Fi(UDP
  // 텔레메트리·명령용)는 loop 의 5 s 재접속이 붙을 때 붙는다. ESP-NOW 는 STA 연결과 무관.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(
      "[WIFI] vehicle Wi-Fi connection failed -> ESP-NOW on channel 6, retry Wi-Fi in loop"
    );
    esp_wifi_set_channel(V2X_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  } else {
    Serial.println("[WIFI] connected to vehicle");
  }

  Serial.print("[WIFI] IP=");
  Serial.println(WiFi.localIP());

  Serial.printf(
    "[WIFI] channel=%ld\n",
    (long)WiFi.channel()
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

// 지팡이 상태를 UDP 4210으로 전송
void sendUdpTelemetry() {
  if (WiFi.status() != WL_CONNECTED) return;

  char udpBuffer[1792];

  int8_t rssiRawSnapshot;
  float rssiFilteredSnapshot;
  bool rssiValidSnapshot;
  uint32_t rssiSampleCountSnapshot;
  uint32_t rssiLastMsSnapshot;
  v2x_uwb_message_t uwbSnapshot = {};
  bool uwbReceivedSnapshot;
  uint32_t uwbRxCountSnapshot;
  uint32_t uwbLastMsSnapshot;

  portENTER_CRITICAL(&rssiMux);
  rssiRawSnapshot = latestVehicleRssiDbm;
  rssiFilteredSnapshot = filteredVehicleRssiDbm;
  rssiValidSnapshot = hasVehicleRssi;
  rssiSampleCountSnapshot = vehicleRssiSampleCount;
  rssiLastMsSnapshot = lastVehicleRssiMs;
  portEXIT_CRITICAL(&rssiMux);

  portENTER_CRITICAL(&uwbMux);
  uwbSnapshot = latestUwbRange;
  uwbReceivedSnapshot = hasUwbRange;
  uwbRxCountSnapshot = uwbRxCount;
  uwbLastMsSnapshot = lastUwbRxMs;
  portEXIT_CRITICAL(&uwbMux);

  uint32_t telemetryMs = millis();
  long rssiAgeMs = rssiValidSnapshot
    ? (long)(telemetryMs - rssiLastMsSnapshot)
    : -1L;
  long vehicleRiskAgeMs = vehicleRiskMs > 0
    ? (long)(telemetryMs - vehicleRiskMs)
    : -1L;
  long rsuRiskAgeMs = rsuRiskMs > 0
    ? (long)(telemetryMs - rsuRiskMs)
    : -1L;
  long uwbAgeMs = uwbReceivedSnapshot
    ? (long)(telemetryMs - uwbLastMsSnapshot)
    : -1L;
  bool uwbFreshSnapshot =
    uwbReceivedSnapshot && uwbAgeMs >= 0 && uwbAgeMs <= 1000L;

  if (cfgLogMode == 0) {
    // 키 이름은 V4와 같게 둔다(뷰어 요약·기록 CSV가 이 이름을 쓴다).
    int essentialWritten = snprintf(
      udpBuffer, sizeof(udpBuffer),
      "시각ms:%lu\n위험:%u\n차량위험:%u\n차량위험경과ms:%ld\n"
      "RSU위험:%u\nRSU위험경과ms:%ld\n"
      "GPS유효:%u\n위도:%.6f\n경도:%.6f\n속도:%.2f\n"
      "GPS위성:%lu\nGPS_HDOP:%.2f\nGPS복구횟수:%lu\n"
      "방향:%.1f\nIMU방향유효:%u\nIMU방향:%.1f\nIMU보정:%u/%u/%u/%u\nIMU정렬:%u\n"
      "IMU정렬중:%u\nIMU정렬표본:%u\n"
      "UWB유효:%u\nUWB보정:%u\nUWB보정거리:%.3f\nUWB원시거리:%.3f\n"
      "UWB접근속도:%.3f\nUWB경과ms:%ld\n"
      "RSSI원시:%d\nRSSI평활:%.1f\nRSSI경과ms:%ld\n"
      "송신:%lu\n차량수신:%lu\n부팅횟수:%lu\n리셋원인:%s\n",
      (unsigned long)telemetryMs,
      currentRisk == 255 ? 0 : currentRisk,
      vehicleRiskLevel, vehicleRiskAgeMs, rsuRiskLevel, rsuRiskAgeMs,
      lastGpsValid, lastLat, lastLng, lastSpeed,
      (unsigned long)rawGpsSatellites, rawGpsHdop,
      (unsigned long)gps5HzRecoveryCount,
      caneHeadingIsFresh() ? caneImuHeadingDeg : 0.0f,
      caneHeadingIsFresh() ? 1u : 0u, caneImuHeadingDeg,
      imuCalSystem, imuCalGyro, imuCalAccel, imuCalMag,
      imuHeadingAligned ? 1u : 0u,
      walkAlignActive ? 1u : 0u, (unsigned)walkAlignSamples,
      uwbFreshSnapshot && (uwbSnapshot.flags & 0x01) ? 1u : 0u,
      (uwbSnapshot.flags & 0x02) ? 1u : 0u,
      uwbSnapshot.distance_m, uwbSnapshot.raw_distance_m,
      uwbSnapshot.closing_speed_mps, uwbAgeMs,
      (int)rssiRawSnapshot, rssiFilteredSnapshot, rssiAgeMs,
      (unsigned long)sendCount, (unsigned long)vehicleRxCount,
      (unsigned long)caneBootCount, resetReasonName(caneResetReason)
    );
    if (essentialWritten <= 0) return;
    size_t essentialLength = essentialWritten < (int)sizeof(udpBuffer)
      ? (size_t)essentialWritten : sizeof(udpBuffer) - 1;
    if (logUdp.beginPacket(udpBroadcastAddress, CANE_UDP_PORT)) {
      logUdp.write((const uint8_t *)udpBuffer, essentialLength);
      if (logUdp.endPacket() != 1) Serial.println("[UDP] send failed");
    }
    IPAddress vehicleIp = WiFi.gatewayIP();
    if (vehicleIp != IPAddress((uint32_t)0) &&
        logUdp.beginPacket(vehicleIp, CANE_UDP_PORT)) {
      logUdp.write((const uint8_t *)udpBuffer, essentialLength);
      logUdp.endPacket();
    }
    return;
  }

  int written = snprintf(
    udpBuffer,
    sizeof(udpBuffer),
    "시각ms:%lu\n"
    "IMU시각ms:%lu\n"
    "부팅횟수:%lu\n"
    "리셋원인:%s\n"
    "GPS복구횟수:%lu\n"
    "마지막GPS복구ms:%lu\n"
    "위험:%u\n"
    "차량위험:%u\n"
    "차량위험경과ms:%ld\n"
    "RSU위험:%u\n"
    "RSU위험경과ms:%ld\n"
    "GPS유효:%u\n"
    "위도:%.6f\n"
    "경도:%.6f\n"
    "속도:%.3f\n"
    "방향:%.1f\n"
    "GPS원시속도:%.3f\n"
    "GPS원시방향유효:%u\n"
    "GPS원시방향:%.1f\n"
    "가속도:%.3f,%.3f,%.3f\n"
    "자이로:%.3f,%.3f,%.3f\n"
    "자력계:%.3f,%.3f,%.3f\n"
    "IMU방향유효:%u\n"
    "IMU방향:%.1f\n"
    "방향가속도크기:%.2f\n"
    "방향자이로크기:%.2f\n"
    "방향자세cos:%.4f\n"
    "자기모델잔차:%.2f\n"
    "자기모델반경:%.3f\n"
    "방향채택:%lu\n"
    "방향거부:%lu\n"
    "송신:%lu\n"
    "GPS위성:%lu\n"
    "GPS_HDOP:%.2f\n"
    "원시위도:%.6f\n"
    "원시경도:%.6f\n"
    "GPS채택:%lu\n"
    "GPS이상치거부:%lu\n"
    "GPS품질거부:%lu\n"
    "GPS재기준:%lu\n"
    "RSSI원시:%d\n"
    "RSSI평활:%.1f\n"
    "RSSI샘플:%lu\n"
    "RSSI경과ms:%ld\n"
    "차량수신:%lu\n"
    "UWB유효:%u\n"
    "UWB보정:%u\n"
    "UWB원시거리:%.3f\n"
    "UWB보정거리:%.3f\n"
    "UWB접근속도:%.3f\n"
    "UWB오프셋:%.3f\n"
    "UWB경과ms:%ld\n"
    "UWB수신:%lu\n"
    // V5에서 추가된 값 (상세 모드에도 같은 이름으로 보낸다)
    "IMU보정:%u/%u/%u/%u\n"
    "IMU정렬:%u\n"
    "IMU정렬중:%u\n"
    "IMU정렬표본:%u\n",
    (unsigned long)telemetryMs,
    (unsigned long)lastImuSampleMs,
    (unsigned long)caneBootCount,
    resetReasonName(caneResetReason),
    (unsigned long)gps5HzRecoveryCount,
    (unsigned long)lastGps5HzRecoveryStartedMs,
    currentRisk == 255 ? 0 : currentRisk,
    vehicleRiskLevel,
    vehicleRiskAgeMs,
    rsuRiskLevel,
    rsuRiskAgeMs,
    lastGpsValid,
    lastLat,
    lastLng,
    lastSpeed,
    caneHeadingIsFresh() ? caneImuHeadingDeg : 0.0f,
    rawGpsSpeedMps,
    rawGpsCourseValid ? 1u : 0u,
    rawGpsCourseDeg,
    lastAccelX,
    lastAccelY,
    lastAccelZ,
    lastGyroX,
    lastGyroY,
    lastGyroZ,
    lastMagX,
    lastMagY,
    lastMagZ,
    caneHeadingIsFresh() ? 1u : 0u,
    caneImuHeadingDeg,
    caneHeadingAccelNorm,
    caneHeadingGyroNorm,
    caneHeadingPoseCos,
    caneMagModelResidual,
    caneMagModelRadius,
    (unsigned long)caneHeadingAcceptedCount,
    (unsigned long)caneHeadingRejectedCount,
    (unsigned long)sendCount,
    (unsigned long)rawGpsSatellites,
    rawGpsHdop,
    rawGpsLat,
    rawGpsLng,
    (unsigned long)gpsAcceptedCount,
    (unsigned long)gpsRejectedCount,
    (unsigned long)gpsQualityRejectedCount,
    (unsigned long)gpsRelocalizedCount,
    (int)rssiRawSnapshot,
    rssiFilteredSnapshot,
    (unsigned long)rssiSampleCountSnapshot,
    rssiAgeMs,
    (unsigned long)vehicleRxCount,
    uwbFreshSnapshot && (uwbSnapshot.flags & 0x01) ? 1u : 0u,
    (uwbSnapshot.flags & 0x02) ? 1u : 0u,
    uwbSnapshot.raw_distance_m,
    uwbSnapshot.distance_m,
    uwbSnapshot.closing_speed_mps,
    uwbSnapshot.offset_m,
    uwbAgeMs,
    (unsigned long)uwbRxCountSnapshot,
    imuCalSystem, imuCalGyro, imuCalAccel, imuCalMag,
    imuHeadingAligned ? 1u : 0u,
    walkAlignActive ? 1u : 0u,
    (unsigned)walkAlignSamples
  );

  if (written <= 0) return;

  size_t sendLength =
    written < (int)sizeof(udpBuffer)
      ? (size_t)written
      : sizeof(udpBuffer) - 1;

  if (!logUdp.beginPacket(udpBroadcastAddress, CANE_UDP_PORT)) {
    Serial.println("[UDP] beginPacket failed");
    return;
  }

  logUdp.write((const uint8_t *)udpBuffer, sendLength);

  if (logUdp.endPacket() != 1) {
    Serial.println("[UDP] send failed");
  }

  // 차량(AP)에게 같은 내용을 직접 한 번 더 보낸다.
  // 아이패드 웹뷰어는 차량이 중계한 지팡이 값을 보여주는데,
  // AP가 자기 자신에게 온 브로드캐스트를 받지 못하는 경우가 있어
  // 게이트웨이 주소로 직접 보내야 확실하게 전달된다.
  IPAddress vehicleIp = WiFi.gatewayIP();
  if (vehicleIp != IPAddress((uint32_t)0)) {
    if (logUdp.beginPacket(vehicleIp, CANE_UDP_PORT)) {
      logUdp.write((const uint8_t *)udpBuffer, sendLength);
      logUdp.endPacket();
    }
  }
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
    caneHeadingIsFresh() ? caneImuHeadingDeg : 0.0f,
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


void startCaneOutputTest(uint8_t risk) {
  if (risk == RISK_SAFE) {
    manualOutputTestUntilMs = 0;
    manualOutputTestRisk = RISK_SAFE;
    applyRisk(RISK_SAFE);
    cmdReply("진동/부저 테스트 중지");
    return;
  }

  manualOutputTestRisk = risk;
  manualOutputTestUntilMs = millis() + 5000UL;
  applyRisk(risk);
  cmdReply("진동/부저 테스트 risk=%u (5초간)", risk);
}

// 지팡이 전용 명령. 처리했으면 true를 돌려준다.
bool runDeviceCommand(const String &name, bool hasValue, float number) {
  // 숫자 한 글자만 입력하면 기존처럼 바로 액추에이터 테스트.
  if (name.length() == 1 && name[0] >= '0' && name[0] <= '3') {
    startCaneOutputTest((uint8_t)(name[0] - '0'));
    return true;
  }

  if (name == "test") {
    if (!hasValue) {
      cmdReply("사용법: test <0-3> (0=중지)");
      return true;
    }
    int risk = (int)number;
    if (risk < 0 || risk > 3) {
      cmdReply("test 범위는 0~3");
      return true;
    }
    startCaneOutputTest((uint8_t)risk);
    return true;
  }

  if (name == "imu") {
    reportImuStatus();
    return true;
  }
  if (name == "imusave") {
    bool ok = saveImuCalibration();
    cmdReply(ok ? "IMU 보정값 저장 완료" :
                  "IMU 저장 실패: 먼저 S/G/A/M 보정도를 모두 3으로 만들 것");
    return true;
  }
  if (name == "imureset") {
    resetImuCalibration();
    cmdReply("IMU 저장 보정/방향 정렬값 삭제 (재부팅 후 다시 보정)");
    return true;
  }
  if (name == "imuwalk") {
    if (hasValue && number == 0.0f) {
      if (walkAlignActive) stopWalkAlignment("명령");
      else cmdReply("방향 정렬 중이 아님");
    } else {
      startWalkAlignment();
    }
    return true;
  }
  if (name == "imualign") {
    if (!hasValue || number < 0.0f || number >= 360.0f) {
      cmdReply("사용법: imualign <보행자가 향한 실제 방위 0~359.9도> (걸어서 자동은 imuwalk 또는 버튼 2초)");
    } else {
      cmdReply(alignImuHeading(number) ? "IMU 지팡이방향 정렬/저장 완료" :
                                        "IMU 정렬 실패: 센서 샘플을 기다릴 것");
    }
    return true;
  }

  return false;
}

// =====================
// 실시간 튜닝 명령 (USB 시리얼 + WiFi UDP 공용)
// 뷰어 입력창에서 UDP로 보낸 명령과 USB 시리얼 입력을 같은 해석기로 처리한다.
// 응답은 명령을 보낸 쪽으로 되돌려주므로 뷰어 로그에 그대로 표시된다.
//
// 값 변경은 즉시 적용되지만 기본적으로 메모리에만 남는다.
// save 명령을 쓰면 플래시(NVS)에 저장되어 전원을 껐다 켜도 유지되고,
// factory 명령으로 코드에 적힌 기본값으로 되돌릴 수 있다.
// =====================
#define CMD_UDP_PORT 4300
#define TUNING_NVS_NAMESPACE "v2xtune"
// 이 노드의 명령 태그. 맥이 형제 STA(지팡이)에게 직접 못 보내는 문제 때문에
// 명령을 "@cane <명령>" 형태로 브로드캐스트하면 지팡이만 골라서 처리한다.
#define NODE_CMD_TAG "cane"

WiFiUDP cmdUdp;
bool cmdUdpStarted = false;
IPAddress cmdReplyIp;
uint16_t cmdReplyPort = 0;
// 지팡이는 맥에게 직접(유니캐스트) 못 보내므로 응답도 브로드캐스트로 보낸다.
bool cmdReplyBroadcast = true;
Preferences tuningPrefs;

enum TuningValueType {
  TUNING_FLOAT,
  TUNING_UINT32
};

typedef struct {
  const char *name;      // 명령 이름 (NVS 키로도 사용, 15자 이내)
  void *pointer;         // 실제 설정 변수
  uint8_t type;
  float minValue;
  float maxValue;
  const char *unit;      // 응답에 붙일 단위
  const char *note;      // help에 표시할 설명
} TuningParam;

TuningParam tuningParams[] = {
  {"alpha", &cfgGpsFilterAlpha, TUNING_FLOAT, 0.01f, 1.0f, "",
   "위치 보정 세기(클수록 원시GPS를 빨리 따라감)"},
  {"beta", &cfgGpsFilterBeta, TUNING_FLOAT, 0.0f, 1.0f, "",
   "속도 보정 세기"},
  {"gate", &cfgGpsOutlierBaseM, TUNING_FLOAT, 0.5f, 50.0f, "m",
   "이 거리보다 크게 튀면 이상치로 버림"},
  {"hdop", &cfgGpsMaxHdop, TUNING_FLOAT, 0.5f, 99.0f, "",
   "이 값보다 나쁜 HDOP은 품질 거부"},
  {"sats", &cfgGpsMinSatellites, TUNING_UINT32, 0.0f, 20.0f, "개",
   "최소 위성 수"},
  {"predict", &cfgGpsPredictionMaxMs, TUNING_UINT32, 0.0f, 5000.0f, "ms",
   "새 fix가 없을 때 예측할 최대 시간"},
  {"fixage", &cfgGpsFixMaxAgeMs, TUNING_UINT32, 500.0f, 10000.0f, "ms",
   "이 시간 지난 fix는 무효 처리"},
  {"coursespd", &cfgGpsMinCourseSpeed, TUNING_FLOAT, 0.0f, 5.0f, "m/s",
   "이 속도 이상일 때만 GPS 방향을 신뢰"},
  {"maxspeed", &cfgGpsMaxSpeed, TUNING_FLOAT, 0.5f, 60.0f, "m/s",
   "지팡이가 낼 수 있는 최대 속도"},
  {"rssialpha", &cfgRssiAlpha, TUNING_FLOAT, 0.01f, 1.0f, "",
   "RSSI 평활 계수"},
  {"risktimeout", &cfgDirectRiskTimeoutMs, TUNING_UINT32, 200.0f, 10000.0f, "ms",
   "차량 위험 패킷 유효시간"},
  {"rate", &cfgTelemetryIntervalMs, TUNING_UINT32, 20.0f, 5000.0f, "ms",
   "뷰어로 로그 보내는 주기"},
  {"logmode", &cfgLogMode, TUNING_UINT32, 0.0f, 1.0f, "",
   "0=필수값만, 1=V4 전체 진단값"},
};

const size_t tuningParamCount =
  sizeof(tuningParams) / sizeof(tuningParams[0]);

// 부팅 직후 기본값을 기억해 두고 factory 명령에서 사용한다.
float tuningDefaults[sizeof(tuningParams) / sizeof(tuningParams[0])];

void cmdReply(const char *format, ...) {
  char message[200];
  va_list args;
  va_start(args, format);
  vsnprintf(message, sizeof(message), format, args);
  va_end(args);

  Serial.print("[CMD] ");
  Serial.println(message);

  if (cmdReplyPort == 0 || !cmdUdpStarted) return;

  char line[220];
  int written = snprintf(line, sizeof(line), "[CMD] %s\n", message);
  if (written <= 0) return;

  size_t length = written < (int)sizeof(line)
                    ? (size_t)written
                    : sizeof(line) - 1;

  IPAddress replyDst = cmdReplyBroadcast ? udpBroadcastAddress : cmdReplyIp;
  if (cmdUdp.beginPacket(replyDst, cmdReplyPort)) {
    cmdUdp.write((const uint8_t *)line, length);
    cmdUdp.endPacket();
  }
}

float readTuningValue(size_t index) {
  if (tuningParams[index].type == TUNING_UINT32) {
    return (float)(*(uint32_t *)tuningParams[index].pointer);
  }
  return *(float *)tuningParams[index].pointer;
}

void writeTuningValue(size_t index, float value) {
  float clamped = constrain(value,
                            tuningParams[index].minValue,
                            tuningParams[index].maxValue);
  if (tuningParams[index].type == TUNING_UINT32) {
    *(uint32_t *)tuningParams[index].pointer = (uint32_t)(clamped + 0.5f);
  } else {
    *(float *)tuningParams[index].pointer = clamped;
  }
}

int findTuningParam(const String &name) {
  for (size_t i = 0; i < tuningParamCount; i++) {
    if (name.equals(tuningParams[i].name)) return (int)i;
  }
  return -1;
}

void captureTuningDefaults() {
  for (size_t i = 0; i < tuningParamCount; i++) {
    tuningDefaults[i] = readTuningValue(i);
  }
}

// 플래시에 저장된 값이 있으면 덮어쓴다. 부팅 시 한 번 호출한다.
void loadTuningFromFlash() {
  if (!tuningPrefs.begin(TUNING_NVS_NAMESPACE, true)) return;

  uint16_t restored = 0;
  for (size_t i = 0; i < tuningParamCount; i++) {
    if (!tuningPrefs.isKey(tuningParams[i].name)) continue;
    float saved = tuningPrefs.getFloat(tuningParams[i].name,
                                       tuningDefaults[i]);
    writeTuningValue(i, saved);
    restored++;
  }
  tuningPrefs.end();

  if (restored > 0) {
    Serial.printf("[CMD] 플래시에서 저장된 설정 %u개 복원\n",
                  (unsigned)restored);
  }
}

void saveTuningToFlash() {
  if (!tuningPrefs.begin(TUNING_NVS_NAMESPACE, false)) {
    cmdReply("저장 실패: 플래시를 열 수 없다");
    return;
  }
  for (size_t i = 0; i < tuningParamCount; i++) {
    tuningPrefs.putFloat(tuningParams[i].name, readTuningValue(i));
  }
  tuningPrefs.end();
  cmdReply("현재 설정 %u개를 플래시에 저장 (전원 껐다 켜도 유지)",
           (unsigned)tuningParamCount);
}

void restoreTuningDefaults() {
  for (size_t i = 0; i < tuningParamCount; i++) {
    writeTuningValue(i, tuningDefaults[i]);
  }
  if (tuningPrefs.begin(TUNING_NVS_NAMESPACE, false)) {
    tuningPrefs.clear();
    tuningPrefs.end();
  }
  cmdReply("코드에 적힌 기본값으로 되돌리고 플래시 저장분도 삭제");
}

void reportTuningValue(size_t index) {
  if (tuningParams[index].type == TUNING_UINT32) {
    cmdReply("%s = %lu%s", tuningParams[index].name,
             (unsigned long)(*(uint32_t *)tuningParams[index].pointer),
             tuningParams[index].unit);
  } else {
    cmdReply("%s = %.3f%s", tuningParams[index].name,
             *(float *)tuningParams[index].pointer,
             tuningParams[index].unit);
  }
}

void reportAllTuningValues() {
  for (size_t i = 0; i < tuningParamCount; i++) {
    reportTuningValue(i);
  }
}

void reportTuningHelp() {
  cmdReply("== 공통 명령 ==");
  cmdReply("get / save / load / factory / reset / help");
  cmdReply("== 값 변경: <이름> <숫자> ==");
  for (size_t i = 0; i < tuningParamCount; i++) {
    cmdReply("%-11s %.3f~%.3f%s  %s", tuningParams[i].name,
             tuningParams[i].minValue, tuningParams[i].maxValue,
             tuningParams[i].unit, tuningParams[i].note);
  }
  cmdReply("== 지팡이 전용 ==");
  cmdReply("test <0-3>  진동/부저 5초 테스트 (0=중지)");
  cmdReply("imu / imusave / imureset / imualign <0~359.9>");
  cmdReply("imuwalk     걸어서 방향 정렬 (= 버튼 2초, imuwalk 0 = 중지)");
}

void runTuningCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  // 다른 노드의 응답이 되돌아온 경우는 명령으로 해석하지 않는다.
  if (line.startsWith("[CMD]") || line.startsWith(">")) return;

  String name = line;
  String value = "";
  int space = line.indexOf(' ');
  if (space > 0) {
    name = line.substring(0, space);
    value = line.substring(space + 1);
    value.trim();
  }
  name.toLowerCase();

  bool hasValue = value.length() > 0;
  float number = hasValue ? value.toFloat() : 0.0f;

  if (name == "help") { reportTuningHelp(); return; }
  if (name == "get") { reportAllTuningValues(); return; }
  if (name == "save") { saveTuningToFlash(); return; }
  if (name == "load") { loadTuningFromFlash(); reportAllTuningValues(); return; }
  if (name == "factory") { restoreTuningDefaults(); return; }

  if (name == "reset") {
    gpsFilter.initialized = false;
    gpsFilter.consecutiveOutliers = 0;
    gpsFilter.hasRelocationCandidate = false;
    cmdReply("GPS 필터 기준점 초기화");
    return;
  }

  if (runDeviceCommand(name, hasValue, number)) return;

  int index = findTuningParam(name);
  if (index < 0) {
    cmdReply("알 수 없는 명령: %s (help 입력)", name.c_str());
    return;
  }

  if (!hasValue) {
    reportTuningValue((size_t)index);
    return;
  }

  writeTuningValue((size_t)index, number);
  reportTuningValue((size_t)index);
}

void handleUdpCommands() {
  if (!cmdUdpStarted) {
    if (WiFi.status() != WL_CONNECTED) return;
    if (cmdUdp.begin(CMD_UDP_PORT)) {
      cmdUdpStarted = true;
      Serial.printf("[CMD] UDP 명령 대기 포트=%u\n", (unsigned)CMD_UDP_PORT);
    }
    return;
  }

  for (int size = cmdUdp.parsePacket(); size > 0; size = cmdUdp.parsePacket()) {
    char buffer[160];
    int length = cmdUdp.read(buffer, sizeof(buffer) - 1);
    if (length < 0) length = 0;
    buffer[length] = '\0';

    cmdReplyIp = cmdUdp.remoteIP();
    cmdReplyPort = cmdUdp.remotePort();

    // 한 패킷에 여러 줄이 들어올 수 있으므로 줄 단위로 처리한다.
    char *context = NULL;
    for (char *token = strtok_r(buffer, "\r\n", &context);
         token != NULL;
         token = strtok_r(NULL, "\r\n", &context)) {
      char *cmd = token;
      // "@노드 <명령>" 형식이면 태그가 이 노드와 일치할 때만 처리한다.
      // (형제 STA끼리 직접 못 보내 명령을 브로드캐스트로 받기 때문.)
      if (cmd[0] == '@') {
        char *space = strchr(cmd, ' ');
        if (space == NULL) continue;
        *space = '\0';
        bool mine = (strcmp(cmd + 1, NODE_CMD_TAG) == 0);
        cmd = space + 1;
        if (!mine) continue;
      }
      runTuningCommand(String(cmd));
    }
  }
}

void readSerialCommandLines() {
  static char serialBuffer[140];
  static size_t serialLength = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      serialBuffer[serialLength] = '\0';
      if (serialLength > 0) {
        cmdReplyPort = 0;  // USB로 들어온 명령의 응답은 시리얼로만 보낸다.
        runTuningCommand(String(serialBuffer));
      }
      serialLength = 0;
      continue;
    }

    if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = c;
    } else {
      serialLength = 0;  // 너무 긴 입력은 버린다.
    }
  }
}

void handleCaneSerialCommands() {
  readSerialCommandLines();

  if (manualOutputTestUntilMs > 0 &&
      (int32_t)(millis() - manualOutputTestUntilMs) >= 0) {
    manualOutputTestUntilMs = 0;
    manualOutputTestRisk = RISK_SAFE;
    applyRisk(RISK_SAFE);
    Serial.println("[ACT TEST] complete -> SAFE");
  }
}

void setup() {
  forceOutputsOff();
  Serial.begin(115200);
  delay(1000);

  captureBootDiagnostics();
  Serial.printf(
    "[BOOT] count=%lu reset=%s\n",
    (unsigned long)caneBootCount,
    resetReasonName(caneResetReason)
  );

  captureTuningDefaults();
  loadTuningFromFlash();

  Serial.println("=== V2X Cane Status + Risk Alert ===");
  Serial.println("[ACT] output = vibration motor + beep buzzer only, no DFPlayer");
  Serial.printf("[CONFIG] fallback lat=%.6f lng=%.6f\n", (float)CANE_FIXED_LAT, (float)CANE_FIXED_LNG);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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
  handleCaneSerialCommands();
  handleUdpCommands();
  handleCaneButton();

  // 현재 위험 단계에 맞춰 진동과 부저 패턴을 계속 갱신.
  updateActuators();

  readGps();
  // 8/18 패치: 런타임 5 Hz 재설정은 10.7 s 동기 정지(차량과 동일 루틴). 부팅 설정만 둔다.
  // recoverGps5HzIfNeeded();
  readImu();

  uint32_t now = millis();

  // 차량 AP와 연결이 끊기면 5초마다 재접속을 시도한다.
  // (ESP-NOW 통신은 STA 연결 여부와 무관하게 계속 동작한다.)
  if (WiFi.status() != WL_CONNECTED &&
      now - lastWifiReconnectMs >= 5000UL) {
    lastWifiReconnectMs = now;
    Serial.println("[WIFI] disconnected -> reconnecting");
    WiFi.disconnect();
    WiFi.begin(V2X_WIFI_SSID, V2X_WIFI_PASSWORD);
  }

if (now - lastSendMs >= SEND_INTERVAL_MS) {
  lastSendMs = now;

  digitalWrite(LED_PIN, HIGH);
  sendCaneStatus();
  digitalWrite(LED_PIN, LOW);
}

// 지팡이 UDP 로그를 1초마다 전송
if (now - lastUdpTelemetryMs >= cfgTelemetryIntervalMs) {
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
    "[BT CHECK] ESP32-Cane client=%d\n",
    SerialBT.hasClient() ? 1 : 0
  );
}
#endif

  // 수신 콜백이 loop 도중 타임스탬프를 갱신하면 미리 재둔 now보다
  // 미래 값이 되어 부호 없는 뺄셈이 거대한 값으로 오버플로우하고,
  // 방금 온 패킷을 타임아웃으로 오판한다. 최신 시각으로 다시 읽고
  // 부호 있는 비교로 판정한다.
  uint32_t riskNow = millis();
  bool vehicleRiskFresh =
    vehicleRiskMs > 0 &&
    (int32_t)(riskNow - vehicleRiskMs) <=
      (int32_t)cfgDirectRiskTimeoutMs;
  bool rsuRiskFresh =
    rsuRiskMs > 0 &&
    (int32_t)(riskNow - rsuRiskMs) <=
      (int32_t)RSU_RISK_TIMEOUT_MS;
  bool directRiskFresh = vehicleRiskFresh || rsuRiskFresh;

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
