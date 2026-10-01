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
//   DF Pro RX <- GPIO27 (ESP32 TX1)
//   DWM3001CDK J10 UART TX -> GPIO32 (ESP32 RX1)
//   DWM3001CDK를 별도 USB/보조배터리로 켜면 ESP32와 GND는 반드시 공통
//   스피커 한 개: DF Pro L+/L- 또는 R+/R-, 모든 모듈 GND 공통


#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <WebServer.h>
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
#define USE_UWB 1

// GPS가 안 잡히는 실내에서도 테스트할 때 1.
// 실제 도로 주행에서는 반드시 0 권장.
#define USE_DEMO_MOVING_FALLBACK 0

// 야외 RSU/Jetson 시연에서는 위험도 판정과 하행 전송을 RSU 단독으로 맡긴다.
// 차량 상태(GPS/속도/방향) 10Hz 송신과 차량 내부 진단 계산은 유지된다.
#define VEHICLE_CALCULATES_RISK 0
// RSU가 지팡이와 차량에 같은 MSG_RISK_ALERT를 보내며,
// 차량 스피커도 RSU 판정값만 사용한다.
#define ENABLE_RSU_RISK_INPUT 1

// 1: 실내 책상 테스트 거리, 0: 실제 도로용 거리
#define USE_INDOOR_RISK_DISTANCE 0

// 블루투스 디버그 (뷰어/폰용). 시연/실전 때는 0.
#define USE_BT_DEBUG 0

// 아이패드/폰 브라우저용 웹뷰어. V2X-LOG 에 접속 후 http://192.168.4.1
// 시연 때 부하를 줄이고 싶으면 0으로 바꾼다.
#define USE_WEB_VIEWER 1
#define WEB_VIEWER_PORT 80
#define CANE_LOG_UDP_PORT 4210
#define CANE_CMD_REPLY_PORT 4301

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

#define UWB_RX 32       // ESP32 RX1 <- DWM3001CDK J10 UART TX
#define DFPLAYER_TX 27  // ESP32 TX1 -> DFPlayer RX
#define DFPLAYER_BAUD 115200
#define DFPLAYER_VOLUME 15  // 0~30

// DFPlayer Pro는 TX(GPIO27)만 사용하고 같은 UART1의 RX(GPIO32)로
// DWM3001CDK CLI ranging 출력을 받는다. 두 장치 모두 115200bps.
#define UWB_BAUD 115200
#define UWB_FRESH_TIMEOUT_MS 750UL
#define UWB_CAL_REQUIRED_SAMPLES 100U
#define UWB_CAL_TIMEOUT_MS 30000UL
#define UWB_MIN_DISTANCE_M 0.05f
#define UWB_MAX_DISTANCE_M 100.0f

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
// 최종 진단 로그는 10Hz. IMU 자체 처리는 loop에서 가능한 한 빠르게 수행한다.
#define UDP_TELEMETRY_INTERVAL_MS 100UL
#define CANE_TIMEOUT_MS 2000UL
#define RSU_RISK_TIMEOUT_MS 3000UL
#define SENSOR_LOG_INTERVAL_MS 1000UL
#define GPS_FIX_MAX_AGE_MS 3000UL

// ESP-NOW RSSI 평활 계수. 작을수록 순간 흔들림을 더 강하게 줄인다.
#define CANE_RSSI_FILTER_ALPHA 0.20f

// ===== 2026-08-05 RSSI 실측 보정 =====
// 1/3/5/8/10m LOS 중앙값(차량측): -51.0/-65.2/-63.5/-72.7/-76.7dBm.
// 3m와 5m가 역전되므로 RSSI를 정확한 거리계로 쓰지 않고,
// 구간 + 히스테리시스 + 유지시간으로만 사용한다.
#define RSSI_FRESH_TIMEOUT_MS 1000UL
#define RSSI_CLOSER_CONFIRM_MS 800UL
#define RSSI_FARTHER_CONFIRM_MS 2500UL
#define RSSI_VERY_CLOSE_ENTER_DBM (-58.0f)
#define RSSI_VERY_CLOSE_EXIT_DBM  (-62.0f)
#define RSSI_CLOSE_ENTER_DBM      (-69.0f)
#define RSSI_CLOSE_EXIT_DBM       (-73.0f)
#define RSSI_APPROACH_ENTER_DBM   (-75.0f)
#define RSSI_APPROACH_EXIT_DBM    (-79.0f)

// RSSI 연속 거리 추정은 진단용으로만 남긴다.
// CPA 상대벡터의 방향과 크기는 영점보정한 GPS에서 모두 가져온다.
// RSSI로 GPS 벡터 크기를 강제하면 평행 통과 때 횡방향 이격거리까지
// 줄어들어 충돌 경로로 오인할 수 있다.
// 실측 회귀: RSSI = -51.7 - 10 * 2.42 * log10(distance).
#define RSSI_AT_1M_DBM (-51.7f)
#define RSSI_PATH_LOSS_EXPONENT 2.42f
#define RSSI_DISTANCE_MIN_M 0.8f
#define RSSI_DISTANCE_MAX_M 12.0f

// 두 GPS 안테나를 나란히 놓고 BOOT 버튼을 누르면 상대좌표 영점을 잡는다.
// 보정 전/보정 중에는 GPS 단독 위험판정을 금지한다.
#define REL_CAL_BUTTON_PIN 0
#define REL_CAL_STILL_TIME_MS 8000UL
#define REL_CAL_TIMEOUT_MS 30000UL
#define REL_CAL_MIN_SAMPLES 40
#define REL_CAL_MAX_OFFSET_SPREAD_M 5.0f

// GPS 품질/이상치 필터 설정: RC카 기준.
#define GPS_MIN_SATELLITES 4
#define GPS_MAX_HDOP 3.5f
#define GPS_NODE_MAX_SPEED_MPS 6.0f
#define GPS_OUTLIER_BASE_M 10.0f   // 8/18 패치: 4→10 (거부=좌표 동결=젯슨이 이동 못 봄; 젯슨에 자체 게이트 있음)
#define GPS_FILTER_ALPHA 0.50f
#define GPS_FILTER_BETA 0.08f
// GPS가 실제 5Hz면 15회, 1Hz로 떨어지면 3회를 사용해 어느 경우든
// 약 3초 동안 같은 새 위치가 반복될 때 필터 기준 위치를 재설정한다.
#define GPS_RELOCALIZE_AFTER_REJECTS_5HZ 5   // 8/18 패치: 15(3 s)→5(1 s) 최대 동결 1 s
#define GPS_RELOCALIZE_AFTER_REJECTS_1HZ 3
#define GPS_FILTER_RESET_GAP_MS 5000UL
#define GPS_PREDICTION_MAX_MS 700UL
#define GPS_5HZ_RECOVERY_COOLDOWN_MS 60000UL
#define GPS_5HZ_RECOVERY_MIN_UPTIME_MS 15000UL

// 정지 상태용 GPS 필터.
#define GPS_STATIONARY_SPEED_MPS 0.25f   // 8/18 패치: 0.45→0.25 (젯슨 ZUPT 문턱과 동일)
#define GPS_STATIONARY_ALPHA 0.30f   // 8/18 패치: 0.08→0.30 (정지 모드 좌표 지연 2.4 s→0.6 s)
#define GPS_STATIONARY_BETA 0.01f

// GPS 진행 방향을 신뢰하기 위한 최소 차량 속도.
#define MIN_VALID_HEADING_SPEED_MPS 0.25f   // 8/18 패치: 0.40→0.25 (RC 속도대·젯슨 도플러 융합 문턱과 동일)

// ===== 2026-08-03~05 차량 실측 방향 보정 =====
// GPS course는 실제 이동 경로(전진/후진 포함), IMU는 차체 방향과
// GPS가 잠깐 끊긴 동안의 회전을 담당한다.
#define GPS_MOTION_HEADING_TIMEOUT_MS 2000UL
#define VEHICLE_BODY_HEADING_TIMEOUT_MS 5000UL
#define VEHICLE_IMU_FRESH_TIMEOUT_MS 500UL
#define VEHICLE_BODY_BOOTSTRAP_SPEED_MPS 0.70f
#define VEHICLE_BODY_GPS_CORRECTION_ALPHA 0.12f
#define VEHICLE_DRIVE_MODE_CONFIRM_COUNT 3

#define CAR_MAG_CENTER_X_UT   9.1125f
#define CAR_MAG_CENTER_Y_UT (-27.4875f)
#define CAR_MAG_CENTER_Z_UT  (-2.41875f)

#define CAR_MAG_AXIS_A_X    (-3.225f)
#define CAR_MAG_AXIS_A_Y     30.975f
#define CAR_MAG_AXIS_A_Z    (-3.565f)

#define CAR_MAG_AXIS_90_X    30.075f
#define CAR_MAG_AXIS_90_Y     5.325f
#define CAR_MAG_AXIS_90_Z    (-0.825f)

// car_16 전진, car_17 후진, car_18 우회전으로 맞춘 진북 기준 오프셋.
// 실측상 우회전 때 모델 상대각이 감소하므로 offset-relative를 사용한다.
#define CAR_MAG_HEADING_OFFSET_DEG 116.8f

#define CAR_NEUTRAL_AX (-0.44f)
#define CAR_NEUTRAL_AY   0.48f
#define CAR_NEUTRAL_AZ   9.68f

#define CAR_GYRO_BIAS_X_DPS   0.8f
#define CAR_GYRO_BIAS_Y_DPS (-0.1f)
#define CAR_GYRO_BIAS_Z_DPS (-0.5f)
#define CAR_GYRO_YAW_DEADBAND_DPS 3.0f

#define CAR_HEADING_ACCEL_MIN_MPS2 7.0f
#define CAR_HEADING_ACCEL_MAX_MPS2 13.0f
#define CAR_HEADING_GYRO_MAX_DPS 45.0f
#define CAR_HEADING_POSE_COS_MIN 0.96592583f  // cos(15도)
#define CAR_MAG_MODEL_RESIDUAL_MAX_UT 12.0f
#define CAR_MAG_MODEL_RADIUS_MIN 0.55f
#define CAR_MAG_MODEL_RADIUS_MAX 1.50f
#define CAR_MAG_CORRECTION_TAU_S 0.80f

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
// 일반 노면 실측 중 순간 최대 73.56m/s²까지 관측되어 85로 올린다.
// 2026-08-08 울퉁불퉁한 노면/배치 중 비충돌 트리거 97.08m/s² 확인.
// 실제 접근 주행 최대 피크는 42.95m/s²이었으며,
// 비충돌 트리거에 약 13% 여유를 둔다.
#define IMPACT_THRESHOLD_MPS2 110.0f
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

// ===== 경로교차(CPA) 위험판정 =====
#define MIN_PREDICT_VEHICLE_SPEED_MPS 0.40f
#define MIN_PREDICT_CANE_SPEED_MPS 0.50f
#define MIN_RADIAL_CLOSING_SPEED_MPS 0.15f
// 사용자가 안전 평행 간격으로 정한 1.5m에서는 울리지 않도록
// CPA 횡방향 충돌반경을 1.2m 안쪽으로 제한한다.
#define CPA_DANGER_DISTANCE_M 0.5f
#define CPA_WARNING_DISTANCE_M 0.8f
#define CPA_CAUTION_DISTANCE_M 1.2f
#define CPA_PREDICTION_HORIZON_S 5.0f
// RSSI/UWB 결합 전에는 GPS만으로 "방향 무관 즉시 경고"하지 않는다.
#define EMERGENCY_DISTANCE_M 0.2f

// 2026-08-12 재시험에서 네 평행 로그는 -72dBm 미만이거나 후진이었고,
// 정면 접근은 전진 중 -72dBm 이상 + GPS 방위오차 70도 안으로 분리됐다.
// 독립 GPS 상대좌표가 20m 이상 틀어진 경우에도 주의 경보를 살리는
// 근거리 정면접근 보조 조건이다. RSSI 특성상 이 경로는 CAUTION만 낸다.
#define RISK_RSSI_GATE_DBM (-72.0f)
#define DIRECT_APPROACH_CONE_HALF_ANGLE_DEG 70.0f

// 지팡이 노드의 GPS fallback 좌표와 같아야 함.
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
float cfgGpsMaxSpeed = GPS_NODE_MAX_SPEED_MPS;
float cfgGpsStationarySpeed = GPS_STATIONARY_SPEED_MPS;
float cfgGpsStationaryAlpha = GPS_STATIONARY_ALPHA;
float cfgGpsStationaryBeta = GPS_STATIONARY_BETA;
float cfgMinHeadingSpeed = MIN_VALID_HEADING_SPEED_MPS;
float cfgCpaDangerM = CPA_DANGER_DISTANCE_M;
float cfgCpaWarningM = CPA_WARNING_DISTANCE_M;
float cfgCpaCautionM = CPA_CAUTION_DISTANCE_M;
float cfgCpaHorizonS = CPA_PREDICTION_HORIZON_S;
float cfgEmergencyM = EMERGENCY_DISTANCE_M;
float cfgMinPredictVehicleSpeed = MIN_PREDICT_VEHICLE_SPEED_MPS;
float cfgMinPredictCaneSpeed = MIN_PREDICT_CANE_SPEED_MPS;
float cfgMinClosingSpeed = MIN_RADIAL_CLOSING_SPEED_MPS;
float cfgApproachConeDeg = DIRECT_APPROACH_CONE_HALF_ANGLE_DEG;
float cfgForwardConeDeg = FORWARD_CONE_HALF_ANGLE_DEG;
uint32_t cfgRiskEscalateCount = RISK_ESCALATE_CONFIRM_COUNT;
uint32_t cfgRiskClearCount = RISK_CLEAR_CONFIRM_COUNT;
float cfgRssiGateDbm = RISK_RSSI_GATE_DBM;
float cfgRssiAlpha = CANE_RSSI_FILTER_ALPHA;
float cfgImpactThreshold = IMPACT_THRESHOLD_MPS2;
uint32_t cfgDfplayerVolume = DFPLAYER_VOLUME;
uint32_t cfgTelemetryIntervalMs = UDP_TELEMETRY_INTERVAL_MS;
float cfgUwbOffsetM = 0.0f;
float cfgUwbFilterAlpha = 0.35f;
uint32_t cfgUwbTimeoutMs = UWB_FRESH_TIMEOUT_MS;
// 실물 검증 전에 기존 경고를 망치지 않도록 기본 0.
uint32_t cfgUwbRiskEnabled = 0;

#if USE_WEB_VIEWER
// =====================
// 아이패드/폰 웹뷰어
// 차량이 자기 로그와 지팡이 브로드캐스트 로그를 모아 브라우저에 보여주고,
// 브라우저에서 보낸 명령을 자기 자신 또는 지팡이에게 전달한다.
// =====================
WebServer webServer(WEB_VIEWER_PORT);
WiFiUDP caneLogUdp;   // 지팡이 텔레메트리(4210) 수신
WiFiUDP caneCmdUdp;   // 지팡이로 명령 전달 및 응답 수신

char webCarTelemetry[2300] = "";
char webCaneTelemetry[1900] = "";
char webReplyBuffer[700] = "";
uint32_t webCarUpdatedMs = 0;
uint32_t webCaneUpdatedMs = 0;
IPAddress caneNodeIp;
bool caneNodeIpKnown = false;

// ===WEB_PAGE_BEGIN=== (build_web_viewer.py 가 생성. 직접 고치지 말 것)
static const char WEB_PAGE[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>V2X 듀얼 뷰어</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--line:#d8dbe0;--ink:#1a1c1f;--dim:#6b7280;
      --ok:#1a7f37;--warn:#b8860b;--bad:#c62828;--accent:#1a56db}
*{box-sizing:border-box;-webkit-text-size-adjust:100%}
body{margin:0;padding:10px;background:var(--bg);color:var(--ink);
     font:15px/1.45 -apple-system,"Apple SD Gothic Neo",sans-serif}
h1{font-size:17px;margin:0 0 8px}
h2{font-size:15px;margin:0 0 6px;display:flex;align-items:center;gap:8px}
.bar{background:var(--card);border:1px solid var(--line);border-radius:10px;
     padding:8px 10px;margin-bottom:10px;display:flex;flex-wrap:wrap;
     gap:8px;align-items:center}
.grid{display:grid;grid-template-columns:1fr;gap:10px}
@media(min-width:820px){.grid{grid-template-columns:1fr 1fr}}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;
        padding:10px}
table{width:100%;border-collapse:collapse;font-size:14px}
td{padding:3px 4px;border-bottom:1px solid #eef0f3;
   font-variant-numeric:tabular-nums}
td.k{color:var(--dim);width:44%;word-break:keep-all}
td.v{font-family:ui-monospace,Menlo,monospace}
tr.chg td.v{background:#fff6d5}
input,select,button{font:inherit;border-radius:8px;border:1px solid var(--line);
                    padding:7px 10px;background:#fff;color:var(--ink)}
input{flex:1;min-width:80px}
button{background:var(--accent);color:#fff;border-color:transparent;
       cursor:pointer;-webkit-appearance:none}
button.sub{background:#eef1f6;color:var(--ink);border-color:var(--line)}
button:active{opacity:.7}
.row{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
.dot{width:9px;height:9px;border-radius:50%;background:#bbb;display:inline-block}
.dot.on{background:var(--ok)}.dot.old{background:var(--warn)}
.age{font-size:12px;color:var(--dim);font-weight:400}
pre{background:#0f1115;color:#e6e8eb;border-radius:8px;padding:8px;
    font:12px/1.4 ui-monospace,Menlo,monospace;height:230px;overflow:auto;
    margin:0;white-space:pre-wrap;word-break:break-all}
.rec{color:var(--bad);font-weight:600}
a.dl{display:inline-block;background:var(--ok);color:#fff;text-decoration:none;
     padding:7px 10px;border-radius:8px;margin:4px 4px 0 0;font-size:14px}
</style>
</head>
<body>

<h1>V2X 듀얼 뷰어 <span class="age" id="conn"></span></h1>

<div class="bar">
  <input id="recName" placeholder="기록 이름 (예: 직선접근1)">
  <button id="recBtn" onclick="toggleRecord()">기록 시작</button>
  <span id="recInfo" class="age">기록 안 함</span>
  <span style="flex:1"></span>
  <select id="interval" onchange="restartTimer()">
    <option value="200">0.2초</option>
    <option value="500" selected>0.5초</option>
    <option value="1000">1초</option>
  </select>
</div>
<div id="dlBox"></div>

<div class="grid">
  <section>
    <h2><span class="dot" id="dotCar"></span>차량<span class="age" id="ageCar"></span></h2>
    <table id="tblCar"></table>
    <div class="row">
      <input id="cmdCar" placeholder="명령 (예: alpha 0.25)"
             autocapitalize="off" autocorrect="off" spellcheck="false">
      <button onclick="send('car')">전송</button>
    </div>
    <div class="row">
      <button class="sub" onclick="quick('car','get')">get</button>
      <button class="sub" onclick="quick('car','help')">help</button>
      <button class="sub" onclick="quick('car','save')">save</button>
      <button class="sub" onclick="quick('car','reset')">reset</button>
      <button class="sub" onclick="quick('car','play 3')">음성</button>
    </div>
  </section>

  <section>
    <h2><span class="dot" id="dotCane"></span>지팡이<span class="age" id="ageCane"></span></h2>
    <table id="tblCane"></table>
    <div class="row">
      <input id="cmdCane" placeholder="명령 (예: alpha 0.25)"
             autocapitalize="off" autocorrect="off" spellcheck="false">
      <button onclick="send('cane')">전송</button>
    </div>
    <div class="row">
      <button class="sub" onclick="quick('cane','get')">get</button>
      <button class="sub" onclick="quick('cane','help')">help</button>
      <button class="sub" onclick="quick('cane','save')">save</button>
      <button class="sub" onclick="quick('cane','reset')">reset</button>
      <button class="sub" onclick="quick('cane','test 3')">진동</button>
    </div>
  </section>
</div>

<section style="margin-top:10px">
  <h2>수신 로그
    <button class="sub" onclick="paused=!paused" id="pauseBtn"
            style="margin-left:auto;padding:4px 9px">일시정지</button>
    <button class="sub" onclick="logLines=[];draw()"
            style="padding:4px 9px">지우기</button>
  </h2>
  <pre id="log"></pre>
</section>

<script>
var logLines = [], paused = false, timer = null;
var prev = {car:{}, cane:{}};
var recording = false, recStart = 0, recName = "";
var recRows = {car:[], cane:[]}, recKeys = {car:[], cane:[]}, recLog = [];

function esc(s){return String(s).replace(/[&<>]/g,function(c){
  return {'&':'&amp;','<':'&lt;','>':'&gt;'}[c];});}

function stamp(){
  var d = new Date();
  return ("0"+d.getHours()).slice(-2)+":"+("0"+d.getMinutes()).slice(-2)+
         ":"+("0"+d.getSeconds()).slice(-2);
}

function pushLog(line){
  logLines.push(stamp()+"  "+line);
  if (logLines.length > 600) logLines.splice(0, 200);
  if (recording) recLog.push(stamp()+"  "+line);
}

function parseSections(text){
  var out = {}, cur = null, lines = text.split("\n");
  for (var i=0;i<lines.length;i++){
    var l = lines[i];
    if (l.indexOf("###") === 0){ cur = l.slice(3).trim(); out[cur] = []; continue; }
    if (cur) out[cur].push(l);
  }
  return out;
}

function toPairs(lines){
  var pairs = [];
  for (var i=0;i<lines.length;i++){
    var p = lines[i].indexOf(":");
    if (p > 0) pairs.push([lines[i].slice(0,p), lines[i].slice(p+1)]);
  }
  return pairs;
}

function fillTable(id, node, pairs){
  var html = "", changed = 0;
  for (var i=0;i<pairs.length;i++){
    var k = pairs[i][0], v = pairs[i][1];
    var cls = (prev[node][k] !== undefined && prev[node][k] !== v) ? " class='chg'" : "";
    if (cls) changed++;
    prev[node][k] = v;
    html += "<tr"+cls+"><td class='k'>"+esc(k)+"</td><td class='v'>"+esc(v)+"</td></tr>";
  }
  document.getElementById(id).innerHTML = html;
  return changed;
}

function markAge(dotId, ageId, ms){
  var dot = document.getElementById(dotId), age = document.getElementById(ageId);
  if (ms < 0 || ms > 60000){ dot.className = "dot"; age.textContent = "수신 없음"; return false; }
  dot.className = ms < 3000 ? "dot on" : "dot old";
  age.textContent = (ms/1000).toFixed(1)+"초 전";
  return true;
}

function recordRow(node, pairs){
  if (!recording || !pairs.length) return;
  var row = {"시각": stamp(),
             "경과초": ((Date.now()-recStart)/1000).toFixed(3)};
  for (var i=0;i<pairs.length;i++){
    row[pairs[i][0]] = pairs[i][1];
    if (recKeys[node].indexOf(pairs[i][0]) < 0) recKeys[node].push(pairs[i][0]);
  }
  recRows[node].push(row);
}

function refresh(){
  fetch("/data", {cache:"no-store"}).then(function(r){return r.text();})
  .then(function(text){
    var s = parseSections(text);
    var meta = {};
    (s.META||[]).forEach(function(l){
      var p = l.indexOf(":"); if (p>0) meta[l.slice(0,p)] = parseInt(l.slice(p+1),10);
    });

    var carOk = markAge("dotCar","ageCar", meta.carAge===undefined?-1:meta.carAge);
    var caneOk = markAge("dotCane","ageCane", meta.caneAge===undefined?-1:meta.caneAge);
    document.getElementById("conn").textContent =
      (carOk?"차량 O":"차량 X") + " / " + (caneOk?"지팡이 O":"지팡이 X");

    var carPairs = toPairs(s.CAR||[]), canePairs = toPairs(s.CANE||[]);
    if (!paused){
      fillTable("tblCar","car", carPairs);
      fillTable("tblCane","cane", canePairs);
    }
    recordRow("car", carPairs);
    recordRow("cane", canePairs);

    if (!paused){
      if (carPairs.length) pushLog("[차량] " + carPairs.map(function(p){
        return p[0]+":"+p[1];}).join(" "));
      if (canePairs.length) pushLog("[지팡이] " + canePairs.map(function(p){
        return p[0]+":"+p[1];}).join(" "));
    }
    (s.REPLY||[]).forEach(function(l){ if (l.trim()) pushLog(l); });
    draw();
  })
  .catch(function(){
    document.getElementById("conn").textContent = "보드 연결 끊김";
  });
}

function draw(){
  var el = document.getElementById("log");
  el.textContent = logLines.slice(-300).join("\n");
  if (!paused) el.scrollTop = el.scrollHeight;
  document.getElementById("pauseBtn").textContent = paused ? "재개" : "일시정지";
  if (recording){
    var sec = Math.floor((Date.now()-recStart)/1000);
    document.getElementById("recInfo").innerHTML =
      "<span class='rec'>기록 중 " + ("0"+Math.floor(sec/60)).slice(-2) + ":" +
      ("0"+(sec%60)).slice(-2) + " (" +
      (recRows.car.length + recRows.cane.length) + "행)</span>";
  }
}

function send(target, text){
  var el = document.getElementById(target === "car" ? "cmdCar" : "cmdCane");
  if (text === undefined) text = el.value.trim();
  if (!text) return;
  pushLog("> [" + (target==="car"?"차량":"지팡이") + "] " + text);
  if (recording) recLog.push(stamp()+"  > ["+target+"] "+text);
  fetch("/cmd?target="+target+"&text="+encodeURIComponent(text))
    .then(function(){ setTimeout(refresh, 250); });
  if (el.value) el.value = "";
  draw();
}
function quick(target, text){ send(target, text); }

function toggleRecord(){
  if (!recording){
    recName = (document.getElementById("recName").value || "기록").trim();
    recording = true; recStart = Date.now();
    recRows = {car:[], cane:[]}; recKeys = {car:[], cane:[]}; recLog = [];
    document.getElementById("recBtn").textContent = "기록 중지";
    document.getElementById("dlBox").innerHTML = "";
  } else {
    recording = false;
    document.getElementById("recBtn").textContent = "기록 시작";
    document.getElementById("recInfo").textContent =
      "기록 완료 — 아래 버튼으로 저장";
    showDownloads();
  }
}

function toCsv(node){
  var keys = ["시각","경과초"].concat(recKeys[node]);
  var out = "﻿" + keys.join(",") + "\n";
  for (var i=0;i<recRows[node].length;i++){
    var r = recRows[node][i], line = [];
    for (var j=0;j<keys.length;j++){
      var v = r[keys[j]] === undefined ? "" : String(r[keys[j]]);
      line.push(v.indexOf(",") >= 0 ? '"'+v+'"' : v);
    }
    out += line.join(",") + "\n";
  }
  return out;
}

function fileName(suffix){
  var d = new Date(recStart), p = function(n){return ("0"+n).slice(-2);};
  return recName + "_" + d.getFullYear() + "-" + p(d.getMonth()+1) + "-" +
         p(d.getDate()) + "_" + p(d.getHours()) + "-" + p(d.getMinutes()) +
         "-" + p(d.getSeconds()) + "_" + suffix;
}

function makeLink(text, content, name, type){
  var blob = new Blob([content], {type:type||"text/csv;charset=utf-8"});
  var a = document.createElement("a");
  a.className = "dl"; a.textContent = text;
  a.href = URL.createObjectURL(blob); a.download = name;
  return a;
}

function showDownloads(){
  var box = document.getElementById("dlBox");
  box.innerHTML = "";
  if (recRows.car.length)
    box.appendChild(makeLink("차량 CSV 저장", toCsv("car"), fileName("차량.csv")));
  if (recRows.cane.length)
    box.appendChild(makeLink("지팡이 CSV 저장", toCsv("cane"), fileName("지팡이.csv")));
  if (recLog.length)
    box.appendChild(makeLink("로그 저장", recLog.join("\n"),
                             fileName("로그.txt"), "text/plain;charset=utf-8"));
}

function restartTimer(){
  if (timer) clearInterval(timer);
  timer = setInterval(refresh, parseInt(document.getElementById("interval").value,10));
}

document.getElementById("cmdCar").addEventListener("keydown", function(e){
  if (e.key === "Enter") send("car"); });
document.getElementById("cmdCane").addEventListener("keydown", function(e){
  if (e.key === "Enter") send("cane"); });

refresh();
restartTimer();
</script>
</body>
</html>
)HTMLPAGE";
// ===WEB_PAGE_END===
#endif

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
  // v4 근접거리 채널: 보정된 최신 UWB가 있으면 UWB, 아니면 RSSI 추정거리.
  // RSU의 기존 rssi_dist 입력을 깨지 않고 UWB 장착 직후 바로 활용한다.
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

  float distance_m;
  float closing_speed_mps;
  float ttc_s;

  uint32_t timestamp_ms;
  uint16_t seq_num;
} v2x_risk_message_t;

static_assert(sizeof(v2x_risk_message_t) == 35,
              "risk alert packet must be 35 bytes");

// UWB 상세 진단은 기존 40/35바이트 패킷을 변경하지 않고 별도로 보낸다.
// 기존 RSU가 이 패킷을 모르더라도 상태/위험 패킷은 그대로 호환된다.
typedef struct __attribute__((packed)) v2x_uwb_message {
  uint32_t magic;
  uint8_t version;
  uint8_t msg_type;
  uint8_t node_type;
  uint8_t flags;       // bit0=유효, bit1=오프셋 보정 완료
  uint32_t target_id;  // 0은 방송/대상 미지정
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
v2x_uwb_message_t txUwbRange;

uint8_t latestCaneMAC[6] = {0};
bool hasCaneMAC = false;
volatile bool hasLatestCane = false;
volatile bool newCanePacket = false;

uint32_t vehicleId = 0;
uint16_t vehicleSeq = 0;
uint16_t riskSeq = 0;
uint16_t uwbSeq = 0;
uint32_t sendCount = 0;
uint32_t caneRxCount = 0;
uint32_t riskSendCount = 0;
uint32_t lastSendMs = 0;

uint32_t lastBtTelemetryMs = 0;
uint32_t lastBtCheckMs = 0;

uint32_t lastCaneRxMs = 0;
volatile uint32_t lastRsuRiskRxMs = 0;
volatile uint32_t rsuRiskRxCount = 0;
uint32_t lastSensorLogMs = 0;

typedef struct {
  bool valid;
  bool calibrated;
  bool calibrating;
  float rawDistanceM;
  float filteredDistanceM;
  float closingSpeedMps;
  float lastRssiDbm;
  uint32_t lastSampleMs;
  uint32_t sampleCount;
  uint32_t failureCount;
  float medianWindow[5];
  uint8_t medianCount;
  uint8_t medianIndex;
  float previousFilteredM;
  uint32_t previousFilteredMs;
  float calibrationKnownM;
  double calibrationRawSumM;
  uint16_t calibrationSamples;
  uint32_t calibrationStartedMs;
} UwbRangeState;

UwbRangeState uwbRange = {};
uint32_t uwbSendCount = 0;
uint32_t uwbParseOverflowCount = 0;
bool uwbUartReady = false;

void sendUwbRangePacket();

// 지팡이 -> 차량 ESP-NOW 수신 신호 세기 진단값.
// 실제 거리 보정 전에는 거리값으로 단정하지 않고 dBm 원시/평활값을 모두 기록한다.
int8_t latestCaneRssiDbm = -127;
float filteredCaneRssiDbm = -127.0f;
bool hasCaneRssi = false;
uint32_t caneRssiSampleCount = 0;
uint32_t lastCaneRssiMs = 0;

enum RssiProximityZone : uint8_t {
  RSSI_ZONE_UNKNOWN = 0,
  RSSI_ZONE_FAR = 1,
  RSSI_ZONE_APPROACH = 2,
  RSSI_ZONE_CLOSE = 3,
  RSSI_ZONE_VERY_CLOSE = 4
};

uint8_t rssiProximityZone = RSSI_ZONE_UNKNOWN;
uint8_t rssiCandidateZone = RSSI_ZONE_UNKNOWN;
uint32_t rssiCandidateSinceMs = 0;
float rssiEstimatedDistanceM = -1.0f;

typedef struct {
  bool active;
  bool valid;
  double sumEastM;
  double sumNorthM;
  float biasEastM;
  float biasNorthM;
  uint32_t requestedMs;
  uint32_t firstSampleMs;
  uint32_t lastProgressLogMs;
  uint16_t sampleCount;
} RelativeGpsCalibration;

RelativeGpsCalibration relativeGpsCalibration = {};

double vehicleLat = 0.0;
double vehicleLng = 0.0;
float vehicleSpeed = 0.0f;
float vehicleHeading = 0.0f;
bool vehicleHeadingValid = false;
uint8_t vehicleGpsValid = 0;
bool usingDemoGps = false;

// 실제 이동 방향. GPS course가 1차 자료이고 IMU는 짧은 공백을 메운다.
float gpsMotionHeadingDeg = 0.0f;
bool gpsMotionHeadingHasFix = false;
uint32_t lastGpsMotionHeadingMs = 0;
bool vehicleDriveModeKnown = false;
bool vehicleIsReversing = false;
bool candidateReversing = false;
uint8_t driveModeCandidateCount = 0;

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

  bool likelyStationary =
    velocityValid && speedMps < cfgGpsStationarySpeed;

  float positionAlpha =
    likelyStationary ? cfgGpsStationaryAlpha : cfgGpsFilterAlpha;

  float velocityBeta =
    likelyStationary ? cfgGpsStationaryBeta : cfgGpsFilterBeta;

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
  if (filteredSpeed > cfgGpsMaxSpeed) {
    float scale = cfgGpsMaxSpeed / filteredSpeed;
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

// 차량 차체 방향과 자력계 모델 진단값.
float vehicleBodyHeadingDeg = 0.0f;
bool vehicleBodyHeadingHasFix = false;
uint32_t lastVehicleMagAcceptedMs = 0;
uint32_t vehicleHeadingAcceptedCount = 0;
uint32_t vehicleHeadingRejectedCount = 0;
float vehicleHeadingAccelNorm = 0.0f;
float vehicleHeadingGyroNorm = 0.0f;
float vehicleHeadingPoseCos = 0.0f;
float vehicleMagModelResidual = 999.0f;
float vehicleMagModelRadius = 0.0f;
float vehicleYawRateDps = 0.0f;

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

const char *rssiZoneName(uint8_t zone) {
  switch (zone) {
    case RSSI_ZONE_FAR: return "FAR";
    case RSSI_ZONE_APPROACH: return "APPROACH";
    case RSSI_ZONE_CLOSE: return "CLOSE";
    case RSSI_ZONE_VERY_CLOSE: return "VERY_CLOSE";
    default: return "UNKNOWN";
  }
}

uint8_t baseRssiZone(float rssiDbm) {
  if (rssiDbm >= RSSI_VERY_CLOSE_ENTER_DBM) {
    return RSSI_ZONE_VERY_CLOSE;
  }
  if (rssiDbm >= RSSI_CLOSE_ENTER_DBM) {
    return RSSI_ZONE_CLOSE;
  }
  if (rssiDbm >= RSSI_APPROACH_ENTER_DBM) {
    return RSSI_ZONE_APPROACH;
  }
  return RSSI_ZONE_FAR;
}

float estimateDistanceFromRssi(float rssiDbm) {
  float exponent =
    (RSSI_AT_1M_DBM - rssiDbm) /
    (10.0f * RSSI_PATH_LOSS_EXPONENT);
  float distanceM = powf(10.0f, exponent);
  return constrain(
    distanceM,
    RSSI_DISTANCE_MIN_M,
    RSSI_DISTANCE_MAX_M
  );
}

bool caneRssiIsFresh() {
  return hasCaneRssi &&
         lastCaneRssiMs > 0 &&
         millis() - lastCaneRssiMs <= RSSI_FRESH_TIMEOUT_MS;
}

void updateRssiProximity() {
  float rssiDbm;
  bool valid;
  uint32_t lastMs;

  portENTER_CRITICAL(&caneMux);
  rssiDbm = filteredCaneRssiDbm;
  valid = hasCaneRssi;
  lastMs = lastCaneRssiMs;
  portEXIT_CRITICAL(&caneMux);

  uint32_t now = millis();
  if (!valid || lastMs == 0 ||
      now - lastMs > RSSI_FRESH_TIMEOUT_MS) {
    rssiEstimatedDistanceM = -1.0f;
    rssiProximityZone = RSSI_ZONE_UNKNOWN;
    rssiCandidateZone = RSSI_ZONE_UNKNOWN;
    rssiCandidateSinceMs = 0;
    return;
  }

  rssiEstimatedDistanceM = estimateDistanceFromRssi(rssiDbm);

  uint8_t desiredZone = rssiProximityZone;
  if (rssiProximityZone == RSSI_ZONE_UNKNOWN) {
    desiredZone = baseRssiZone(rssiDbm);
  }
  else if (rssiProximityZone == RSSI_ZONE_FAR) {
    if (rssiDbm >= RSSI_APPROACH_ENTER_DBM) {
      desiredZone = baseRssiZone(rssiDbm);
    }
  }
  else if (rssiProximityZone == RSSI_ZONE_APPROACH) {
    if (rssiDbm >= RSSI_CLOSE_ENTER_DBM) {
      desiredZone = baseRssiZone(rssiDbm);
    } else if (rssiDbm <= RSSI_APPROACH_EXIT_DBM) {
      desiredZone = RSSI_ZONE_FAR;
    }
  }
  else if (rssiProximityZone == RSSI_ZONE_CLOSE) {
    if (rssiDbm >= RSSI_VERY_CLOSE_ENTER_DBM) {
      desiredZone = RSSI_ZONE_VERY_CLOSE;
    } else if (rssiDbm <= RSSI_CLOSE_EXIT_DBM) {
      desiredZone = baseRssiZone(rssiDbm);
    }
  }
  else if (rssiProximityZone == RSSI_ZONE_VERY_CLOSE &&
           rssiDbm <= RSSI_VERY_CLOSE_EXIT_DBM) {
    desiredZone = baseRssiZone(rssiDbm);
  }

  if (desiredZone == rssiProximityZone) {
    rssiCandidateZone = desiredZone;
    rssiCandidateSinceMs = 0;
    return;
  }

  if (desiredZone != rssiCandidateZone) {
    rssiCandidateZone = desiredZone;
    rssiCandidateSinceMs = now;
    return;
  }

  uint32_t requiredMs =
    desiredZone > rssiProximityZone
      ? RSSI_CLOSER_CONFIRM_MS
      : RSSI_FARTHER_CONFIRM_MS;

  if (rssiCandidateSinceMs > 0 &&
      now - rssiCandidateSinceMs >= requiredMs) {
    Serial.printf(
      "[RSSI ZONE] %s -> %s rssi=%.1f distance=%.1fm\n",
      rssiZoneName(rssiProximityZone),
      rssiZoneName(desiredZone),
      rssiDbm,
      rssiEstimatedDistanceM
    );
    rssiProximityZone = desiredZone;
    rssiCandidateSinceMs = 0;
  }
}

void calculateRawRelativeVector(
  const v2x_status_message_t &cane,
  float *outEastM,
  float *outNorthM
) {
  float meanLatRad =
    0.5f * ((float)vehicleLat + cane.latitude) * DEG_TO_RAD;

  *outEastM =
    (cane.longitude - (float)vehicleLng) *
    111320.0f * cosf(meanLatRad);
  *outNorthM =
    (cane.latitude - (float)vehicleLat) * 111132.0f;
}

void resetRelativeCalibrationSamples() {
  relativeGpsCalibration.sumEastM = 0.0;
  relativeGpsCalibration.sumNorthM = 0.0;
  relativeGpsCalibration.firstSampleMs = 0;
  relativeGpsCalibration.sampleCount = 0;
}

void startRelativeGpsCalibration() {
  relativeGpsCalibration.active = true;
  relativeGpsCalibration.valid = false;
  relativeGpsCalibration.requestedMs = millis();
  relativeGpsCalibration.lastProgressLogMs = 0;
  resetRelativeCalibrationSamples();
  resetAllCaneRiskStates();

  rawRiskLevel = RISK_SAFE;
  stableRiskLevel = RISK_SAFE;
  candidateRiskLevel = RISK_SAFE;
  candidateRiskCount = 0;
#if !ENABLE_RSU_RISK_INPUT
  lastRiskLevel = RISK_SAFE;
  announceRisk(RISK_SAFE);
#endif

  Serial.println(
    "[REL CAL] started: keep both GPS antennas together and still for 8s"
  );
}

void updateRelativeGpsCalibration(
  const v2x_status_message_t &cane
) {
  if (!relativeGpsCalibration.active) return;

  uint32_t now = millis();
  if (now - relativeGpsCalibration.requestedMs >
      REL_CAL_TIMEOUT_MS) {
    relativeGpsCalibration.active = false;
    resetRelativeCalibrationSamples();
    Serial.println("[REL CAL] timeout -> calibration invalid");
    return;
  }

  if (!vehicleGpsValid || !cane.gps_valid) {
    resetRelativeCalibrationSamples();
    return;
  }

  float rawEastM;
  float rawNorthM;
  calculateRawRelativeVector(cane, &rawEastM, &rawNorthM);
  if (!isfinite(rawEastM) || !isfinite(rawNorthM)) return;

  if (relativeGpsCalibration.sampleCount > 0) {
    float meanEastM =
      (float)(relativeGpsCalibration.sumEastM /
              relativeGpsCalibration.sampleCount);
    float meanNorthM =
      (float)(relativeGpsCalibration.sumNorthM /
              relativeGpsCalibration.sampleCount);
    float spreadM = hypotf(
      rawEastM - meanEastM,
      rawNorthM - meanNorthM
    );

    if (spreadM > REL_CAL_MAX_OFFSET_SPREAD_M) {
      Serial.printf(
        "[REL CAL] offset moved %.1fm -> sample window restarted\n",
        spreadM
      );
      resetRelativeCalibrationSamples();
      return;
    }
  }

  if (relativeGpsCalibration.firstSampleMs == 0) {
    relativeGpsCalibration.firstSampleMs = now;
  }

  relativeGpsCalibration.sumEastM += rawEastM;
  relativeGpsCalibration.sumNorthM += rawNorthM;
  relativeGpsCalibration.sampleCount++;

  if (relativeGpsCalibration.lastProgressLogMs == 0 ||
      now - relativeGpsCalibration.lastProgressLogMs >= 1000UL) {
    relativeGpsCalibration.lastProgressLogMs = now;
    Serial.printf(
      "[REL CAL] collecting %lus samples=%u offset=(%.1f,%.1f)m\n",
      (unsigned long)((now - relativeGpsCalibration.firstSampleMs) / 1000UL),
      relativeGpsCalibration.sampleCount,
      rawEastM,
      rawNorthM
    );
  }

  bool enoughTime =
    now - relativeGpsCalibration.firstSampleMs >=
    REL_CAL_STILL_TIME_MS;
  bool enoughSamples =
    relativeGpsCalibration.sampleCount >= REL_CAL_MIN_SAMPLES;
  if (!enoughTime || !enoughSamples) return;

  relativeGpsCalibration.biasEastM =
    (float)(relativeGpsCalibration.sumEastM /
            relativeGpsCalibration.sampleCount);
  relativeGpsCalibration.biasNorthM =
    (float)(relativeGpsCalibration.sumNorthM /
            relativeGpsCalibration.sampleCount);
  relativeGpsCalibration.valid = true;
  relativeGpsCalibration.active = false;

  Serial.printf(
    "[REL CAL] complete samples=%u bias=(%.2f,%.2f)m\n",
    relativeGpsCalibration.sampleCount,
    relativeGpsCalibration.biasEastM,
    relativeGpsCalibration.biasNorthM
  );
}


// 차량 전용 명령. 처리했으면 true를 돌려준다.
bool runDeviceCommand(const String &name, bool hasValue, float number) {
  // 숫자 한 글자만 입력하면 기존처럼 바로 음성 테스트.
  if (name.length() == 1 && name[0] >= '1' && name[0] <= '4') {
    playTestAudio((uint8_t)(name[0] - '0'));
    return true;
  }

  if (name == "play") {
    if (!hasValue) {
      cmdReply("사용법: play <1-4> (1주의 2경고 3위험 4충격)");
      return true;
    }
    int track = (int)number;
    if (track < 1 || track > 4) {
      cmdReply("play 범위는 1~4");
      return true;
    }
    playTestAudio((uint8_t)track);
    return true;
  }

  if (name == "cal" || name == "c") {
    startRelativeGpsCalibration();
    cmdReply("GPS 상대좌표 영점 보정 시작");
    return true;
  }

  // 음량은 값을 저장한 뒤 스피커에 즉시 반영해야 한다.
  if (name == "vol") {
    int index = findTuningParam("vol");
    if (index < 0) return false;

    if (hasValue) {
      writeTuningValue((size_t)index, number);
      char command[24];
      snprintf(command, sizeof(command), "AT+VOL=%lu",
               (unsigned long)cfgDfplayerVolume);
      sendDfPlayerAtCommand(command);
    }
    reportTuningValue((size_t)index);
    return true;
  }

  return false;
}

void playTestAudio(uint8_t track) {
  if (track == 1) {
    playAudioFile(TRACK_CAUTION_FILE);
    riskAudioPlayCount++;
    lastPlayedRiskAudio = RISK_CAUTION;
  } else if (track == 2) {
    playAudioFile(TRACK_WARNING_FILE);
    riskAudioPlayCount++;
    lastPlayedRiskAudio = RISK_WARNING;
  } else if (track == 3) {
    playAudioFile(TRACK_DANGER_FILE);
    riskAudioPlayCount++;
    lastPlayedRiskAudio = RISK_DANGER;
  } else if (track == 4) {
    playAudioFile(TRACK_IMPACT_FILE);
    impactAudioPlayCount++;
  }
  cmdReply("음성 테스트 재생: %u", track);
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
// 이 노드의 명령 태그. "@cane" 로 브로드캐스트된 지팡이 명령은 무시한다.
#define NODE_CMD_TAG "car"

WiFiUDP cmdUdp;
bool cmdUdpStarted = false;
IPAddress cmdReplyIp;
uint16_t cmdReplyPort = 0;
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
   "위치 보정 세기(클수록 원시GPS를 빨리 추종)"},
  {"beta", &cfgGpsFilterBeta, TUNING_FLOAT, 0.0f, 1.0f, "",
   "속도 보정 세기"},
  {"gate", &cfgGpsOutlierBaseM, TUNING_FLOAT, 0.5f, 50.0f, "m",
   "이보다 크게 튀면 이상치로 버림"},
  {"hdop", &cfgGpsMaxHdop, TUNING_FLOAT, 0.5f, 99.0f, "",
   "이보다 나쁜 HDOP은 품질 거부"},
  {"sats", &cfgGpsMinSatellites, TUNING_UINT32, 0.0f, 20.0f, "개",
   "최소 위성 수"},
  {"predict", &cfgGpsPredictionMaxMs, TUNING_UINT32, 0.0f, 5000.0f, "ms",
   "fix 없을 때 예측 최대시간"},
  {"fixage", &cfgGpsFixMaxAgeMs, TUNING_UINT32, 500.0f, 10000.0f, "ms",
   "이 시간 지난 fix는 무효"},
  {"maxspeed", &cfgGpsMaxSpeed, TUNING_FLOAT, 0.5f, 60.0f, "m/s",
   "차량 최대 속도 상한"},
  {"statspeed", &cfgGpsStationarySpeed, TUNING_FLOAT, 0.0f, 5.0f, "m/s",
   "이 속도 미만이면 정지로 간주"},
  {"statalpha", &cfgGpsStationaryAlpha, TUNING_FLOAT, 0.0f, 1.0f, "",
   "정지 중 위치 보정 세기"},
  {"statbeta", &cfgGpsStationaryBeta, TUNING_FLOAT, 0.0f, 1.0f, "",
   "정지 중 속도 보정 세기"},
  {"headspeed", &cfgMinHeadingSpeed, TUNING_FLOAT, 0.0f, 5.0f, "m/s",
   "GPS 방향을 믿을 최소 속도"},
  {"danger", &cfgCpaDangerM, TUNING_FLOAT, 0.1f, 20.0f, "m",
   "위험 판정 CPA 거리"},
  {"warn", &cfgCpaWarningM, TUNING_FLOAT, 0.1f, 30.0f, "m",
   "경고 판정 CPA 거리"},
  {"caution", &cfgCpaCautionM, TUNING_FLOAT, 0.1f, 40.0f, "m",
   "주의 판정 CPA 거리"},
  {"horizon", &cfgCpaHorizonS, TUNING_FLOAT, 0.5f, 30.0f, "s",
   "CPA 예측 시간창"},
  {"emergency", &cfgEmergencyM, TUNING_FLOAT, 0.0f, 10.0f, "m",
   "방향 무관 즉시 위험 거리"},
  {"vspeed", &cfgMinPredictVehicleSpeed, TUNING_FLOAT, 0.0f, 10.0f, "m/s",
   "CPA 계산 최소 차량속도"},
  {"cspeed", &cfgMinPredictCaneSpeed, TUNING_FLOAT, 0.0f, 10.0f, "m/s",
   "CPA 계산 최소 지팡이속도"},
  {"closing", &cfgMinClosingSpeed, TUNING_FLOAT, 0.0f, 10.0f, "m/s",
   "최소 접근속도"},
  {"cone", &cfgApproachConeDeg, TUNING_FLOAT, 5.0f, 180.0f, "도",
   "정면접근 인정 반각"},
  {"fwdcone", &cfgForwardConeDeg, TUNING_FLOAT, 5.0f, 180.0f, "도",
   "전방 판정 반각"},
  {"escalate", &cfgRiskEscalateCount, TUNING_UINT32, 1.0f, 50.0f, "회",
   "위험 상승 확정 횟수"},
  {"clearcnt", &cfgRiskClearCount, TUNING_UINT32, 1.0f, 100.0f, "회",
   "위험 해제 확정 횟수"},
  {"rssigate", &cfgRssiGateDbm, TUNING_FLOAT, -100.0f, -30.0f, "dBm",
   "이보다 약하면 근거리 보조판정 제외"},
  {"rssialpha", &cfgRssiAlpha, TUNING_FLOAT, 0.01f, 1.0f, "",
   "RSSI 평활 계수"},
  {"impact", &cfgImpactThreshold, TUNING_FLOAT, 5.0f, 300.0f, "m/s2",
   "충격 감지 임계값"},
  {"vol", &cfgDfplayerVolume, TUNING_UINT32, 0.0f, 30.0f, "",
   "스피커 음량 (즉시 반영)"},
  {"uwboffset", &cfgUwbOffsetM, TUNING_FLOAT, -5.0f, 5.0f, "m",
   "UWB 거리 보정 오프셋"},
  {"uwbalpha", &cfgUwbFilterAlpha, TUNING_FLOAT, 0.05f, 1.0f, "",
   "UWB 거리 평활 계수"},
  {"uwbtimeout", &cfgUwbTimeoutMs, TUNING_UINT32, 200.0f, 5000.0f, "ms",
   "UWB 최신값 유효 시간"},
  {"uwbrisk", &cfgUwbRiskEnabled, TUNING_UINT32, 0.0f, 1.0f, "",
   "실물 검증 후 UWB 직접 위험계산 사용(0/1)"},
  {"rate", &cfgTelemetryIntervalMs, TUNING_UINT32, 20.0f, 5000.0f, "ms",
   "뷰어 로그 전송 주기"},
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

#if USE_WEB_VIEWER
  webLogReply("[CMD] ", message);
#endif

  if (cmdReplyPort == 0 || !cmdUdpStarted) return;

  char line[220];
  int written = snprintf(line, sizeof(line), "[CMD] %s\n", message);
  if (written <= 0) return;

  size_t length = written < (int)sizeof(line)
                    ? (size_t)written
                    : sizeof(line) - 1;

  if (cmdUdp.beginPacket(cmdReplyIp, cmdReplyPort)) {
    cmdUdp.write((const uint8_t *)line, length);
    cmdUdp.endPacket();
  }
}

bool uwbIsFresh(uint32_t now) {
  return uwbRange.valid &&
         uwbRange.lastSampleMs > 0 &&
         now - uwbRange.lastSampleMs <= cfgUwbTimeoutMs;
}

float medianUwbWindow() {
  float values[5];
  uint8_t count = uwbRange.medianCount;
  for (uint8_t i = 0; i < count; i++) values[i] = uwbRange.medianWindow[i];

  for (uint8_t i = 1; i < count; i++) {
    float value = values[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = value;
  }
  return count > 0 ? values[count / 2] : -1.0f;
}

void persistUwbOffset() {
  if (!tuningPrefs.begin(TUNING_NVS_NAMESPACE, false)) {
    cmdReply("UWB 보정값 저장 실패");
    return;
  }
  tuningPrefs.putFloat("uwboffset", cfgUwbOffsetM);
  tuningPrefs.putBool("uwbcal", true);
  tuningPrefs.end();
}

void finishUwbCalibration() {
  if (!uwbRange.calibrating || uwbRange.calibrationSamples == 0) return;

  float meanRawM = (float)(uwbRange.calibrationRawSumM /
                           uwbRange.calibrationSamples);
  cfgUwbOffsetM = uwbRange.calibrationKnownM - meanRawM;
  uwbRange.calibrated = true;
  uwbRange.calibrating = false;
  uwbRange.medianCount = 0;
  uwbRange.medianIndex = 0;
  uwbRange.previousFilteredMs = 0;
  persistUwbOffset();
  cmdReply("UWB 보정 완료: %u회 평균=%.3fm 기준=%.3fm 오프셋=%+.3fm (자동저장)",
           (unsigned)uwbRange.calibrationSamples,
           meanRawM,
           uwbRange.calibrationKnownM,
           cfgUwbOffsetM);
}

void acceptUwbMeasurement(float rawDistanceM, float rssiDbm) {
  if (!isfinite(rawDistanceM) ||
      rawDistanceM < UWB_MIN_DISTANCE_M ||
      rawDistanceM > UWB_MAX_DISTANCE_M) {
    uwbRange.failureCount++;
    return;
  }

  uint32_t now = millis();
  uwbRange.valid = true;
  uwbRange.rawDistanceM = rawDistanceM;
  uwbRange.lastRssiDbm = rssiDbm;
  uwbRange.lastSampleMs = now;
  uwbRange.sampleCount++;

  if (uwbRange.calibrating) {
    uwbRange.calibrationRawSumM += rawDistanceM;
    uwbRange.calibrationSamples++;
    if (uwbRange.calibrationSamples >= UWB_CAL_REQUIRED_SAMPLES) {
      finishUwbCalibration();
    }
  }

  float correctedM = rawDistanceM + cfgUwbOffsetM;
  correctedM = constrain(correctedM, UWB_MIN_DISTANCE_M, UWB_MAX_DISTANCE_M);
  uwbRange.medianWindow[uwbRange.medianIndex] = correctedM;
  uwbRange.medianIndex = (uwbRange.medianIndex + 1) % 5;
  if (uwbRange.medianCount < 5) uwbRange.medianCount++;

  float medianM = medianUwbWindow();
  float previousM = uwbRange.filteredDistanceM;
  uint32_t previousMs = uwbRange.previousFilteredMs;
  if (previousMs == 0) {
    uwbRange.filteredDistanceM = medianM;
  } else {
    uwbRange.filteredDistanceM =
      cfgUwbFilterAlpha * medianM +
      (1.0f - cfgUwbFilterAlpha) * uwbRange.filteredDistanceM;
  }

  if (previousMs > 0 && now > previousMs) {
    float dt = (now - previousMs) / 1000.0f;
    if (dt >= 0.03f && dt <= 2.0f) {
      float instantClosingMps = (previousM - uwbRange.filteredDistanceM) / dt;
      instantClosingMps = constrain(instantClosingMps, -15.0f, 15.0f);
      uwbRange.closingSpeedMps =
        0.30f * instantClosingMps +
        0.70f * uwbRange.closingSpeedMps;
    }
  }
  uwbRange.previousFilteredM = uwbRange.filteredDistanceM;
  uwbRange.previousFilteredMs = now;

  sendUwbRangePacket();
}

void parseUwbLine(char *line) {
  char *distanceField = strstr(line, "distance[cm]=");
  if (distanceField != nullptr &&
      (strstr(line, "status=\"SUCCESS\"") != nullptr ||
       strstr(line, "status=SUCCESS") != nullptr)) {
    distanceField += strlen("distance[cm]=");
    float distanceCm = strtof(distanceField, nullptr);

    float rssiDbm = NAN;
    char *rssiField = strstr(line, "RSSI[dBm]=");
    if (rssiField != nullptr) {
      rssiDbm = strtof(rssiField + strlen("RSSI[dBm]="), nullptr);
    }
    acceptUwbMeasurement(distanceCm / 100.0f, rssiDbm);
    return;
  }

  if (strstr(line, "status=\"") != nullptr &&
      strstr(line, "SUCCESS") == nullptr) {
    uwbRange.failureCount++;
  }
}

void updateUwb() {
#if USE_UWB
  static char line[256];
  static size_t length = 0;

  while (uwbUartReady && dfSerial.available() > 0) {
    char c = (char)dfSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (length > 0) {
        line[length] = '\0';
        parseUwbLine(line);
        length = 0;
      }
      continue;
    }
    if (length < sizeof(line) - 1) {
      line[length++] = c;
    } else {
      length = 0;
      uwbParseOverflowCount++;
    }
  }

  if (uwbRange.calibrating &&
      millis() - uwbRange.calibrationStartedMs > UWB_CAL_TIMEOUT_MS) {
    uwbRange.calibrating = false;
    cmdReply("UWB 보정 시간초과: %u/%u회 수신 (배선/CDK ranging 상태 확인)",
             (unsigned)uwbRange.calibrationSamples,
             (unsigned)UWB_CAL_REQUIRED_SAMPLES);
  }
#endif
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
  uwbRange.calibrated = tuningPrefs.getBool("uwbcal", false);
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
  tuningPrefs.putBool("uwbcal", uwbRange.calibrated);
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
  uwbRange.calibrated = false;
  uwbRange.calibrating = false;
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
  cmdReply("== 차량 전용 ==");
  cmdReply("play <1-4>  음성 테스트 "
           "(1주의 2경고 3위험 4충격)");
  cmdReply("cal         GPS 상대좌표 영점 보정 시작");
  cmdReply("uwb status  UWB 수신/보정 상태 확인");
  cmdReply("uwb cal <m> 알고 있는 거리에서 100회 자동보정");
  cmdReply("uwb stop    진행 중인 UWB 보정 취소");
  cmdReply("uwb reset   UWB 오프셋/보정 저장값 삭제");
}

bool runUwbCommand(String line) {
  String lower = line;
  lower.toLowerCase();
  if (lower != "uwb" && !lower.startsWith("uwb ")) return false;

  String args = line.substring(3);
  args.trim();
  String sub = args;
  String value = "";
  int space = args.indexOf(' ');
  if (space > 0) {
    sub = args.substring(0, space);
    value = args.substring(space + 1);
    value.trim();
  }
  sub.toLowerCase();

  if (sub.length() == 0 || sub == "status") {
    long ageMs = uwbRange.lastSampleMs > 0
      ? (long)(millis() - uwbRange.lastSampleMs) : -1L;
    cmdReply("UWB fresh=%u raw=%.3fm filtered=%.3fm closing=%.3fm/s age=%ldms samples=%lu fail=%lu",
             uwbIsFresh(millis()) ? 1u : 0u,
             uwbRange.rawDistanceM,
             uwbRange.filteredDistanceM,
             uwbRange.closingSpeedMps,
             ageMs,
             (unsigned long)uwbRange.sampleCount,
             (unsigned long)uwbRange.failureCount);
    cmdReply("UWB calibrated=%u calibrating=%u progress=%u/%u offset=%+.3fm risk=%lu",
             uwbRange.calibrated ? 1u : 0u,
             uwbRange.calibrating ? 1u : 0u,
             (unsigned)uwbRange.calibrationSamples,
             (unsigned)UWB_CAL_REQUIRED_SAMPLES,
             cfgUwbOffsetM,
             (unsigned long)cfgUwbRiskEnabled);
    return true;
  }

  if (sub == "cal") {
    float knownM = value.toFloat();
    if (value.length() == 0 || knownM < 0.20f || knownM > 50.0f) {
      cmdReply("사용법: uwb cal <실제거리m>  예) uwb cal 3");
      return true;
    }
    uwbRange.calibrating = true;
    uwbRange.calibrationKnownM = knownM;
    uwbRange.calibrationRawSumM = 0.0;
    uwbRange.calibrationSamples = 0;
    uwbRange.calibrationStartedMs = millis();
    cmdReply("UWB %.3fm 자동보정 시작: 두 장치를 움직이지 말 것 (%u회 수집)",
             knownM, (unsigned)UWB_CAL_REQUIRED_SAMPLES);
    return true;
  }

  if (sub == "stop") {
    uwbRange.calibrating = false;
    cmdReply("UWB 보정 취소");
    return true;
  }

  if (sub == "reset") {
    cfgUwbOffsetM = 0.0f;
    uwbRange.calibrated = false;
    uwbRange.calibrating = false;
    uwbRange.medianCount = 0;
    uwbRange.previousFilteredMs = 0;
    if (tuningPrefs.begin(TUNING_NVS_NAMESPACE, false)) {
      tuningPrefs.remove("uwboffset");
      tuningPrefs.remove("uwbcal");
      tuningPrefs.end();
    }
    cmdReply("UWB 보정값 삭제, offset=0");
    return true;
  }

  cmdReply("알 수 없는 UWB 명령: %s", sub.c_str());
  return true;
}

void runTuningCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  // 다른 노드의 응답이 되돌아온 경우는 명령으로 해석하지 않는다.
  if (line.startsWith("[CMD]") || line.startsWith(">")) return;

  if (runUwbCommand(line)) return;

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

void handleVehicleSerialCommands() {
  static bool previousButtonPressed = false;
  static uint32_t lastButtonEventMs = 0;
  uint32_t now = millis();

  bool buttonPressed = digitalRead(REL_CAL_BUTTON_PIN) == LOW;
  if (buttonPressed && !previousButtonPressed &&
      now - lastButtonEventMs >= 500UL) {
    lastButtonEventMs = now;
    startRelativeGpsCalibration();
  }
  previousButtonPressed = buttonPressed;

  readSerialCommandLines();
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
#if USE_DFPLAYER || USE_UWB
  // ESP32는 하드웨어 UART가 3개뿐이므로 UART1의 방향을 나눠 쓴다.
  // RX=UWB 거리 출력, TX=DFPlayer Pro 음성 명령.
  dfSerial.begin(DFPLAYER_BAUD, SERIAL_8N1,
                 USE_UWB ? UWB_RX : -1,
                 USE_DFPLAYER ? DFPLAYER_TX : -1);
  uwbUartReady = USE_UWB;
#if USE_DFPLAYER
  delay(1000);  // DFPlayer Pro 전원 안정화
  dfPlayerReady = true;
  sendDfPlayerAtCommand("AT");
  Serial.printf("[DFPLAYER PRO] TX-only ready GPIO=%d baud=%lu\n",
                DFPLAYER_TX, (unsigned long)DFPLAYER_BAUD);
#endif
#if USE_UWB
  Serial.printf("[UWB] DWM3001CDK CLI RX ready GPIO=%d baud=%lu\n",
                UWB_RX, (unsigned long)UWB_BAUD);
#endif
#endif
}

void initializeDfPlayer() {
#if USE_DFPLAYER
  if (!dfPlayerReady) return;
  char volumeCommand[20];
  snprintf(volumeCommand,
           sizeof(volumeCommand),
           "AT+VOL=%u",
           (unsigned int)constrain(cfgDfplayerVolume, 0, 30));
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

// =====================
// 차량 IMU 차체방향 + GPS 실제 이동방향 융합
// =====================
float vehicleDot3(float ax, float ay, float az,
                  float bx, float by, float bz) {
  return ax * bx + ay * by + az * bz;
}

float vehicleNorm3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

float signedHeadingDifference(float fromDeg, float toDeg) {
  return fmodf(toDeg - fromDeg + 540.0f, 360.0f) - 180.0f;
}

bool vehicleBodyHeadingIsFresh() {
  return vehicleBodyHeadingHasFix &&
         imuReady &&
         lastImuSampleMs > 0 &&
         millis() - lastImuSampleMs <=
           VEHICLE_IMU_FRESH_TIMEOUT_MS;
}

bool estimateVehicleBodyHeadingFromModel(float *outHeadingDeg) {
  vehicleHeadingAccelNorm = vehicleNorm3(accelX, accelY, accelZ);
  vehicleHeadingGyroNorm = vehicleNorm3(gyroX, gyroY, gyroZ);

  const float neutralNorm =
    vehicleNorm3(CAR_NEUTRAL_AX, CAR_NEUTRAL_AY, CAR_NEUTRAL_AZ);

  if (!isfinite(vehicleHeadingAccelNorm) ||
      !isfinite(vehicleHeadingGyroNorm) ||
      vehicleHeadingAccelNorm < CAR_HEADING_ACCEL_MIN_MPS2 ||
      vehicleHeadingAccelNorm > CAR_HEADING_ACCEL_MAX_MPS2 ||
      vehicleHeadingGyroNorm > CAR_HEADING_GYRO_MAX_DPS) {
    return false;
  }

  vehicleHeadingPoseCos =
    vehicleDot3(accelX, accelY, accelZ,
                CAR_NEUTRAL_AX, CAR_NEUTRAL_AY, CAR_NEUTRAL_AZ) /
    (vehicleHeadingAccelNorm * neutralNorm);

  if (!isfinite(vehicleHeadingPoseCos) ||
      vehicleHeadingPoseCos < CAR_HEADING_POSE_COS_MIN) {
    return false;
  }

  float mx = magX - CAR_MAG_CENTER_X_UT;
  float my = magY - CAR_MAG_CENTER_Y_UT;
  float mz = magZ - CAR_MAG_CENTER_Z_UT;

  float projectionA =
    vehicleDot3(mx, my, mz,
                CAR_MAG_AXIS_A_X,
                CAR_MAG_AXIS_A_Y,
                CAR_MAG_AXIS_A_Z);

  float projection90 =
    vehicleDot3(mx, my, mz,
                CAR_MAG_AXIS_90_X,
                CAR_MAG_AXIS_90_Y,
                CAR_MAG_AXIS_90_Z);

  // 네 방향 실측축 Gram 행렬의 역행렬.
  float coefficientA =
    0.00102335589f * projectionA -
    0.00007771140f * projection90;

  float coefficient90 =
   -0.00007771140f * projectionA +
    0.00107709046f * projection90;

  vehicleMagModelRadius =
    sqrtf(coefficientA * coefficientA +
          coefficient90 * coefficient90);

  float predictedX =
    coefficientA * CAR_MAG_AXIS_A_X +
    coefficient90 * CAR_MAG_AXIS_90_X;
  float predictedY =
    coefficientA * CAR_MAG_AXIS_A_Y +
    coefficient90 * CAR_MAG_AXIS_90_Y;
  float predictedZ =
    coefficientA * CAR_MAG_AXIS_A_Z +
    coefficient90 * CAR_MAG_AXIS_90_Z;

  vehicleMagModelResidual =
    vehicleNorm3(mx - predictedX,
                 my - predictedY,
                 mz - predictedZ);

  if (!isfinite(vehicleMagModelRadius) ||
      !isfinite(vehicleMagModelResidual) ||
      vehicleMagModelRadius < CAR_MAG_MODEL_RADIUS_MIN ||
      vehicleMagModelRadius > CAR_MAG_MODEL_RADIUS_MAX ||
      vehicleMagModelResidual > CAR_MAG_MODEL_RESIDUAL_MAX_UT) {
    return false;
  }

  float relativeHeadingDeg =
    atan2f(coefficient90, coefficientA) * RAD_TO_DEG;

  *outHeadingDeg = normalizeHeading(
    CAR_MAG_HEADING_OFFSET_DEG - relativeHeadingDeg
  );
  return true;
}

void updateVehicleBodyHeading(uint32_t now, float dt) {
  // 우회전 실측에서 중력축 자이로가 음수였으므로 부호를 반전한다.
  float gravityNorm = vehicleNorm3(gravityX, gravityY, gravityZ);
  vehicleYawRateDps = 0.0f;

  if (vehicleBodyHeadingHasFix &&
      gravityNorm > 1.0f &&
      dt > 0.0f && dt <= 0.10f) {
    float correctedGx = gyroX - CAR_GYRO_BIAS_X_DPS;
    float correctedGy = gyroY - CAR_GYRO_BIAS_Y_DPS;
    float correctedGz = gyroZ - CAR_GYRO_BIAS_Z_DPS;

    vehicleYawRateDps = -vehicleDot3(
      correctedGx, correctedGy, correctedGz,
      gravityX / gravityNorm,
      gravityY / gravityNorm,
      gravityZ / gravityNorm
    );

    if (fabsf(vehicleYawRateDps) <= CAR_GYRO_YAW_DEADBAND_DPS) {
      vehicleYawRateDps = 0.0f;
    } else {
      vehicleYawRateDps -=
        copysignf(CAR_GYRO_YAW_DEADBAND_DPS, vehicleYawRateDps);
    }

    vehicleBodyHeadingDeg = normalizeHeading(
      vehicleBodyHeadingDeg + vehicleYawRateDps * dt
    );
  }

  float measuredHeadingDeg = 0.0f;
  if (estimateVehicleBodyHeadingFromModel(&measuredHeadingDeg)) {
    bool previousFresh = vehicleBodyHeadingIsFresh();

    if (!previousFresh) {
      vehicleBodyHeadingDeg = measuredHeadingDeg;
    } else {
      float correctionDt =
        (now - lastVehicleMagAcceptedMs) / 1000.0f;
      correctionDt = constrain(correctionDt, 0.001f, 0.50f);
      float correctionAlpha =
        1.0f - expf(-correctionDt / CAR_MAG_CORRECTION_TAU_S);

      vehicleBodyHeadingDeg = blendHeading(
        vehicleBodyHeadingDeg,
        measuredHeadingDeg,
        correctionAlpha
      );
    }

    vehicleBodyHeadingHasFix = true;
    lastVehicleMagAcceptedMs = now;
    vehicleHeadingAcceptedCount++;
  } else {
    vehicleHeadingRejectedCount++;

    // 이번 실측에서는 전원/장착 환경이 달라져 기존 자력계 모델이
    // 정지 중에도 모두 거절됐다. 이미 GPS로 초기화한 차체방향은
    // 자력계 거절만으로 폐기하지 않고 자이로 적분을 계속 사용한다.
    // IMU 자체가 끊기면 vehicleBodyHeadingIsFresh()가 즉시 false가 된다.
  }
}

void updateVehicleDriveModeFromGps(float gpsHeadingDeg) {
  // 자력계 절대방향이 유효하지 않아도 첫 확실한 전진 구간의 GPS course로
  // 차체방향을 시작한다. 이후 차체를 돌리면 자이로도 함께 회전하고,
  // 차체 그대로 후진하면 GPS course만 180도 바뀌므로 두 동작을 구분한다.
  if (!vehicleBodyHeadingIsFresh()) {
    if (!imuReady || lastImuSampleMs == 0 ||
        millis() - lastImuSampleMs > VEHICLE_IMU_FRESH_TIMEOUT_MS ||
        rawGpsSpeedMps < VEHICLE_BODY_BOOTSTRAP_SPEED_MPS) {
      return;
    }

    vehicleBodyHeadingDeg = normalizeHeading(gpsHeadingDeg);
    vehicleBodyHeadingHasFix = true;
    vehicleDriveModeKnown = true;
    vehicleIsReversing = false;
    candidateReversing = false;
    driveModeCandidateCount = VEHICLE_DRIVE_MODE_CONFIRM_COUNT;

    Serial.printf(
      "[BODY HEADING] GPS bootstrap heading=%.1f speed=%.2f\n",
      vehicleBodyHeadingDeg,
      rawGpsSpeedMps
    );
    return;
  }

  float difference = fabsf(
    signedHeadingDifference(vehicleBodyHeadingDeg, gpsHeadingDeg)
  );
  bool measuredReversing = difference > 90.0f;

  if (driveModeCandidateCount == 0 ||
      measuredReversing != candidateReversing) {
    candidateReversing = measuredReversing;
    driveModeCandidateCount = 1;
  } else if (driveModeCandidateCount < 255) {
    driveModeCandidateCount++;
  }

  if (driveModeCandidateCount >= VEHICLE_DRIVE_MODE_CONFIRM_COUNT) {
    bool modeChanged =
      !vehicleDriveModeKnown ||
      vehicleIsReversing != candidateReversing;

    vehicleIsReversing = candidateReversing;
    vehicleDriveModeKnown = true;

    float gpsBodyHeadingDeg = normalizeHeading(
      gpsHeadingDeg + (vehicleIsReversing ? 180.0f : 0.0f)
    );
    vehicleBodyHeadingDeg = blendHeading(
      vehicleBodyHeadingDeg,
      gpsBodyHeadingDeg,
      VEHICLE_BODY_GPS_CORRECTION_ALPHA
    );

    if (modeChanged) {
      Serial.printf(
        "[DRIVE MODE] %s body=%.1f gpsCourse=%.1f diff=%.1f\n",
        vehicleIsReversing ? "REVERSE" : "FORWARD",
        vehicleBodyHeadingDeg,
        gpsHeadingDeg,
        difference
      );
    }
  }
}

void updateVehiclePathHeading() {
  if (usingDemoGps) return;

  uint32_t now = millis();
  bool moving = vehicleSpeed >= cfgMinHeadingSpeed;
  bool gpsHeadingFresh =
    gpsMotionHeadingHasFix &&
    lastGpsMotionHeadingMs > 0 &&
    now - lastGpsMotionHeadingMs <= GPS_MOTION_HEADING_TIMEOUT_MS;

  if (moving && gpsHeadingFresh) {
    // 실제 경로가 살짝 휘거나 후진해도 GPS course가 그대로 반영된다.
    vehicleHeading = gpsMotionHeadingDeg;
    vehicleHeadingValid = true;
    return;
  }

  bool imuFallbackFresh =
    moving &&
    vehicleDriveModeKnown &&
    vehicleBodyHeadingIsFresh() &&
    lastGpsMotionHeadingMs > 0 &&
    now - lastGpsMotionHeadingMs <= VEHICLE_BODY_HEADING_TIMEOUT_MS;

  if (imuFallbackFresh) {
    vehicleHeading = normalizeHeading(
      vehicleBodyHeadingDeg + (vehicleIsReversing ? 180.0f : 0.0f)
    );
    vehicleHeadingValid = true;
  } else {
    vehicleHeadingValid = false;
  }
}

// =====================
// GPS
// =====================
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

bool configureNeo6m5Hz() {
  // 9600bps에서 5Hz NMEA 전체를 보내면 대역폭이 부족할 수 있다.
  // TinyGPSPlus에 필요한 GGA(위성 수/HDOP)와 RMC(위치/속도/방향)만 남긴다.
  static const uint8_t nmeaMessageRates[][3] = {
    {0xF0, 0x00, 1},  // GGA ON
    {0xF0, 0x01, 0},  // GLL OFF
    {0xF0, 0x02, 0},  // GSA OFF
    {0xF0, 0x03, 0},  // GSV OFF
    {0xF0, 0x04, 1},  // RMC ON
    {0xF0, 0x05, 0}   // VTG OFF
  };

  // UBX-CFG-RATE: measRate=200ms, navRate=1, timeRef=GPS time.
  static const uint8_t rate5Hz[] = {
    0xC8, 0x00,
    0x01, 0x00,
    0x01, 0x00
  };
  // 부팅 직후 쌓인 NMEA로 RX 버퍼가 가득 차면 UBX-ACK가 유실될 수
  // 있으므로 각 명령 전에 버퍼를 비우고 ACK를 즉시 소비한다.
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    discardGpsSerialInput(100);
    for (size_t i = 0;
         i < sizeof(nmeaMessageRates) / sizeof(nmeaMessageRates[0]);
         i++) {
      discardGpsSerialInput(20);
      sendGpsUbxCommand(0x06, 0x01, nmeaMessageRates[i], 3);
      waitForGpsUbxAck(0x06, 0x01, 250UL);
    }

    discardGpsSerialInput(30);
    sendGpsUbxCommand(0x06, 0x08, rate5Hz, sizeof(rate5Hz));
    bool rateAck = waitForGpsUbxAck(0x06, 0x08, 700UL);
    uint16_t actualRateMs = 0;
    bool rateReadback = pollGpsMeasurementRate(actualRateMs, 700UL);
    if (rateReadback) {
      Serial.printf("[GPS] CFG-RATE readback=%ums (%s)\n",
                    actualRateMs,
                    actualRateMs == 200 ? "5Hz" : "not 5Hz");
    }
    if (actualRateMs == 200 || (rateAck && !rateReadback)) {
      return true;
    }

    Serial.printf("[GPS] NEO-6M 5Hz no response attempt=%u/3\n", attempt);
    delay(300);
  }

  return false;
}

void setupGps() {
#if USE_GPS
  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1500);
  bool gps5HzConfigured = configureNeo6m5Hz();
  Serial.printf(
    "[GPS] NEO-6M 5Hz config=%s, GGA+RMC, 9600bps\n",
    gps5HzConfigured ? "ACK" : "FAILED"
  );
  Serial.println("[GPS] ready: TX->GPIO16, RX<-GPIO17");
#endif
}

void recoverGps5HzIfNeeded() {
#if USE_GPS
  if (!gps5HzRecoveryRequested || relativeGpsCalibration.active) return;

  uint32_t now = millis();
  if (now < GPS_5HZ_RECOVERY_MIN_UPTIME_MS) return;
  if (lastGps5HzRecoveryAttemptMs > 0 &&
      now - lastGps5HzRecoveryAttemptMs <
        GPS_5HZ_RECOVERY_COOLDOWN_MS) {
    return;
  }

  lastGps5HzRecoveryAttemptMs = now;
  Serial.printf(
    "[GPS] observed %.0fms updates -> retrying 5Hz configuration\n",
    gpsObservedUpdateIntervalMs
  );

  bool recovered = configureNeo6m5Hz();
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

        // GPS course는 차체 전방이 아니라 실제 이동 경로다.
        // 따라서 후진과 완벽하지 않은 직선 주행도 그대로 반영한다.
        if (courseOk &&
            rawSpeed >= cfgMinHeadingSpeed) {
          float rawHeading = rawGpsCourseDeg;
          bool previousCourseFresh =
            gpsMotionHeadingHasFix &&
            lastGpsMotionHeadingMs > 0 &&
            now - lastGpsMotionHeadingMs <=
              GPS_MOTION_HEADING_TIMEOUT_MS;

          gpsMotionHeadingDeg =
            previousCourseFresh
              ? blendHeading(gpsMotionHeadingDeg, rawHeading, 0.35f)
              : rawHeading;
          gpsMotionHeadingHasFix = true;
          lastGpsMotionHeadingMs = now;
          updateVehicleDriveModeFromGps(gpsMotionHeadingDeg);
        }
      }
    }
  }

  if (gpsFilter.initialized &&
      now - gpsFilter.lastFixMs <= cfgGpsFixMaxAgeMs) {
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
    // 기본 ±2g는 일반 노면에서도 포화됐다. car_20/21과 같은 ±8g 사용.
    ICM_20948_fss_t imuFullScale;
    imuFullScale.a = gpm8;
    imu.setFullScale(ICM_20948_Internal_Acc, imuFullScale);

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

  uint32_t now = millis();
  float imuDt =
    lastImuSampleMs > 0
      ? (now - lastImuSampleMs) / 1000.0f
      : 0.0f;

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
  lastImuSampleMs = now;

  if (!imuHasSample) {
    gravityX = accelX;
    gravityY = accelY;
    gravityZ = accelZ;
    imuHasSample = true;
    updateVehicleBodyHeading(now, 0.0f);
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

  if (linearAccelMagnitude > impactPeakSinceTelemetry) {
    impactPeakSinceTelemetry = linearAccelMagnitude;
  }

  updateVehicleBodyHeading(now, imuDt);

  bool cooldownDone =
    lastImpactMs == 0 ||
    now - lastImpactMs >= IMPACT_COOLDOWN_MS;

  if (cooldownDone && linearAccelMagnitude >= cfgImpactThreshold) {
    lastImpactMs = now;
    lastImpactTriggerMagnitude = linearAccelMagnitude;
    lastImpactTriggerAtMs = now;
    Serial.printf("[IMU IMPACT] linear=%.2f m/s^2\n", linearAccelMagnitude);
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
  if (vehicleSpeed < cfgMinHeadingSpeed) return false;

  return fabsf(headingErrorDeg) <= cfgForwardConeDeg;
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
      ? cfgRiskEscalateCount
      : cfgRiskClearCount;

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
  *outDistance = -1.0f;
  *outClosingSpeed = 0.0f;
  *outTtc = 999.0f;
  *outBearing = 0.0f;
  *outHeadingError = 0.0f;
  *outInVehiclePath = false;

  bool useUwbRange =
    cfgUwbRiskEnabled != 0 &&
    uwbRange.calibrated &&
    uwbIsFresh(millis()) &&
    isfinite(uwbRange.filteredDistanceM) &&
    uwbRange.filteredDistanceM >= UWB_MIN_DISTANCE_M;

  // 보정 전 GPS 좌표만으로는 20~50m 상대 바이어스가 생겼으므로
  // BOOT 영점보정은 GPS 방향을 위해 계속 필요하다.
  // UWB가 검증/활성화되면 RSSI 근접 게이트 대신 UWB 거리를 쓴다.
  if (!relativeGpsCalibration.valid ||
      relativeGpsCalibration.active ||
      (!useUwbRange &&
       (!caneRssiIsFresh() ||
        rssiProximityZone == RSSI_ZONE_UNKNOWN ||
        rssiProximityZone == RSSI_ZONE_FAR ||
        rssiEstimatedDistanceM <= 0.0f))) {
    return RISK_SAFE;
  }

  float rx;
  float ry;
  calculateRawRelativeVector(cane, &rx, &ry);
  rx -= relativeGpsCalibration.biasEastM;
  ry -= relativeGpsCalibration.biasNorthM;

  // 영점보정 GPS 벡터의 방향과 크기를 그대로 사용한다.
  // 2026-08-12 평행 실측에서 RSSI 크기로 치환하면 실제 횡방향 이격거리
  // 약 4~7m가 2.5~4m로 축소되어 CAUTION/WARNING 오경보가 발생했다.
  float correctedGpsDistanceM = hypotf(rx, ry);
  if (!isfinite(correctedGpsDistanceM) ||
      correctedGpsDistanceM < 0.75f) {
    return RISK_SAFE;
  }

  float distanceM = correctedGpsDistanceM;
  if (useUwbRange) {
    float scale = uwbRange.filteredDistanceM / correctedGpsDistanceM;
    rx *= scale;
    ry *= scale;
    distanceM = uwbRange.filteredDistanceM;
  }
  float bearing = normalizeHeading(
    atan2f(rx, ry) * RAD_TO_DEG
  );
  float headingError =
    angleDifferenceDegrees(vehicleHeading, bearing);

  *outDistance = distanceM;
  *outBearing = bearing;
  *outHeadingError = headingError;

  // 먼 거리 RSSI에서는 독립 GPS 두 대의 순간 상대오차만으로 경보하지 않는다.
  // 05-2 평행 로그의 GPS가 실제 약 4m를 0.9m로 오인했지만 RSSI는
  // -72dBm보다 약했으므로 이 게이트에서 제거된다.
  if (!useUwbRange && filteredCaneRssiDbm < cfgRssiGateDbm) {
    return RISK_SAFE;
  }

  if (distanceM < cfgEmergencyM) {
    *outInVehiclePath = true;
    return RISK_DANGER;
  }

  bool vehicleMoving =
    vehicleHeadingValid &&
    vehicleSpeed >= cfgMinPredictVehicleSpeed;
  if (!vehicleMoving) return RISK_SAFE;

  bool frontalRssiApproach =
    vehicleDriveModeKnown &&
    !vehicleIsReversing &&
    fabsf(headingError) <= cfgApproachConeDeg;
  if (frontalRssiApproach) {
    *outInVehiclePath = true;
  }

  float vehicleRad = vehicleHeading * DEG_TO_RAD;
  float vehicleVx = vehicleSpeed * sinf(vehicleRad); // 동쪽 +
  float vehicleVy = vehicleSpeed * cosf(vehicleRad); // 북쪽 +

  bool caneMoving =
    cane.heading_valid &&
    cane.speed_mps >= cfgMinPredictCaneSpeed;

  float caneVx = 0.0f;
  float caneVy = 0.0f;
  if (caneMoving) {
    float caneRad = cane.heading_deg * DEG_TO_RAD;
    caneVx = cane.speed_mps * sinf(caneRad);
    caneVy = cane.speed_mps * cosf(caneRad);
  }

  // 지팡이-차량 상대속도.
  float rvx = caneVx - vehicleVx;
  float rvy = caneVy - vehicleVy;
  float relativeSpeedSq = rvx * rvx + rvy * rvy;
  if (relativeSpeedSq < 0.01f) {
    return frontalRssiApproach ? RISK_CAUTION : RISK_SAFE;
  }

  float dot = rx * rvx + ry * rvy;
  float closingSpeed = -dot / fmaxf(distanceM, 0.1f);
  float tCpa = -dot / relativeSpeedSq;

  if (useUwbRange && isfinite(uwbRange.closingSpeedMps)) {
    closingSpeed = uwbRange.closingSpeedMps;
  }

  *outClosingSpeed = closingSpeed;

  if (closingSpeed < cfgMinClosingSpeed ||
      tCpa <= 0.0f ||
      tCpa > cfgCpaHorizonS) {
    return frontalRssiApproach ? RISK_CAUTION : RISK_SAFE;
  }

  float closestX = rx + rvx * tCpa;
  float closestY = ry + rvy * tCpa;
  float dCpa = sqrtf(closestX * closestX + closestY * closestY);

  float riskTtc = tCpa;
  if (useUwbRange && closingSpeed > cfgMinClosingSpeed) {
    riskTtc = distanceM / closingSpeed;
  }

  *outTtc = riskTtc;
  *outInVehiclePath = dCpa <= cfgCpaCautionM;

  if (dCpa <= cfgCpaDangerM &&
      riskTtc <= RISK_DANGER_TTC_S) {
    return RISK_DANGER;
  }

  if (dCpa <= cfgCpaWarningM &&
      riskTtc <= RISK_WARNING_TTC_S) {
    return RISK_WARNING;
  }

  if (dCpa <= cfgCpaCautionM &&
      riskTtc <= RISK_CAUTION_TTC_S) {
    return RISK_CAUTION;
  }

  return frontalRssiApproach ? RISK_CAUTION : RISK_SAFE;
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
  txVehicleStatus.heading_valid = vehicleHeadingValid ? 1 : 0;
  txVehicleStatus.node_id = vehicleId;
  txVehicleStatus.latitude = (float)vehicleLat;
  txVehicleStatus.longitude = (float)vehicleLng;
  txVehicleStatus.speed_mps = vehicleSpeed;
  txVehicleStatus.heading_deg = vehicleHeading;
  txVehicleStatus.timestamp_ms = millis();
  txVehicleStatus.seq_num = vehicleSeq++;
  bool exportUwbDistance =
    uwbRange.calibrated &&
    uwbIsFresh(millis()) &&
    isfinite(uwbRange.filteredDistanceM) &&
    uwbRange.filteredDistanceM >= UWB_MIN_DISTANCE_M;
  txVehicleStatus.rssi_distance_m =
    exportUwbDistance ? uwbRange.filteredDistanceM : rssiEstimatedDistanceM;
}

void sendVehicleStatus() {
  buildVehicleStatusPacket();
  esp_err_t result = esp_now_send(broadcastMAC,
                                  (uint8_t *)&txVehicleStatus,
                                  sizeof(txVehicleStatus));
  sendCount++;

  Serial.printf(
    "[VEHICLE TX] seq=%u gps=%u%s headingOk=%u heading=%.1f "
    "lat=%.6f lng=%.6f speed=%.2f risk=%u result=%s\n",
    txVehicleStatus.seq_num,
    txVehicleStatus.gps_valid,
    usingDemoGps ? "(DEMO)" : "",
    txVehicleStatus.heading_valid,
    txVehicleStatus.heading_deg,
    txVehicleStatus.latitude,
    txVehicleStatus.longitude,
    txVehicleStatus.speed_mps,
    txVehicleStatus.risk_level,
    result == ESP_OK ? "OK" : "ERR"
  );
}

void sendUwbRangePacket() {
#if USE_UWB
  if (!uwbIsFresh(millis())) return;

  memset(&txUwbRange, 0, sizeof(txUwbRange));
  txUwbRange.magic = V2X_MAGIC;
  txUwbRange.version = V2X_VERSION;
  txUwbRange.msg_type = MSG_UWB_RANGE;
  txUwbRange.node_type = NODE_VEHICLE;
  txUwbRange.flags = 0x01 | (uwbRange.calibrated ? 0x02 : 0x00);
  // 지팽이 ID를 아직 못 받은 시점에도 진단값은 보낼 수 있게 방송한다.
  txUwbRange.target_id = 0;
  txUwbRange.src_id = vehicleId;
  txUwbRange.raw_distance_m = uwbRange.rawDistanceM;
  txUwbRange.distance_m = uwbRange.filteredDistanceM;
  txUwbRange.closing_speed_mps = uwbRange.closingSpeedMps;
  txUwbRange.offset_m = cfgUwbOffsetM;
  txUwbRange.timestamp_ms = millis();
  txUwbRange.seq_num = uwbSeq++;

  if (esp_now_send(broadcastMAC,
                   (uint8_t *)&txUwbRange,
                   sizeof(txUwbRange)) == ESP_OK) {
    uwbSendCount++;
  }
#endif
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
#if !ENABLE_RSU_RISK_INPUT
  Serial.printf("[RSU RX] ignored risk=%u seq=%u (direct vehicle mode)\n",
                message.risk_level,
                message.seq_num);
  return;
#endif
  if (message.risk_level > RISK_DANGER) return;
  lastRsuRiskRxMs = millis();
  rsuRiskRxCount++;
  lastRiskLevel = message.risk_level;
  announceRisk(lastRiskLevel);
  Serial.printf("[RSU RX LEGACY] risk=%u seq=%u\n",
                message.risk_level,
                message.seq_num);
}

bool isRiskForThisVehicle(uint32_t targetId) {
  return targetId == 0 ||
         targetId == vehicleId ||
         targetId == 0xFFFFFFFFUL;
}

void handleRsuRiskAlert(const v2x_risk_message_t &message) {
#if !ENABLE_RSU_RISK_INPUT
  Serial.printf("[RSU RISK RX] ignored risk=%u seq=%u\n",
                message.risk_level,
                message.seq_num);
  return;
#endif
  if (!isRiskForThisVehicle(message.target_id) ||
      message.risk_level > RISK_DANGER) {
    return;
  }

  lastRsuRiskRxMs = millis();
  rsuRiskRxCount++;
  lastRiskLevel = message.risk_level;
  announceRisk(lastRiskLevel);

  Serial.printf(
    "[RSU RISK RX] risk=%u target=%lu src=%lu "
    "distance=%.2f closing=%.2f ttc=%.2f seq=%u\n",
    message.risk_level,
    (unsigned long)message.target_id,
    (unsigned long)message.src_id,
    message.distance_m,
    message.closing_speed_mps,
    message.ttc_s,
    message.seq_num
  );
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(v2x_risk_message_t)) {
    v2x_risk_message_t riskMessage;
    memcpy(&riskMessage, data, sizeof(riskMessage));
    if (riskMessage.magic == V2X_MAGIC &&
        riskMessage.version == V2X_VERSION &&
        riskMessage.msg_type == MSG_RISK_ALERT &&
        riskMessage.node_type == NODE_RSU) {
      handleRsuRiskAlert(riskMessage);
    }
    return;
  }

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
        cfgRssiAlpha * (float)receivedRssiDbm +
        (1.0f - cfgRssiAlpha) * filteredCaneRssiDbm;
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

  updateRssiProximity();
  updateRelativeGpsCalibration(cane);

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

#if !ENABLE_RSU_RISK_INPUT
    lastRiskLevel = RISK_SAFE;
    announceRisk(RISK_SAFE);
#endif

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

#if !ENABLE_RSU_RISK_INPUT
  lastRiskLevel = risk;
  announceRisk(risk);
#endif

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

#if !ENABLE_RSU_RISK_INPUT
  lastRiskLevel = RISK_SAFE;
  announceRisk(RISK_SAFE);
#endif

  // 기존 거리 및 접근속도 기록을 모두 초기화.
  resetAllCaneRiskStates();

  Serial.println("[CANE] timeout -> risk reset");
}

void resetStaleRsuRisk() {
#if ENABLE_RSU_RISK_INPUT
  uint32_t receivedMs = lastRsuRiskRxMs;
  if (receivedMs == 0 ||
      millis() - receivedMs <= RSU_RISK_TIMEOUT_MS) {
    return;
  }

  lastRsuRiskRxMs = 0;
  lastRiskLevel = RISK_SAFE;
  announceRisk(RISK_SAFE);
  Serial.println("[RSU RISK] timeout -> SAFE");
#endif
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
  char udpBuffer[2560];

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
  long uwbAgeMs = uwbRange.lastSampleMs > 0
    ? (long)(telemetryMs - uwbRange.lastSampleMs)
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
    "이동방향유효:%u\n"
    "GPS이동방향:%.1f\n"
    "차체방향유효:%u\n"
    "차체방향:%.1f\n"
    "후진판정:%u\n"
    "IMU요회전속도:%.2f\n"
    "방향가속도크기:%.2f\n"
    "방향자이로크기:%.2f\n"
    "방향자세cos:%.4f\n"
    "자기모델잔차:%.2f\n"
    "자기모델반경:%.3f\n"
    "방향채택:%lu\n"
    "방향거부:%lu\n"
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
    "RSSI구간:%u\n"
    "RSSI거리:%.2f\n"
    "상대보정:%u\n"
    "상대보정중:%u\n"
    "보정동쪽:%.2f\n"
    "보정북쪽:%.2f\n"
    "UWB유효:%u\n"
    "UWB원시거리:%.3f\n"
    "UWB보정거리:%.3f\n"
    "UWB접근속도:%.3f\n"
    "UWB_RSSI:%.1f\n"
    "UWB경과ms:%ld\n"
    "UWB샘플:%lu\n"
    "UWB실패:%lu\n"
    "UWB보정:%u\n"
    "UWB보정중:%u\n"
    "UWB보정샘플:%u\n"
    "UWB보정기준:%.3f\n"
    "UWB오프셋:%.3f\n"
    "UWB위험사용:%lu\n"
    "UWB전송:%lu\n"
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
    vehicleHeadingValid ? 1u : 0u,
    gpsMotionHeadingDeg,
    vehicleBodyHeadingIsFresh() ? 1u : 0u,
    vehicleBodyHeadingDeg,
    vehicleDriveModeKnown && vehicleIsReversing ? 1u : 0u,
    vehicleYawRateDps,
    vehicleHeadingAccelNorm,
    vehicleHeadingGyroNorm,
    vehicleHeadingPoseCos,
    vehicleMagModelResidual,
    vehicleMagModelRadius,
    (unsigned long)vehicleHeadingAcceptedCount,
    (unsigned long)vehicleHeadingRejectedCount,
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
    rssiProximityZone,
    rssiEstimatedDistanceM,
    relativeGpsCalibration.valid ? 1u : 0u,
    relativeGpsCalibration.active ? 1u : 0u,
    relativeGpsCalibration.biasEastM,
    relativeGpsCalibration.biasNorthM,
    uwbIsFresh(telemetryMs) ? 1u : 0u,
    uwbRange.rawDistanceM,
    uwbRange.filteredDistanceM,
    uwbRange.closingSpeedMps,
    uwbRange.lastRssiDbm,
    uwbAgeMs,
    (unsigned long)uwbRange.sampleCount,
    (unsigned long)uwbRange.failureCount,
    uwbRange.calibrated ? 1u : 0u,
    uwbRange.calibrating ? 1u : 0u,
    (unsigned)uwbRange.calibrationSamples,
    uwbRange.calibrationKnownM,
    cfgUwbOffsetM,
    (unsigned long)cfgUwbRiskEnabled,
    (unsigned long)uwbSendCount,
    (unsigned long)riskSendCount
  );

  if (written <= 0) {
    return;
  }

  size_t sendLength =
    written < (int)sizeof(udpBuffer)
      ? (size_t)written
      : sizeof(udpBuffer) - 1;

#if USE_WEB_VIEWER
  strlcpy(webCarTelemetry, udpBuffer, sizeof(webCarTelemetry));
  webCarUpdatedMs = millis();
#endif

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
#if USE_WEB_VIEWER
// 웹 화면 하단에 표시할 응답을 모아 둔다. 가득 차면 앞쪽 절반을 버린다.
void webLogReply(const char *prefix, const char *message) {
  size_t used = strlen(webReplyBuffer);
  size_t need = strlen(prefix) + strlen(message) + 2;

  if (used + need >= sizeof(webReplyBuffer)) {
    size_t drop = sizeof(webReplyBuffer) / 2;
    if (drop < used) {
      memmove(webReplyBuffer, webReplyBuffer + drop, used - drop + 1);
    } else {
      webReplyBuffer[0] = '\0';
    }
  }

  strlcat(webReplyBuffer, prefix, sizeof(webReplyBuffer));
  strlcat(webReplyBuffer, message, sizeof(webReplyBuffer));
  strlcat(webReplyBuffer, "\n", sizeof(webReplyBuffer));
}

void handleWebRoot() {
  webServer.send_P(200, "text/html; charset=utf-8", WEB_PAGE);
}

// 애플/안드로이드 기기의 인터넷 확인 요청에 정상 응답을 돌려준다.
void handleCaptiveCheck() {
  webServer.send(200, "text/html",
                 "<HTML><HEAD><TITLE>Success</TITLE></HEAD>"
                 "<BODY>Success</BODY></HTML>");
}

void handleWebData() {
  uint32_t now = millis();
  long carAge = webCarUpdatedMs == 0 ? -1L : (long)(now - webCarUpdatedMs);
  long caneAge = webCaneUpdatedMs == 0 ? -1L : (long)(now - webCaneUpdatedMs);

  String out;
  out.reserve(3200);
  out += "###META\ncarAge:";
  out += carAge;
  out += "\ncaneAge:";
  out += caneAge;
  out += "\n###CAR\n";
  out += webCarTelemetry;
  out += "\n###CANE\n";
  out += webCaneTelemetry;
  out += "\n###REPLY\n";
  out += webReplyBuffer;

  // 한 번 보낸 응답은 비워서 같은 줄이 반복 표시되지 않게 한다.
  webReplyBuffer[0] = '\0';

  webServer.send(200, "text/plain; charset=utf-8", out);
}

void handleWebCmd() {
  String target = webServer.arg("target");
  String text = webServer.arg("text");
  text.trim();

  if (text.length() == 0) {
    webServer.send(200, "text/plain", "empty");
    return;
  }

  if (target == "cane") {
    if (!caneNodeIpKnown) {
      webLogReply("[지팡이] ", "아직 지팡이 로그를 받지 못해 주소를 모른다");
    } else {
      String line = text + "\n";
      if (caneCmdUdp.beginPacket(caneNodeIp, CMD_UDP_PORT)) {
        caneCmdUdp.write((const uint8_t *)line.c_str(), line.length());
        caneCmdUdp.endPacket();
      }
    }
  } else {
    cmdReplyPort = 0;  // 응답은 시리얼과 웹 버퍼로만 보낸다.
    runTuningCommand(text);
  }

  webServer.send(200, "text/plain", "ok");
}

// 지팡이가 브로드캐스트한 텔레메트리를 받아 웹 화면용으로 저장한다.
void pollCaneTelemetry() {
  for (int size = caneLogUdp.parsePacket(); size > 0;
       size = caneLogUdp.parsePacket()) {
    int length = caneLogUdp.read(webCaneTelemetry,
                                 sizeof(webCaneTelemetry) - 1);
    if (length < 0) length = 0;
    webCaneTelemetry[length] = '\0';

    if (!caneNodeIpKnown) {
      Serial.print("[WEB] 지팡이 로그 수신 시작: ");
      Serial.println(caneLogUdp.remoteIP());
    }

    caneNodeIp = caneLogUdp.remoteIP();
    caneNodeIpKnown = true;
    webCaneUpdatedMs = millis();
  }
}

// 지팡이에게 보낸 명령의 응답을 받아 웹 화면에 표시한다.
void pollCaneCommandReplies() {
  for (int size = caneCmdUdp.parsePacket(); size > 0;
       size = caneCmdUdp.parsePacket()) {
    char buffer[240];
    int length = caneCmdUdp.read(buffer, sizeof(buffer) - 1);
    if (length < 0) length = 0;
    buffer[length] = '\0';

    char *context = NULL;
    for (char *token = strtok_r(buffer, "\r\n", &context);
         token != NULL;
         token = strtok_r(NULL, "\r\n", &context)) {
      webLogReply("[지팡이] ", token);
      Serial.printf("[CANE CMD] %s\n", token);
    }
  }
}

void setupWebViewer() {
  webServer.on("/", handleWebRoot);
  webServer.on("/data", handleWebData);
  webServer.on("/cmd", handleWebCmd);

  // iOS/맥은 와이파이에 붙으면 인터넷 확인용 주소를 먼저 찔러본다.
  // 여기서 엉뚱한 페이지를 돌려주면 "로그인이 필요한 와이파이"로 오해해
  // 로그인 창을 띄우거나 연결을 제한한다. 기대하는 Success 응답을 준다.
  webServer.on("/hotspot-detect.html", handleCaptiveCheck);
  webServer.on("/library/test/success.html", handleCaptiveCheck);
  webServer.on("/generate_204", handleCaptiveCheck);   // 안드로이드용

  // 그 밖의 주소는 페이지 대신 404를 돌려준다.
  webServer.onNotFound([]() {
    webServer.send(404, "text/plain", "not found");
  });

  webServer.begin();

  caneLogUdp.begin(CANE_LOG_UDP_PORT);
  caneCmdUdp.begin(CANE_CMD_REPLY_PORT);

  Serial.print("[WEB] 아이패드에서 V2X-LOG 접속 후 http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(" 열기");
}
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(REL_CAL_BUTTON_PIN, INPUT_PULLUP);

  captureTuningDefaults();
  loadTuningFromFlash();

  Serial.println("\n=== V2X Vehicle + GPS + ICM-20948 + DFPlayer Pro ===");
  Serial.printf("[CONFIG] CPA distance: %.1f / %.1f / %.1f m\n",
                (float)cfgCpaCautionM,
                (float)cfgCpaWarningM,
                (float)cfgCpaDangerM);

  setupGps();
  setupImu();
  setupDfPlayer();
  initializeDfPlayer();
  setupEspNow();

#if USE_BT_DEBUG
  SerialBT.begin("ESP32-Car");
  Serial.println("[BT] debug started: ESP32-Car");
#endif

#if USE_WEB_VIEWER
  setupWebViewer();
#endif

  Serial.println("[VEHICLE] system ready");
}

void loop() {
  handleVehicleSerialCommands();
  handleUdpCommands();
#if USE_WEB_VIEWER
  webServer.handleClient();
  pollCaneTelemetry();
  pollCaneCommandReplies();
#endif
  readGps();
  // 8/18 패치: 런타임 5 Hz 재설정(recoverGps5HzIfNeeded)은 configureNeo6m5Hz()가 3회 동기
  // 재시도로 10.7 s 동안 loop를 멈추고, 실측(8/17·8/18)에서 매분 :13~:24 송신 정지 →
  // RSU 판정 공백·재개 유령의 원인이었다. 부팅 시 5 Hz 설정만 두고 런타임 재설정은 끈다.
  // recoverGps5HzIfNeeded();
  readImu();
  updateVehiclePathHeading();
  updateUwb();
  processLatestCanePacket();
  resetStaleCaneRisk();
  resetStaleRsuRisk();
  logSensors();

uint32_t now = millis();

if (now - lastSendMs >= SEND_INTERVAL_MS) {
  lastSendMs = now;
  sendVehicleStatus();
}

  // 차량 로그는 1초마다 UDP 4211로 전송
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
    "[BT CHECK] ESP32-Car client=%d\n",
    SerialBT.hasClient() ? 1 : 0
  );
}
#endif

  // 상대 GPS 영점보정 중에는 빠르게 점멸한다.
  if (relativeGpsCalibration.active) {
    digitalWrite(LED_PIN, (now / 150UL) % 2);
  }
  // 최근 지팡이 패킷이 있으면 점등, 없으면 천천히 점멸.
  else if (lastCaneRxMs > 0 && now - lastCaneRxMs < 1000UL) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, (now / 500UL) % 2);
  }

  delay(5);
}
