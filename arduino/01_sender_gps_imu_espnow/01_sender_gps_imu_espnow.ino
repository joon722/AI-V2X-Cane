// [박중선] V2X Smart Cane Sender
// GPS + ICM-20948 IMU + ESP-NOW 송신기
// + 부저/진동모터 제어 추가

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi_types.h"
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <ICM_20948.h>
#include <math.h>

// =====================
// 기본 설정
// =====================
#define NODE_CANE 0x01

#define RISK_SAFE    0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER  3

#define I2C_SDA 21
#define I2C_SCL 22

// GPS 배선
// GPS TX -> ESP32 GPIO16
// GPS RX -> ESP32 GPIO17
#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

// 부저/진동모터 핀
#define BUZZER_PIN 25
#define MOTOR_PIN 26

// 부저 모듈이 Active LOW라고 가정
#define BUZZER_ON  LOW
#define BUZZER_OFF HIGH

// 진동모터 모듈은 Active HIGH라고 가정
#define MOTOR_ON  HIGH
#define MOTOR_OFF LOW

// 100ms = 10Hz 송신
#define SEND_PERIOD_MS 100

// ICM-20948 주소 설정
// 0x69면 AD0_VAL = 1
// 0x68이면 AD0_VAL = 0
#define AD0_VAL 1

// =====================
// 강현준 수신기 MAC 주소
// 1C:C3:AB:D1:73:5C
// =====================
uint8_t receiverMAC[] = {0x1C, 0xC3, 0xAB, 0xD1, 0x73, 0x5C};

// =====================
// 송신기/수신기 공통 구조체
// 수신기 코드와 100% 동일해야 함
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
// 객체 생성
// =====================
ICM_20948_I2C imu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

v2x_message_t packet;

uint16_t seq = 0;
uint32_t lastSendMs = 0;

// GPS 마지막 값 저장
float lastLat = 0.0;
float lastLng = 0.0;
float lastSpeed = 0.0;
float lastHeading = 0.0;
uint8_t lastGpsValid = 0;

// 부저 제어 변수
bool beepActive = false;
uint32_t beepStartMs = 0;
const uint32_t BEEP_DURATION_MS = 30;

uint8_t currentRisk = 255;

// =====================
// ESP-NOW 전송 결과 콜백
// ESP32 Core 3.3.8 기준
// =====================
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW TX] seq=");
  Serial.print(seq - 1);

  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println(" result=OK");
  } else {
    Serial.println(" result=FAIL");
  }
}

// =====================
// 액추에이터 초기화
// =====================
void setupActuator() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, BUZZER_OFF);
  digitalWrite(MOTOR_PIN, MOTOR_OFF);

  Serial.println("[ACT] Buzzer/Motor ready");
}

// =====================
// 부저 짧게 울리기 시작
// =====================
void startShortBeep() {
  digitalWrite(BUZZER_PIN, BUZZER_ON);
  beepActive = true;
  beepStartMs = millis();
}

// =====================
// 부저 자동으로 끄기
// =====================
void updateBeep() {
  if (beepActive && millis() - beepStartMs >= BEEP_DURATION_MS) {
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }
}

// =====================
// risk_level에 따른 부저/진동모터 동작
// =====================
void applyRisk(uint8_t risk) {
  if (risk == RISK_SAFE) {
    digitalWrite(MOTOR_PIN, MOTOR_OFF);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }

  else if (risk == RISK_CAUTION) {
    digitalWrite(MOTOR_PIN, MOTOR_ON);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }

  else if (risk == RISK_WARNING) {
    digitalWrite(MOTOR_PIN, MOTOR_ON);

    if (risk != currentRisk) {
      startShortBeep();
    }
  }

  else {
    digitalWrite(MOTOR_PIN, MOTOR_ON);

    if (risk != currentRisk) {
      startShortBeep();
    }
  }

  currentRisk = risk;
}

// =====================
// IMU 초기화
// =====================
void setupImu() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  imu.begin(Wire, AD0_VAL);

  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.println("[IMU] connected");
  } else {
    Serial.print("[IMU] failed. status=");
    Serial.println(imu.statusString());
    Serial.println("[IMU] hint: AD0_VAL 1/0 바꾸거나 SDA/SCL 확인");
  }
}

// =====================
// GPS 초기화
// =====================
void setupGps() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] UART2 started");
  Serial.println("[GPS] GPS TX -> ESP32 GPIO16, GPS RX -> ESP32 GPIO17");
}

// =====================
// ESP-NOW 초기화
// =====================
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);

  Serial.print("[ESP-NOW] Sender MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed. restart.");
    ESP.restart();
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.println("[ESP-NOW] receiver peer added.");
  } else {
    Serial.println("[ESP-NOW] peer add failed. Check receiverMAC[].");
  }
}

// =====================
// GPS 읽기
// =====================
void readGps() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    lastGpsValid = gps.location.isValid() ? 1 : 0;
  }

  if (gps.speed.isUpdated()) {
    lastSpeed = gps.speed.mps();
  }

  if (gps.course.isUpdated()) {
    lastHeading = gps.course.deg();
  }
}

// =====================
// IMU 값을 packet에 넣기
// =====================
void readImuIntoPacket() {
  if (imu.dataReady()) {
    imu.getAGMT();

    packet.accel_x = (imu.accX() / 1000.0) * 9.80665;
    packet.accel_y = (imu.accY() / 1000.0) * 9.80665;
    packet.accel_z = (imu.accZ() / 1000.0) * 9.80665;

    packet.gyro_x = imu.gyrX();
    packet.gyro_y = imu.gyrY();
    packet.gyro_z = imu.gyrZ();
  }
}

// =====================
// 테스트용 위험도 판단
// =====================
uint8_t decideRiskForTest(float speed, float ax, float ay, float az) {
  static int cautionCount = 0;
  static int warningCount = 0;

  float totalA = sqrt(ax * ax + ay * ay + az * az);
  float motionA = fabs(totalA - 9.80665);

  bool warningNow = (speed > 3.0 || motionA > 9.0);
  bool cautionNow = (speed > 1.8 || motionA > 5.0);

  if (warningNow) {
    warningCount++;
  } else {
    warningCount = 0;
  }

  if (cautionNow) {
    cautionCount++;
  } else {
    cautionCount = 0;
  }

  if (warningCount >= 3) {
    return RISK_WARNING;
  }

  if (cautionCount >= 3) {
    return RISK_CAUTION;
  }

  return RISK_SAFE;
}

// =====================
// 패킷 채우고 전송
// =====================
void fillAndSendPacket() {
  packet.node_type = NODE_CANE;

  packet.gps_valid = lastGpsValid;
  packet.latitude = lastLat;
  packet.longitude = lastLng;
  packet.speed_mps = lastSpeed;
  packet.heading_deg = lastHeading;

  packet.risk_level = decideRiskForTest(
    packet.speed_mps,
    packet.accel_x,
    packet.accel_y,
    packet.accel_z
  );

  // 지팡이 자체에서 부저/진동 동작
  applyRisk(packet.risk_level);

  packet.timestamp_ms = millis();
  packet.seq_num = seq++;

  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (result != ESP_OK) {
    Serial.println("[ESP-NOW] send function error.");
  }

  Serial.printf(
    "[SEND] seq=%u gps=%u lat=%.6f lng=%.6f spd=%.2f "
    "ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f "
    "risk=%u size=%d\n",
    packet.seq_num,
    packet.gps_valid,
    packet.latitude,
    packet.longitude,
    packet.speed_mps,
    packet.accel_x,
    packet.accel_y,
    packet.accel_z,
    packet.gyro_x,
    packet.gyro_y,
    packet.gyro_z,
    packet.risk_level,
    sizeof(packet)
  );
}

// =====================
// setup
// =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== V2X Smart Cane Sender: GPS + IMU + ESP-NOW + ACT ===");

  memset(&packet, 0, sizeof(packet));

  setupActuator();
  setupImu();
  setupGps();
  setupEspNow();

  Serial.println("[TX] Sender Ready");
}

// =====================
// loop
// =====================
void loop() {
  updateBeep();

  readGps();
  readImuIntoPacket();

  if (millis() - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = millis();
    fillAndSendPacket();
  }
}
