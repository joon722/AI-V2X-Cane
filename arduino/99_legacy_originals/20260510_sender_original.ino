// [박중선] V2X Smart Cane Sender
// GPS + ICM-20948 IMU + ESP-NOW 송신기

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

    // accX/Y/Z 단위가 mg라서 m/s^2로 변환
    packet.accel_x = (imu.accX() / 1000.0) * 9.80665;
    packet.accel_y = (imu.accY() / 1000.0) * 9.80665;
    packet.accel_z = (imu.accZ() / 1000.0) * 9.80665;

    // gyro는 deg/s 계열
    packet.gyro_x = imu.gyrX();
    packet.gyro_y = imu.gyrY();
    packet.gyro_z = imu.gyrZ();
  }
}

// =====================
// 테스트용 위험도 판단
// =====================
uint8_t decideRiskForTest(float speed, float ax, float ay, float az) {
  float totalA = sqrt(ax * ax + ay * ay + az * az);

  // 가만히 있으면 중력 때문에 약 9.8m/s^2
  // 그래서 9.8에서 얼마나 벗어났는지 봄
  float motionA = fabs(totalA - 9.80665);

  if (speed > 2.0 || motionA > 5.0) {
    return RISK_WARNING;   // 2
  }

  if (speed > 1.0 || motionA > 2.5) {
    return RISK_CAUTION;   // 1
  }

  return RISK_SAFE;        // 0
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

  packet.timestamp_ms = millis();
  packet.seq_num = seq++;

  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (result != ESP_OK) {
    Serial.println("[ESP-NOW] send function error.");
  }

  Serial.printf(
    "[SEND] seq=%u gps=%u lat=%.6f lng=%.6f spd=%.2f ax=%.2f ay=%.2f az=%.2f risk=%u size=%d\n",
    packet.seq_num,
    packet.gps_valid,
    packet.latitude,
    packet.longitude,
    packet.speed_mps,
    packet.accel_x,
    packet.accel_y,
    packet.accel_z,
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

  Serial.println("=== V2X Smart Cane Sender: GPS + IMU + ESP-NOW ===");

  memset(&packet, 0, sizeof(packet));

  setupImu();
  setupGps();
  setupEspNow();

  Serial.println("[TX] Sender Ready");
}

// =====================
// loop
// =====================
void loop() {
  readGps();
  readImuIntoPacket();

  if (millis() - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = millis();
    fillAndSendPacket();
  }
}
