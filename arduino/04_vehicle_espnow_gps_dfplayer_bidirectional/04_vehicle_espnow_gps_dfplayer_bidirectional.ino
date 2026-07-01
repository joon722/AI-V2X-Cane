// [강현준/박중선] V2X Vehicle Node
// GPS 수집 + ESP-NOW 양방향 통신 + 차량 부저/등/DFPlayer 음성 출력

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <TinyGPSPlus.h>

// =====================
// 기능 ON/OFF
// =====================
#define USE_GPS       1
#define USE_BUZZER    1
#define USE_LAMP      1
#define USE_DFPLAYER  1

// 처음 통신/출력 확인 때는 1, 실제 위험도 알고리즘 연결 후에는 0 권장
#define TEST_RISK_CYCLE 1

// =====================
// 핀 설정
// =====================
#define LED_PIN 2

// 차량 출력 장치
#define BUZZER_PIN 25
#define LAMP_PIN   27

// GPS UART
#define GPS_RX 16   // ESP32 RX2 <- GPS TX
#define GPS_TX 17   // ESP32 TX2 -> GPS RX
#define GPS_BAUD 9600

// DFPlayer UART
#define MP3_RX 32   // ESP32 RX1 <- DFPlayer TX
#define MP3_TX 33   // ESP32 TX1 -> DFPlayer RX
#define MP3_BAUD 9600

// =====================
// 출력 동작 방식
// =====================
// 연결한 부저 모듈 기준 Active LOW
#define BUZZER_ON  LOW
#define BUZZER_OFF HIGH

// 차량 등/LED 모듈은 Active HIGH로 가정
#define LAMP_ON  HIGH
#define LAMP_OFF LOW

// =====================
// risk level 정의
// =====================
#define NODE_VEHICLE    0x10
#define NODE_CANE_REPLY 0x02

#define RISK_SAFE    0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER  3

// =====================
// 지팡이/수신기 ESP32 MAC 주소
// 수신기 시리얼 모니터의 [RX] STA MAC Address 값을 넣기
// 예: 24:6F:28:AA:BB:CC -> {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}
// =====================
uint8_t caneMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // TODO: 지팡이 ESP32 MAC으로 교체

// =====================
// 송신기/수신기 공통 구조체
// 지팡이 수신기 코드와 100% 동일해야 함
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
// 전역 변수
// =====================
HardwareSerial gpsSerial(2);
HardwareSerial mp3Serial(1);
TinyGPSPlus gps;

v2x_message_t txPacket;
v2x_message_t rxReply;

uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t ackCount = 0;

uint32_t lastSendMs = 0;
uint32_t lastAckMs = 0;
const uint32_t SEND_INTERVAL_MS = 100;  // 10Hz 송신

int forcedRisk = -1;  // -1이면 자동 모드
uint8_t currentRisk = 255;

bool beepActive = false;
uint32_t beepStartMs = 0;
uint32_t beepDurationMs = 50;

uint32_t lastLampToggleMs = 0;
bool lampBlinkState = false;

uint32_t lastMp3PlayMs = 0;
const uint32_t MP3_MIN_INTERVAL_MS = 1500;

// =====================
// 출력 장치 즉시 OFF
// =====================
void forceOutputsOff() {
#if USE_BUZZER
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
#endif

#if USE_LAMP
  pinMode(LAMP_PIN, OUTPUT);
  digitalWrite(LAMP_PIN, LAMP_OFF);
#endif
}

// =====================
// MAC 주소 출력
// =====================
void printVehicleMac() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);

  Serial.print("[TX] STA MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

// =====================
// DFPlayer Mini UART 명령
// microSD에 0001.mp3, 0002.mp3, 0003.mp3 파일 사용
// =====================
void dfSendCommand(uint8_t cmd, uint16_t param) {
#if USE_DFPLAYER
  uint8_t packet[10];

  packet[0] = 0x7E;
  packet[1] = 0xFF;
  packet[2] = 0x06;
  packet[3] = cmd;
  packet[4] = 0x00;
  packet[5] = (param >> 8) & 0xFF;
  packet[6] = param & 0xFF;

  uint16_t checksum = 0 - (packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6]);
  packet[7] = (checksum >> 8) & 0xFF;
  packet[8] = checksum & 0xFF;
  packet[9] = 0xEF;

  mp3Serial.write(packet, 10);
#endif
}

void dfSetVolume(uint8_t volume) {
#if USE_DFPLAYER
  if (volume > 30) volume = 30;
  dfSendCommand(0x06, volume);
#endif
}

void dfPlayTrack(uint16_t trackNum) {
#if USE_DFPLAYER
  dfSendCommand(0x03, trackNum);
#endif
}

// =====================
// 부저 제어
// =====================
void startBeep(uint32_t durationMs) {
#if USE_BUZZER
  digitalWrite(BUZZER_PIN, BUZZER_ON);
  beepActive = true;
  beepStartMs = millis();
  beepDurationMs = durationMs;
#endif
}

void updateBeep() {
#if USE_BUZZER
  if (beepActive && millis() - beepStartMs >= beepDurationMs) {
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }
#endif
}

// =====================
// 등/LED 제어
// =====================
void updateLampByRisk() {
#if USE_LAMP
  uint32_t now = millis();

  if (currentRisk == RISK_SAFE) {
    digitalWrite(LAMP_PIN, LAMP_OFF);
    lampBlinkState = false;
  } else if (currentRisk == RISK_CAUTION) {
    if (now - lastLampToggleMs >= 600) {
      lastLampToggleMs = now;
      lampBlinkState = !lampBlinkState;
      digitalWrite(LAMP_PIN, lampBlinkState ? LAMP_ON : LAMP_OFF);
    }
  } else if (currentRisk == RISK_WARNING) {
    if (now - lastLampToggleMs >= 250) {
      lastLampToggleMs = now;
      lampBlinkState = !lampBlinkState;
      digitalWrite(LAMP_PIN, lampBlinkState ? LAMP_ON : LAMP_OFF);
    }
  } else {
    digitalWrite(LAMP_PIN, LAMP_ON);
    lampBlinkState = true;
  }
#endif
}

// =====================
// 차량 출력
// risk에 따라 부저/등/음성 재생
// =====================
void applyVehicleOutput(uint8_t risk) {
  if (risk == currentRisk) return;

  Serial.printf("[OUT] risk changed %u -> %u\n", currentRisk, risk);

  if (risk == RISK_SAFE) {
#if USE_BUZZER
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
#endif
#if USE_LAMP
    digitalWrite(LAMP_PIN, LAMP_OFF);
#endif
    beepActive = false;
    lampBlinkState = false;
  } else if (risk == RISK_CAUTION) {
    startBeep(40);

    if (millis() - lastMp3PlayMs > MP3_MIN_INTERVAL_MS) {
      dfPlayTrack(1);  // 0001.mp3
      lastMp3PlayMs = millis();
    }
  } else if (risk == RISK_WARNING) {
    startBeep(80);

    if (millis() - lastMp3PlayMs > MP3_MIN_INTERVAL_MS) {
      dfPlayTrack(2);  // 0002.mp3
      lastMp3PlayMs = millis();
    }
  } else {
    startBeep(150);

    if (millis() - lastMp3PlayMs > MP3_MIN_INTERVAL_MS) {
      dfPlayTrack(3);  // 0003.mp3
      lastMp3PlayMs = millis();
    }
  }

  currentRisk = risk;
  lastLampToggleMs = millis();
}

// =====================
// GPS 읽기
// =====================
void updateGps() {
#if USE_GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
#endif
}

// =====================
// risk 결정
// 현재는 테스트용 자동 순환. 이후 거리/TTC 알고리즘으로 교체.
// =====================
uint8_t decideRiskLevel() {
  if (forcedRisk >= 0) {
    return (uint8_t)forcedRisk;
  }

#if TEST_RISK_CYCLE
  // 100ms 송신 기준, 약 2초마다 0 -> 1 -> 2 -> 3 반복
  return (seq / 20) % 4;
#else
  return RISK_SAFE;
#endif
}

// =====================
// 송신 패킷 만들기
// =====================
void buildPacket() {
  txPacket.node_type = NODE_VEHICLE;
  txPacket.risk_level = decideRiskLevel();

#if USE_GPS
  bool gpsOk = gps.location.isValid() && gps.location.age() < 3000;

  txPacket.gps_valid = gpsOk ? 1 : 0;
  txPacket.latitude = gpsOk ? gps.location.lat() : 0.0;
  txPacket.longitude = gpsOk ? gps.location.lng() : 0.0;
  txPacket.speed_mps = gps.speed.isValid() ? gps.speed.mps() : 0.0;
  txPacket.heading_deg = gps.course.isValid() ? gps.course.deg() : 0.0;
#else
  txPacket.gps_valid = 0;
  txPacket.latitude = 0.0;
  txPacket.longitude = 0.0;
  txPacket.speed_mps = 0.0;
  txPacket.heading_deg = 0.0;
#endif

  // 차량 노드는 일단 IMU 없음. 구조체 호환을 위해 0으로 채움.
  txPacket.accel_x = 0.0;
  txPacket.accel_y = 0.0;
  txPacket.accel_z = 0.0;

  txPacket.gyro_x = 0.0;
  txPacket.gyro_y = 0.0;
  txPacket.gyro_z = 0.0;

  txPacket.timestamp_ms = millis();
  txPacket.seq_num = seq++;
}

// =====================
// ESP-NOW 송신
// =====================
void sendPacket() {
  buildPacket();

  esp_err_t result = esp_now_send(caneMAC, (uint8_t *)&txPacket, sizeof(txPacket));
  sendCount++;

  Serial.printf(
    "[SEND] seq=%u risk=%u gps=%u lat=%.6f lng=%.6f spd=%.2f send=%lu result=%s\n",
    txPacket.seq_num,
    txPacket.risk_level,
    txPacket.gps_valid,
    txPacket.latitude,
    txPacket.longitude,
    txPacket.speed_mps,
    (unsigned long)sendCount,
    result == ESP_OK ? "OK" : "ERR"
  );

  applyVehicleOutput(txPacket.risk_level);
}

// =====================
// 지팡이에서 돌아오는 응답 수신
// 지팡이 코드에 reply 송신 기능이 있어야 [RX BACK]이 찍힘
// =====================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(v2x_message_t)) {
    Serial.printf("[RX BACK] size mismatch. len=%d expected=%d\n", len, sizeof(v2x_message_t));
    return;
  }

  memcpy(&rxReply, data, sizeof(rxReply));

  ackCount++;
  lastAckMs = millis();

  Serial.printf(
    "[RX BACK] seq=%u node=%u risk=%u ack=%lu rx_ms=%lu\n",
    rxReply.seq_num,
    rxReply.node_type,
    rxReply.risk_level,
    (unsigned long)ackCount,
    (unsigned long)lastAckMs
  );
}

// =====================
// 시리얼 모니터 명령
// 0/1/2/3: risk 강제 설정
// a: 자동 순환, p: 0001.mp3 재생, v: 볼륨 25
// =====================
void handleSerialCommand() {
  if (!Serial.available()) return;

  char c = Serial.read();

  if (c >= '0' && c <= '3') {
    forcedRisk = c - '0';
    Serial.printf("[CMD] forced risk = %d\n", forcedRisk);
  } else if (c == 'a' || c == 'A') {
    forcedRisk = -1;
    Serial.println("[CMD] auto risk cycle mode");
  } else if (c == 'p' || c == 'P') {
    Serial.println("[CMD] play 0001.mp3");
    dfPlayTrack(1);
  } else if (c == 'v' || c == 'V') {
    Serial.println("[CMD] volume 25");
    dfSetVolume(25);
  }
}

// =====================
// ESP-NOW 초기화
// =====================
void setupEspNowVehicle() {
  WiFi.mode(WIFI_STA);
  delay(300);

  Serial.println("[ESP-NOW] Vehicle Start");
  printVehicleMac();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed. restart.");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, caneMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.println("[ESP-NOW] cane peer added.");
  } else {
    Serial.println("[ESP-NOW] cane peer add failed.");
  }

  Serial.println("[ESP-NOW] Vehicle Ready");
}

// =====================
// setup
// =====================
void setup() {
  forceOutputsOff();

  Serial.begin(115200);
  delay(1000);

  Serial.println("=== V2X Vehicle Node ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if USE_BUZZER
  Serial.println("[BUZZER] ready");
#endif

#if USE_LAMP
  Serial.println("[LAMP] ready");
#endif

#if USE_GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] ready");
  Serial.println("[GPS] GPS TX -> ESP32 GPIO16 RX2");
  Serial.println("[GPS] GPS RX -> ESP32 GPIO17 TX2");
#endif

#if USE_DFPLAYER
  mp3Serial.begin(MP3_BAUD, SERIAL_8N1, MP3_RX, MP3_TX);
  delay(500);
  dfSetVolume(22);
  Serial.println("[DFPlayer] ready");
  Serial.println("[DFPlayer] ESP32 GPIO33 TX1 -> DFPlayer RX");
  Serial.println("[DFPlayer] ESP32 GPIO32 RX1 <- DFPlayer TX");
#endif

  setupEspNowVehicle();

  Serial.println("[TX] System Ready");
  Serial.println("[CMD] 0/1/2/3 = force risk, a = auto, p = play 0001.mp3, v = volume 25");
}

// =====================
// loop
// =====================
void loop() {
  updateGps();
  updateBeep();
  updateLampByRisk();
  handleSerialCommand();

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    digitalWrite(LED_PIN, HIGH);
    sendPacket();
    digitalWrite(LED_PIN, LOW);
  }

  // 1초 이상 지팡이 응답이 없으면 내장 LED 깜빡임으로 경고
  if (ackCount == 0 || millis() - lastAckMs > 1000) {
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  }

  delay(5);
}
