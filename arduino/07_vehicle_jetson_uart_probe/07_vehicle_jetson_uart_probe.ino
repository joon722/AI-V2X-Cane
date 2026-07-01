// V2X Vehicle -> Jetson UART Probe
// 차량 ESP 단독 테스트용: 차량 상태 JSON을 USB Serial과 Jetson UART로 동시에 송신.
// 지팡이 ESP가 없어도 Jetson 입력/파싱/risk 회신 흐름을 먼저 확인할 수 있음.

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <math.h>

// =====================
// 기능 설정
// =====================
#define USE_SIMULATED_GPS 1
#define USE_ESPNOW_BROADCAST 1

// =====================
// 핀 설정
// =====================
#define LED_PIN 2

// Jetson UART
// ESP32 GPIO33 TX1 -> Jetson RX
// ESP32 GPIO32 RX1 <- Jetson TX
// GND 공통 연결 필수
#define JETSON_RX 32
#define JETSON_TX 33
#define JETSON_BAUD 115200

// =====================
// 송신 설정
// =====================
#define SEND_INTERVAL_MS 100   // 10Hz
#define V2X_MAGIC 0x56325831UL // "V2X1"

// 테스트 기준 좌표. 실험 장소 근처 좌표로 바꾸면 Jetson 거리 계산 확인이 쉬움.
#define START_LAT 37.000000
#define START_LNG 127.000000

typedef struct __attribute__((packed)) v2x_message {
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
} v2x_message_t;

HardwareSerial jetsonSerial(1);

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
v2x_message_t packet;

uint32_t vehicleId = 0;
uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t lastSendMs = 0;
uint8_t lastJetsonRisk = 0;
uint32_t lastJetsonRxMs = 0;

char jetsonLine[160];
size_t jetsonLineLen = 0;

void printMacAddress(const char *prefix, const uint8_t *mac) {
  Serial.print(prefix);
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

uint32_t macToNodeId(const uint8_t *mac) {
  return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
}

void setupVehicleId() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  vehicleId = macToNodeId(mac);
  printMacAddress("[VEHICLE] STA MAC Address: ", mac);
  Serial.printf("[VEHICLE] node_id=%lu\n", (unsigned long)vehicleId);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);

  setupVehicleId();

#if USE_ESPNOW_BROADCAST
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.println("[ESP-NOW] broadcast peer added");
  } else {
    Serial.println("[ESP-NOW] broadcast peer add failed");
  }
#endif
}

float metersToLatDeg(float meters) {
  return meters / 111320.0f;
}

float metersToLngDeg(float meters, float latDeg) {
  float latRad = latDeg * 0.017453292519943295f;
  float denom = 111320.0f * cosf(latRad);
  if (fabsf(denom) < 0.001f) return 0.0f;
  return meters / denom;
}

void buildSimulatedVehicleState(float &lat, float &lng, float &speedMps, float &headingDeg) {
  float t = millis() / 1000.0f;

  speedMps = 2.0f + 0.5f * sinf(t * 0.5f);
  headingDeg = 90.0f;

  // 동쪽으로 천천히 이동하는 가상 차량.
  float eastMeters = t * speedMps;
  lat = START_LAT;
  lng = START_LNG + metersToLngDeg(eastMeters, START_LAT);
}

void buildPacket() {
  memset(&packet, 0, sizeof(packet));

  packet.magic = V2X_MAGIC;
  packet.version = 1;
  packet.msg_type = 1;       // vehicle status
  packet.node_type = 0x10;   // vehicle
  packet.risk_level = 0;     // 차량은 risk 판단 안 함
  packet.node_id = vehicleId;

#if USE_SIMULATED_GPS
  packet.gps_valid = 1;
  float lat = 0.0f;
  float lng = 0.0f;
  float speedMps = 0.0f;
  float headingDeg = 0.0f;
  buildSimulatedVehicleState(lat, lng, speedMps, headingDeg);
  packet.latitude = lat;
  packet.longitude = lng;
  packet.speed_mps = speedMps;
  packet.heading_deg = headingDeg;
#else
  packet.gps_valid = 0;
  packet.latitude = 0.0f;
  packet.longitude = 0.0f;
  packet.speed_mps = 0.0f;
  packet.heading_deg = 0.0f;
#endif

  packet.timestamp_ms = millis();
  packet.seq_num = seq++;
}

void sendJsonToJetson(const char *espnowResult) {
  char line[240];
  snprintf(
    line,
    sizeof(line),
    "{\"type\":\"vehicle\",\"node_id\":%lu,\"seq\":%u,\"gps_valid\":%u,"
    "\"lat\":%.6f,\"lng\":%.6f,\"speed_mps\":%.3f,\"heading_deg\":%.2f,"
    "\"tx_ms\":%lu,\"espnow\":\"%s\"}",
    (unsigned long)packet.node_id,
    packet.seq_num,
    packet.gps_valid,
    packet.latitude,
    packet.longitude,
    packet.speed_mps,
    packet.heading_deg,
    (unsigned long)packet.timestamp_ms,
    espnowResult
  );

  Serial.println(line);
  jetsonSerial.println(line);
}

void sendVehicleState() {
  buildPacket();

  const char *espnowText = "off";

#if USE_ESPNOW_BROADCAST
  esp_err_t result = esp_now_send(broadcastMAC, (uint8_t *)&packet, sizeof(packet));
  espnowText = result == ESP_OK ? "ok" : "err";
#endif

  sendCount++;
  sendJsonToJetson(espnowText);
}

int extractRiskFromLine(const char *line) {
  const char *riskKey = strstr(line, "\"risk\"");
  if (!riskKey) riskKey = strstr(line, "risk");
  if (!riskKey) return -1;

  while (*riskKey && *riskKey != ':' && *riskKey != '=') riskKey++;
  if (!*riskKey) return -1;
  riskKey++;

  while (*riskKey == ' ' || *riskKey == '\"') riskKey++;
  if (*riskKey < '0' || *riskKey > '3') return -1;
  return *riskKey - '0';
}

void handleJetsonLine(const char *line) {
  int risk = extractRiskFromLine(line);
  Serial.print("[FROM JETSON] ");
  Serial.println(line);

  if (risk >= 0) {
    lastJetsonRisk = (uint8_t)risk;
    lastJetsonRxMs = millis();
    Serial.printf("[JETSON RISK] risk=%u\n", lastJetsonRisk);
  }
}

void readJetsonUart() {
  while (jetsonSerial.available()) {
    char c = (char)jetsonSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      jetsonLine[jetsonLineLen] = '\0';
      if (jetsonLineLen > 0) {
        handleJetsonLine(jetsonLine);
      }
      jetsonLineLen = 0;
      continue;
    }

    if (jetsonLineLen < sizeof(jetsonLine) - 1) {
      jetsonLine[jetsonLineLen++] = c;
    } else {
      jetsonLineLen = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== V2X Vehicle Jetson UART Probe ===");
  Serial.println("[WIRING] ESP32 GPIO33 TX1 -> Jetson RX");
  Serial.println("[WIRING] ESP32 GPIO32 RX1 <- Jetson TX");
  Serial.println("[WIRING] ESP32 GND <-> Jetson GND");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  jetsonSerial.begin(JETSON_BAUD, SERIAL_8N1, JETSON_RX, JETSON_TX);
  Serial.println("[JETSON UART] ready 115200");

  setupEspNow();
  Serial.println("[VEHICLE] System ready");
}

void loop() {
  readJetsonUart();

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    digitalWrite(LED_PIN, HIGH);
    sendVehicleState();
    digitalWrite(LED_PIN, LOW);
  }

  delay(5);
}
