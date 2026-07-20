// V2X Vehicle Node
// GPS status broadcast only. No risk decision is made on the vehicle side.

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <TinyGPSPlus.h>

// =====================
// Feature switches
// =====================
#define USE_GPS 1
#define SEND_BROADCAST 1
#define USE_DEMO_GPS_FALLBACK 1

// =====================
// Pins
// =====================
#define LED_PIN 2

#define GPS_RX 16   // ESP32 RX2 <- GPS TX
#define GPS_TX 17   // ESP32 TX2 -> GPS RX
#define GPS_BAUD 9600

// =====================
// V2X constants
// =====================
#define V2X_MAGIC   0x56325831UL  // "V2X1"
#define V2X_VERSION 1

#define MSG_VEHICLE_STATUS 1
#define MSG_RSU_REPLY      2
#define MSG_CANE_STATUS    3
#define MSG_RISK_ALERT     4

#define NODE_VEHICLE 0x10
#define NODE_CANE    0x20
#define NODE_RSU     0x30

#define RISK_SAFE    0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER  3
#define SEND_INTERVAL_MS 100

#define DEMO_VEHICLE_LAT 37.000000
#define DEMO_VEHICLE_LNG 127.000150

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
TinyGPSPlus gps;

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t caneMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Optional: replace with cane MAC for directed send.

v2x_status_message_t txPacket;
v2x_status_message_t rxReply;
v2x_risk_message_t rxRisk;

uint32_t vehicleId = 0;
uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t ackCount = 0;
uint32_t lastSendMs = 0;
uint32_t lastAckMs = 0;
uint8_t lastRiskLevel = RISK_SAFE;

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

void addPeer(const uint8_t *mac, const char *name) {
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.printf("[ESP-NOW] %s peer added\n", name);
  } else {
    Serial.printf("[ESP-NOW] %s peer add failed\n", name);
  }
}

void updateGps() {
#if USE_GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
#endif
}

void buildPacket() {
  memset(&txPacket, 0, sizeof(txPacket));
  txPacket.magic = V2X_MAGIC;
  txPacket.version = V2X_VERSION;
  txPacket.msg_type = MSG_VEHICLE_STATUS;
  txPacket.node_type = NODE_VEHICLE;
  txPacket.risk_level = RISK_SAFE;
  txPacket.node_id = vehicleId;

#if USE_GPS
  bool gpsOk = gps.location.isValid() && gps.location.age() < 3000;
  txPacket.gps_valid = gpsOk || USE_DEMO_GPS_FALLBACK ? 1 : 0;
  txPacket.latitude = gpsOk ? gps.location.lat() : (USE_DEMO_GPS_FALLBACK ? DEMO_VEHICLE_LAT : 0.0);
  txPacket.longitude = gpsOk ? gps.location.lng() : (USE_DEMO_GPS_FALLBACK ? DEMO_VEHICLE_LNG : 0.0);
  txPacket.speed_mps = gps.speed.isValid() ? gps.speed.mps() : 0.0;
  txPacket.heading_deg = gps.course.isValid() ? gps.course.deg() : 0.0;
#else
  txPacket.gps_valid = USE_DEMO_GPS_FALLBACK ? 1 : 0;
  txPacket.latitude = USE_DEMO_GPS_FALLBACK ? DEMO_VEHICLE_LAT : 0.0;
  txPacket.longitude = USE_DEMO_GPS_FALLBACK ? DEMO_VEHICLE_LNG : 0.0;
  txPacket.speed_mps = 0.0;
  txPacket.heading_deg = 0.0;
#endif

  txPacket.timestamp_ms = millis();
  txPacket.seq_num = seq++;
}

void sendPacket() {
  buildPacket();

#if SEND_BROADCAST
  esp_err_t result = esp_now_send(broadcastMAC, (uint8_t *)&txPacket, sizeof(txPacket));
#else
  esp_err_t result = esp_now_send(caneMAC, (uint8_t *)&txPacket, sizeof(txPacket));
#endif

  sendCount++;
  Serial.printf(
    "[SEND] id=%lu seq=%u gps=%u lat=%.6f lng=%.6f spd=%.2f heading=%.2f send=%lu result=%s\n",
    (unsigned long)txPacket.node_id,
    txPacket.seq_num,
    txPacket.gps_valid,
    txPacket.latitude,
    txPacket.longitude,
    txPacket.speed_mps,
    txPacket.heading_deg,
    (unsigned long)sendCount,
    result == ESP_OK ? "OK" : "ERR"
  );
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(v2x_risk_message_t)) {
    memcpy(&rxRisk, data, sizeof(rxRisk));
    bool validRisk =
      rxRisk.magic == V2X_MAGIC &&
      rxRisk.version == V2X_VERSION &&
      rxRisk.msg_type == MSG_RISK_ALERT &&
      (rxRisk.target_id == 0 || rxRisk.target_id == vehicleId || rxRisk.target_id == 0xFFFFFFFFUL);

    if (!validRisk) {
      Serial.println("[RX RISK] invalid or not for this vehicle");
      return;
    }

    ackCount++;
    lastAckMs = millis();
    lastRiskLevel = rxRisk.risk_level;
    Serial.printf(
      "[RX RISK] risk=%u target=%lu src=%lu seq=%u ack=%lu rx_ms=%lu\n",
      rxRisk.risk_level,
      (unsigned long)rxRisk.target_id,
      (unsigned long)rxRisk.src_id,
      rxRisk.seq_num,
      (unsigned long)ackCount,
      (unsigned long)lastAckMs
    );
    return;
  }

  if (len == sizeof(v2x_status_message_t)) {
    memcpy(&rxReply, data, sizeof(rxReply));
    if (rxReply.magic != V2X_MAGIC || rxReply.version != V2X_VERSION || rxReply.msg_type != MSG_RSU_REPLY) {
      Serial.println("[RX BACK] invalid status reply header");
      return;
    }

    ackCount++;
    lastAckMs = millis();
    lastRiskLevel = rxReply.risk_level;
    Serial.printf(
      "[RX BACK] seq=%u risk=%u ack=%lu rx_ms=%lu\n",
      rxReply.seq_num,
      rxReply.risk_level,
      (unsigned long)ackCount,
      (unsigned long)lastAckMs
    );
    return;
  }

  Serial.printf("[RX BACK] size mismatch len=%d expected_status=%d expected_risk=%d\n",
                len, sizeof(v2x_status_message_t), sizeof(v2x_risk_message_t));
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);

  Serial.println("[ESP-NOW] Vehicle start");
  setupVehicleId();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed, restart");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  addPeer(broadcastMAC, "broadcast");

#if !SEND_BROADCAST
  addPeer(caneMAC, "cane");
#endif

  Serial.println("[ESP-NOW] Vehicle ready");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== V2X Vehicle GPS Broadcast ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if USE_GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] ready");
  Serial.println("[GPS] GPS TX -> ESP32 GPIO16 RX2");
  Serial.println("[GPS] GPS RX -> ESP32 GPIO17 TX2");
#endif

  setupEspNow();
  Serial.println("[VEHICLE] System ready");
}

void loop() {
  updateGps();

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();
    digitalWrite(LED_PIN, HIGH);
    sendPacket();
    digitalWrite(LED_PIN, LOW);
  }

  if (ackCount == 0 || millis() - lastAckMs > 1000) {
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  }

  delay(5);
}
