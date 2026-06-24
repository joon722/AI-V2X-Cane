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
#define MSG_CANE_REPLY     2

#define NODE_VEHICLE 0x10
#define NODE_CANE    0x20

#define RISK_SAFE    0
#define SEND_INTERVAL_MS 100

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

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t caneMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Optional: replace with cane MAC for directed send.

v2x_message_t txPacket;
v2x_message_t rxReply;

uint32_t vehicleId = 0;
uint16_t seq = 0;
uint32_t sendCount = 0;
uint32_t ackCount = 0;
uint32_t lastSendMs = 0;
uint32_t lastAckMs = 0;

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
  if (len != sizeof(v2x_message_t)) {
    Serial.printf("[RX BACK] size mismatch len=%d expected=%d\n", len, sizeof(v2x_message_t));
    return;
  }

  memcpy(&rxReply, data, sizeof(rxReply));
  if (rxReply.magic != V2X_MAGIC || rxReply.version != V2X_VERSION || rxReply.msg_type != MSG_CANE_REPLY) {
    Serial.println("[RX BACK] invalid packet header");
    return;
  }

  ackCount++;
  lastAckMs = millis();
  Serial.printf(
    "[RX BACK] seq=%u cane_risk=%u ack=%lu rx_ms=%lu\n",
    rxReply.seq_num,
    rxReply.risk_level,
    (unsigned long)ackCount,
    (unsigned long)lastAckMs
  );
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
