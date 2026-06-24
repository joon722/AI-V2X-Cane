// V2X RSU Bridge ESP32
// ESP-NOW wireless receiver + USB Serial bridge for Jetson.
//
// Role:
// 1. Receive vehicle packets over ESP-NOW.
// 2. Print one JSON line to USB Serial so Jetson can read it.
// 3. Read Jetson risk JSON from USB Serial.
// 4. Send a compact ESP-NOW reply back to the vehicle.

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// =====================
// Pins / settings
// =====================
#define LED_PIN 2
#define SERIAL_BAUD 115200

#define MAX_VEHICLES 8
#define JETSON_LINE_MAX 180

// =====================
// V2X packet constants
// Keep these aligned with vehicle sketches 06/07.
// =====================
#define V2X_MAGIC   0x56325831UL  // "V2X1"
#define V2X_VERSION 1

#define MSG_VEHICLE_STATUS 1
#define MSG_RSU_REPLY      2

#define NODE_VEHICLE 0x10
#define NODE_RSU     0x30

#define RISK_SAFE    0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER  3

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

typedef struct vehicle_slot {
  bool active;
  uint8_t mac[6];
  uint32_t node_id;
  uint16_t last_seq;
  uint32_t recv_count;
  uint32_t lost_count;
  uint32_t last_rx_ms;
  uint8_t last_risk;
} vehicle_slot_t;

vehicle_slot_t vehicles[MAX_VEHICLES];
v2x_message_t rxPacket;
v2x_message_t replyPacket;

char jetsonLine[JETSON_LINE_MAX];
size_t jetsonLineLen = 0;

uint8_t defaultRisk = RISK_SAFE;
uint32_t bridgeRecvCount = 0;

void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
}

void printOwnMac() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.print("[BRIDGE] STA MAC Address: ");
  printMac(mac);
  Serial.println();
}

bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

void addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.print("[BRIDGE] peer added ");
    printMac(mac);
    Serial.println();
  } else {
    Serial.print("[BRIDGE] peer add failed ");
    printMac(mac);
    Serial.println();
  }
}

int findOrCreateVehicle(const uint8_t *mac, uint32_t nodeId) {
  int freeIndex = -1;
  int oldestIndex = 0;
  uint32_t oldestMs = UINT32_MAX;

  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (vehicles[i].active && sameMac(vehicles[i].mac, mac)) {
      return i;
    }

    if (!vehicles[i].active && freeIndex < 0) {
      freeIndex = i;
    }

    if (vehicles[i].last_rx_ms < oldestMs) {
      oldestMs = vehicles[i].last_rx_ms;
      oldestIndex = i;
    }
  }

  int index = freeIndex >= 0 ? freeIndex : oldestIndex;
  memset(&vehicles[index], 0, sizeof(vehicles[index]));
  vehicles[index].active = true;
  memcpy(vehicles[index].mac, mac, 6);
  vehicles[index].node_id = nodeId;
  vehicles[index].last_risk = defaultRisk;
  return index;
}

int findVehicleByNodeId(uint32_t nodeId) {
  for (int i = 0; i < MAX_VEHICLES; i++) {
    if (vehicles[i].active && vehicles[i].node_id == nodeId) {
      return i;
    }
  }
  return -1;
}

uint8_t clampRisk(int risk) {
  if (risk < RISK_SAFE) return RISK_SAFE;
  if (risk > RISK_DANGER) return RISK_DANGER;
  return (uint8_t)risk;
}

void sendReplyToVehicle(int index, uint16_t seqForAck) {
  if (index < 0 || index >= MAX_VEHICLES || !vehicles[index].active) return;

  memset(&replyPacket, 0, sizeof(replyPacket));
  replyPacket.magic = V2X_MAGIC;
  replyPacket.version = V2X_VERSION;
  replyPacket.msg_type = MSG_RSU_REPLY;
  replyPacket.node_type = NODE_RSU;
  replyPacket.risk_level = vehicles[index].last_risk;
  replyPacket.gps_valid = 0;
  replyPacket.node_id = vehicles[index].node_id;
  replyPacket.timestamp_ms = millis();
  replyPacket.seq_num = seqForAck;

  esp_err_t result = esp_now_send(vehicles[index].mac, (uint8_t *)&replyPacket, sizeof(replyPacket));

  Serial.printf(
    "{\"type\":\"rsu_reply\",\"node_id\":%lu,\"seq\":%u,\"risk\":%u,\"espnow\":\"%s\"}\n",
    (unsigned long)replyPacket.node_id,
    replyPacket.seq_num,
    replyPacket.risk_level,
    result == ESP_OK ? "ok" : "err"
  );
}

void printVehicleJson(const v2x_message_t &m, const vehicle_slot_t &slot, const uint8_t *srcMac, int rssi) {
  Serial.print("{\"type\":\"vehicle\"");
  Serial.printf(",\"node_id\":%lu", (unsigned long)m.node_id);
  Serial.printf(",\"seq\":%u", m.seq_num);
  Serial.printf(",\"gps_valid\":%u", m.gps_valid);
  Serial.printf(",\"lat\":%.6f", m.latitude);
  Serial.printf(",\"lng\":%.6f", m.longitude);
  Serial.printf(",\"speed_mps\":%.3f", m.speed_mps);
  Serial.printf(",\"heading_deg\":%.2f", m.heading_deg);
  Serial.printf(",\"tx_ms\":%lu", (unsigned long)m.timestamp_ms);
  Serial.printf(",\"rx_ms\":%lu", (unsigned long)millis());
  Serial.printf(",\"recv_count\":%lu", (unsigned long)slot.recv_count);
  Serial.printf(",\"lost_count\":%lu", (unsigned long)slot.lost_count);
  Serial.printf(",\"rssi\":%d", rssi);
  Serial.print(",\"src_mac\":\"");
  printMac(srcMac);
  Serial.println("\"}");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(v2x_message_t)) {
    Serial.printf("{\"type\":\"drop\",\"reason\":\"size\",\"len\":%d,\"expected\":%d}\n", len, sizeof(v2x_message_t));
    return;
  }

  memcpy(&rxPacket, data, sizeof(rxPacket));

  if (rxPacket.magic != V2X_MAGIC ||
      rxPacket.version != V2X_VERSION ||
      rxPacket.msg_type != MSG_VEHICLE_STATUS ||
      rxPacket.node_type != NODE_VEHICLE) {
    Serial.println("{\"type\":\"drop\",\"reason\":\"header\"}");
    return;
  }

  const uint8_t *srcMac = info->src_addr;
  int rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;

  addPeerIfNeeded(srcMac);
  int index = findOrCreateVehicle(srcMac, rxPacket.node_id);
  vehicle_slot_t &slot = vehicles[index];

  if (slot.recv_count > 0) {
    uint16_t expected = slot.last_seq + 1;
    if (rxPacket.seq_num != expected) {
      slot.lost_count += (uint16_t)(rxPacket.seq_num - expected);
    }
  }

  slot.recv_count++;
  slot.last_seq = rxPacket.seq_num;
  slot.last_rx_ms = millis();
  bridgeRecvCount++;

  digitalWrite(LED_PIN, HIGH);
  printVehicleJson(rxPacket, slot, srcMac, rssi);
  sendReplyToVehicle(index, rxPacket.seq_num);
  digitalWrite(LED_PIN, LOW);
}

int extractIntAfterKey(const char *line, const char *key, int fallback) {
  const char *p = strstr(line, key);
  if (!p) return fallback;

  while (*p && *p != ':' && *p != '=') p++;
  if (!*p) return fallback;
  p++;

  while (*p == ' ' || *p == '\"') p++;
  bool neg = false;
  if (*p == '-') {
    neg = true;
    p++;
  }

  if (*p < '0' || *p > '9') return fallback;

  int value = 0;
  while (*p >= '0' && *p <= '9') {
    value = value * 10 + (*p - '0');
    p++;
  }
  return neg ? -value : value;
}

void handleJetsonLine(const char *line) {
  int risk = extractIntAfterKey(line, "risk", -1);
  if (risk < 0) {
    Serial.print("{\"type\":\"jetson_ignore\",\"line\":\"");
    Serial.print(line);
    Serial.println("\"}");
    return;
  }

  int nodeIdInt = extractIntAfterKey(line, "node_id", -1);
  uint8_t newRisk = clampRisk(risk);

  if (nodeIdInt >= 0) {
    int index = findVehicleByNodeId((uint32_t)nodeIdInt);
    if (index >= 0) {
      vehicles[index].last_risk = newRisk;
      sendReplyToVehicle(index, vehicles[index].last_seq);
    }
  } else {
    defaultRisk = newRisk;
    for (int i = 0; i < MAX_VEHICLES; i++) {
      if (vehicles[i].active) {
        vehicles[i].last_risk = newRisk;
        sendReplyToVehicle(i, vehicles[i].last_seq);
      }
    }
  }

  Serial.printf("{\"type\":\"jetson_risk\",\"node_id\":%d,\"risk\":%u}\n", nodeIdInt, newRisk);
}

void readJetsonSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
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

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);

  printOwnMac();

  if (esp_now_init() != ESP_OK) {
    Serial.println("{\"type\":\"error\",\"where\":\"esp_now_init\"}");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("{\"type\":\"bridge_ready\",\"radio\":\"esp-now\"}");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println("=== V2X RSU ESP-NOW Jetson Bridge ===");
  Serial.println("{\"type\":\"boot\",\"role\":\"rsu_bridge\",\"baud\":115200}");

  setupEspNow();
}

void loop() {
  readJetsonSerial();

  if (bridgeRecvCount == 0) {
    digitalWrite(LED_PIN, (millis() / 500) % 2);
  }

  delay(5);
}
