// RSU multi-node bridge: forward vehicle/cane ESP-NOW status to Jetson and relay Jetson risk alerts.

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

#define LED_PIN 2
#define SERIAL_BAUD 115200
#define MAX_DEVICES 12
#define JETSON_LINE_MAX 220

#define V2X_MAGIC 0x56325831UL  // "V2X1"
#define V2X_VERSION 1

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

typedef struct device_slot {
  bool active;
  uint8_t mac[6];
  uint8_t node_type;
  uint32_t node_id;
  uint16_t last_seq;
  uint32_t recv_count;
  uint32_t lost_count;
  uint32_t last_rx_ms;
  uint8_t last_risk;
} device_slot_t;

device_slot_t devices[MAX_DEVICES];
v2x_status_message_t rxStatus;
v2x_status_message_t legacyReply;
v2x_risk_message_t riskAlert;

char jetsonLine[JETSON_LINE_MAX];
size_t jetsonLineLen = 0;
uint16_t riskSeq = 0;
uint32_t totalRecv = 0;

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
  Serial.print("[RSU] STA MAC Address: ");
  printMac(mac);
  Serial.println();
}

bool sameMac(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

const char *nodeTypeName(uint8_t nodeType) {
  if (nodeType == NODE_VEHICLE) return "vehicle";
  if (nodeType == NODE_CANE) return "cane";
  return "unknown";
}

uint8_t clampRisk(int risk) {
  if (risk < RISK_SAFE) return RISK_SAFE;
  if (risk > RISK_DANGER) return RISK_DANGER;
  return (uint8_t)risk;
}

void addPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    Serial.print("{\"type\":\"peer_added\",\"mac\":\"");
    printMac(mac);
    Serial.println("\"}");
  }
}

int findOrCreateDevice(const uint8_t *mac, uint8_t nodeType, uint32_t nodeId) {
  int freeIndex = -1;
  int oldestIndex = 0;
  uint32_t oldestMs = UINT32_MAX;

  for (int i = 0; i < MAX_DEVICES; i++) {
    if (devices[i].active && sameMac(devices[i].mac, mac)) return i;
    if (!devices[i].active && freeIndex < 0) freeIndex = i;
    if (devices[i].last_rx_ms < oldestMs) {
      oldestMs = devices[i].last_rx_ms;
      oldestIndex = i;
    }
  }

  int index = freeIndex >= 0 ? freeIndex : oldestIndex;
  memset(&devices[index], 0, sizeof(devices[index]));
  devices[index].active = true;
  memcpy(devices[index].mac, mac, 6);
  devices[index].node_type = nodeType;
  devices[index].node_id = nodeId;
  devices[index].last_risk = RISK_SAFE;
  return index;
}

int findDeviceByNodeId(uint32_t nodeId) {
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (devices[i].active && devices[i].node_id == nodeId) return i;
  }
  return -1;
}

void sendLegacyReply(int index, uint8_t risk) {
  memset(&legacyReply, 0, sizeof(legacyReply));
  legacyReply.magic = V2X_MAGIC;
  legacyReply.version = V2X_VERSION;
  legacyReply.msg_type = MSG_RSU_REPLY;
  legacyReply.node_type = NODE_RSU;
  legacyReply.risk_level = risk;
  legacyReply.node_id = devices[index].node_id;
  legacyReply.timestamp_ms = millis();
  legacyReply.seq_num = devices[index].last_seq;
  esp_err_t result = esp_now_send(devices[index].mac, (uint8_t *)&legacyReply, sizeof(legacyReply));
  Serial.printf("{\"type\":\"risk_tx\",\"mode\":\"legacy\",\"target_id\":%lu,\"risk\":%u,\"espnow\":\"%s\"}\n",
                (unsigned long)devices[index].node_id, risk, result == ESP_OK ? "ok" : "err");
}

void sendRiskAlert(int index, uint8_t risk, uint32_t srcId) {
  memset(&riskAlert, 0, sizeof(riskAlert));
  riskAlert.magic = V2X_MAGIC;
  riskAlert.version = V2X_VERSION;
  riskAlert.msg_type = MSG_RISK_ALERT;
  riskAlert.node_type = NODE_RSU;
  riskAlert.risk_level = risk;
  riskAlert.target_id = devices[index].node_id;
  riskAlert.src_id = srcId;
  riskAlert.timestamp_ms = millis();
  riskAlert.seq_num = riskSeq++;
  esp_err_t result = esp_now_send(devices[index].mac, (uint8_t *)&riskAlert, sizeof(riskAlert));
  Serial.printf("{\"type\":\"risk_tx\",\"mode\":\"alert\",\"target_id\":%lu,\"src_id\":%lu,\"risk\":%u,\"espnow\":\"%s\"}\n",
                (unsigned long)riskAlert.target_id, (unsigned long)riskAlert.src_id, risk, result == ESP_OK ? "ok" : "err");
}

void sendRiskToDevice(int index, uint8_t risk, uint32_t srcId) {
  if (index < 0 || index >= MAX_DEVICES || !devices[index].active) return;
  devices[index].last_risk = risk;
  sendRiskAlert(index, risk, srcId);
}

void printStatusJson(const v2x_status_message_t &m, const device_slot_t &slot, const uint8_t *srcMac, int rssi) {
  Serial.printf("{\"type\":\"%s\",\"node_id\":%lu,\"seq\":%u,\"gps_valid\":%u,",
                nodeTypeName(m.node_type), (unsigned long)m.node_id, m.seq_num, m.gps_valid);
  Serial.printf("\"lat\":%.6f,\"lng\":%.6f,\"speed_mps\":%.3f,\"heading_deg\":%.2f,",
                m.latitude, m.longitude, m.speed_mps, m.heading_deg);
  Serial.printf("\"node_risk\":%u,\"tx_ms\":%lu,\"rx_ms\":%lu,",
                m.risk_level, (unsigned long)m.timestamp_ms, (unsigned long)millis());
  Serial.printf("\"recv_count\":%lu,\"lost_count\":%lu,\"rssi\":%d,\"src_mac\":\"",
                (unsigned long)slot.recv_count, (unsigned long)slot.lost_count, rssi);
  printMac(srcMac);
  Serial.println("\"}");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(v2x_status_message_t)) {
    Serial.printf("{\"type\":\"drop\",\"reason\":\"size\",\"len\":%d,\"expected\":%d}\n", len, sizeof(v2x_status_message_t));
    return;
  }

  memcpy(&rxStatus, data, sizeof(rxStatus));

  bool isStatus =
    rxStatus.magic == V2X_MAGIC &&
    rxStatus.version == V2X_VERSION &&
    (rxStatus.msg_type == MSG_VEHICLE_STATUS || rxStatus.msg_type == MSG_CANE_STATUS) &&
    (rxStatus.node_type == NODE_VEHICLE || rxStatus.node_type == NODE_CANE);

  if (!isStatus) {
    Serial.println("{\"type\":\"drop\",\"reason\":\"header\"}");
    return;
  }

  const uint8_t *srcMac = info->src_addr;
  int rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  addPeerIfNeeded(srcMac);

  int index = findOrCreateDevice(srcMac, rxStatus.node_type, rxStatus.node_id);
  device_slot_t &slot = devices[index];

  if (slot.recv_count > 0) {
    uint16_t expected = slot.last_seq + 1;
    if (rxStatus.seq_num != expected) {
      slot.lost_count += (uint16_t)(rxStatus.seq_num - expected);
    }
  }

  slot.node_type = rxStatus.node_type;
  slot.node_id = rxStatus.node_id;
  slot.recv_count++;
  slot.last_seq = rxStatus.seq_num;
  slot.last_rx_ms = millis();
  totalRecv++;

  digitalWrite(LED_PIN, HIGH);
  printStatusJson(rxStatus, slot, srcMac, rssi);
  digitalWrite(LED_PIN, LOW);
}

bool extractUint32AfterKey(const char *line, const char *key, uint32_t &value) {
  const char *p = strstr(line, key);
  if (!p) return false;
  while (*p && *p != ':' && *p != '=') p++;
  if (!*p) return false;
  p++;
  while (*p == ' ' || *p == '\"') p++;
  if (*p < '0' || *p > '9') return false;
  value = 0;
  while (*p >= '0' && *p <= '9') {
    uint8_t digit = *p - '0';
    if (value > (UINT32_MAX - digit) / 10UL) return false;
    value = value * 10UL + digit;
    p++;
  }
  return true;
}

void handleJetsonLine(const char *line) {
  uint32_t riskRaw = 0;
  if (!extractUint32AfterKey(line, "risk", riskRaw)) {
    Serial.print("{\"type\":\"jetson_ignore\",\"line\":\"");
    Serial.print(line);
    Serial.println("\"}");
    return;
  }

  uint8_t risk = clampRisk((int)riskRaw);
  uint32_t targetId = 0;
  uint32_t srcId = 0;
  bool hasTarget = extractUint32AfterKey(line, "target_id", targetId);
  extractUint32AfterKey(line, "src_id", srcId);

  if (hasTarget) {
    if (targetId == 0 || targetId == 0xFFFFFFFFUL) {
      for (int i = 0; i < MAX_DEVICES; i++) {
        if (devices[i].active) sendRiskToDevice(i, risk, srcId);
      }
      Serial.printf("{\"type\":\"risk_broadcast_to_seen\",\"target_id\":%lu,\"risk\":%u}\n",
                    (unsigned long)targetId, risk);
      return;
    }

    int index = findDeviceByNodeId(targetId);
    if (index >= 0) {
      sendRiskToDevice(index, risk, srcId);
    } else {
      Serial.printf("{\"type\":\"risk_drop\",\"reason\":\"target_not_seen\",\"target_id\":%lu,\"risk\":%u}\n",
                    (unsigned long)targetId, risk);
    }
    return;
  }

  for (int i = 0; i < MAX_DEVICES; i++) {
    if (devices[i].active) sendRiskToDevice(i, risk, srcId);
  }
  Serial.printf("{\"type\":\"risk_broadcast_to_seen\",\"risk\":%u}\n", risk);
}

void readJetsonSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      jetsonLine[jetsonLineLen] = '\0';
      if (jetsonLineLen > 0) handleJetsonLine(jetsonLine);
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
  Serial.println("{\"type\":\"bridge_ready\",\"mode\":\"multi_node\"}");
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println("=== V2X RSU Multi ESP-NOW Jetson Bridge ===");
  Serial.println("{\"type\":\"boot\",\"role\":\"rsu_multi_bridge\",\"baud\":115200}");
  setupEspNow();
}

void loop() {
  readJetsonSerial();
  if (totalRecv == 0) digitalWrite(LED_PIN, (millis() / 500) % 2);
  delay(5);
}
