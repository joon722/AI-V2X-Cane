// V2X Cane Node
// ESP-NOW receive -> coarse distance alert -> Jetson UART bridge -> actuator/DFPlayer output

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <TinyGPSPlus.h>
#include <math.h>

// =====================
// Feature switches
// =====================
#define USE_ACTUATOR      1
#define USE_DFPLAYER      1
#define USE_JETSON_UART   1
#define USE_CANE_GPS      0
#define USE_FIXED_CANE_POS 1

// =====================
// Pins
// =====================
#define LED_PIN 2

#define BUZZER_PIN 25
#define MOTOR_PIN  26

#define JETSON_RX 16   // ESP32 RX2 <- Jetson TX
#define JETSON_TX 17   // ESP32 TX2 -> Jetson RX
#define JETSON_BAUD 115200

#define MP3_RX 32      // ESP32 RX1 <- DFPlayer TX
#define MP3_TX 33      // ESP32 TX1 -> DFPlayer RX
#define MP3_BAUD 9600

#define GPS_RX 34      // Optional cane GPS RX. Change to your wiring if used.
#define GPS_TX 35      // Optional cane GPS TX. GPIO35 is input-only, so change if GPS RX is needed.
#define GPS_BAUD 9600

// =====================
// Output polarity
// =====================
#define BUZZER_ON  LOW
#define BUZZER_OFF HIGH
#define MOTOR_ON   HIGH
#define MOTOR_OFF  LOW

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
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER  3

// Coarse fail-safe thresholds aligned with the main cane risk logic.
#define DIST_CAUTION_M 10.0f
#define DIST_WARNING_M  6.0f
#define DIST_DANGER_M   3.0f

#define VEHICLE_TIMEOUT_MS 1200
#define JETSON_RISK_TIMEOUT_MS 1200
#define BEEP_DURATION_MS 40
#define MP3_MIN_INTERVAL_MS 1500
#define MAX_TRACKED_VEHICLES 6

// If USE_CANE_GPS is 0, set the demo cane location here.
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000

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

typedef struct vehicle_track {
  bool active;
  uint8_t mac[6];
  uint32_t node_id;
  uint16_t last_seq;
  uint32_t recv_count;
  uint32_t lost_count;
  uint32_t last_rx_ms;
  float distance_m;
  uint8_t coarse_risk;
} vehicle_track_t;

HardwareSerial jetsonSerial(2);
HardwareSerial mp3Serial(1);
HardwareSerial gpsSerial(0);
TinyGPSPlus caneGps;

vehicle_track_t tracks[MAX_TRACKED_VEHICLES];
v2x_message_t rxPacket;
v2x_message_t replyPacket;

uint8_t currentRisk = 255;
uint8_t jetsonRisk = RISK_SAFE;
uint32_t lastJetsonRiskMs = 0;

bool beepActive = false;
uint32_t beepStartMs = 0;
uint32_t lastMp3PlayMs = 0;

char jetsonLine[160];
size_t jetsonLineLen = 0;

void forceOutputsOff() {
#if USE_ACTUATOR
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
  digitalWrite(MOTOR_PIN, MOTOR_OFF);
#endif
}

void printMacAddress(const char *prefix, const uint8_t *mac) {
  Serial.print(prefix);
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

void printCaneMac() {
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  printMacAddress("[CANE] STA MAC Address: ", mac);
}

uint32_t macToNodeId(const uint8_t *mac) {
  return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
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
    printMacAddress("[ESP-NOW] peer added: ", mac);
  } else {
    printMacAddress("[ESP-NOW] peer add failed: ", mac);
  }
}

int findOrCreateTrack(const uint8_t *mac, uint32_t nodeId) {
  int freeIndex = -1;

  for (int i = 0; i < MAX_TRACKED_VEHICLES; i++) {
    if (tracks[i].active && sameMac(tracks[i].mac, mac)) {
      return i;
    }
    if (!tracks[i].active && freeIndex < 0) {
      freeIndex = i;
    }
  }

  if (freeIndex < 0) {
    uint32_t oldestMs = UINT32_MAX;
    for (int i = 0; i < MAX_TRACKED_VEHICLES; i++) {
      if (tracks[i].last_rx_ms < oldestMs) {
        oldestMs = tracks[i].last_rx_ms;
        freeIndex = i;
      }
    }
  }

  vehicle_track_t &t = tracks[freeIndex];
  memset(&t, 0, sizeof(t));
  t.active = true;
  memcpy(t.mac, mac, 6);
  t.node_id = nodeId;
  t.distance_m = NAN;
  t.coarse_risk = RISK_SAFE;
  return freeIndex;
}

float degToRad(float deg) {
  return deg * 0.017453292519943295f;
}

float haversineMeters(float lat1, float lon1, float lat2, float lon2) {
  const float earthRadiusM = 6371000.0f;
  float dLat = degToRad(lat2 - lat1);
  float dLon = degToRad(lon2 - lon1);
  float rLat1 = degToRad(lat1);
  float rLat2 = degToRad(lat2);

  float a = sinf(dLat / 2.0f) * sinf(dLat / 2.0f) +
            cosf(rLat1) * cosf(rLat2) * sinf(dLon / 2.0f) * sinf(dLon / 2.0f);
  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return earthRadiusM * c;
}

bool getCanePosition(float &lat, float &lng) {
#if USE_CANE_GPS
  bool gpsOk = caneGps.location.isValid() && caneGps.location.age() < 3000;
  if (gpsOk) {
    lat = caneGps.location.lat();
    lng = caneGps.location.lng();
    return true;
  }
#endif

#if USE_FIXED_CANE_POS
  lat = CANE_FIXED_LAT;
  lng = CANE_FIXED_LNG;
  return true;
#else
  return false;
#endif
}

uint8_t riskFromDistance(float distanceM) {
  if (isnan(distanceM)) return RISK_SAFE;
  if (distanceM < DIST_DANGER_M) return RISK_DANGER;
  if (distanceM < DIST_WARNING_M) return RISK_WARNING;
  if (distanceM < DIST_CAUTION_M) return RISK_CAUTION;
  return RISK_SAFE;
}

uint8_t maxRisk(uint8_t a, uint8_t b) {
  return a > b ? a : b;
}

uint8_t getCoarseSystemRisk() {
  uint8_t risk = RISK_SAFE;
  uint32_t now = millis();

  for (int i = 0; i < MAX_TRACKED_VEHICLES; i++) {
    if (!tracks[i].active) continue;
    if (now - tracks[i].last_rx_ms > VEHICLE_TIMEOUT_MS) continue;
    risk = maxRisk(risk, tracks[i].coarse_risk);
  }

  return risk;
}

uint8_t getOutputRisk() {
  uint8_t coarse = getCoarseSystemRisk();
  bool jetsonFresh = (millis() - lastJetsonRiskMs) <= JETSON_RISK_TIMEOUT_MS;

  if (jetsonFresh) {
    return jetsonRisk;
  }

  return coarse;
}

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

void startBeep() {
#if USE_ACTUATOR
  digitalWrite(BUZZER_PIN, BUZZER_ON);
  beepActive = true;
  beepStartMs = millis();
#endif
}

void updateBeep() {
#if USE_ACTUATOR
  if (beepActive && millis() - beepStartMs >= BEEP_DURATION_MS) {
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  }
#endif
}

void applyRiskOutput(uint8_t risk) {
  if (risk == currentRisk) return;

  Serial.printf("[OUT] risk %u -> %u\n", currentRisk, risk);

#if USE_ACTUATOR
  if (risk == RISK_SAFE) {
    digitalWrite(MOTOR_PIN, MOTOR_OFF);
    digitalWrite(BUZZER_PIN, BUZZER_OFF);
    beepActive = false;
  } else {
    digitalWrite(MOTOR_PIN, MOTOR_ON);
    startBeep();
  }
#endif

#if USE_DFPLAYER
  if (risk > RISK_SAFE && millis() - lastMp3PlayMs > MP3_MIN_INTERVAL_MS) {
    dfPlayTrack(risk);
    lastMp3PlayMs = millis();
  }
#endif

  currentRisk = risk;
}

void sendJsonToJetson(const v2x_message_t &m, const vehicle_track_t &t) {
#if USE_JETSON_UART
  jetsonSerial.printf(
    "{\"type\":\"vehicle\",\"node_id\":%lu,\"seq\":%u,\"gps_valid\":%u,"
    "\"lat\":%.6f,\"lng\":%.6f,\"speed\":%.3f,\"heading\":%.2f,"
    "\"distance_m\":%.2f,\"coarse_risk\":%u,\"lost\":%lu,"
    "\"tx_ms\":%lu,\"rx_ms\":%lu}\n",
    (unsigned long)m.node_id,
    m.seq_num,
    m.gps_valid,
    m.latitude,
    m.longitude,
    m.speed_mps,
    m.heading_deg,
    t.distance_m,
    t.coarse_risk,
    (unsigned long)t.lost_count,
    (unsigned long)m.timestamp_ms,
    (unsigned long)millis()
  );
#endif
}

void sendReplyToVehicle(const uint8_t *mac, const v2x_message_t &m) {
  memset(&replyPacket, 0, sizeof(replyPacket));
  replyPacket.magic = V2X_MAGIC;
  replyPacket.version = V2X_VERSION;
  replyPacket.msg_type = MSG_CANE_REPLY;
  replyPacket.node_type = NODE_CANE;
  replyPacket.risk_level = getOutputRisk();
  replyPacket.gps_valid = 0;
  replyPacket.node_id = macToNodeId(mac);
  replyPacket.timestamp_ms = millis();
  replyPacket.seq_num = m.seq_num;

  esp_err_t result = esp_now_send(mac, (uint8_t *)&replyPacket, sizeof(replyPacket));
  Serial.printf("[TX BACK] seq=%u risk=%u result=%s\n", replyPacket.seq_num, replyPacket.risk_level, result == ESP_OK ? "OK" : "ERR");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(v2x_message_t)) {
    Serial.printf("[RX] size mismatch len=%d expected=%d\n", len, sizeof(v2x_message_t));
    return;
  }

  memcpy(&rxPacket, data, sizeof(rxPacket));

  if (rxPacket.magic != V2X_MAGIC || rxPacket.version != V2X_VERSION || rxPacket.msg_type != MSG_VEHICLE_STATUS) {
    Serial.println("[RX] invalid packet header");
    return;
  }

  const uint8_t *srcMac = info->src_addr;
  addPeerIfNeeded(srcMac);

  int idx = findOrCreateTrack(srcMac, rxPacket.node_id);
  vehicle_track_t &t = tracks[idx];

  if (t.recv_count > 0) {
    uint16_t expected = t.last_seq + 1;
    if (rxPacket.seq_num != expected) {
      t.lost_count += (uint16_t)(rxPacket.seq_num - expected);
    }
  }

  t.recv_count++;
  t.last_seq = rxPacket.seq_num;
  t.last_rx_ms = millis();

  float caneLat = 0.0f;
  float caneLng = 0.0f;
  bool caneGpsOk = getCanePosition(caneLat, caneLng);

  if (rxPacket.gps_valid && caneGpsOk) {
    t.distance_m = haversineMeters(caneLat, caneLng, rxPacket.latitude, rxPacket.longitude);
  } else {
    t.distance_m = NAN;
  }

  t.coarse_risk = riskFromDistance(t.distance_m);

  Serial.printf(
    "[RX] id=%lu seq=%u gps=%u dist=%.2f coarse=%u recv=%lu lost=%lu\n",
    (unsigned long)t.node_id,
    rxPacket.seq_num,
    rxPacket.gps_valid,
    t.distance_m,
    t.coarse_risk,
    (unsigned long)t.recv_count,
    (unsigned long)t.lost_count
  );

  sendJsonToJetson(rxPacket, t);
  sendReplyToVehicle(srcMac, rxPacket);
  applyRiskOutput(getOutputRisk());
}

void updateCaneGps() {
#if USE_CANE_GPS
  while (gpsSerial.available()) {
    caneGps.encode(gpsSerial.read());
  }
#endif
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
  if (risk < 0) {
    Serial.print("[JETSON] ignore: ");
    Serial.println(line);
    return;
  }

  jetsonRisk = (uint8_t)risk;
  lastJetsonRiskMs = millis();
  Serial.printf("[JETSON] risk=%u\n", jetsonRisk);
  applyRiskOutput(getOutputRisk());
}

void readJetsonUart() {
#if USE_JETSON_UART
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
#endif
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  delay(300);

  Serial.println("[ESP-NOW] Cane start");
  printCaneMac();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed, restart");
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("[ESP-NOW] Cane ready");
}

void setup() {
  forceOutputsOff();

  Serial.begin(115200);
  delay(1000);
  Serial.println("=== V2X Cane Distance + Jetson Bridge ===");
  Serial.printf("[CONFIG] fixed_cane_lat=%.6f fixed_cane_lng=%.6f\n", (float)CANE_FIXED_LAT, (float)CANE_FIXED_LNG);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#if USE_JETSON_UART
  jetsonSerial.begin(JETSON_BAUD, SERIAL_8N1, JETSON_RX, JETSON_TX);
  Serial.println("[JETSON] UART ready");
#endif

#if USE_DFPLAYER
  mp3Serial.begin(MP3_BAUD, SERIAL_8N1, MP3_RX, MP3_TX);
  delay(500);
  dfSetVolume(22);
  Serial.println("[DFPlayer] ready");
#endif

#if USE_CANE_GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS] cane GPS ready");
#endif

  setupEspNow();
  Serial.println("[CANE] System ready");
}

void loop() {
  updateCaneGps();
  updateBeep();
  readJetsonUart();
  applyRiskOutput(getOutputRisk());

  digitalWrite(LED_PIN, getOutputRisk() > RISK_SAFE ? HIGH : LOW);
  delay(5);
}
