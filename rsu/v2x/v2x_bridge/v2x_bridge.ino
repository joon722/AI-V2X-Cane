// V2X RSU 브리지 ESP32
//
// 역할:
//   지팡이/차량 --(ESP-NOW)--> [이 보드] --(USB 시리얼, JSON 한 줄씩)--> 젯슨
//   지팡이/차량 <--(ESP-NOW MSG_RSU_REPLY 브로드캐스트)-- [이 보드] <--(시리얼 {"target_id":0,"risk":N})-- 젯슨
//
// 이전 브리지에서 확인된 문제 2개를 설계로 방지:
//   1) 먹통(freeze): 수신 콜백에서는 큐에 넣기만 하고, JSON 출력은
//      메인 루프에서 수행. 워치독 10초 + 모든 버퍼에 오버플로우 가드.
//   2) target_not_seen: 다운링크는 대상 조회 없이 MSG_RSU_REPLY를
//      브로드캐스트한다. 지팡이(handleLegacyReply)와 차량(handleRsuReply)
//      모두 이 메시지를 이미 처리한다. target_id 값은 무시한다.
//
// 채널: 차량 ESP32가 만드는 V2X-LOG AP가 채널 6이므로 고정 6번을 쓴다.
//       (AP 접속은 하지 않는다 - 차량이 꺼져 있어도 브리지는 계속 동작)
//
// 젯슨이 기대하는 업링크 JSON (기존 브리지와 동일 형식):
//   {"type":"cane","node_id":...,"seq":...,"gps_valid":...,"lat":...,"lng":...,
//    "speed_mps":...,"heading_deg":...,"node_risk":...,"tx_ms":...,"rx_ms":...,
//    "recv_count":...,"lost_count":...,"rssi":...,"src_mac":"AA:BB:CC:DD:EE:FF"}

#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <esp_task_wdt.h>
#include <string.h>
#include <stdlib.h>

// =====================
// 설정
// =====================
#define LED_PIN 2
#define SERIAL_BAUD 115200

// 차량 노드의 V2X_WIFI_CHANNEL과 반드시 같아야 한다.
#define V2X_CHANNEL 6

// 루프가 이 시간(ms) 동안 멈추면 자동 재부팅.
#define WDT_TIMEOUT_MS 10000

// 1로 바꾸면 다운링크 성공도 JSON으로 보고한다.
// (젯슨 step8이 모르는 타입이라 journal에 경고가 쌓이므로 기본 0)
#define BRIDGE_VERBOSE 0

// =====================
// V2X 프로토콜 (지팡이/차량 노드와 반드시 동일)
// =====================
#define V2X_MAGIC 0x56325831UL
#define V2X_VERSION 2

#define MSG_VEHICLE_STATUS 1
#define MSG_RSU_REPLY 2
#define MSG_CANE_STATUS 3
#define MSG_RISK_ALERT 4

#define NODE_VEHICLE 0x10
#define NODE_CANE 0x20
#define NODE_RSU 0x30

#define RISK_SAFE 0
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

// =====================
// 수신 큐: 콜백(WiFi 태스크)에서는 넣기만, loop에서 꺼내 처리
// =====================
#define RX_QUEUE_SIZE 16

typedef struct {
  v2x_status_message_t msg;
  uint8_t mac[6];
  int8_t rssi;
  uint32_t rx_ms;
} rx_item_t;

static rx_item_t rxQueue[RX_QUEUE_SIZE];
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;
static volatile uint32_t rxDropped = 0;

// =====================
// 노드별 수신 통계 (recv_count / lost_count 계산용)
// =====================
#define MAX_NODES 6

typedef struct {
  bool used;
  uint32_t node_id;
  uint16_t last_seq;
  uint32_t recv_count;
  uint32_t lost_count;
  uint32_t last_rx_ms;
} node_stats_t;

static node_stats_t nodeStats[MAX_NODES];

static uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint32_t rsuId = 0;
static uint16_t replySeq = 0;

// 시리얼 다운링크 명령 수신 버퍼
static char serialLine[192];
static size_t serialLineLen = 0;

// =====================
// 유틸
// =====================
static uint32_t macToNodeId(const uint8_t *mac) {
  return ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) |
         ((uint32_t)mac[4] << 8) | mac[5];
}

static node_stats_t *getNodeStats(uint32_t nodeId, uint32_t now) {
  for (int i = 0; i < MAX_NODES; i++) {
    if (nodeStats[i].used && nodeStats[i].node_id == nodeId) {
      return &nodeStats[i];
    }
  }
  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodeStats[i].used) {
      memset(&nodeStats[i], 0, sizeof(node_stats_t));
      nodeStats[i].used = true;
      nodeStats[i].node_id = nodeId;
      return &nodeStats[i];
    }
  }
  // 테이블이 꽉 차면 가장 오래 조용했던 슬롯 재사용
  int oldest = 0;
  for (int i = 1; i < MAX_NODES; i++) {
    if (nodeStats[i].last_rx_ms < nodeStats[oldest].last_rx_ms) oldest = i;
  }
  memset(&nodeStats[oldest], 0, sizeof(node_stats_t));
  nodeStats[oldest].used = true;
  nodeStats[oldest].node_id = nodeId;
  return &nodeStats[oldest];
}

// =====================
// ESP-NOW 수신 콜백: 검증 후 큐에 복사만 한다 (printf 금지)
// =====================
static void onDataRecv(const esp_now_recv_info_t *info,
                       const uint8_t *data,
                       int len) {
  if (len != (int)sizeof(v2x_status_message_t)) return;

  uint32_t magic = 0;
  memcpy(&magic, data, sizeof(magic));
  if (magic != V2X_MAGIC || data[4] != V2X_VERSION) return;

  uint8_t msgType = data[5];
  if (msgType != MSG_CANE_STATUS && msgType != MSG_VEHICLE_STATUS) return;

  uint8_t nextHead = (rxHead + 1) % RX_QUEUE_SIZE;
  if (nextHead == rxTail) {  // 큐 가득 -> 버리고 카운트만 (블로킹 금지)
    rxDropped++;
    return;
  }

  rx_item_t *slot = &rxQueue[rxHead];
  memcpy(&slot->msg, data, sizeof(v2x_status_message_t));
  memcpy(slot->mac, info->src_addr, 6);
  slot->rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  slot->rx_ms = millis();
  rxHead = nextHead;
}

// =====================
// 업링크: 큐에서 꺼내 JSON 한 줄로 출력
// =====================
static void forwardQueuedPackets() {
  // 한 번의 loop에서 큐 전체를 비운다 (최대 RX_QUEUE_SIZE개)
  while (rxTail != rxHead) {
    rx_item_t item;
    memcpy(&item, &rxQueue[rxTail], sizeof(item));
    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;

    const v2x_status_message_t *m = &item.msg;
    const char *typeName =
      m->msg_type == MSG_CANE_STATUS ? "cane" : "vehicle";

    node_stats_t *stats = getNodeStats(m->node_id, item.rx_ms);
    if (stats->recv_count > 0) {
      // uint16 순번 되돌림(wrap)을 고려한 유실 계산
      uint16_t gap = (uint16_t)(m->seq_num - stats->last_seq);
      if (gap > 1) stats->lost_count += gap - 1;
    }
    stats->recv_count++;
    stats->last_seq = m->seq_num;
    stats->last_rx_ms = item.rx_ms;

    char json[352];
    int written = snprintf(
      json, sizeof(json),
      "{\"type\":\"%s\",\"node_id\":%lu,\"seq\":%u,"
      "\"gps_valid\":%u,\"lat\":%.6f,\"lng\":%.6f,"
      "\"speed_mps\":%.3f,\"heading_deg\":%.2f,\"node_risk\":%u,"
      "\"tx_ms\":%lu,\"rx_ms\":%lu,"
      "\"recv_count\":%lu,\"lost_count\":%lu,\"rssi\":%d,"
      "\"src_mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}",
      typeName,
      (unsigned long)m->node_id,
      m->seq_num,
      m->gps_valid,
      m->latitude,
      m->longitude,
      m->speed_mps,
      m->heading_deg,
      m->risk_level,
      (unsigned long)m->timestamp_ms,
      (unsigned long)item.rx_ms,
      (unsigned long)stats->recv_count,
      (unsigned long)stats->lost_count,
      (int)item.rssi,
      item.mac[0], item.mac[1], item.mac[2],
      item.mac[3], item.mac[4], item.mac[5]
    );

    if (written > 0 && written < (int)sizeof(json)) {
      Serial.println(json);
    }
  }
}

// =====================
// 다운링크: {"target_id":X,"risk":N} -> MSG_RSU_REPLY 브로드캐스트
// =====================
static bool jsonFindLong(const char *line, const char *key, long *out) {
  char pattern[32];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(line, pattern);
  if (p == NULL) return false;
  p = strchr(p + strlen(pattern), ':');
  if (p == NULL) return false;
  *out = strtol(p + 1, NULL, 10);
  return true;
}

static void sendRsuReply(uint8_t risk) {
  v2x_status_message_t reply;
  memset(&reply, 0, sizeof(reply));
  reply.magic = V2X_MAGIC;
  reply.version = V2X_VERSION;
  reply.msg_type = MSG_RSU_REPLY;
  reply.node_type = NODE_RSU;
  reply.risk_level = risk;
  reply.gps_valid = 0;
  reply.node_id = rsuId;
  reply.timestamp_ms = millis();
  reply.seq_num = replySeq++;

  esp_err_t result =
    esp_now_send(broadcastMAC, (uint8_t *)&reply, sizeof(reply));

#if BRIDGE_VERBOSE
  Serial.printf(
    "{\"type\":\"risk_tx\",\"risk\":%u,\"result\":\"%s\"}\n",
    risk, result == ESP_OK ? "OK" : "ERR"
  );
#else
  if (result != ESP_OK) {
    Serial.printf(
      "{\"type\":\"risk_error\",\"reason\":\"espnow_send_failed\",\"risk\":%u}\n",
      risk
    );
  }
#endif
}

static void handleSerialCommand(const char *line) {
  long risk = -1;
  if (!jsonFindLong(line, "risk", &risk)) {
    Serial.println(
      "{\"type\":\"risk_error\",\"reason\":\"missing_risk_field\"}"
    );
    return;
  }

  if (risk < RISK_SAFE || risk > RISK_DANGER) {
    Serial.printf(
      "{\"type\":\"risk_error\",\"reason\":\"invalid_risk_value\",\"risk\":%ld}\n",
      risk
    );
    return;
  }

  // target_id는 읽되 사용하지 않는다. MSG_RSU_REPLY 브로드캐스트는
  // 지팡이/차량 펌웨어가 모두 수신하므로 대상 조회가 필요 없다.
  sendRsuReply((uint8_t)risk);
}

static void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (serialLineLen > 0) {
        serialLine[serialLineLen] = '\0';
        if (serialLine[0] == '{') {
          handleSerialCommand(serialLine);
        }
        serialLineLen = 0;
      }
      continue;
    }

    if (serialLineLen < sizeof(serialLine) - 1) {
      serialLine[serialLineLen++] = c;
    } else {
      // 줄이 비정상적으로 길면 버리고 처음부터 (오버플로우 가드)
      serialLineLen = 0;
    }
  }
}

// =====================
// 워치독: 코어 2.x/3.x 모두 지원
// =====================
static void setupWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t config = {};
  config.timeout_ms = WDT_TIMEOUT_MS;
  config.idle_core_mask = 0;
  config.trigger_panic = true;
  esp_task_wdt_reconfigure(&config);
#else
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
  esp_task_wdt_add(NULL);
}

// =====================
// setup / loop
// =====================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(500);

  // AP 접속 없이 채널만 고정: 차량 AP가 꺼져 있어도 브리지는 동작한다.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_channel(V2X_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  rsuId = macToNodeId(mac);

  if (esp_now_init() != ESP_OK) {
    Serial.println(
      "{\"type\":\"bridge_error\",\"reason\":\"espnow_init_failed\"}"
    );
    delay(1000);
    ESP.restart();
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMAC, 6);
  peer.channel = 0;  // 현재 채널 사용
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  setupWatchdog();

  Serial.printf(
    "{\"type\":\"bridge_boot\",\"node_id\":%lu,\"channel\":%d,"
    "\"version\":%d,\"wdt_ms\":%d}\n",
    (unsigned long)rsuId, V2X_CHANNEL, V2X_VERSION, WDT_TIMEOUT_MS
  );
}

void loop() {
  esp_task_wdt_reset();

  forwardQueuedPackets();
  readSerialCommands();

  // 콜백에서 버린 패킷이 있으면 10초에 한 번만 보고
  static uint32_t lastDropReportMs = 0;
  static uint32_t reportedDrops = 0;
  uint32_t now = millis();
  if (rxDropped != reportedDrops && now - lastDropReportMs >= 10000UL) {
    lastDropReportMs = now;
    reportedDrops = rxDropped;
    Serial.printf(
      "{\"type\":\"bridge_warn\",\"reason\":\"rx_queue_drop\",\"count\":%lu}\n",
      (unsigned long)rxDropped
    );
  }

  // 살아있음 표시: 0.5초 간격 깜빡임
  digitalWrite(LED_PIN, (now / 500UL) % 2);

  delay(2);
}
