// [강현준] B-5: ESP-NOW 수신기 전체 코드 (Receiver.ino)
#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 2 // ESP32 내장 LED

// --- 구조체 (송신기와 완전히 동일!)
#define NODE_VEHICLE 0x02
#define RISK_SAFE 0
#define RISK_CAUTION 1
#define RISK_WARNING 2
#define RISK_DANGER 3

typedef struct v2x_message {
  uint8_t node_type;
  uint8_t risk_level;
  float latitude;
  float longitude;
  float speed_mps;
  float heading_deg;
  float accel_x;
  float accel_y;
  float accel_z;
  uint32_t timestamp_ms;
  uint8_t seq_num;
} v2x_message_t;

v2x_message_t incoming;
uint8_t prevSeq = 0;
uint32_t recvCount = 0;
uint32_t lostCount = 0;

// 데이터가 도착하면 자동으로 이 함수가 실행됨
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // 받은 바이트 구조체로 변환
  memcpy(&incoming, data, sizeof(incoming));
  recvCount++;

  // 패킷 유실 감지
  if (recvCount > 1) {
    uint8_t expected = prevSeq + 1;
    if (incoming.seq_num != expected) {
      uint8_t gap = incoming.seq_num - expected;
      lostCount += gap;
      Serial.print(" 패킷 ");
      Serial.print(gap);
      Serial.println("개 유실!");
    }
  }
  prevSeq = incoming.seq_num;

  // 수신 내용 출력
  Serial.println("-------------------------");
  Serial.print("[RX] #");
  Serial.print(incoming.seq_num);
  Serial.print(" | 노드: ");
  Serial.println(incoming.node_type == NODE_VEHICLE ? "차량" : "기타");
  Serial.print(" 좌표: (");
  Serial.print(incoming.latitude, 4);
  Serial.print(", ");
  Serial.print(incoming.longitude, 4);
  Serial.println(")");
  Serial.print(" 속도: ");
  Serial.print(incoming.speed_mps * 3.6, 1);
  Serial.print("km/h | 방향: ");
  Serial.print(incoming.heading_deg, 0);
  Serial.println("°");
  Serial.print(" 위험등급: ");
  Serial.print(incoming.risk_level);
  Serial.print(" | 총수신: ");
  Serial.print(recvCount);
  Serial.print(" | 유실: ");
  Serial.println(lostCount);

  // LED 깜빡 (수신확인)
  digitalWrite(LED_PIN, HIGH);
  delay(30);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("=== 수신기(지팡이 노드) 시작 ===");
  
  WiFi.mode(WIFI_STA);
  delay(1000); // <--- Wi-Fi 칩이 잠에서 깰 수 있게 1초 기다려주는 마법의 코드!
  
  Serial.print("수신기 MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패!");
    ESP.restart();
  }
  // 콜백 등록
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("수신 대기 중...");
}

void loop() {
  // 모든 처리는 콜백에서 자동 실행
  delay(1000);
}


//송신기
// [강현준] B-4: ESP-NOW 송신기 전체 코드 (Sender.ino)
#include <esp_now.h>
#include <WiFi.h>

// *** 네가 찾은 수신기 MAC 주소가 여기 쏙 들어갔어! ***
uint8_t receiverMAC[] = {0x1C, 0xC3, 0xAB, 0xD1, 0x73, 0x5C};

//--- 구조체 
#define NODE_VEHICLE 0x02
#define RISK_SAFE 0

typedef struct v2x_message {
  uint8_t node_type;
  uint8_t risk_level;
  float latitude;
  float longitude;
  float speed_mps;
  float heading_deg;
  float accel_x;
  float accel_y;
  float accel_z;
  uint32_t timestamp_ms;
  uint8_t seq_num;
} v2x_message_t;

v2x_message_t outgoing;
uint8_t seq = 0;

// 전송 결과 알림
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("[TX] #");
  Serial.print(seq);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " 성공" : " 실패 x");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== 송신기(차량 노드) 시작 ===");
  
  WiFi.mode(WIFI_STA);
  Serial.print("송신기 MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패!");
    ESP.restart();
  }
  
 // 변경 후
esp_now_register_send_cb((esp_now_send_cb_t)onSent);
  
  // 수신기를 Peer(친구)로 등록
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Peer 등록 실패!");
  }
  Serial.println("초기화 완료. 전송 시작...");
}

void loop() {
  // 더미 데이터 채우기 
  outgoing.node_type = NODE_VEHICLE;
  outgoing.risk_level = RISK_SAFE;
  outgoing.latitude = 37.4563;
  outgoing.longitude = 126.9520;
  outgoing.speed_mps = 8.33; // 약 30km/h
  outgoing.heading_deg = 180.0; // 남쪽
  outgoing.accel_x = 0.05;
  outgoing.accel_y = -0.02;
  outgoing.accel_z = -9.81;
  outgoing.timestamp_ms = millis();
  outgoing.seq_num = seq++;
  
  // 전송!
  esp_now_send(receiverMAC, (uint8_t *)&outgoing, sizeof(outgoing));
  delay(100); // 100ms 간격으로 쏨
}
