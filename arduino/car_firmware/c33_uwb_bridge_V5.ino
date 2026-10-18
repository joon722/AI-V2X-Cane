// V5 vehicle UWB bridge
// Board: Arduino Portenta C33 + Arduino Portenta UWB Shield
// Wiring: C33 D14/TX (Serial1 TX) -> vehicle ESP32 GPIO32/RX1, GND common
// Counterpart: stella_uwb_responder_V5.ino

#include <PortentaUWBShield.h>

// 1로 바꾸면 UWB 스택의 상세 로그를 USB 시리얼로 낸다. 측거 세션이 시작되지
// 않으면(GitHub Truesense-it/PortentaUWBShield 이슈 #5, event 162) 이 로그를 남겨 둔다.
#ifndef UWB_VERBOSE_LOG
#define UWB_VERBOSE_LOG 0
#endif

static const uint32_t UWB_SESSION_ID = 0x11223344UL;
static uint8_t controllerBytes[] = {0x11, 0x11};
static uint8_t responderBytes[] = {0x22, 0x22};
static UWBMacAddress controllerAddress(UWBMacAddress::Size::SHORT,
                                      controllerBytes);
static UWBMacAddress responderAddress(UWBMacAddress::Size::SHORT,
                                     responderBytes);
static UWBRangingController controller(UWB_SESSION_ID,
                                       controllerAddress,
                                       responderAddress);

static volatile uint32_t successfulRanges = 0;
static volatile uint32_t failedRanges = 0;
// USB에서도 실패 원인을 확인한다. 측정 status는 init/start 반환값과 별개다.
static volatile uint8_t lastMeasurementStatus = 0xFF;
static volatile uint16_t lastRawDistanceCm = 0xFFFF;
static volatile uint16_t lastPeerAddress = 0;
static volatile uint32_t lastMeasurementMs = 0;
static bool sessionStarted = false;
static uint8_t sessionInitStatus = 0xFF;
static uint8_t sessionStartStatus = 0xFF;
static volatile float lastAzimuthDeg = 0.0f;
static volatile uint8_t lastAzimuthFom = 0;
static volatile uint8_t lastNlos = 255;

void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() !=
      (uint8_t)uwb::MeasurementType::TWO_WAY) return;

  RangingMeasures measures = rangingData.twoWayRangingMeasure();
  for (int i = 0; i < rangingData.available(); i++) {
    lastMeasurementStatus = measures[i].status;
    lastRawDistanceCm = measures[i].distance;
    lastPeerAddress = ((uint16_t)measures[i].peer_addr[1] << 8) |
                      measures[i].peer_addr[0];
    lastMeasurementMs = millis();
    if (measures[i].status == 0 && measures[i].distance != 0xFFFF) {
      // 앞부분은 DWM3001CDK CLI와 같은 형식이라 차량 V4/V5 파서가 그대로 읽는다.
      // 뒤에 쉴드가 잰 Stella 방향(AoA)을 덧붙인다. aoa_azimuth는 Q9.7 형식(/128 = 도),
      // aoa_fom은 각도 신뢰도 0~100, nlos는 0=가림 없음·1=가림·255=판단 불가.
      float azimuthDeg = measures[i].aoa_azimuth / 128.0f;
      Serial1.print("status=\"SUCCESS\", distance[cm]=");
      Serial1.print(measures[i].distance);
      Serial1.print(", RSSI[dBm]=nan, azimuth[deg]=");
      Serial1.print(azimuthDeg, 1);
      Serial1.print(", aoa_fom=");
      Serial1.print(measures[i].aoa_azimuth_fom);
      Serial1.print(", nlos=");
      Serial1.println(measures[i].nlos);
      lastAzimuthDeg = azimuthDeg;
      lastAzimuthFom = measures[i].aoa_azimuth_fom;
      lastNlos = measures[i].nlos;
      successfulRanges++;
    } else {
      Serial1.print("status=\"FAIL\", code=");
      Serial1.println(measures[i].status);
      failedRanges++;
    }
  }
}

void printUwbSessionResult(const char *operation, uwb::Status status) {
  Serial.print("[V5 UWB] ");
  Serial.print(operation);
  Serial.print("=0x");
  if ((uint8_t)status < 0x10) Serial.print('0');
  Serial.print((uint8_t)status, HEX);
  Serial.println(status == uwb::Status::SUCCESS ? " SUCCESS" : " FAILED");
}

void setup() {
  Serial.begin(115200);   // USB 진단
  Serial1.begin(115200);  // D14/TX -> ESP32 GPIO32
  uint32_t waitStarted = millis();
  while (!Serial && millis() - waitStarted < 3000UL) delay(10);
  Serial.println("[V5 UWB] diagnostic=peer-address-fix-1");

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, HIGH);
#endif

  UWB.registerRangingCallback(rangingHandler);
#if UWB_VERBOSE_LOG
  UWB.begin(Serial, uwb::LogLevel::UWB_RX_LEVEL);
#else
  UWB.begin(Serial, uwb::LogLevel::UWB_INFO_LEVEL);
#endif
  Serial.println("[V5 UWB] Portenta controller stack starting");

  uint32_t stackStarted = millis();
  while (UWB.state() != 0 && millis() - stackStarted < 15000UL) delay(10);
  if (UWB.state() != 0) {
    Serial.println("[V5 UWB] ERROR: stack init timeout");
    Serial1.println("status=\"FAIL\", code=STACK_TIMEOUT");
    return;
  }

  // PortentaUWBShield 1.0.2: controller 생성자는 dstAddr를 값으로 받고,
  // appParams에는 그 임시 객체의 주소 버퍼 포인터만 남긴다. init 직전에
  // 수명이 유지되는 전역 responderAddress로 다시 연결해야 주소가 깨지지 않는다.
  bool peerBound = controller.appParams.destinationMacAddr(responderAddress);
  uwb::AppConfig *peerConfig =
    controller.appParams.findParam(uwb::AppConfigId::PeerAddress);
  if (!peerBound || peerConfig == nullptr ||
      peerConfig->param_type != uwb::AppParamType::ARRAY_U8 ||
      peerConfig->param_value.au8.param_len != 2 ||
      peerConfig->param_value.au8.param_value != responderAddress.getData()) {
    Serial.println("[V5 UWB] ERROR: persistent peer address binding failed");
    return;
  }
  Serial.println("[V5 UWB] peer_config=0x2222 persistent=1");

  UWBSessionManager.addSession(controller);
  uwb::Status initStatus = controller.init();
  sessionInitStatus = (uint8_t)initStatus;
  printUwbSessionResult("init", initStatus);
  if (initStatus != uwb::Status::SUCCESS) return;
  uwb::Status startStatus = controller.start();
  sessionStartStatus = (uint8_t)startStatus;
  printUwbSessionResult("start", startStatus);
  if (startStatus != uwb::Status::SUCCESS) return;
  sessionStarted = true;
  Serial.println("[V5 UWB] session=0x11223344 controller=0x1111 responder=0x2222 rate=5Hz");
}

void loop() {
  static uint32_t lastHeartbeatMs = 0;
  if (millis() - lastHeartbeatMs >= 1000UL) {
    lastHeartbeatMs = millis();
#if defined(ARDUINO_PORTENTA_C33)
    digitalWrite(LEDR, !digitalRead(LEDR));
#endif
    Serial.print("[V5 UWB] ok=");
    Serial.print((uint32_t)successfulRanges);
    Serial.print(" fail=");
    Serial.print((uint32_t)failedRanges);
    Serial.print(" session=");
    Serial.print(sessionStarted ? "STARTED" : "NOT_STARTED");
    Serial.print(" init=0x");
    if (sessionInitStatus < 0x10) Serial.print('0');
    Serial.print(sessionInitStatus, HEX);
    Serial.print(" start=0x");
    if (sessionStartStatus < 0x10) Serial.print('0');
    Serial.print(sessionStartStatus, HEX);
    if (lastMeasurementMs > 0) {
      Serial.print(" status=0x");
      uint8_t status = lastMeasurementStatus;
      if (status < 0x10) Serial.print('0');
      Serial.print(status, HEX);
      Serial.print(" raw_cm=");
      Serial.print((uint16_t)lastRawDistanceCm);
      Serial.print(" peer=0x");
      Serial.print((uint16_t)lastPeerAddress, HEX);
      Serial.print(" age_ms=");
      Serial.print(millis() - (uint32_t)lastMeasurementMs);
    } else {
      Serial.print(" measurement=NONE");
    }
    Serial.print(" azimuth=");
    Serial.print((float)lastAzimuthDeg, 1);
    Serial.print(" fom=");
    Serial.print((uint8_t)lastAzimuthFom);
    Serial.print(" nlos=");
    Serial.println((uint8_t)lastNlos);
  }
  delay(5);
}
