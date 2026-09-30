// V5 vehicle UWB bridge
// Board: Arduino Portenta C33 + Arduino Portenta UWB Shield
// Wiring: C33 D14/TX (Serial1 TX) -> vehicle ESP32 GPIO32/RX1, GND common
// Counterpart: stella_uwb_responder_V5.ino

#include <PortentaUWBShield.h>

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

void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() !=
      (uint8_t)uwb::MeasurementType::TWO_WAY) return;

  RangingMeasures measures = rangingData.twoWayRangingMeasure();
  for (int i = 0; i < rangingData.available(); i++) {
    if (measures[i].status == 0 && measures[i].distance != 0xFFFF) {
      // ESP32 V4/V5 파서와 바이트 단위로 맞춘 한 줄 형식. distance 단위는 cm.
      Serial1.print("status=\"SUCCESS\", distance[cm]=");
      Serial1.print(measures[i].distance);
      Serial1.println(", RSSI[dBm]=nan");
      successfulRanges++;
    } else {
      Serial1.print("status=\"FAIL\", code=");
      Serial1.println(measures[i].status);
      failedRanges++;
    }
  }
}

void setup() {
  Serial.begin(115200);   // USB 진단
  Serial1.begin(115200);  // D14/TX -> ESP32 GPIO32
  uint32_t waitStarted = millis();
  while (!Serial && millis() - waitStarted < 3000UL) delay(10);

#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, HIGH);
#endif

  UWB.registerRangingCallback(rangingHandler);
  UWB.begin();
  Serial.println("[V5 UWB] Portenta controller stack starting");

  uint32_t stackStarted = millis();
  while (UWB.state() != 0 && millis() - stackStarted < 15000UL) delay(10);
  if (UWB.state() != 0) {
    Serial.println("[V5 UWB] ERROR: stack init timeout");
    Serial1.println("status=\"FAIL\", code=STACK_TIMEOUT");
    return;
  }

  UWBSessionManager.addSession(controller);
  controller.init();
  controller.start();
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
    Serial.println((uint32_t)failedRanges);
  }
  delay(5);
}
