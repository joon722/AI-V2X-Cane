// V5 cane UWB responder
// Board: Arduino Stella
// Counterpart: c33_uwb_bridge_V5.ino on Portenta C33 + UWB Shield

#include <StellaUWB.h>

static const uint32_t UWB_SESSION_ID = 0x11223344UL;
static uint8_t responderBytes[] = {0x22, 0x22};
static uint8_t controllerBytes[] = {0x11, 0x11};
static UWBMacAddress responderAddress(UWBMacAddress::Size::SHORT,
                                     responderBytes);
static UWBMacAddress controllerAddress(UWBMacAddress::Size::SHORT,
                                      controllerBytes);
static UWBRangingControlee responder(UWB_SESSION_ID,
                                     responderAddress,
                                     controllerAddress);

static volatile uint32_t successfulRanges = 0;
static volatile uint32_t failedRanges = 0;

void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() !=
      (uint8_t)uwb::MeasurementType::TWO_WAY) return;
  RangingMeasures measures = rangingData.twoWayRangingMeasure();
  for (int i = 0; i < rangingData.available(); i++) {
    if (measures[i].status == 0 && measures[i].distance != 0xFFFF) {
      successfulRanges++;
    } else {
      failedRanges++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStarted = millis();
  while (!Serial && millis() - waitStarted < 3000UL) delay(10);

  UWB.registerRangingCallback(rangingHandler);
  UWB.begin();
  Serial.println("[V5 UWB] Stella responder stack starting");

  uint32_t stackStarted = millis();
  while (UWB.state() != 0 && millis() - stackStarted < 15000UL) delay(10);
  if (UWB.state() != 0) {
    Serial.println("[V5 UWB] ERROR: stack init timeout");
    return;
  }

  UWBSessionManager.addSession(responder);
  responder.init();
  responder.start();
  Serial.println("[V5 UWB] session=0x11223344 responder=0x2222 controller=0x1111 rate=5Hz");
}

void loop() {
  static uint32_t lastHeartbeatMs = 0;
  if (millis() - lastHeartbeatMs >= 1000UL) {
    lastHeartbeatMs = millis();
    Serial.print("[V5 UWB] ok=");
    Serial.print((uint32_t)successfulRanges);
    Serial.print(" fail=");
    Serial.println((uint32_t)failedRanges);
  }
  delay(10);
}
