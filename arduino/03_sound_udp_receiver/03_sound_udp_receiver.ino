#include <WiFi.h>
#include <WiFiUdp.h>

// Copy config.example.h to config.h and fill in local Wi-Fi credentials.
#include "config.h"

const int LOCAL_PORT = 6001;

#define VIB_PIN 25
#define BUZZER_PIN 27

#define BUZZER_ON LOW
#define BUZZER_OFF HIGH

#define VIB_ON HIGH
#define VIB_OFF LOW

WiFiUDP udp;

int riskLevel = 0;

const unsigned long VIB_PWM_PERIOD_MS = 20;
unsigned long vibPwmStartTime = 0;
int vibDuty = 0;

unsigned long lastBeepTime = 0;
bool buzzerState = false;
int beepStep = 0;

void buzzerOn() {
  digitalWrite(BUZZER_PIN, BUZZER_ON);
}

void buzzerOff() {
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
}

void vibrationOn() {
  digitalWrite(VIB_PIN, VIB_ON);
}

void vibrationOff() {
  digitalWrite(VIB_PIN, VIB_OFF);
}

void allOff() {
  vibDuty = 0;
  vibrationOff();
  buzzerOff();
  buzzerState = false;
  beepStep = 0;
}

void updateVibration() {
  if (vibDuty <= 0) {
    vibrationOff();
    return;
  }

  if (vibDuty >= 255) {
    vibrationOn();
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = (now - vibPwmStartTime) % VIB_PWM_PERIOD_MS;
  unsigned long onTime = (VIB_PWM_PERIOD_MS * vibDuty) / 255;

  if (elapsed < onTime) {
    vibrationOn();
  } else {
    vibrationOff();
  }
}

void updateRisk2Beep() {
  unsigned long now = millis();

  if (!buzzerState && now - lastBeepTime >= 800) {
    buzzerState = true;
    lastBeepTime = now;
    buzzerOn();
  }

  if (buzzerState && now - lastBeepTime >= 200) {
    buzzerState = false;
    lastBeepTime = now;
    buzzerOff();
  }
}

void updateRisk3Beep() {
  unsigned long now = millis();

  switch (beepStep) {
    case 0:
      buzzerOn();
      lastBeepTime = now;
      beepStep = 1;
      break;

    case 1:
      if (now - lastBeepTime >= 100) {
        buzzerOff();
        lastBeepTime = now;
        beepStep = 2;
      }
      break;

    case 2:
      if (now - lastBeepTime >= 100) {
        buzzerOn();
        lastBeepTime = now;
        beepStep = 3;
      }
      break;

    case 3:
      if (now - lastBeepTime >= 100) {
        buzzerOff();
        lastBeepTime = now;
        beepStep = 4;
      }
      break;

    case 4:
      if (now - lastBeepTime >= 100) {
        buzzerOn();
        lastBeepTime = now;
        beepStep = 5;
      }
      break;

    case 5:
      if (now - lastBeepTime >= 100) {
        buzzerOff();
        lastBeepTime = now;
        beepStep = 6;
      }
      break;

    case 6:
      if (now - lastBeepTime >= 500) {
        beepStep = 0;
      }
      break;
  }
}

void setRiskLevel(int level) {
  riskLevel = level;

  buzzerOff();
  buzzerState = false;
  beepStep = 0;
  lastBeepTime = millis();
  vibPwmStartTime = millis();

  if (riskLevel == 0) {
    vibDuty = 0;
    vibrationOff();
    buzzerOff();
    Serial.println("[RISK 0] SAFE - ALL OFF");
  } else if (riskLevel == 1) {
    vibDuty = 80;
    buzzerOff();
    Serial.println("[RISK 1] Weak vibration only");
  } else if (riskLevel == 2) {
    vibDuty = 80;
    buzzerOff();
    Serial.println("[RISK 2] Weak vibration + slow beep");
  } else if (riskLevel == 3) {
    vibDuty = 255;
    buzzerOff();
    Serial.println("[RISK 3] Strong vibration + triple beep");
  } else {
    riskLevel = 0;
    allOff();
    Serial.println("[UNKNOWN RISK] Force SAFE");
  }
}

void receiveRiskCommand() {
  int packetSize = udp.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  char buffer[256];
  int len = udp.read(buffer, sizeof(buffer) - 1);
  buffer[len] = '\0';

  String msg = String(buffer);

  Serial.print("[FROM JETSON] ");
  Serial.println(msg);

  if (msg.indexOf("\"risk\":0") >= 0) {
    setRiskLevel(0);
  } else if (msg.indexOf("\"risk\":1") >= 0) {
    setRiskLevel(1);
  } else if (msg.indexOf("\"risk\":2") >= 0) {
    setRiskLevel(2);
  } else if (msg.indexOf("\"risk\":3") >= 0) {
    setRiskLevel(3);
  } else {
    Serial.println("[UNKNOWN MESSAGE] Ignore");
  }
}

void updateOutputByRisk() {
  updateVibration();

  if (riskLevel == 0 || riskLevel == 1) {
    buzzerOff();
  } else if (riskLevel == 2) {
    updateRisk2Beep();
  } else if (riskLevel == 3) {
    updateRisk3Beep();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(VIB_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  allOff();

  Serial.println();
  Serial.println("================================");
  Serial.println("CANE ESP32 UDP RISK RECEIVER");
  Serial.println("BUZZER: LOW ON / HIGH OFF");
  Serial.println("VIB: HIGH ON / LOW OFF");
  Serial.println("BOOT STATE: SAFE");
  Serial.println("================================");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    allOff();
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("Cane ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(LOCAL_PORT);

  Serial.print("UDP listening port: ");
  Serial.println(LOCAL_PORT);

  setRiskLevel(0);
}

void loop() {
  receiveRiskCommand();
  updateOutputByRisk();
}
