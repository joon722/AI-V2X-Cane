# 05_cane_v2x_distance_jetson_bridge

New main cane-side prototype sketch. This does not replace the older `02` file.

## Role

- Receive vehicle ESP32 packets over ESP-NOW
- Validate packet header and track `seq_num` loss per vehicle
- Compute coarse distance cutoff on the cane side
- Send vehicle data to Jetson over UART as JSON lines
- Read Jetson risk replies such as `{"risk":2}`
- Drive vibration motor, buzzer, and DFPlayer voice output
- Reply to the vehicle ESP32 over ESP-NOW

## Default wiring

```text
[Actuator]
Buzzer SIG -> GPIO25
Motor  SIG -> GPIO26

[Jetson UART]
ESP32 GPIO17 TX2 -> Jetson RX
ESP32 GPIO16 RX2 <- Jetson TX
GND shared

[DFPlayer]
DFPlayer TX -> ESP32 GPIO32
DFPlayer RX -> ESP32 GPIO33
Speaker -> DFPlayer L+/L-
```

## Important setup

The sketch defaults to a fixed demo cane position:

```cpp
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000
```

Change these to your test location, or enable `USE_CANE_GPS` and adjust the GPS pins.

Jetson should receive lines like:

```json
{"type":"vehicle","node_id":123,"seq":10,"distance_m":12.3,"coarse_risk":2}
```

Jetson can reply over UART with:

```json
{"risk":2}
```

