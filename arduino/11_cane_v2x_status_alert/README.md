# 11_cane_v2x_status_alert

Cane-side endpoint for the `09_vehicle_v2x_direct_risk_alert` vehicle flow,
with optional RSU/Jetson risk input.

## Role

- Broadcast cane position/status as `MSG_CANE_STATUS` every 100 ms.
- Receive vehicle `MSG_VEHICLE_STATUS` and calculate cane-side distance/TTC risk.
- Receive higher-level irregular/AI risk from RSU/Jetson when available.
- Ignore vehicle-side `MSG_RISK_ALERT` for final output because it duplicates the cane-side distance/TTC calculation.
- Apply `final_risk = max(cane_local_risk, jetson_risk)`.
- Drive vibration/buzzer output from `final_risk`.

## Upload Combinations

Main direct ESP-to-ESP flow:

```text
Vehicle ESP32: 09_vehicle_v2x_direct_risk_alert
Cane ESP32:    11_cane_v2x_status_alert
```

RSU/Jetson can still be added for irregular/AI risk:

```text
Vehicle ESP32: 09_vehicle_v2x_direct_risk_alert
RSU ESP32:     10_rsu_espnow_multi_jetson_bridge
Cane ESP32:    11_cane_v2x_status_alert
```

## Normal System Flow

```text
Cane ESP32 11    --MSG_CANE_STATUS--> Vehicle ESP32 09
Vehicle ESP32 09 --MSG_VEHICLE_STATUS-> Cane ESP32 11
Cane ESP32 11 calculates cane_local_risk from vehicle status
Cane ESP32 11 output uses final_risk = max(cane_local_risk, jetson_risk)
```

## Optional Jetson Risk Flow

```text
Cane ESP32 11    --MSG_CANE_STATUS--> RSU 10 --> Jetson
Vehicle ESP32 09 --MSG_VEHICLE_STATUS-> RSU 10 --> Jetson
Jetson           --{"risk":N}--------> RSU 10
Cane ESP32 11    <--MSG_RISK_ALERT---- RSU 10
```

## Pins

```text
BUZZER_PIN = GPIO25  // Active LOW
MOTOR_PIN  = GPIO26  // Active HIGH
LED_PIN    = GPIO2
```

Optional GPS is disabled by default:

```cpp
#define USE_CANE_GPS 0
#define USE_FIXED_CANE_POS 1
```

Set the test position before upload:

```cpp
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000
```

## Risk Ownership

```text
Vehicle ESP32 09: close distance/TTC risk from vehicle side
Cane ESP32 11: close distance/TTC risk from cane side and physical output
Jetson: irregular/AI risk
Cane ESP32 11 final output: max(cane_local_risk, jetson_risk)
```
