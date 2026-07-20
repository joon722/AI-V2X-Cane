# 09_vehicle_v2x_direct_risk_alert

Vehicle-side V2X endpoint for a no-Jetson direct ESP-NOW test.

This sketch is not the cane endpoint. It runs on the vehicle ESP32:

- broadcasts vehicle GPS status with `MSG_VEHICLE_STATUS`
- receives cane position/status with `MSG_CANE_STATUS`
- calculates simple distance/TTC risk on the vehicle ESP32
- sends `MSG_RISK_ALERT` directly back to the cane ESP32

Use this sketch only when testing direct ESP-to-ESP risk calculation without
the RSU/Jetson bridge.

## Topology

```text
Cane ESP32 --MSG_CANE_STATUS--> Vehicle ESP32
Cane ESP32 <--MSG_RISK_ALERT--- Vehicle ESP32
```

For the Jetson-centered project flow, use these sketches instead:

```text
06_vehicle_v2x_gps_broadcast
08_rsu_espnow_jetson_bridge
```

or, when both vehicle and cane packets must pass through the RSU:

```text
10_rsu_espnow_multi_jetson_bridge
```

## Pins

```text
[GPS]
GPS TX -> ESP32 GPIO16
GPS RX -> ESP32 GPIO17

[Status]
LED_PIN = GPIO2
```

## Test Position

When GPS is not fixed yet, the sketch can use the demo moving fallback enabled
by:

```cpp
#define USE_DEMO_MOVING_FALLBACK 1
```

The cane fallback reference position is:

```cpp
#define CANE_FIXED_LAT 37.000000
#define CANE_FIXED_LNG 127.000000
```

Set `USE_DEMO_MOVING_FALLBACK` to `0` when you want to rely only on live GPS.
