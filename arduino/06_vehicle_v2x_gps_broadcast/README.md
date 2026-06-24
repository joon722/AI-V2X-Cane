# 06_vehicle_v2x_gps_broadcast

New main vehicle-side prototype sketch. The vehicle only broadcasts its state and does not calculate risk.

## Role

- Read GPS from GPIO16/17
- Generate a vehicle `node_id` from the ESP32 MAC address
- Broadcast `v2x_message_t` at 10 Hz over ESP-NOW
- Receive cane replies and print `[RX BACK]`

## Wiring

```text
[GPS]
GPS VCC -> ESP32 3V3
GPS GND -> ESP32 GND
GPS TX  -> ESP32 GPIO16
GPS RX  -> ESP32 GPIO17
```

## Notes

By default the sketch uses ESP-NOW broadcast:

```cpp
#define SEND_BROADCAST 1
```

This is useful for 1:N and N:N demo setup. If directed send is needed, set it to `0` and replace `caneMAC[]`.

