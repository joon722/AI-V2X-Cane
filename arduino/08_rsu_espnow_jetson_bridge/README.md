# 08_rsu_espnow_jetson_bridge

Blank ESP32 bridge for the Jetson-side prototype.

This ESP32 replaces the wireless receive module that would be integrated inside
a real RSU. It has no sensors. It only bridges:

```text
Vehicle ESP32 -- ESP-NOW --> Bridge ESP32 -- USB Serial --> Jetson
Vehicle ESP32 <-- ESP-NOW -- Bridge ESP32 <-- USB Serial -- Jetson risk
```

## Upload target

- Board: `ESP32 Dev Module`
- Baud: `115200`

## Jetson connection

Use USB first. Plug this bridge ESP32 into the Jetson. The Jetson reads the USB
serial port, usually `/dev/ttyUSB0`.

## Output to Jetson

One JSON line per received vehicle packet:

```json
{"type":"vehicle","node_id":123,"seq":10,"gps_valid":1,"lat":37.0,"lng":127.0,"speed_mps":2.0,"heading_deg":90.0,"recv_count":10,"lost_count":0,"rssi":-42,"src_mac":"AA:BB:CC:DD:EE:FF"}
```

## Input from Jetson

Send one JSON line back to the bridge:

```json
{"risk":2}
```

or, for a specific vehicle:

```json
{"node_id":123,"risk":2}
```

The bridge sends the latest risk back to the vehicle over ESP-NOW.
