# 10_rsu_espnow_multi_jetson_bridge

Final RSU bridge sketch for the current vehicle/cane/Jetson protocol.

## Role

- Receive vehicle `MSG_VEHICLE_STATUS` packets.
- Receive cane `MSG_CANE_STATUS` packets.
- Print each status packet to USB Serial as one JSON line for Jetson.
- Read Jetson risk JSON from USB Serial.
- Relay Jetson risk to vehicle/cane endpoints as `MSG_RISK_ALERT`.
- Do not calculate risk on the RSU bridge.

## Status JSON To Jetson

Each received status packet is printed with at least:

```text
type, node_id, seq, gps_valid, lat, lng, speed_mps, heading_deg,
node_risk, tx_ms, rx_ms, recv_count, lost_count, rssi, src_mac
```

Example:

```json
{"type":"vehicle","node_id":123,"seq":1,"gps_valid":1,"lat":37.0,"lng":127.0,"speed_mps":2.0,"heading_deg":90.0,"node_risk":0,"tx_ms":100,"rx_ms":120,"recv_count":1,"lost_count":0,"rssi":-52,"src_mac":"AA:BB:CC:DD:EE:FF"}
```

## Risk JSON From Jetson

Broadcast the same risk to every seen endpoint:

```json
{"risk":2}
```

Send risk to one endpoint:

```json
{"risk":2,"target_id":456,"src_id":123}
```

`target_id` is the endpoint that should receive the warning. `src_id` is optional
and can identify the vehicle or Jetson-side source of the risk.

## Relay Behavior

- Missing `target_id` broadcasts to every seen vehicle/cane endpoint.
- `target_id` of `0` or `0xFFFFFFFF` also broadcasts to every seen endpoint.
- Known `target_id` sends only to that endpoint.
- Unknown `target_id` is dropped and logged.
- Vehicle and cane endpoints both receive risk as `MSG_RISK_ALERT`.
