# Python / Jetson Utilities

## `jetson_uart_vehicle_risk_probe.py`

Vehicle ESP32 -> Jetson UART receive/reply test.

Install dependency on Jetson:

```bash
python3 -m pip install pyserial
```

Run with Jetson Nano GPIO UART:

```bash
python3 jetson_uart_vehicle_risk_probe.py --port /dev/ttyTHS1 --baud 115200 --cane-lat 37.000000 --cane-lng 127.000000
```

If using a USB-UART adapter:

```bash
python3 jetson_uart_vehicle_risk_probe.py --port /dev/ttyUSB0 --baud 115200
```

Expected ESP32 input line:

```json
{"type":"vehicle","node_id":123,"seq":1,"gps_valid":1,"lat":37.000000,"lng":127.000010,"speed_mps":2.1,"heading_deg":90.0}
```

Reply sent back to ESP32:

```json
{"risk":1,"distance_m":8.5,"reason":"distance<10m","jetson_ms":123456789}
```

## `risk_engine.py`

Shared RISK calculation module used by Jetson live input and CSV replay.

Inputs follow the 9-column integration schema:

```csv
ts_ms,ped_x,ped_y,veh_x,veh_y,ped_speed_mps,veh_speed_mps,distance_m,rel_speed_mps
```

The engine computes:

```text
rule_risk = distance/TTC based risk
zone_risk = zone_definition.csv based base risk
transformer_risk = placeholder until Minseo's .onnx model is ready
final_risk = max(rule_risk, zone_risk, transformer_risk)
```

## `jetson_rsu_bridge.py`

Recommended Jetson integration entry point.

Run on Jetson:

```bash
python3 jetson_rsu_bridge.py --port /dev/ttyUSB0 --baud 115200 --zone-file zone_definition.csv
```

Expected RSU ESP32 input lines:

```json
{"type":"cane","node_id":456,"x":0.0,"y":0.0,"speed_mps":1.0}
{"type":"vehicle","node_id":123,"x":20.0,"y":0.0,"speed_mps":6.0,"rel_speed_mps":5.0}
```

Reply sent back to RSU ESP32:

```json
{"type":"risk","target_id":456,"src_id":123,"risk":2,"distance_m":5.5,"ttc_s":4.0,"reason":"winner=rule;...","jetson_ms":123456789}
```
