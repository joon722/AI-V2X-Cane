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
{"risk":1,"distance_m":12.3,"reason":"distance<=30m","jetson_ms":123456789}
```
