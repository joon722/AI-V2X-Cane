import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

while True:
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"opened {PORT} at {BAUD}")
        break
    except PermissionError:
        print("permission denied. Run with sudo or add user to dialout.")
        raise
    except Exception as e:
        print(f"waiting for {PORT}: {e}")
        time.sleep(1)

while True:
    line = ser.readline().decode(errors="replace").strip()
    if line:
        print(line)
