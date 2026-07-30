import socket
import time

CANE_IP = "192.168.219.109"
CANE_PORT = 6001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("[JETSON] Sending risk 2 every 1 second")
print(f"[TARGET] {CANE_IP}:{CANE_PORT}")
print("Stop: Ctrl + C")

try:
    while True:
        text = '{"risk":2}'
        sock.sendto(text.encode("utf-8"), (CANE_IP, CANE_PORT))
        print("[SEND]", text)
        time.sleep(1)

except KeyboardInterrupt:
    print("\n[STOP] Sending risk 0 before exit")
    text = '{"risk":0}'
    sock.sendto(text.encode("utf-8"), (CANE_IP, CANE_PORT))
    print("[SEND]", text)

finally:
    sock.close()