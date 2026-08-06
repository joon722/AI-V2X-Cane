#!/usr/bin/env python3
"""브리지 ESP32가 살아있는지 시리얼만으로 확인한다.

ESP-NOW로 아무것도 안 들어올 때, 원인이 브리지인지 노드인지 가르는 도구.
raw 로그로는 판별할 수 없다. 브리지는 수신할 게 없으면 아무 말도 안 하므로,
"조용한 브리지"와 "죽은 브리지"가 로그상 똑같이 보이기 때문이다.

대신 브리지는 범위를 벗어난 risk 값을 받으면 반드시 risk_error 를 되돌려준다.
노드가 전부 꺼져 있어도 이 응답만으로 브리지 자체의 생존을 확인할 수 있다.

사용법 (젯슨에서, 서비스를 먼저 멈춰야 한다 - 포트를 서로 뺏는다):
    sudo systemctl stop v2x-risk-engine
    python3 ~/v2x/03_jetson/scripts/check_bridge.py
    sudo systemctl start v2x-risk-engine
"""

import argparse
import sys
import time


# 브리지가 받아들이는 범위는 0~3. 이 값은 반드시 거부당하므로 응답이 온다.
PROBE_RISK = 99
BOOT_WAIT_S = 4.0
REPLY_WAIT_S = 1.5


def read_for(connection, seconds):
    """seconds 동안 들어오는 줄을 모두 모은다."""
    deadline = time.monotonic() + seconds
    lines = []
    while time.monotonic() < deadline:
        raw = connection.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
    return lines


def report(booted, replied, boot_lines, reply_lines):
    print("")
    for line in boot_lines + reply_lines:
        print(f"  받은 줄: {line[:100]}")
    print("")

    if replied:
        print("[결과] 브리지 정상. 시리얼 명령에 응답합니다.")
        print("       ESP-NOW로 아무것도 안 들어온다면 원인은 노드(지팡이/차량) 쪽입니다.")
        return 0
    if booted:
        print("[결과] 브리지가 부팅은 했는데 명령에 응답하지 않습니다.")
        print("       루프가 멈춘 상태일 수 있습니다. 전원을 뺐다 꽂아 보세요.")
        return 1
    if boot_lines or reply_lines:
        print("[결과] 뭔가 오긴 하는데 브리지 형식이 아닙니다.")
        print("       다른 ESP32가 꽂혀 있거나 baud rate가 다를 수 있습니다.")
        return 1
    print("[결과] 브리지가 한 글자도 보내지 않습니다.")
    print("       USB 케이블(데이터 지원 여부), 보드 전원 LED를 확인하고,")
    print("       그래도 조용하면 브리지 펌웨어를 다시 올려야 합니다.")
    return 1


def parse_args():
    parser = argparse.ArgumentParser(description="브리지 ESP32 생존 확인")
    parser.add_argument("--port", default="/dev/v2x-rsu", help="시리얼 포트")
    parser.add_argument("--baud", type=int, default=115200)
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial이 필요합니다: pip3 install pyserial")

    try:
        connection = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f"[실패] 포트를 열 수 없습니다: {exc}", file=sys.stderr)
        print("", file=sys.stderr)
        print("서비스가 포트를 쥐고 있을 수 있습니다. 먼저 멈추세요:", file=sys.stderr)
        print("    sudo systemctl stop v2x-risk-engine", file=sys.stderr)
        raise SystemExit(1)

    with connection:
        # 포트를 여는 순간 DTR 신호로 ESP32가 리셋되므로 부팅 메시지를 볼 수 있다.
        print(f"[확인] {args.port} 열림. 브리지 재부팅을 {BOOT_WAIT_S:.0f}초 기다립니다...")
        boot_lines = read_for(connection, BOOT_WAIT_S)
        booted = any('"bridge_boot"' in line for line in boot_lines)

        print(f'[확인] 거부당할 명령을 보냅니다: {{"risk":{PROBE_RISK}}}')
        connection.write(f'{{"target_id":0,"risk":{PROBE_RISK}}}\n'.encode("utf-8"))
        connection.flush()
        reply_lines = read_for(connection, REPLY_WAIT_S)
        replied = any('"risk_error"' in line for line in reply_lines)

    raise SystemExit(report(booted, replied, boot_lines, reply_lines))


if __name__ == "__main__":
    main()
