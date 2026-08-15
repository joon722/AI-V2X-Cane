#!/usr/bin/env python3
"""raw 로그를 요약해 링크 상태와 실험 진척을 몇 초마다 보여준다.

raw 로그는 초당 20줄씩 흘러서 tail -f 로는 "지금 실험할 수 있는 상태인가"를
판단하기 어렵다. 이 도구는 그 판단에 필요한 것만 본다.

  1행  링크 상태 - 지팡이와 차량이 둘 다 들어오는가, GPS는 잡혔는가, 신호는 쓸 만한가
  2행  실험 진척 - GPS 실제 주기, 지금 거리와 TTC, 근접 이벤트 누적, 안전 하한 발동

2행이 있는 이유는 실험이 끝난 뒤에야 성패를 아는 상황을 피하기 위해서다.
확인해야 할 셋이 모두 여기 나온다.

  GPS 주기    step7_risk의 안전 하한 2.0초는 5Hz(200ms)를 전제로 계산한 값이다.
              1Hz면 필요한 하한이 2.8초가 되어 전제가 무너진다.
  근접 이벤트 모델 검증에는 차량이 3 m/s 이상으로 움직이며 2m 이내로 접근한
              사례가 30건 이상 필요하다. 몇 건 모였는지 현장에서 알아야 한다.
  하한 발동   TTC 2.0초 이하에서 레벨 3이 나가는지. 배포가 실제로 동작하는지 확인.

사용법 (젯슨에서):
    python3 ~/v2x/03_jetson/scripts/watch_link.py
    python3 ~/v2x/03_jetson/scripts/watch_link.py --simple   # 1행만

종료: Ctrl+C
"""

import argparse
import json
import math
import time
from pathlib import Path


# 창이 길수록 건수는 안정적이지만 화면이 그만큼 늦게 바뀐다. 이 도구는
# "지금 노드가 살아있나"를 보는 용도라 반응 속도를 택했다.
WINDOW_S = 2.0
INTERVAL_S = 1.0
# ESP-NOW는 -90 아래로 내려가면 유실이 급격히 늘어난다.
RSSI_WEAK = -85

# step6_kinematics와 같은 평면 근사 상수. 젯슨에 numpy를 얹지 않으려고
# 여기서도 순수 파이썬으로 계산한다.
METERS_PER_DEGREE_LAT = 111320.0

# 접촉으로 볼 거리. oracle.D_CRIT_M과 같은 값.
D_CRIT_M = 2.0
# 모델 검증에 쓸 만한 접근으로 볼 최소 차량 속도.
FAST_MPS = 3.0
# 하한이 발동하는 TTC. 값을 복사해 두면 step7이 바뀔 때 조용히 갈라지므로
# 정본에서 직접 읽는다. import가 실패해도 화면 도구가 죽으면 안 되니 대비값을 둔다.
try:
    import sys as _sys

    _sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from step7_risk import T_FLOOR_TTC_S
except ImportError:  # 03_jetson 밖에서 로그만 볼 때
    T_FLOOR_TTC_S = 2.8
# 근접 이벤트 목표치. EXPERIMENT_PLAN.md 기준.
TARGET_EVENTS = 30
# 이보다 큰 TTC는 숫자로 보여주지 않는다. 접근속도가 0에 가까우면 TTC가
# 100초를 넘어가는데, 그것은 "위험이 멀다"가 아니라 "TTC로는 잴 수 없다"는
# 뜻이다. 큰 숫자를 그대로 띄우면 화면에서 오히려 눈을 끈다.
TTC_SHOW_MAX_S = 30.0


def latest_log(log_dir):
    files = sorted(Path(log_dir).glob("raw_*.log"), key=lambda p: p.stat().st_mtime)
    return files[-1] if files else None


def tail_lines(path, count):
    """파일 끝부분만 읽는다. 실험이 길어지면 로그가 수십 MB가 되기 때문."""
    with path.open("rb") as handle:
        handle.seek(0, 2)
        size = handle.tell()
        block = min(size, count * 300)
        handle.seek(size - block)
        text = handle.read().decode("utf-8", errors="replace")
    lines = text.splitlines()
    # 중간부터 읽었으면 첫 줄은 잘린 조각이다. 깨진 줄로 오해하지 않도록 버린다.
    if block < size and lines:
        lines = lines[1:]
    return lines[-count:]


def summarize(lines, now, window_s):
    """최근 window_s 초 안의 줄만 종류별로 모은다.

    깨진 줄도 센다. 보드레이트가 어긋나면 JSON이 아니라 쓰레기 바이트가
    쏟아지는데(2026-08-08에 11분을 그렇게 날렸다), 조용히 건너뛰면 화면에는
    "신호없음"으로만 보여서 브리지가 꺼진 것과 구분이 안 된다.
    """
    nodes = {"cane": [], "vehicle": []}
    tx = []
    good = bad = 0
    for line in lines:
        parts = line.split(" ", 2)
        if len(parts) < 3:
            # 쓰레기 바이트에는 개행도 공백도 없어서 "시각 RX 본문" 꼴이 되지 않는다.
            if line.strip():
                bad += 1
            continue
        try:
            stamp = float(parts[0])
        except ValueError:
            bad += 1
            continue
        if now - stamp > window_s:
            continue
        if parts[1] == "TX":
            tx.append(parts[2])
            continue
        try:
            record = json.loads(parts[2])
        except json.JSONDecodeError:
            bad += 1
            continue
        good += 1
        if record.get("type") in nodes:
            nodes[record["type"]].append(record)
    return nodes, tx, good, bad


def heading_rate(records):
    """최근 창에서 heading_valid=1 비율.

    차량은 GPS 진행방향이라 멈추면 0%가 정상이고, 지팡이는 IMU라 멈춰 있어도
    나와야 한다. 그래서 이 값은 노드마다 다르게 읽어야 한다.
    """
    if not records:
        return None
    return sum(1 for r in records if str(r.get("heading_valid")) == "1") / len(records)


def describe(label, records):
    if not records:
        return f"{label} 없음"
    last = records[-1]
    rssi = sum(int(r.get("rssi", 0)) for r in records) / len(records)
    warn = " 약함!" if rssi < RSSI_WEAK else ""
    head = heading_rate(records)
    head_part = "방향 --" if head is None else f"방향{head*100:3.0f}%"
    return (f"{label} {len(records):3d}건 gps={last.get('gps_valid')} "
            f"{head_part} rssi={rssi:.0f}{warn}")


def verdict(nodes):
    """실험을 시작해도 되는 상태인지 한 단어로."""
    if not nodes["cane"] and not nodes["vehicle"]:
        return "신호없음"
    if not nodes["cane"]:
        return "지팡이없음"
    if not nodes["vehicle"]:
        return "차량없음"
    if not all(
        str(records[-1].get("gps_valid")) == "1" for records in nodes.values()
    ):
        return "GPS대기"
    return "READY"


def status_line(now, nodes, tx, good, bad):
    clock = time.strftime("%H:%M:%S", time.localtime(now))
    tx_part = f"TX {len(tx)}건"
    if tx:
        tx_part += f" {tx[-1]}"
    # 깨진 줄이 섞이면 보드레이트 불일치나 브리지 부팅 루프다. 노드가 안 보이는
    # 것과 원인이 전혀 다르므로 따로 띄운다.
    broken = ""
    if bad and bad > good:
        broken = f" !!시리얼깨짐 {bad}줄 - 브리지 재부팅/보드레이트 확인"
    elif bad:
        broken = f" !깨진줄 {bad}"
    return (
        f"{clock} [{verdict(nodes)}] "
        f"{describe('지팡이', nodes['cane'])} | "
        f"{describe('차량', nodes['vehicle'])} | {tx_part}{broken}"
    )


def distance_m(cane, vehicle):
    """두 노드 사이 거리. 평면 근사이고 수백 미터 범위에서 GPS 오차 안쪽이다."""
    try:
        lat0 = float(cane["lat"])
        dlat = (float(vehicle["lat"]) - lat0) * METERS_PER_DEGREE_LAT
        dlng = ((float(vehicle["lng"]) - float(cane["lng"]))
                * METERS_PER_DEGREE_LAT * math.cos(math.radians(lat0)))
    except (KeyError, TypeError, ValueError):
        return None
    return math.hypot(dlat, dlng)


class Progress:
    """실험 진척을 누적한다.

    화면에 매번 새로 계산할 수 없는 것들 - GPS 갱신 주기, 근접 이벤트 횟수,
    하한 발동 횟수 - 을 호출 사이에 들고 있는다. 로그를 처음부터 다시 읽지 않고
    최근 창만 보므로, 누적값은 이 객체가 유일한 기억이다.
    """

    def __init__(self):
        self.last_pos = {}        # 노드별 (시각, 좌표) - GPS 주기 측정용
        self.gaps = {"cane": [], "vehicle": []}
        self.track = []           # (시각, 거리) - 접근속도 계산용
        self.events = 0           # 2m 이내 진입 횟수
        self.fast_events = 0      # 그중 차량이 빠르게 움직이던 경우
        self.inside = False       # 직전에 2m 안에 있었는지 (중복 계수 방지)
        self.floor_hits = 0       # 하한이 발동했을 상황의 TX 횟수
        self.seen_tx = 0

    def feed(self, now, nodes, tx):
        for kind in ("cane", "vehicle"):
            records = nodes[kind]
            if not records:
                continue
            last = records[-1]
            pos = (last.get("lat"), last.get("lng"))
            prev = self.last_pos.get(kind)
            # 같은 좌표의 재전송은 세지 않는다. 노드는 10Hz로 올리지만 GPS 자체는
            # 그보다 느리므로, 좌표가 바뀐 간격이라야 GPS 주기가 된다.
            if prev and prev[1] != pos and 0.05 < now - prev[0] < 3.0:
                self.gaps[kind].append(now - prev[0])
                self.gaps[kind] = self.gaps[kind][-20:]
            if not prev or prev[1] != pos:
                self.last_pos[kind] = (now, pos)

        cane = nodes["cane"][-1] if nodes["cane"] else None
        veh = nodes["vehicle"][-1] if nodes["vehicle"] else None
        if not (cane and veh and str(cane.get("gps_valid")) == "1"
                and str(veh.get("gps_valid")) == "1"):
            return

        d = distance_m(cane, veh)
        if d is None:
            return
        self.track.append((now, d))
        self.track = self.track[-12:]

        veh_speed = _to_float(veh.get("speed_mps"))
        if d <= D_CRIT_M and not self.inside:
            self.events += 1
            if veh_speed is not None and veh_speed >= FAST_MPS:
                self.fast_events += 1
            self.inside = True
        elif d > D_CRIT_M * 1.5:
            # 경계에서 떨었을 때 같은 접근을 여러 번 세지 않도록 여유를 둔다.
            self.inside = False

        ttc = self.ttc()
        new_tx = tx[len(tx) - max(len(tx) - self.seen_tx, 0):] if tx else []
        self.seen_tx = len(tx)
        if ttc is not None and ttc <= T_FLOOR_TTC_S:
            for raw in new_tx:
                try:
                    if json.loads(raw).get("risk") == 3:
                        self.floor_hits += 1
                except (json.JSONDecodeError, AttributeError):
                    pass

    def gps_hz(self, kind):
        gaps = self.gaps[kind]
        if len(gaps) < 3:
            return None
        return 1.0 / (sorted(gaps)[len(gaps) // 2])

    def closing(self):
        """거리 변화율. 양수면 가까워지는 중."""
        if len(self.track) < 4:
            return None
        (t0, d0), (t1, d1) = self.track[0], self.track[-1]
        if t1 - t0 < 0.4:
            return None
        return (d0 - d1) / (t1 - t0)

    def ttc(self):
        closing = self.closing()
        if not closing or closing <= 0 or not self.track:
            return None
        return self.track[-1][1] / closing


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def progress_line(progress):
    """2행: 실험 진척. 확인해야 할 셋을 한 줄에 담는다."""
    hz = []
    for kind, label in (("cane", "지팡이"), ("vehicle", "차량")):
        value = progress.gps_hz(kind)
        hz.append("--" if value is None else f"{value:.1f}")
    rate = f"GPS {hz[0]}/{hz[1]}Hz"
    if all(h != "--" for h in hz) and min(float(h) for h in hz) < 3.5:
        rate += "!"   # 5Hz가 아니면 하한 2.0초의 전제가 무너진다

    if progress.track:
        d = progress.track[-1][1]
        closing = progress.closing()
        ttc = progress.ttc()
        near = f"거리 {d:5.1f}m"
        if closing is not None:
            near += f" {'접근' if closing > 0 else '이탈'} {abs(closing):4.1f}m/s"
        if ttc is None:
            near += " TTC   --"
        elif ttc > TTC_SHOW_MAX_S:
            near += " TTC  >30s"
        else:
            near += f" TTC {ttc:4.1f}s"
    else:
        near = "거리    --"

    return (f"         {rate} | {near} | "
            f"근접 {progress.events}회(고속 {progress.fast_events}/{TARGET_EVENTS}) | "
            f"하한 {progress.floor_hits}회")


def parse_args():
    parser = argparse.ArgumentParser(description="V2X 링크 상태를 요약해서 보여준다.")
    parser.add_argument(
        "--log-dir",
        default=str(Path.home() / "v2x/03_jetson/logs"),
        help="raw_*.log 가 쌓이는 폴더",
    )
    parser.add_argument("--window-s", type=float, default=WINDOW_S,
                        help=f"최근 몇 초를 집계할지 (기본 {WINDOW_S})")
    parser.add_argument("--interval-s", type=float, default=INTERVAL_S,
                        help=f"몇 초마다 갱신할지 (기본 {INTERVAL_S})")
    parser.add_argument("--once", action="store_true", help="한 번만 출력하고 끝낸다")
    parser.add_argument("--simple", action="store_true",
                        help="링크 상태만 보여준다(2행 생략)")
    return parser.parse_args()


def main():
    args = parse_args()
    path = latest_log(args.log_dir)
    if path is None:
        raise SystemExit(f"raw 로그가 없습니다: {args.log_dir}")

    print(f"[감시] {path.name} (최근 {args.window_s:.0f}초 기준, 종료는 Ctrl+C)")
    if not args.simple:
        print(f"       2행: GPS 주기 / 거리·TTC / 근접 이벤트(고속 {TARGET_EVENTS}건 목표) "
              f"/ 안전 하한 발동")
    progress = Progress()
    try:
        while True:
            now = time.time()
            nodes, tx, good, bad = summarize(tail_lines(path, 400), now, args.window_s)
            print(status_line(now, nodes, tx, good, bad), flush=True)
            if not args.simple:
                progress.feed(now, nodes, tx)
                print(progress_line(progress), flush=True)
            if args.once:
                return
            time.sleep(args.interval_s)
    except KeyboardInterrupt:
        print("\n[종료]")


if __name__ == "__main__":
    main()
