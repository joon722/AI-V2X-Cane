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
# step7_risk.T_FLOOR_TTC_S와 같은 값. 이 아래면 하한이 발동한다.
T_FLOOR_TTC_S = 2.0
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
    return text.splitlines()[-count:]


def summarize(lines, now, window_s):
    """최근 window_s 초 안의 줄만 종류별로 모은다."""
    nodes = {"cane": [], "vehicle": []}
    tx = []
    for line in lines:
        parts = line.split(" ", 2)
        if len(parts) < 3:
            continue
        try:
            stamp = float(parts[0])
        except ValueError:
            continue
        if now - stamp > window_s:
            continue
        if parts[1] == "TX":
            tx.append(parts[2])
            continue
        try:
            record = json.loads(parts[2])
        except json.JSONDecodeError:
            continue
        if record.get("type") in nodes:
            nodes[record["type"]].append(record)
    return nodes, tx


def describe(label, records):
    if not records:
        return f"{label} 없음"
    last = records[-1]
    rssi = sum(int(r.get("rssi", 0)) for r in records) / len(records)
    warn = " 약함!" if rssi < RSSI_WEAK else ""
    return f"{label} {len(records):3d}건 gps={last.get('gps_valid')} rssi={rssi:.0f}{warn}"


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


def status_line(path, now, window_s):
    nodes, tx = summarize(tail_lines(path, 400), now, window_s)
    clock = time.strftime("%H:%M:%S", time.localtime(now))
    tx_part = f"TX {len(tx)}건"
    if tx:
        tx_part += f" {tx[-1]}"
    return (
        f"{clock} [{verdict(nodes)}] "
        f"{describe('지팡이', nodes['cane'])} | "
        f"{describe('차량', nodes['vehicle'])} | {tx_part}"
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
            nodes, tx = summarize(tail_lines(path, 400), now, args.window_s)
            print(status_line(path, now, args.window_s), flush=True)
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
