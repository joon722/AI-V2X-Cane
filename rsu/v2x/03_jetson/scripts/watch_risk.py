#!/usr/bin/env python3
"""촬영용 위험도 표시 — 두 가지 보기.

  · 기본(로그) : watch_link 처럼 매 갱신마다 아래로 한 줄씩 쌓인다. 위로 스크롤
                 해 지난 기록을 볼 수 있다. 값 확인·디버깅에 좋다.
  · --big      : htop 처럼 전체화면을 독점해 위험 레벨을 큰 색깔 박스로 화면 가득
                 그린다. 촬영 화면에 좋다. Ctrl-C 로 원래 터미널 복귀.

둘 다 같은 정보를 보여준다: RSU(젯슨) 레벨 · 지팡이/차량 수신 위험 · 거리 · TTC ·
접근/멀어짐 · GPS · RSSI. RSU 가 보낸 레벨과 노드가 든 레벨이 다르면(다운링크가
아직 안 따라옴) RSU 값을 빨갛게 표시한다.

읽기만 하는 관측 도구다(경보 경로와 무관): risk_tx CSV + raw_*.log.

    python3 scripts/watch_risk.py                  # 로그(확인용)
    python3 scripts/watch_risk.py --big            # 큰 색 박스(촬영용)
    python3 scripts/watch_risk.py --logs logs_sim  # 리허설(가짜 접근) 볼 때
"""
import argparse
import csv
import glob
import json
import os
import re
import shutil
import time

LEVELS = {
    0: ("● 안 전",   "48;5;22",  "97"),
    1: ("▲ 주 의",   "48;5;100", "97"),
    2: ("▲▲ 경 고",  "48;5;208", "30"),
    3: ("■ 위 험 ■", "48;5;196", "97"),
}
WAIT = ("… 대 기 …", "48;5;238", "97")
RESET = "\033[0m"
RED = "\033[91;1m"
DIM = "\033[90m"
HOME = "\033[H"
CLR_EOL = "\033[K"
CLR_DOWN = "\033[J"
ENTER_ALT = "\033[?1049h\033[?25l\033[2J\033[H"   # 전체화면 진입 + 커서 숨김
EXIT_ALT = "\033[?25h\033[?1049l"                 # 원래 화면 복귀
LINE_RE = re.compile(r"^(\d+\.\d+) RX (\{.*\})\s*$")


def _latest(log_dir, pattern):
    files = glob.glob(os.path.join(log_dir, pattern))
    return max(files, key=os.path.getmtime) if files else None


def last_row(path):
    # 전원이 급차단되면 파일 꼬리가 NUL로 채워진 채 남는다(8/17 12:49 실제 발생).
    # 파이썬 3.10 csv 는 NUL 줄에서 예외를 던지므로 지우고 읽고, 잘려서 열이
    # 모자란 마지막 행은 건너뛰어 마지막 "정상" 행을 돌려준다.
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            reader = csv.DictReader(line.replace("\0", "") for line in fh)
            rows = [r for r in reader if None not in r.values()]
    except (OSError, csv.Error):
        return None
    return rows[-1] if rows else None


def latest_rssi(path, tail_bytes=48000):
    out = {"cane": {}, "vehicle": {}}
    if not path:
        return out
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as fh:
            if size > tail_bytes:
                fh.seek(size - tail_bytes)
                fh.readline()
            text = fh.read().decode("utf-8", errors="replace")
    except OSError:
        return out
    for line_ in reversed(text.splitlines()):
        m = LINE_RE.match(line_)
        if not m:
            continue
        try:
            j = json.loads(m.group(2))
        except json.JSONDecodeError:
            continue
        typ = j.get("type")
        if typ in out and not out[typ]:
            out[typ] = {"rssi": j.get("rssi"), "gps_valid": j.get("gps_valid")}
        if out["cane"] and out["vehicle"]:
            break
    return out


def _f(row, key):
    try:
        return float(row.get(key))
    except (TypeError, ValueError):
        return None


def _i(row, key):
    v = _f(row, key)
    return int(v) if v is not None else None


# 방향 라벨용 거리 이력. 순간 접근속도(closing)는 저속에서 GPS 잡음으로 거리와 반대로
# 뒤집힌다(2026-08-19 실기: 거리 7→4.6 m 다가오는데 화면은 '멀어짐'). 화면 방향은
# 사람이 보는 거리 자체의 최근 변화로 정한다 — 거리가 줄면 무조건 접근중.
_DIST_HIST = []
_APPROACH_WINDOW_S = 4.0
_APPROACH_DELTA_M = 0.4


def _approach_label(t, dist):
    if t is None or dist is None:
        return "정지"
    _DIST_HIST.append((t, dist))
    cutoff = t - _APPROACH_WINDOW_S
    while len(_DIST_HIST) > 1 and _DIST_HIST[0][0] < cutoff:
        _DIST_HIST.pop(0)
    delta = _DIST_HIST[0][1] - dist  # +면 그만큼 가까워짐
    if delta > _APPROACH_DELTA_M:
        return "접근중"
    if delta < -_APPROACH_DELTA_M:
        return "멀어짐"
    return "정지"


def _read(row, rssi, now, stale_s):
    """공통: 한 판정에서 화면에 쓸 값들을 꺼낸다."""
    level = _i(row, "effective_level") if row else None
    t = _f(row, "pc_time") if row else None
    age = (now - t) if t is not None else None
    if level is None or age is None or age > stale_s:
        return None
    d = dict(
        level=level,
        dist=_f(row, "distance_m"),
        ttc=_f(row, "ttc_s"),
        closing=_f(row, "closing_mps") or 0.0,
        cane_nr=_i(row, "cane_node_risk"),
        veh_nr=_i(row, "veh_node_risk"),
        cane_gv=_i(row, "cane_gps_valid"),
        veh_gv=_i(row, "veh_gps_valid"),
        cane_sp=_f(row, "cane_speed_mps"),
        veh_sp=_f(row, "veh_speed_mps"),
        reason=row.get("reason") or "",
        c_rssi=rssi.get("cane", {}).get("rssi"),
        v_rssi=rssi.get("vehicle", {}).get("rssi"),
    )
    d["approach"] = _approach_label(t, d["dist"])
    d["ttc_s"] = f"{d['ttc']:.1f}s" if (d["ttc"] is not None and d["ttc"] < 999) else "--"
    d["mismatch"] = ((d["cane_nr"] is not None and d["cane_nr"] != level)
                     or (d["veh_nr"] is not None and d["veh_nr"] != level))
    return d


def _gv(x):
    return "✓" if x == 1 else "✗"


def line(row, rssi, now, stale_s):
    """로그 한 줄(색 배지 포함)."""
    ts = time.strftime("%H:%M:%S")
    d = _read(row, rssi, now, stale_s)
    if d is None:
        label, bg, fg = WAIT
        badge = f"\033[{bg};{fg};1m  {label}  \033[0m"
        note = "젯슨 신호 없음 — 브리지·엔진 확인" if row is None else "신호 끊김"
        return f"{DIM}{ts}{RESET} {badge}  {note}"
    label, bg, fg = LEVELS.get(d["level"], LEVELS[0])
    badge = f"\033[{bg};{fg};1m  {label} · LV{d['level']}  \033[0m"
    rsu = f"{RED}{d['level']}{RESET}" if d["mismatch"] else f"{d['level']}"
    dist_s = f"{d['dist']:.1f}m" if d["dist"] is not None else "--"
    return (f"{DIM}{ts}{RESET} {badge}  거리 {dist_s} TTC {d['ttc_s']} {d['approach']}"
            f"  {DIM}|{RESET} RSU {rsu} 지팡이 {d['cane_nr']} 차량 {d['veh_nr']}"
            f"  {DIM}|{RESET} GPS {_gv(d['cane_gv'])}{_gv(d['veh_gv'])}"
            f" RSSI {d['c_rssi'] if d['c_rssi'] is not None else '--'}/{d['v_rssi'] if d['v_rssi'] is not None else '--'}")


def big_frame(row, rssi, now, stale_s, scroll=False):
    """큰 색 박스 프레임. scroll=False 면 제자리 갱신(전체화면), True 면 아래로 쌓인다."""
    cols, rows_n = shutil.get_terminal_size((80, 24))
    d = _read(row, rssi, now, stale_s)
    if d is None:
        label, bg, fg = WAIT
        level = None
        panel = [f"  {'젯슨 신호 없음 — 브리지 연결·엔진 확인' if row is None else '신호 끊김'}"]
    else:
        level = d["level"]
        label, bg, fg = LEVELS.get(level, LEVELS[0])
        rsu = f"{RED}{level}  ← 전달중{RESET}" if d["mismatch"] else f"\033[1m{level}\033[0m"
        dist = f"{d['dist']:.1f} m" if d["dist"] is not None else "--"
        panel = [
            f"  RSU 계산(젯슨) {rsu}   {DIM}(신뢰 O · {d['reason']}){RESET}",
            f"  지팡이 수신 risk {d['cane_nr']}     차량 수신 risk {d['veh_nr']}",
            "",
            f"  거리 {dist}   TTC {d['ttc_s']}   {d['approach']}",
            "",
            f"  지팡이  GPS{_gv(d['cane_gv'])}  {('%.2f' % d['cane_sp']) if d['cane_sp'] is not None else '--'} m/s  "
            f"RSSI {d['c_rssi'] if d['c_rssi'] is not None else '--'}",
            f"  차량    GPS{_gv(d['veh_gv'])}  {('%.2f' % d['veh_sp']) if d['veh_sp'] is not None else '--'} m/s  "
            f"RSSI {d['v_rssi'] if d['v_rssi'] is not None else '--'}",
        ]

    band = f"\033[{bg};{fg};1m"

    def vlen(s):
        return sum(1 if ord(c) < 128 else 2 for c in s)

    def ctr(s):
        p = max(0, (cols - vlen(s)) // 2)
        return " " * p + s + " " * max(0, cols - p - vlen(s))

    # 쌓임 모드는 박스를 낮게(6줄) 해서 여러 개가 화면에 들어오게, 제자리 모드는
    # 화면 절반을 채워 크게.
    box_h = 5 if scroll else max(6, int(rows_n * 0.5))
    lbl_i, lvl_i = box_h // 2 - 1, box_h // 2 + 1
    out = []
    for i in range(box_h):
        if i == lbl_i:
            out.append(band + ctr(label) + RESET)
        elif i == lvl_i:
            out.append(band + ctr(f"LEVEL {level if level is not None else '-'}") + RESET)
        else:
            out.append(band + " " * cols + RESET)
    out.append("")
    out.extend(panel)
    out.append(f"  {DIM}{time.strftime('%H:%M:%S')}  ·  촬영용 관측(경보 경로와 무관){RESET}")
    if scroll:
        return "\n".join(out) + "\n"                                   # 아래로 쌓임(위로 스크롤)
    return HOME + "".join(r + CLR_EOL + "\n" for r in out) + CLR_DOWN  # 제자리 갱신


def parse_args():
    p = argparse.ArgumentParser(description="촬영용 위험도 표시")
    default_logs = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
    p.add_argument("--logs", default=default_logs, help="risk_tx CSV / raw 로그 폴더")
    p.add_argument("--big", action="store_true", help="큰 색 박스(촬영용)")
    p.add_argument("--scroll", action="store_true", help="--big 을 아래로 쌓이게(위로 스크롤). 없으면 전체화면 고정")
    p.add_argument("--interval", type=float, default=1.0, help="갱신 주기 초")
    p.add_argument("--stale", type=float, default=3.0)
    p.add_argument("--once", action="store_true")
    return p.parse_args()


def _pull(args):
    csv_path = _latest(args.logs, "risk_tx_*.csv")
    raw_path = _latest(args.logs, "raw_*.log")
    row = last_row(csv_path) if csv_path else None
    return row, latest_rssi(raw_path)


def main():
    args = parse_args()
    if args.big:
        use_alt = not args.scroll and not args.once   # 쌓임 모드는 전체화면 독점 안 함
        if use_alt:
            print(ENTER_ALT, end="", flush=True)
        try:
            while True:
                row, rssi = _pull(args)
                print(big_frame(row, rssi, time.time(), args.stale, scroll=args.scroll),
                      end="", flush=True)
                if args.once:
                    if not args.scroll:
                        print()
                    break
                time.sleep(args.interval)
        except KeyboardInterrupt:
            pass
        finally:
            if use_alt:
                print(EXIT_ALT, end="", flush=True)
            print(RESET + "[종료]", flush=True)
    else:
        print(f"{DIM}[watch_risk] 위험도 로그 · 매 {args.interval:g}초 한 줄 · 위로 스크롤 가능 · Ctrl-C 종료{RESET}")
        try:
            while True:
                row, rssi = _pull(args)
                print(line(row, rssi, time.time(), args.stale), flush=True)
                if args.once:
                    break
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print(RESET + "[종료]")


if __name__ == "__main__":
    main()
