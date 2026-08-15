#!/usr/bin/env python3
"""지팡이/차량의 현재 위치와 위험 판정을 위험지도 서버로 실시간 스트리밍.

기존 upload_events.py와 목적이 정반대라 별도 경로다:

  upload_events.py   위험 이벤트(레벨 1+)를 남김없이 아카이브. 1분 타이머로
                     돌고, 오프라인이면 기억했다가 나중에 밀린 것을 올린다.
  stream_live.py     "지금 어디에 있는가"를 화면에 그리기 위한 현재 상태.
                     0.5초마다 돌고, 못 보낸 것은 버린다 - 3초 늦게 도착한
                     위치는 지도에 찍을 값이 아니라 거짓말이기 때문이다.

step8은 건드리지 않는다. 같은 CSV를 각자의 오프셋으로 읽을 뿐이라
이 프로세스가 죽어도 이벤트 업로드와 RSU 다운링크는 그대로 돈다.

설정은 upload_events.py와 같은 환경 변수(RISKMAP_URL, RISKMAP_API_KEY)를 쓴다.

서버 없이 페이로드만 확인:
    python3 stream_live.py --dry-run --logs-dir ../logs
"""
import argparse
import csv
import json
import os
import socket
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_INTERVAL_S = 0.5
# step8의 heartbeat는 1초다. 그 10배 동안 새 행이 없다면 엔진이 멈춘 것이므로,
# 마지막 위치를 계속 보내 화면을 살아있는 것처럼 보이게 하면 안 된다.
DEFAULT_MAX_AGE_S = 10.0
SEND_TIMEOUT_S = 3.0


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class CsvTail:
    """CSV 끝에 새로 붙은 행만 읽는다.

    헤더는 처음 한 번만 읽고 이후에는 오프셋부터 seek한다. 0.5초마다 도는
    루프라 매번 파일 전체를 읽으면 안 된다 - 하루치 로그는 수십 MB가 된다.
    쓰다 만 행(개행 없음)은 step8이 마저 쓸 때까지 소비하지 않는다.
    """

    def __init__(self, path):
        self.path = Path(path)
        self.fields = None
        self.offset = 0

    def new_rows(self):
        with self.path.open("rb") as handle:
            if self.fields is None:
                header = handle.readline()
                if not header.endswith(b"\n"):
                    return []
                self.fields = next(csv.reader([header.decode("utf-8")]))
                self.offset = handle.tell()
            handle.seek(self.offset)
            chunk = handle.read()

        last_newline = chunk.rfind(b"\n")
        if last_newline < 0:
            return []
        text = chunk[: last_newline + 1].decode("utf-8")
        self.offset += last_newline + 1
        return [
            dict(zip(self.fields, values))
            for values in csv.reader(text.splitlines())
            if values
        ]


def _node(row, prefix):
    """한 노드의 위치/속도/방위. 좌표가 없으면 None으로 비운다.

    (0, 0)은 실제 좌표가 아니라 펌웨어의 "픽스 없음" 표식이다. 그대로 보내면
    화면이 기니 만 앞바다에 마커를 찍는다. gps_valid는 따로 실어보내서
    화면이 "위치를 아는데 정확도가 낮음"과 "위치를 모름"을 구분하게 한다.
    """
    lat = _to_float(row.get(f"{prefix}_lat"))
    lng = _to_float(row.get(f"{prefix}_lng"))
    if lat is None or lng is None or (lat == 0.0 and lng == 0.0):
        lat = lng = None
    return {
        "lat": lat,
        "lng": lng,
        "speed_mps": _to_float(row.get(f"{prefix}_speed_mps")),
        "heading_deg": _to_float(row.get(f"{prefix}_heading_deg")),
        "gps_valid": _to_float(row.get(f"{prefix}_gps_valid")) == 1,
    }


def snapshot_from_row(row, device_id):
    """step8 CSV 한 행 -> /api/live 페이로드. 시각이 없으면 None."""
    row_time = _to_float(row.get("pc_time"))
    if row_time is None:
        return None
    vehicle = _node(row, "veh")
    # 차량 1인칭 화면은 차량 GPS(자주 무효) 대신 지팡이 기준 상대좌표를 쓴다.
    # 이 값을 부호 반전 후 vehicle.heading_deg 로 회전하면 차량 전방 기준 배치가 된다.
    vehicle["rel_east_m"] = _to_float(row.get("rel_east_m"))
    vehicle["rel_north_m"] = _to_float(row.get("rel_north_m"))
    return {
        "device_id": device_id,
        "ts": datetime.fromtimestamp(row_time, tz=timezone.utc).isoformat(),
        "cane": _node(row, "cane"),
        "vehicle": vehicle,
        "risk": {
            "level": int(_to_float(row.get("effective_level")) or 0),
            "distance_m": _to_float(row.get("distance_m")),
            "ttc_s": _to_float(row.get("ttc_s")),
            "closing_mps": _to_float(row.get("closing_mps")),
            "source": row.get("level_source") or None,
        },
    }


class LiveStreamer:
    """마지막 스냅샷을 들고 있다가 주기마다 내보낸다.

    step8은 1초 heartbeat라 0.5초 주기의 절반은 새 행이 없다. 그때 아무것도
    안 보내면 화면은 갱신이 없는 것과 신호가 끊긴 것을 구분할 수 없으므로,
    직전 스냅샷을 age_s와 함께 다시 보낸다.
    """

    def __init__(self, device_id, max_age_s=DEFAULT_MAX_AGE_S):
        self.device_id = device_id
        self.max_age_s = max_age_s
        self.snapshot = None
        self.row_time = None

    def observe(self, rows):
        # 실시간 화면에 지나간 위치는 의미가 없다. 밀린 행은 버리고 최신만 쓴다.
        for row in reversed(rows):
            snapshot = snapshot_from_row(row, self.device_id)
            if snapshot is not None:
                self.snapshot = snapshot
                self.row_time = _to_float(row.get("pc_time"))
                return

    def payload(self, now):
        if self.snapshot is None:
            return None
        age = now - self.row_time
        if age > self.max_age_s:
            return None
        return {**self.snapshot, "age_s": round(age, 2)}


def http_sender(base_url, api_key):
    endpoint = base_url.rstrip("/") + "/api/live"

    def send(payload):
        request = urllib.request.Request(
            endpoint,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json", "X-API-Key": api_key},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=SEND_TIMEOUT_S) as response:
                return 200 <= response.status < 300
        except (urllib.error.URLError, OSError):
            # 실시간 전송은 재시도하지 않는다. 다음 주기에 더 새로운 값이 간다.
            return False

    return send


def stdout_sender(payload):
    print(json.dumps(payload, ensure_ascii=False), flush=True)
    return True


def latest_csv(logs_dir):
    """가장 최근에 쓰인 CSV. step8은 세션마다 새 파일을 만든다.

    파일명이 아니라 수정 시각으로 고르는 이유는 logs/에 실험용 CSV가 섞여
    있어서다. 지금 기록 중인 파일이 곧 지금 실행 중인 세션이다.
    """
    files = list(Path(logs_dir).glob("*.csv"))
    if not files:
        return None
    return max(files, key=lambda path: path.stat().st_mtime)


def parse_args():
    parser = argparse.ArgumentParser(
        description="지팡이/차량 현재 상태를 위험지도 서버로 실시간 전송"
    )
    parser.add_argument("--logs-dir", default=str(Path(__file__).parent.parent / "logs"))
    parser.add_argument("--csv", default=None, help="특정 CSV 고정 (기본: logs 중 최신)")
    parser.add_argument("--url", default=os.environ.get("RISKMAP_URL", ""))
    parser.add_argument("--api-key", default=os.environ.get("RISKMAP_API_KEY", ""))
    parser.add_argument("--device-id", default=socket.gethostname())
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S)
    parser.add_argument("--max-age-s", type=float, default=DEFAULT_MAX_AGE_S)
    parser.add_argument("--dry-run", action="store_true",
                        help="전송 대신 페이로드를 stdout에 출력")
    parser.add_argument("--once", action="store_true",
                        help="1건만 보내고 종료 (서버 연결 확인용)")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.dry_run and (not args.url or not args.api_key):
        print("[SKIP] RISKMAP_URL / RISKMAP_API_KEY 미설정 -> 전송 안 함", file=sys.stderr)
        return 0

    sender = stdout_sender if args.dry_run else http_sender(args.url, args.api_key)
    streamer = LiveStreamer(args.device_id, max_age_s=args.max_age_s)
    fixed = Path(args.csv) if args.csv else None
    print(f"[INFO] device_id={args.device_id} interval={args.interval}s "
          f"max_age={args.max_age_s}s dry_run={args.dry_run}", file=sys.stderr)

    tail = None
    fail_streak = 0
    try:
        while True:
            path = fixed or latest_csv(args.logs_dir)
            if path is not None:
                if tail is None or tail.path != path:
                    print(f"[INFO] 로그 파일: {path}", file=sys.stderr)
                    tail = CsvTail(path)
                try:
                    streamer.observe(tail.new_rows())
                except OSError as ex:
                    print(f"[WARN] 로그 읽기 실패: {ex}", file=sys.stderr)

            payload = streamer.payload(time.time())
            sent = payload is not None and sender(payload)
            if payload is None:
                fail_streak = 0
            elif sent:
                if fail_streak:
                    print(f"[OK] 전송 재개 (연속 실패 {fail_streak}건)", file=sys.stderr)
                fail_streak = 0
            else:
                fail_streak += 1
                # 0.5초마다 같은 경고를 찍으면 로그가 순식간에 덮인다.
                if fail_streak == 1:
                    print("[WARN] 전송 실패 (오프라인?) - 다음 주기에 최신 값으로 재시도",
                          file=sys.stderr)

            if args.once:
                # stdout은 --dry-run 페이로드 전용이다. 진단 메시지가 섞이면
                # `stream_live.py --dry-run | jq` 같은 파이프가 깨진다.
                if payload is None:
                    print("[FAIL] 보낼 상태가 없음 (step8이 도는 중인지 확인)",
                          file=sys.stderr)
                    return 1
                print("[OK] 1건 전송됨" if sent else "[FAIL] 전송 실패", file=sys.stderr)
                return 0 if sent else 1
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[INFO] stopped", file=sys.stderr)
        return 0


if __name__ == "__main__":
    sys.exit(main())
