#!/usr/bin/env python3
"""위험 이벤트를 위험지도 서버(riskmap-api)로 업로드.

systemd 타이머(v2x-event-upload.timer)가 1분마다 실행한다.
step8이 남기는 logs/*.csv에서 위험 레벨 1 이상 행만 골라
POST /api/events 로 보낸다. 읽은 위치(오프셋)를 상태 파일에 기억하므로
새로 추가된 행만 처리하고, 오프라인이면 다음 주기에 이어서 재시도한다.
event_uid가 행마다 고정이라 재전송해도 서버가 중복 저장하지 않는다.

설정은 환경 변수(RISKMAP_URL, RISKMAP_API_KEY)로 받는다.
젯슨에서는 /etc/default/v2x-riskmap 에 넣고 systemd가 읽는다.

수동 실행 예:
    RISKMAP_URL=https://... RISKMAP_API_KEY=... python3 upload_events.py \
        --logs-dir ../logs --state-file ../upload_events_state.json

서버 연결 확인용 테스트 이벤트 1건 전송:
    RISKMAP_URL=... RISKMAP_API_KEY=... python3 upload_events.py --self-test
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

DEFAULT_MIN_INTERVAL_S = 10.0  # 같은 레벨 지속 중 전송 간격
SELF_TEST_LAT = 37.4963       # 숭실대 근처 (지도 도로 스냅 범위 안)
SELF_TEST_LNG = 126.9575


def read_new_rows(path, offset):
    """offset 이후의 완성된 행만 (row_dict, 행 끝 오프셋) 목록으로 반환.

    헤더는 매번 파일 첫 줄에서 읽는다. 쓰다 만 행(개행 없음)은 소비하지 않는다.
    """
    data = Path(path).read_bytes()
    header_end = data.find(b"\n")
    if header_end < 0:
        return [], offset
    fields = next(csv.reader([data[: header_end + 1].decode("utf-8")]))
    pos = max(offset, header_end + 1)
    rows = []
    while True:
        line_end = data.find(b"\n", pos)
        if line_end < 0:
            break
        line = data[pos : line_end + 1].decode("utf-8")
        values = next(csv.reader([line]), None)
        if values:
            rows.append((dict(zip(fields, values)), line_end + 1))
        pos = line_end + 1
    return rows, pos


def _to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def build_event(row, device_id):
    """CSV 행 -> /api/events 페이로드. 좌표가 없으면 None."""
    lat = _to_float(row.get("cane_lat"))
    lng = _to_float(row.get("cane_lng"))
    if lat is None or lng is None:
        return None
    if _to_float(row.get("cane_gps_valid")) != 1:
        return None
    pc_time = _to_float(row.get("pc_time"))
    if pc_time is None:
        return None
    return {
        "event_uid": f"{device_id}-{row['pc_time']}",
        "source": "live",
        "lat": lat,
        "lng": lng,
        "risk": int(_to_float(row.get("effective_level")) or 0),
        "device_id": device_id,
        "ttc": _to_float(row.get("ttc_s")),
        "distance_m": _to_float(row.get("distance_m")),
        "occurred_at": datetime.fromtimestamp(pc_time, tz=timezone.utc).isoformat(),
    }


def process_file(path, file_state, sender, min_interval=DEFAULT_MIN_INTERVAL_S,
                 device_id="jetson"):
    """한 CSV 파일의 새 행을 선별·전송. 전송 실패 시 False (오프셋은 성공 지점까지만).

    file_state: {"offset": int, "last_sent_t": float, "last_level": int} (없으면 채움)
    """
    rows, empty_offset = read_new_rows(path, file_state.get("offset", 0))
    if not rows:
        file_state["offset"] = empty_offset
        return True
    last_sent_t = file_state.get("last_sent_t")
    last_level = file_state.get("last_level", 0)
    for row, offset_after in rows:
        level = int(_to_float(row.get("effective_level")) or 0)
        if level <= 0:
            last_level = 0
        else:
            event = build_event(row, device_id)
            if event is not None:
                row_t = _to_float(row.get("pc_time")) or 0.0
                rise = level > last_level
                due = last_sent_t is None or row_t - last_sent_t >= min_interval
                if rise or due:
                    if not sender(event):
                        # 실패한 행 앞에서 멈춤 -> 다음 주기에 이 행부터 재시도
                        file_state["last_sent_t"] = last_sent_t
                        file_state["last_level"] = last_level
                        return False
                    last_sent_t = row_t
                    last_level = level
        file_state["offset"] = offset_after
    file_state["last_sent_t"] = last_sent_t
    file_state["last_level"] = last_level
    return True


def load_state(state_path):
    try:
        return json.loads(Path(state_path).read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {"files": {}}


def save_state(state_path, state):
    path = Path(state_path)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=1), encoding="utf-8")
    tmp.replace(path)


def http_sender(base_url, api_key):
    endpoint = base_url.rstrip("/") + "/api/events"

    def send(event):
        request = urllib.request.Request(
            endpoint,
            data=json.dumps(event).encode("utf-8"),
            headers={"Content-Type": "application/json", "X-API-Key": api_key},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=15) as response:
                return 200 <= response.status < 300
        except urllib.error.HTTPError as ex:
            print(f"[WARN] 서버 응답 오류 {ex.code}: {ex.reason}", flush=True)
            return False
        except (urllib.error.URLError, OSError) as ex:
            print(f"[WARN] 전송 실패 (오프라인?): {ex}", flush=True)
            return False

    return send


def run_self_test(sender, device_id):
    event = {
        "event_uid": f"test-{device_id}-{int(time.time())}",
        "source": "live",
        "lat": SELF_TEST_LAT,
        "lng": SELF_TEST_LNG,
        "risk": 2,
        "device_id": device_id,
        "ttc": 3.5,
        "distance_m": 8.0,
        "occurred_at": datetime.now(timezone.utc).isoformat(),
    }
    if sender(event):
        print(f"[OK] 테스트 이벤트 전송됨: {event['event_uid']}")
        return 0
    print("[FAIL] 테스트 이벤트 전송 실패")
    return 1


def parse_args():
    parser = argparse.ArgumentParser(description="위험 이벤트를 위험지도 서버로 업로드")
    parser.add_argument("--logs-dir", default=str(Path(__file__).parent.parent / "logs"))
    parser.add_argument(
        "--state-file",
        default=str(Path(__file__).parent.parent / "upload_events_state.json"),
    )
    parser.add_argument("--url", default=os.environ.get("RISKMAP_URL", ""))
    parser.add_argument("--api-key", default=os.environ.get("RISKMAP_API_KEY", ""))
    parser.add_argument("--min-interval", type=float, default=DEFAULT_MIN_INTERVAL_S)
    parser.add_argument("--device-id", default=socket.gethostname())
    parser.add_argument("--self-test", action="store_true",
                        help="테스트 이벤트 1건만 보내고 종료")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.url or not args.api_key:
        print("[SKIP] RISKMAP_URL / RISKMAP_API_KEY 미설정 -> 업로드 안 함")
        return 0
    sender = http_sender(args.url, args.api_key)
    if args.self_test:
        return run_self_test(sender, args.device_id)

    logs_dir = Path(args.logs_dir)
    if not logs_dir.is_dir():
        print(f"[SKIP] 로그 폴더 없음: {logs_dir}")
        return 0
    state = load_state(args.state_file)
    files_state = state.setdefault("files", {})
    csv_files = sorted(logs_dir.glob("*.csv"))
    # 사라진 파일의 상태는 정리
    names = {f.name for f in csv_files}
    for stale in [k for k in files_state if k not in names]:
        del files_state[stale]

    sent_before = 0
    ok = True
    for csv_path in csv_files:
        file_state = files_state.setdefault(csv_path.name, {})
        counting = _CountingSender(sender)
        ok = process_file(csv_path, file_state, counting,
                          min_interval=args.min_interval, device_id=args.device_id)
        sent_before += counting.count
        if not ok:
            break
    save_state(args.state_file, state)
    if ok:
        print(f"[OK] 이벤트 {sent_before}건 전송, 파일 {len(csv_files)}개 확인")
        return 0
    print(f"[WARN] 전송 중단 (성공 {sent_before}건) -> 다음 주기에 재시도")
    return 0  # 타이머 재시도 구조라 실패도 정상 종료


class _CountingSender:
    def __init__(self, sender):
        self.sender = sender
        self.count = 0

    def __call__(self, event):
        ok = self.sender(event)
        if ok:
            self.count += 1
        return ok


if __name__ == "__main__":
    sys.exit(main())
