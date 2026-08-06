#!/usr/bin/env python3
"""
서버 자동 정리 (디스크 보호)

1. Jetson이 추론 결과까지 업로드한 시나리오 폴더는 삭제한다.
   - 원본 데이터는 Jetson(incoming_data/)에 그대로 보관되므로 손실 없음
   - 최신 KEEP_RECENT개는 안전을 위해 무조건 보존
2. 하루 지난 결과 CSV는 gzip 압축한다 (약 85% 절약).
   - 결과 원본도 Jetson(inference_results/)에 보관되어 있음

cron이 10분마다 실행한다.
"""
import gzip
import shutil
import time
from pathlib import Path

BASE = Path.home() / "SUMO_project"
GEN = BASE / "generated_data"
RES = BASE / "results"
KEEP_RECENT = 30          # 최신 시나리오는 항상 보존
GZIP_AFTER_HOURS = 0.1      # 이 시간 지난 결과 CSV는 압축 (pandas는 .gz도 바로 읽음)
DELETE_GZ_AFTER_DAYS = 14  # 이 기간 지난 압축 결과는 삭제 (원본은 Jetson에 보관됨)

deleted = 0
if GEN.exists():
    dirs = sorted(GEN.glob("scenario_*"))
    for d in dirs[:-KEEP_RECENT]:
        has_result = (RES / f"{d.name}_result.csv").exists() or \
                     (RES / f"{d.name}_result.csv.gz").exists()
        if has_result and (d / "DONE").exists():
            shutil.rmtree(d, ignore_errors=True)
            deleted += 1

gzipped = 0
now = time.time()
if RES.exists():
    for f in RES.glob("*_result.csv"):
        if now - f.stat().st_mtime > GZIP_AFTER_HOURS * 3600:
            with open(f, "rb") as src, gzip.open(f"{f}.gz", "wb") as dst:
                shutil.copyfileobj(src, dst)
            f.unlink()
            gzipped += 1

removed_gz = 0
if RES.exists():
    for f in RES.glob("*_result.csv.gz"):
        if now - f.stat().st_mtime > DELETE_GZ_AFTER_DAYS * 86400:
            f.unlink()
            removed_gz += 1

if deleted or gzipped or removed_gz:
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} "
          f"시나리오 {deleted}개 삭제, 결과 {gzipped}개 압축, "
          f"오래된 결과 {removed_gz}개 삭제")
