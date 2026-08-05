import os, hmac, csv, io, datetime
from typing import Optional
from fastapi import FastAPI, Header, HTTPException
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from google.cloud import storage
import psycopg
from psycopg_pool import ConnectionPool

import roads  # [변경] 실제 도로망 스냅 모듈 (roads.json 필요)

app = FastAPI(title="V2X Risk Map")

_pool = None

def _get_pool():
    # 커넥션 풀: 매 요청마다 새로 접속하지 않고 미리 만든 연결을 재사용
    global _pool
    if _pool is None:
        inst = os.environ["INSTANCE_CONNECTION_NAME"]
        conninfo = (f"host=/cloudsql/{inst} "
                    f"dbname={os.environ.get('DB_NAME', 'riskmap')} "
                    f"user={os.environ.get('DB_USER', 'riskmap_app')} "
                    f"password={os.environ['DB_PASS']}")
        _pool = ConnectionPool(conninfo, min_size=1, max_size=4, timeout=20,
                               kwargs={"autocommit": False})
    return _pool

def get_conn():
    # 풀에서 연결을 꺼내고, close() 호출 시 풀로 반환되도록 감싼다.
    # (기존 코드가 conn = get_conn() / conn.close() 방식이라 이 형태를 유지)
    cm = _get_pool().connection()
    conn = cm.__enter__()
    orig_close = conn.close
    def _release(*a, **k):
        try:
            cm.__exit__(None, None, None)
        except Exception:
            orig_close()
    conn.close = _release
    return conn

def require_key(x_api_key):
    expected = os.environ.get("API_KEY", "")
    if not expected or not hmac.compare_digest(x_api_key or "", expected):
        raise HTTPException(status_code=401, detail="invalid api key")

def edge_of(lat, lng):
    # [변경] 기존: 격자 id ("g%.3f_%.3f") -> 이제: 가장 가까운 실제 도로 엣지 id
    # 도로망에서 너무 멀면(300m+) None 저장 -> 집계/지도에서 제외됨
    return roads.nearest_edge_id(lat, lng)

def reject_if_off_road(lat, lng):
    """좌표를 두 번 거른다. 통과하면 확정된 엣지 id를 돌려준다."""
    center_km = roads.distance_from_center_km(lat, lng)
    if center_km > roads.MAX_CENTER_DISTANCE_KM:
        raise HTTPException(status_code=400, detail=(
            f"서비스 지역 밖입니다. 기준 좌표에서 {center_km:.1f}km "
            f"(허용 {roads.MAX_CENTER_DISTANCE_KM}km)"))
    edge_id, dist_m = roads.nearest_edge(lat, lng)
    if edge_id is None or dist_m > roads.MAX_SNAP_DISTANCE_M:
        raise HTTPException(status_code=400, detail=(
            f"가까운 도로가 없습니다. 최단 거리 {dist_m:.1f}m "
            f"(허용 {roads.MAX_SNAP_DISTANCE_M}m)"))
    return edge_id

def fetch_stats(cur, source):
    cur.execute("SELECT edge_id,p1_lat,p1_lng,p2_lat,p2_lng,event_count,avg_risk,grade,avg_ttc,updated_at "
                "FROM road_segment_stats WHERE source=%s AND p1_lat IS NOT NULL ORDER BY edge_id", (source,))
    rows = {}
    for r in cur.fetchall():
        rows[r[0]] = {"edge_id": r[0], "points": [[r[1], r[2]], [r[3], r[4]]], "events": r[5],
                      "avg": round(r[6], 2) if r[6] is not None else None, "grade": r[7],
                      "ttc": round(r[8], 2) if r[8] is not None else None,
                      "updated_at": r[9].isoformat() if r[9] else None}
    return rows

@app.get("/api/health")
def health():
    return {"status": "ok"}

@app.get("/api/segments")
def segments(source: str = "live"):
    try:
        conn = get_conn()
        try:
            cur = conn.cursor()
            if source == "combined":
                live, sumo = fetch_stats(cur, "live"), fetch_stats(cur, "sumo")
                out = []
                for e in sorted(set(live) | set(sumo)):
                    l, s = live.get(e), sumo.get(e)
                    lg = l["grade"] if l else 0
                    sg = s["grade"] if s else 0
                    agr = "both" if lg >= 1 and sg >= 1 else ("live_only" if lg >= 1 else "sumo_only")
                    base = l if l else s
                    out.append({"edge_id": e, "points": base["points"], "grade": max(lg, sg),
                                "events": (l["events"] if l else 0) + (s["events"] if s else 0),
                                "avg": None, "ttc": None, "source": "combined",
                                "agreement": agr, "updated_at": base["updated_at"]})
                return JSONResponse(out)
            rows = fetch_stats(cur, source)
            out = []
            for e in sorted(rows):
                d = rows[e]; d["source"] = source; d["agreement"] = None; out.append(d)
            return JSONResponse(out)
        finally:
            conn.close()
    except Exception as ex:
        print("DB error:", ex); return JSONResponse([])

@app.get("/api/zones")
def zones():
    try:
        conn = get_conn()
        try:
            cur = conn.cursor()
            cur.execute("SELECT zone_id,zone_name,zone_type,center_lat,center_lng,radius_m,base_risk "
                        "FROM zones WHERE center_lat IS NOT NULL")
            return JSONResponse([{"zone_id": r[0], "zone_name": r[1], "zone_type": r[2], "lat": r[3],
                                  "lng": r[4], "radius": r[5], "base_risk": r[6]} for r in cur.fetchall()])
        finally:
            conn.close()
    except Exception as ex:
        print("DB error:", ex); return JSONResponse([])

@app.get("/api/status")
def status():
    try:
        conn = get_conn()
        try:
            cur = conn.cursor()
            cur.execute("SELECT source,count(*),max(updated_at) FROM road_segment_stats GROUP BY source")
            counts, last = {}, None
            for r in cur.fetchall():
                counts[r[0]] = r[1]
                if r[2] and (last is None or r[2] > last): last = r[2]
            cur.execute("SELECT source,count(*) FROM events WHERE edge_id IS NULL GROUP BY source")
            unsnapped = {r[0]: r[1] for r in cur.fetchall()}
            return JSONResponse({"counts": counts, "unsnapped": unsnapped,
                                 "last_updated": last.isoformat() if last else None})
        finally:
            conn.close()
    except Exception as ex:
        print("DB error:", ex); return JSONResponse({"counts": {}, "unsnapped": {}, "last_updated": None})

class EventIn(BaseModel):
    event_uid: str
    source: str
    lat: float
    lng: float
    risk: int
    device_id: Optional[str] = None
    scenario_id: Optional[str] = None
    ttc: Optional[float] = None
    distance_m: Optional[float] = None
    zone_id: Optional[str] = None
    occurred_at: Optional[str] = None

@app.post("/api/events")
def add_event(ev: EventIn, x_api_key: str = Header(default="")):
    require_key(x_api_key)
    edge_id = reject_if_off_road(ev.lat, ev.lng)
    ttc = None if (ev.ttc is not None and ev.ttc >= 9999) else ev.ttc
    conn = get_conn()
    try:
        cur = conn.cursor()
        cur.execute("INSERT INTO events (event_uid,source,device_id,scenario_id,lat,lng,edge_id,"
                    "risk,ttc,distance_m,zone_id,occurred_at) "
                    "VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,COALESCE(%s::timestamptz,now())) "
                    "ON CONFLICT (event_uid) DO NOTHING",
                    (ev.event_uid, ev.source, ev.device_id, ev.scenario_id, ev.lat, ev.lng,
                     edge_id, ev.risk, ttc, ev.distance_m, ev.zone_id, ev.occurred_at))
        inserted = cur.rowcount; conn.commit()
        return {"ok": True, "inserted": inserted}
    finally:
        conn.close()

class ImportIn(BaseModel):
    gcs_uri: str
    source: str
    scenario_id: Optional[str] = None

@app.post("/api/import")
def import_csv(body: ImportIn, x_api_key: str = Header(default="")):
    require_key(x_api_key)
    conn = get_conn()
    try:
        cur = conn.cursor()
        cur.execute("SELECT 1 FROM ingest_log WHERE gcs_uri=%s", (body.gcs_uri,))
        if cur.fetchone():
            return {"ok": False, "reason": "already imported"}
        started = datetime.datetime.now(datetime.timezone.utc)
        bucket_name, _, blob_path = body.gcs_uri[5:].partition("/")
        data = storage.Client().bucket(bucket_name).blob(blob_path).download_as_text()
        row_count = inserted = 0
        for row in csv.DictReader(io.StringIO(data)):
            row_count += 1
            lat = float(row["lat"]); lng = float(row["lng"]); risk = int(float(row["risk"]))
            t = row.get("ttc"); ttc = None if not t or float(t) >= 9999 else float(t)
            d = row.get("distance_m"); dist = float(d) if d else None
            uid = row.get("event_uid") or ((body.scenario_id or "row") + "-" + str(row_count))
            cur.execute("INSERT INTO events (event_uid,source,scenario_id,lat,lng,edge_id,risk,ttc,"
                        "distance_m,zone_id,occurred_at) "
                        "VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,COALESCE(%s::timestamptz,now())) "
                        "ON CONFLICT (event_uid) DO NOTHING",
                        (uid, body.source, body.scenario_id, lat, lng, edge_of(lat, lng), risk, ttc,
                         dist, row.get("zone_id") or None, row.get("occurred_at") or None))
            inserted += cur.rowcount
        cur.execute("INSERT INTO ingest_log (gcs_uri,source,scenario_id,row_count,inserted_count,"
                    "skipped_count,status,started_at,finished_at) VALUES (%s,%s,%s,%s,%s,%s,'ok',%s,now())",
                    (body.gcs_uri, body.source, body.scenario_id, row_count, inserted, row_count-inserted, started))
        conn.commit()
        return {"ok": True, "row_count": row_count, "inserted": inserted, "skipped": row_count-inserted}
    finally:
        conn.close()

@app.post("/api/admin/refresh-stats")
def refresh_stats(x_api_key: str = Header(default="")):
    require_key(x_api_key)
    conn = get_conn()
    try:
        cur = conn.cursor()
        cur.execute("DELETE FROM road_segment_stats")
        cur.execute("SELECT edge_id,source,count(*),avg(risk),avg(ttc) "
                    "FROM events WHERE edge_id IS NOT NULL GROUP BY edge_id,source")
        rows = cur.fetchall()
        made = 0
        for edge_id, source, cnt, avgrisk, avgttc in rows:
            # [변경] 구간 양 끝점을 평균좌표±0.0004 대신 실제 도로 엣지 좌표에서 가져옴.
            # 도로망에 없는 엣지 id(예전 격자 데이터 등)는 지도에 안 올림.
            try:
                p1, p2 = roads.get_edge_points(edge_id)
            except KeyError:
                continue
            grade = int(round(float(avgrisk)))
            cur.execute("INSERT INTO road_segment_stats (edge_id,source,p1_lat,p1_lng,p2_lat,p2_lng,"
                        "event_count,avg_risk,grade,avg_ttc,updated_at) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,now())",
                        (edge_id, source, p1[0], p1[1], p2[0], p2[1], cnt, float(avgrisk), grade,
                         float(avgttc) if avgttc is not None else None))
            made += 1
        conn.commit()
        return {"ok": True, "segments": made}
    finally:
        conn.close()

app.mount("/", StaticFiles(directory="static", html=True), name="static")
