"""OSM 도로망(roads.json)을 불러와 좌표를 가장 가까운 도로 구간(엣지)에 스냅.

- roads.json: Overpass API로 받아둔 숭실대/상도동 일대 도로(way) 목록.
  각 way는 [lat, lng] 점들의 리스트이며, 인접한 두 점씩 잘라 '엣지'로 취급.
- nearest_edge_id(lat, lng): 좌표에서 가장 가까운 엣지 id 반환 (이벤트 저장 시 사용)
- get_edge_points(edge_id): 엣지 양 끝점 반환 (집계/지도 표시 시 사용)

측정 지역을 바꾸려면 Overpass API로 그 지역 도로망을 다시 받아
roads.json만 교체하면 됨. (README 참고)
"""
import json
import math
import os
from pathlib import Path

ROADS_PATH = Path(__file__).resolve().parent / "roads.json"

_LAT_TO_M = 110_540.0  # 위도 1도 ≈ 110.54km

# [변경] 스냅 상한을 300m -> 50m. 상도동 도로 밀도면 300m 안에는 거의 항상
# 도로가 있어서 사실상 아무것도 걸러지지 않았다. 실측 GPS 오차는 σ≈2m.
MAX_SNAP_DISTANCE_M = float(os.environ.get("MAX_SNAP_DISTANCE_M", "50"))
# [변경] 서비스 지역 밖 좌표를 먼저 거른다. 실내 fallback 좌표(37.0/127.0)처럼
# 수십 km 떨어진 값이 들어와도 스냅만으로는 "먼 도로"인지 구분이 안 된다.
MAX_CENTER_DISTANCE_KM = float(os.environ.get("MAX_CENTER_DISTANCE_KM", "5"))


class _Edge:
    __slots__ = ("id", "p1", "p2", "x1", "y1", "x2", "y2")

    def __init__(self, edge_id, p1, p2, lng_to_m):
        self.id = edge_id
        self.p1 = p1
        self.p2 = p2
        self.x1, self.y1 = p1[1] * lng_to_m, p1[0] * _LAT_TO_M
        self.x2, self.y2 = p2[1] * lng_to_m, p2[0] * _LAT_TO_M


def _load():
    with open(ROADS_PATH, encoding="utf-8") as f:
        roads = json.load(f)

    # 도로망 데이터 자체의 중심 위도를 기준으로 경도->미터 변환 계수 계산
    lats = [p[0] for r in roads for p in r["points"]]
    lngs = [p[1] for r in roads for p in r["points"]]
    center_lat = (min(lats) + max(lats)) / 2
    center_lng = (min(lngs) + max(lngs)) / 2  # [변경] 서비스 지역 판정 기준점
    lng_to_m = 111_320.0 * math.cos(math.radians(center_lat))

    edges = []
    for road in roads:
        pts = road["points"]
        for i in range(len(pts) - 1):
            p1 = (pts[i][0], pts[i][1])
            p2 = (pts[i + 1][0], pts[i + 1][1])
            edges.append(_Edge(f"{road['id']}_{i}", p1, p2, lng_to_m))
    return edges, lng_to_m, (center_lat, center_lng)


_EDGES, _LNG_TO_M, _CENTER = _load()
_EDGES_BY_ID = {e.id: e for e in _EDGES}


def _dist_sq(px, py, ax, ay, bx, by):
    """점(px,py)에서 선분(a-b)까지 최단거리 제곱 (m^2)"""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return (px - ax) ** 2 + (py - ay) ** 2
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return (px - ax - t * dx) ** 2 + (py - ay - t * dy) ** 2


def nearest_edge(lat: float, lng: float):
    """[변경] 가장 가까운 엣지 id와 그 거리(m)를 함께 반환. 상한 판정은 부르는
    쪽에서 한다 — 거부할 때 "몇 m 떨어졌다"를 응답에 담아야 원인을 알 수 있다.
    도로망이 비어 있으면 (None, inf)."""
    px, py = lng * _LNG_TO_M, lat * _LAT_TO_M
    best_id, best_d = None, math.inf
    for e in _EDGES:
        d = _dist_sq(px, py, e.x1, e.y1, e.x2, e.y2)
        if d < best_d:
            best_d, best_id = d, e.id
    return best_id, math.sqrt(best_d)


def distance_from_center_km(lat: float, lng: float) -> float:
    """[변경] 도로망 중심에서의 거리(km). 서비스 지역 밖 좌표를 거르는 데 쓴다."""
    dx = (lng - _CENTER[1]) * _LNG_TO_M
    dy = (lat - _CENTER[0]) * _LAT_TO_M
    return math.hypot(dx, dy) / 1000.0


def nearest_edge_id(lat: float, lng: float):
    """가장 가까운 도로 엣지 id. 상한을 넘게 벗어난 좌표는 None
    (엉뚱한 지역 좌표가 지도에 이상하게 찍히는 것을 방지)"""
    edge_id, dist_m = nearest_edge(lat, lng)
    return edge_id if dist_m <= MAX_SNAP_DISTANCE_M else None


def get_edge_points(edge_id: str):
    """엣지 id -> ((lat,lng), (lat,lng)). 없는 id면 KeyError"""
    e = _EDGES_BY_ID[edge_id]
    return e.p1, e.p2
