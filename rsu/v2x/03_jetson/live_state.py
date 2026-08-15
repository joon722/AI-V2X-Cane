#!/usr/bin/env python3
"""판정 결과의 최신 한 벌. 화면이 여기서 읽어간다.

step8은 판정할 때마다 update()를 부르고, HTTP 서버 스레드는 snapshot()으로
읽는다. 갱신이 dict를 통째로 바꾸는 것이라 락이 없어도 반쯤 쓰인 값이 읽히지
않는다 - 읽는 쪽은 완결된 이전 것이나 완결된 새 것 중 하나를 본다.

RSU로 가는 경보와는 별개 경로다. 여기서 무슨 일이 나도 지팡이는 영향받지 않는다.
그래서 step8은 update() 호출을 try로 감싸고, 이 파일은 예외를 던져도 되는
자리에서만 던진다.
"""
import copy

# 이보다 큰 TTC는 값이 아니라 "접근하지 않는다"는 뜻이므로 화면에 내보내지 않는다.
# step7의 calculate_ttc는 그런 쌍에 9999를 준다. 실제 판정에 쓰이는 TTC는
# model_features.TTC_MAX_S(30초)에서 이미 잘리므로 999는 넉넉한 경계다.
TTC_UNBOUNDED_S = 999.0

# 화면 스냅샷의 모양. 데이터가 오기 전에도 같은 키를 내보내야 화면 코드가
# "아직 없음"과 "값이 0"을 갈라 다루지 않아도 된다.
_EMPTY = {
    "t": None,
    "age_s": None,
    "level": 0,
    "level_source": None,
    "reason": None,
    "trusted": False,
    "target_id": None,
    "distance_m": None,
    "ttc_s": None,
    "closing_mps": None,
    "dcpa_m": None,
    "rel_east_m": None,
    "rel_north_m": None,
    "model_proba": None,
    "cane": {"lat": None, "lng": None, "heading_deg": None,
             "speed_mps": None, "gps_valid": None},
    "veh": {"speed_mps": None, "heading_deg": None, "gps_valid": None},
}


def _to_float(value, digits=2):
    """store 값은 JSON에서 와 문자열일 수 있다. 화면에는 숫자만 보낸다."""
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def _bounded_ttc(ttc):
    value = _to_float(ttc)
    if value is None or value >= TTC_UNBOUNDED_S:
        return None
    return value


class LiveState:
    """최신 판정 하나를 들고 있는다. 이력은 남기지 않는다."""

    def __init__(self):
        self._snap = None

    def update(self, now, store, decision, assessment, kinematics,
               gate=None, target_id=0):
        """판정 한 번의 결과를 스냅샷으로 굳힌다.

        전송 여부(decision.should_send)와 무관하게 매 판정마다 불려야 한다.
        전송은 등급이 바뀌거나 heartbeat일 때만 일어나지만, 거리와 TTC는 그
        사이에도 계속 변하고 화면은 그것을 보여줘야 한다.
        """
        cane = store.latest["cane"]
        vehicle = store.latest["vehicle"]
        self._snap = {
            "t": now,
            "age_s": None,  # 읽는 시점에 채운다
            # 지팡이가 실제로 받은 등급. 신뢰할 수 없는 GPS로 계산된 등급은
            # 지팡이에 안 갔으므로 화면에도 뜨면 안 된다.
            "level": decision.effective_level,
            "level_source": "table" if gate is None else gate.source,
            "reason": assessment.reason,
            "trusted": bool(decision.trusted),
            "target_id": target_id,
            "distance_m": _to_float(assessment.distance_m),
            "ttc_s": _bounded_ttc(assessment.ttc),
            # 양수 = 접근 중, 음수 = 멀어지는 중.
            "closing_mps": _to_float(assessment.closing_los),
            "dcpa_m": _to_float(assessment.dcpa),
            "rel_east_m": _to_float(kinematics.rel_east),
            "rel_north_m": _to_float(kinematics.rel_north),
            "model_proba": None if gate is None else _to_float(gate.proba, 4),
            "cane": {
                "lat": _to_float(cane["lat"], 7),
                "lng": _to_float(cane["lng"], 7),
                "heading_deg": _to_float(cane["heading_deg"]),
                "speed_mps": _to_float(cane["speed_mps"]),
                "gps_valid": _to_float(cane["gps_valid"], 0),
            },
            "veh": {
                "speed_mps": _to_float(vehicle["speed_mps"]),
                "heading_deg": _to_float(vehicle["heading_deg"]),
                "gps_valid": _to_float(vehicle["gps_valid"], 0),
            },
        }

    def snapshot(self, now):
        """읽는 시점 기준의 한 벌. 호출자가 마음대로 고쳐도 안전한 사본이다."""
        # 참조를 한 번만 읽는다. 그 뒤에 update가 돌아도 이 사본은 온전하다.
        snap = self._snap
        if snap is None:
            return copy.deepcopy(_EMPTY)
        out = copy.deepcopy(snap)
        # 시계가 뒤로 가도 화면에 음수 나이가 뜨지 않게 한다.
        out["age_s"] = round(max(0.0, now - snap["t"]), 2)
        return out
