# zones — 정적 위험구역 정의

위치만으로도 기본 위험도를 올려주는 캠퍼스 내 상시 위험 지점 정의입니다. `scripts/zone_detector.py`(라벨링)와 실시간 zone 판정 양쪽에서 사용합니다.

## zone_definition.csv

원형 구역(중심 좌표 + 반경)으로 정의하며, 좌표는 SUMO 로컬 좌표계(미터)입니다.

| zone_id | 이름 | 유형 | 반경 | base_risk | 비고 |
| --- | --- | --- | --- | --- | --- |
| Z01 | Main Gate | Entrance | 30m | 3 | 정문, 보행자 밀집 |
| Z02 | Middle Gate | Intersection | 30m | 4 | 건물에 가려진 사각 교차로 |
| Z03 | Student Center | Pedestrian Area | 30m | 2 | 학생회관, 보행 빈번 |
| Z04 | Parking Exit | Parking | 30m | 5 | 주차장 출구, 시야 차단 |

스키마: `zone_id, zone_name, zone_type, center_x, center_y, radius_m, base_risk, speed_limit, description`
