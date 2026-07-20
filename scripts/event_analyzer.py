import csv
from pathlib import Path

from zone_detector import load_zones, detect_zone
from risk_calculator import analyze_risk
from event_classifier import classify_event


PROJECT_DIR = Path(__file__).resolve().parent.parent
INPUT_FILE = PROJECT_DIR / "dataset" / "scenario_005" / "feature.csv"
OUTPUT_FILE = PROJECT_DIR / "dataset" / "scenario_005" / "event_result.csv"


def calculate_event_position(ped_x, ped_y, veh_x, veh_y):
    """
    위험 이벤트 위치를 차량과 보행자의 중간 지점으로 계산한다.
    """

    event_x = (ped_x + veh_x) / 2
    event_y = (ped_y + veh_y) / 2

    return event_x, event_y


def main():
    zones = load_zones()

    total_count = 0
    risk_counts = {
        0: 0,
        1: 0,
        2: 0,
        3: 0,
    }

    event_counts = {}

    with open(INPUT_FILE, "r", encoding="utf-8-sig", newline="") as input_file:
        reader = csv.DictReader(input_file)

        if reader.fieldnames is None:
            raise ValueError("입력 CSV의 헤더를 읽을 수 없습니다.")

        output_fieldnames = reader.fieldnames + [
            "event_id",
            "event_x",
            "event_y",
            "ttc",
            "risk_score",
            "risk_level",
            "zone_id",
            "zone_name",
            "zone_type",
            "zone_distance_m",
            "zone_base_risk",
            "zone_speed_limit_kmh",
            "event_type",
        ]

        with open(
            OUTPUT_FILE,
            "w",
            encoding="utf-8-sig",
            newline="",
        ) as output_file:
            writer = csv.DictWriter(
                output_file,
                fieldnames=output_fieldnames,
            )

            writer.writeheader()

            for row_number, row in enumerate(reader, start=1):
                total_count += 1

                ped_x = float(row["ped_x"])
                ped_y = float(row["ped_y"])
                veh_x = float(row["veh_x"])
                veh_y = float(row["veh_y"])

                distance_m = float(row["distance_m"])
                relative_speed_mps = float(row["rel_speed_mps"])
                vehicle_speed_mps = float(row["veh_speed_mps"])

                event_x, event_y = calculate_event_position(
                    ped_x=ped_x,
                    ped_y=ped_y,
                    veh_x=veh_x,
                    veh_y=veh_y,
                )

                zone = detect_zone(
                    x=event_x,
                    y=event_y,
                    zones=zones,
                )

                risk = analyze_risk(
                    distance_m=distance_m,
                    relative_speed_mps=relative_speed_mps,
                    vehicle_speed_mps=vehicle_speed_mps,
                    zone_base_risk=zone["zone_base_risk"],
                )

                event_type = classify_event(
                    distance_m=distance_m,
                    relative_speed_mps=relative_speed_mps,
                    vehicle_speed_mps=vehicle_speed_mps,
                    ttc=risk["ttc"],
                    risk_level=risk["risk_level"],
                    zone_type=zone["zone_type"],
               )

                risk_counts[risk["risk_level"]] += 1
                event_counts[event_type] = event_counts.get(event_type, 0) + 1

                row["event_id"] = f"E{row_number:06d}"
                row["event_x"] = round(event_x, 2)
                row["event_y"] = round(event_y, 2)
                row["ttc"] = risk["ttc"]
                row["risk_score"] = risk["risk_score"]
                row["risk_level"] = risk["risk_level"]
                row["zone_id"] = zone["zone_id"]
                row["zone_name"] = zone["zone_name"]
                row["zone_type"] = zone["zone_type"]
                row["zone_distance_m"] = (
                    ""
                    if zone["zone_distance_m"] is None
                    else zone["zone_distance_m"]
                )
                row["zone_base_risk"] = zone["zone_base_risk"]
                row["zone_speed_limit_kmh"] = (
                    ""
                    if zone["zone_speed_limit_kmh"] is None
                    else zone["zone_speed_limit_kmh"]
                )
                row["event_type"] = event_type

                writer.writerow(row)

    print("이벤트 분석 완료")
    print(f"전체 데이터 수: {total_count}")
    print(f"안전(Level 0): {risk_counts[0]}")
    print(f"주의(Level 1): {risk_counts[1]}")
    print(f"경고(Level 2): {risk_counts[2]}")
    print(f"위험(Level 3): {risk_counts[3]}")

    print("\n이벤트 유형별 개수")

    for event_type, count in sorted(event_counts.items()):
        print(f"{event_type}: {count}")

    print(f"\n저장 파일: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
