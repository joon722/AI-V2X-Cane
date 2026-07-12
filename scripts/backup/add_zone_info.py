import csv
import math
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent.parent
ZONE_FILE = PROJECT_DIR / "zones" / "zone_definition.csv"
INPUT_FILE = PROJECT_DIR / "feature_9cols.csv"
OUTPUT_FILE = PROJECT_DIR / "dataset" / "feature_with_zone.csv"


def load_zones():
    zones = []

    with open(ZONE_FILE, "r", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)

        for row in reader:
            zones.append(
                {
                    "zone_id": row["zone_id"],
                    "zone_name": row["zone_name"],
                    "zone_type": row["zone_type"],
                    "center_x": float(row["center_x"]),
                    "center_y": float(row["center_y"]),
                    "radius_m": float(row["radius_m"]),
                    "base_risk": int(row["base_risk"]),
                    "speed_limit": float(row["speed_limit"]),
                }
            )

    return zones


def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def detect_zone(x, y, zones):
    nearest_zone = None
    nearest_distance = float("inf")

    for zone in zones:
        distance = calculate_distance(
            x,
            y,
            zone["center_x"],
            zone["center_y"],
        )

        if distance <= zone["radius_m"] and distance < nearest_distance:
            nearest_zone = zone
            nearest_distance = distance

    return nearest_zone, nearest_distance


def main():
    zones = load_zones()
    total_count = 0
    zone_count = 0

    with open(INPUT_FILE, "r", encoding="utf-8-sig", newline="") as input_file:
        reader = csv.DictReader(input_file)

        new_fieldnames = reader.fieldnames + [
            "zone_id",
            "zone_name",
            "zone_type",
            "zone_distance_m",
            "zone_base_risk",
            "zone_speed_limit_kmh",
        ]

        with open(
            OUTPUT_FILE,
            "w",
            encoding="utf-8-sig",
            newline="",
        ) as output_file:
            writer = csv.DictWriter(
                output_file,
                fieldnames=new_fieldnames,
            )

            writer.writeheader()

            for row in reader:
                total_count += 1

                ped_x = float(row["ped_x"])
                ped_y = float(row["ped_y"])
                veh_x = float(row["veh_x"])
                veh_y = float(row["veh_y"])

event_x = (ped_x + veh_x) / 2
event_y = (ped_y + veh_y) / 2

zone, distance = detect_zone(event_x, event_y, zones)

                if zone is None:
                    row["zone_id"] = "OUT"
                    row["zone_name"] = "Outside Zone"
                    row["zone_type"] = "Normal"
                    row["zone_distance_m"] = ""
                    row["zone_base_risk"] = 0
                    row["zone_speed_limit_kmh"] = ""
                else:
                    zone_count += 1

                    row["zone_id"] = zone["zone_id"]
                    row["zone_name"] = zone["zone_name"]
                    row["zone_type"] = zone["zone_type"]
                    row["zone_distance_m"] = f"{distance:.2f}"
                    row["zone_base_risk"] = zone["base_risk"]
                    row["zone_speed_limit_kmh"] = zone["speed_limit"]

                writer.writerow(row)

    print("Zone 정보 추가 완료")
    print(f"전체 데이터 수: {total_count}")
    print(f"Zone 내부 데이터 수: {zone_count}")
    print(f"Zone 외부 데이터 수: {total_count - zone_count}")
    print(f"저장 파일: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()