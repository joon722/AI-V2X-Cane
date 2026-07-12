import csv
import math
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent.parent
ZONE_FILE = PROJECT_DIR / "zones" / "zone_definition.csv"


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
                    "description": row["description"],
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

    print("SUMO 좌표를 입력하세요.")

    x = float(input("x 좌표: "))
    y = float(input("y 좌표: "))

    zone, distance = detect_zone(x, y, zones)

    if zone is None:
        print("\n현재 좌표는 등록된 위험 Zone 밖에 있습니다.")
    else:
        print("\n현재 Zone 정보")
        print(f"Zone ID: {zone['zone_id']}")
        print(f"Zone 이름: {zone['zone_name']}")
        print(f"Zone 유형: {zone['zone_type']}")
        print(f"중심점과의 거리: {distance:.2f} m")
        print(f"기본 위험도: {zone['base_risk']}")
        print(f"제한 속도: {zone['speed_limit']} km/h")
        print(f"설명: {zone['description']}")


if __name__ == "__main__":
    main()