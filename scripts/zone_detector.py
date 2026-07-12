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

    if nearest_zone is None:
        return {
            "zone_id": "OUT",
            "zone_name": "Outside Zone",
            "zone_type": "Normal Area",
            "zone_distance_m": None,
            "zone_base_risk": 0,
            "zone_speed_limit_kmh": None,
            "description": "Outside registered risk zones",
        }

    return {
        "zone_id": nearest_zone["zone_id"],
        "zone_name": nearest_zone["zone_name"],
        "zone_type": nearest_zone["zone_type"],
        "zone_distance_m": round(nearest_distance, 2),
        "zone_base_risk": nearest_zone["base_risk"],
        "zone_speed_limit_kmh": nearest_zone["speed_limit"],
        "description": nearest_zone["description"],
    }