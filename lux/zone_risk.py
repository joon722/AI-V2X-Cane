#!/usr/bin/env python3
"""Map a pedestrian position to a static hazard-zone risk level (0-3).

The circular-zone detection mirrors the team's scripts/zone_detector.py so the
two stay consistent. This module adds the two things the realtime fusion stage
needs and the audit script does not: an explicit base_risk(0-5) -> risk_level
(0-3) table, and a single zone_level() entry point.

OPEN CONTRACT (see docs/integration_contract.md):
  - Coordinate frame: callers must hand in (x, y) already in the SAME frame as
    the zone csv's center_x/center_y. That frame is NOT yet agreed. The zone
    csvs in the repo disagree with each other (zones/zone_definition.csv uses
    SUMO local ~3600/1400; lux/zone_definition.csv uses tiny ~10/5 values), and
    neither matches the realtime GPS lat/lng frame. Until Part 2 of the contract
    is settled, the caller is responsible for the transform.
  - BASE_RISK_TO_LEVEL below is PROVISIONAL (contract Part 5). Swap this one dict
    when the mapping is confirmed.
"""

import csv
import math


# base_risk (0-5, from the team zone csv) -> fusion risk_level (0-3).
# PROVISIONAL — confirm against docs/integration_contract.md Part 5.
BASE_RISK_TO_LEVEL = {0: 0, 1: 0, 2: 1, 3: 1, 4: 2, 5: 3}


def load_zones(csv_path):
    """Load zones from csv. Reads only the columns fusion needs, so the two
    schema variants in the repo (with/without speed_limit) both parse."""
    zones = []
    with open(csv_path, "r", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            zones.append(
                {
                    "zone_id": row["zone_id"],
                    "zone_name": row["zone_name"],
                    "zone_type": row["zone_type"],
                    "center_x": float(row["center_x"]),
                    "center_y": float(row["center_y"]),
                    "radius_m": float(row["radius_m"]),
                    "base_risk": int(row["base_risk"]),
                }
            )
    return zones


def _distance(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


def detect_zone(x, y, zones):
    """Nearest zone whose radius contains (x, y); OUT sentinel if none."""
    nearest = None
    nearest_distance = float("inf")
    for zone in zones:
        d = _distance(x, y, zone["center_x"], zone["center_y"])
        if d <= zone["radius_m"] and d < nearest_distance:
            nearest, nearest_distance = zone, d
    if nearest is None:
        return {"zone_id": "OUT", "zone_base_risk": 0, "zone_distance_m": None}
    return {
        "zone_id": nearest["zone_id"],
        "zone_base_risk": nearest["base_risk"],
        "zone_distance_m": round(nearest_distance, 2),
    }


def zone_level(x, y, zones):
    """Static zone risk as a 0-3 level for the fusion stage."""
    base = detect_zone(x, y, zones)["zone_base_risk"]
    return BASE_RISK_TO_LEVEL.get(base, 0)
