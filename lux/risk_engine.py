#!/usr/bin/env python3
"""
Risk calculation engine for the AI-V2X cane project.

This module is intentionally independent from UART/Serial code so the same
logic can be used by Jetson live input, CSV replay, and Minseo's SUMO dataset.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class RiskInput:
    ts_ms: int
    ped_x: float
    ped_y: float
    veh_x: float
    veh_y: float
    ped_speed_mps: float
    veh_speed_mps: float
    distance_m: float | None = None
    rel_speed_mps: float | None = None


@dataclass(frozen=True)
class Zone:
    zone_id: str
    zone_name: str
    zone_type: str
    center_x: float
    center_y: float
    radius_m: float
    base_risk: int
    description: str = ""


@dataclass(frozen=True)
class RiskResult:
    ts_ms: int
    distance_m: float
    rel_speed_mps: float
    ttc_s: float | None
    rule_risk: int
    zone_risk: int
    transformer_risk: int
    final_risk: int
    reason: str


def haversine_m(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lam = math.radians(lng2 - lng1)

    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lam / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


def distance_xy_m(ped_x: float, ped_y: float, veh_x: float, veh_y: float) -> float:
    return math.hypot(veh_x - ped_x, veh_y - ped_y)


def estimate_relative_speed(input_data: RiskInput) -> float:
    if input_data.rel_speed_mps is not None:
        return max(0.0, input_data.rel_speed_mps)

    # Conservative fallback until heading/vector data is available.
    # Positive means the vehicle can close distance faster than the pedestrian.
    return max(0.0, input_data.veh_speed_mps - input_data.ped_speed_mps)


def estimate_ttc(distance_m: float, rel_speed_mps: float) -> float | None:
    if rel_speed_mps <= 0.2:
        return None
    return distance_m / rel_speed_mps


def classify_rule_risk(distance_m: float, ttc_s: float | None) -> tuple[int, str]:
    if distance_m < 3.0:
        return 3, "rule:distance<3m"
    if ttc_s is not None and ttc_s < 1.5:
        return 3, "rule:ttc<1.5s"
    if distance_m < 6.0:
        return 2, "rule:distance<6m"
    if ttc_s is not None and ttc_s < 3.0:
        return 2, "rule:ttc<3s"
    if distance_m < 10.0:
        return 1, "rule:distance<10m"
    if ttc_s is not None and ttc_s < 5.0:
        return 1, "rule:ttc<5s"
    return 0, "rule:safe"


def load_zones(path: str | Path) -> list[Zone]:
    zone_path = Path(path)
    if not zone_path.exists():
        return []

    zones: list[Zone] = []
    with zone_path.open("r", encoding="utf-8-sig", newline="") as fp:
        for row in csv.DictReader(fp):
            zones.append(
                Zone(
                    zone_id=row["zone_id"],
                    zone_name=row["zone_name"],
                    zone_type=row["zone_type"],
                    center_x=float(row["center_x"]),
                    center_y=float(row["center_y"]),
                    radius_m=float(row["radius_m"]),
                    base_risk=int(row["base_risk"]),
                    description=row.get("description", ""),
                )
            )
    return zones


def classify_zone_risk(ped_x: float, ped_y: float, zones: list[Zone]) -> tuple[int, str]:
    matched: list[Zone] = []
    for zone in zones:
        if math.hypot(ped_x - zone.center_x, ped_y - zone.center_y) <= zone.radius_m:
            matched.append(zone)

    if not matched:
        return 0, "zone:none"

    highest = max(matched, key=lambda zone: zone.base_risk)
    return highest.base_risk, f"zone:{highest.zone_id}:{highest.zone_type}"


def transformer_risk_placeholder(input_data: RiskInput) -> tuple[int, str]:
    # Minseo's .onnx model will be called here later.
    return 0, "ai:placeholder"


def merge_risks(rule: tuple[int, str], zone: tuple[int, str], ai: tuple[int, str]) -> tuple[int, str]:
    candidates = [
        ("rule", rule[0], rule[1]),
        ("zone", zone[0], zone[1]),
        ("ai", ai[0], ai[1]),
    ]
    winner = max(candidates, key=lambda item: item[1])
    details = ";".join(f"{name}={risk}({reason})" for name, risk, reason in candidates)
    return winner[1], f"winner={winner[0]};{details}"


def evaluate_risk(input_data: RiskInput, zones: list[Zone] | None = None) -> RiskResult:
    zones = zones or []
    distance_m = input_data.distance_m
    if distance_m is None:
        distance_m = distance_xy_m(
            input_data.ped_x,
            input_data.ped_y,
            input_data.veh_x,
            input_data.veh_y,
        )

    rel_speed_mps = estimate_relative_speed(input_data)
    ttc_s = estimate_ttc(distance_m, rel_speed_mps)

    rule = classify_rule_risk(distance_m, ttc_s)
    zone = classify_zone_risk(input_data.ped_x, input_data.ped_y, zones)
    ai = transformer_risk_placeholder(input_data)
    final_risk, reason = merge_risks(rule, zone, ai)

    return RiskResult(
        ts_ms=input_data.ts_ms,
        distance_m=distance_m,
        rel_speed_mps=rel_speed_mps,
        ttc_s=ttc_s,
        rule_risk=rule[0],
        zone_risk=zone[0],
        transformer_risk=ai[0],
        final_risk=final_risk,
        reason=reason,
    )
