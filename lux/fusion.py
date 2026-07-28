#!/usr/bin/env python3
"""Combine the three risk sources into one level, safety-first.

final = max(rule, zone, predicted). The highest wins so a miss on any single
path cannot silence the warning (the team's agreed structure). reason records
which source(s) produced the winning level, in priority order rule > zone >
predicted, so a log line explains itself. A None prediction (model absent or
buffer not ready) simply drops out of the max.

This module is pure: no coordinate frame, no onnx, no serial. It only maxes
three integers, so it is complete regardless of the still-open integration
contract (see docs/integration_contract.md).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class FusedRisk:
    level: int
    reason: str
    sources: dict


def fuse_risk(rule_level, zone_level, predicted_level):
    sources = {"rule": rule_level, "zone": zone_level}
    if predicted_level is not None:
        sources["predicted"] = predicted_level

    level = max(sources.values())
    if level == 0:
        return FusedRisk(level=0, reason="none", sources=sources)

    order = ("rule", "zone", "predicted")
    winners = [name for name in order if sources.get(name) == level]
    return FusedRisk(level=level, reason="+".join(winners), sources=sources)
