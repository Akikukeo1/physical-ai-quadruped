"""Simulator-only warehouse state; never pass this object to the planner."""

import math
import random
from copy import deepcopy

from pai_motion.custom_map import validate_map

CAUSES = ("power_loss", "cooler_fault", "door_open")
INSPECTION_RADIUS = 0.25


def warehouse_scene(seed=1102, *, development=True, fault=None):
    """A is editable development data. B uses a separate, reproducible generator."""
    if development:
        positions = [[3.4, -1.5], [1.0, 0.0], [3.4, 1.5]]
        obstacles = [[1.8, -0.4, 2.25, 0.4]]
        cause = fault or "power_loss"
    else:
        rng = random.Random(seed)
        positions = [[rng.uniform(2.9, 3.6), y] for y in (-1.6, 0.0, 1.6)]
        rng.shuffle(positions)
        obstacles = [[1.4, -0.5, 1.9, 0.5]]
        cause = rng.choice(("cooler_fault", "door_open"))
    if cause not in CAUSES:
        raise ValueError("Unknown warehouse fault")
    points = [dict(id=name, position=p) for name, p in zip(("supply", "cooler", "door"), positions, strict=True)]
    geometry = validate_map(dict(start=[0, 0], obstacles=obstacles, targets={}, areas=[], points=points))
    return InspectionWorld(geometry, cause)


class InspectionWorld:
    def __init__(self, geometry, cause):
        self.geometry = deepcopy(geometry)
        self.cause = cause

    def inventory(self):
        descriptions = {
            "supply": "冷却専用電源盤。出力電圧、遮断器。正常230V。",
            "cooler": "冷却機。受電電圧、冷却要求、圧縮機運転、吹出温度。",
            "door": "独立電池式の扉開閉・開放継続時間センサー。",
        }
        return [{**deepcopy(p), "description": descriptions[p["id"]]} for p in self.geometry["points"]]

    def inspect(self, equipment, state):
        point = next((p for p in self.geometry["points"] if p["id"] == equipment), None)
        if point is None or math.dist(state["position"][:2], point["position"]) > INSPECTION_RADIUS:
            raise ValueError("Equipment inspection requires physical proximity")
        if math.hypot(*state["body_velocity"][:2]) > 0.05 or abs(state["yaw_rate"]) > 0.08:
            raise ValueError("Equipment inspection requires a settled robot")
        powered = self.cause != "power_loss"
        running = self.cause == "door_open"
        readings = {
            "supply": {"output_voltage_v": 230 if powered else 0, "breaker": "on" if powered else "tripped"},
            "cooler": {
                "input_voltage_v": 230 if powered else 0,
                "cooling_requested": True,
                "compressor_running": running,
                "supply_air_c": -12 if running else 8,
            },
            "door": {"closed": self.cause != "door_open", "open_seconds": 900 if self.cause == "door_open" else 0},
        }
        return {
            "equipment": equipment,
            "readings": deepcopy(readings[equipment]),
            "time": state["time"],
            "position": list(state["position"][:2]),
            "interface": "simulated_local_inspection",
        }

    def evaluate(self, report, observations):
        """Post-run evaluator; truth and sufficiency rules are absent from planner inputs."""
        by_id = {o["id"]: o for o in observations}
        evidence = {by_id[i]["equipment"]: by_id[i]["readings"] for i in report.get("evidence_ids", []) if i in by_id}
        supply, cooler, door = (evidence.get(k, {}) for k in ("supply", "cooler", "door"))
        supported = {
            "power_loss": supply.get("output_voltage_v") == 0
            and cooler.get("input_voltage_v") == 0
            and cooler.get("compressor_running") is False,
            "cooler_fault": supply.get("output_voltage_v") == 230
            and cooler.get("input_voltage_v") == 230
            and cooler.get("cooling_requested") is True
            and cooler.get("compressor_running") is False,
            "door_open": cooler.get("compressor_running") is True
            and cooler.get("supply_air_c", 99) < 0
            and door.get("closed") is False
            and door.get("open_seconds", 0) >= 600,
        }
        cause = report.get("conclusion")
        return {
            "truth": self.cause,
            "correct_cause": cause == self.cause,
            "evidence_sufficient": bool(supported.get(cause, False)),
        }
