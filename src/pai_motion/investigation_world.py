"""Simulator-only warehouse state; never pass this object to the planner."""

import math
import random
from copy import deepcopy

from pai_motion.custom_map import validate_map

CAUSES = ("power_loss", "cooler_fault", "door_open")
INSPECTION_RADIUS = 0.25


def warehouse_scene(seed=1102, *, development=True, fault=None, probe=False):
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
    if cause not in (*CAUSES, *(("airflow_fault",) if probe else ())):
        raise ValueError("Unknown warehouse fault")
    points = [dict(id=name, position=p) for name, p in zip(("supply", "cooler", "door"), positions, strict=True)]
    geometry = validate_map(dict(start=[0, 0], obstacles=obstacles, targets={}, areas=[], points=points))
    return (ProbeInspectionWorld if probe else InspectionWorld)(geometry, cause)


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


class ProbeInspectionWorld(InspectionWorld):
    """Same warehouse task; air data is available only through the deployed probe."""

    def inventory(self):
        from pai_motion.sensor_probe import measurement_ports

        points = super().inventory()
        cooler = next(p for p in points if p["id"] == "cooler")
        cooler["description"] = "冷却機の受電電圧・冷却要求・圧縮機運転を局所点検。空気温度・風速は別途プローブ計測。"
        cooler["measurements"] = measurement_ports(cooler["position"])
        return points

    def inspect(self, equipment, state):
        observation = super().inspect(equipment, state)
        if equipment == "cooler":
            observation["readings"] = {
                "input_voltage_v": 0 if self.cause == "power_loss" else 230,
                "cooling_requested": True,
                "compressor_running": self.cause != "power_loss",
            }
        return observation

    def measure(self, item, sink):
        from pai_motion.sensor_probe import measurement_ports, vent_sample

        state = sink.observe()
        cooler = next(p for p in self.geometry["points"] if p["id"] == "cooler")
        port = next((p for p in measurement_ports(cooler["position"]) if p["item"] == item), None)
        if port is None:
            raise ValueError("Unsupported measurement")
        pose = sink.probe_pose()
        outlet_c = 8.0 if self.cause in ("power_loss", "cooler_fault") else -12.0
        speed = 0.0 if self.cause == "power_loss" else 0.05 if self.cause == "airflow_fault" else 2.0
        result = vent_sample(
            item,
            port["source"],
            pose,
            outlet_c=outlet_c,
            outlet_speed=speed,
            clear=sink.probe_line_clear(port["source"]),
        )
        if math.hypot(*state["body_velocity"][:2]) > 0.05 or abs(state["yaw_rate"]) > 0.08:
            result.update(valid=False, value=None, reason="robot must be stationary")
        return {
            "equipment": "cooler",
            "item": item,
            **result,
            "pose": pose,
            "time": state["time"],
            "position": list(state["position"][:2]),
            "interface": "geometric_probe_field",
        }

    def evaluate(self, report, observations):
        cited = [o for o in observations if o["id"] in report.get("evidence_ids", [])]
        readings = {o["equipment"]: o["readings"] for o in cited if "readings" in o}
        measured = {o["item"]: o["value"] for o in cited if o.get("valid") is True}
        temp, flow = measured.get("air_temperature"), measured.get("airflow")
        cooler, supply, door = (readings.get(k, {}) for k in ("cooler", "supply", "door"))
        sufficient = {
            "power_loss": supply.get("output_voltage_v") == 0 and cooler.get("input_voltage_v") == 0,
            "cooler_fault": temp is not None
            and temp > 0
            and flow is not None
            and flow > 0.8
            and cooler.get("input_voltage_v") == 230,
            "airflow_fault": flow is not None
            and flow < 0.2
            and temp is not None
            and temp < 0
            and cooler.get("input_voltage_v") == 230,
            "door_open": temp is not None
            and temp < 0
            and flow is not None
            and flow > 0.8
            and door.get("closed") is False
            and door.get("open_seconds", 0) >= 600,
        }
        cause = report.get("conclusion")
        return {
            "truth": self.cause,
            "correct_cause": cause == self.cause,
            "evidence_sufficient": bool(sufficient.get(cause, False)),
        }
