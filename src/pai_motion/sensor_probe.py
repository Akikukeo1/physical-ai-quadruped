"""Kinematic yaw probe and analytic vent field; no CFD or hidden-state planner access."""

import math
import xml.etree.ElementTree as ET

LENGTH = 0.22
LIMIT = 1.1
ITEMS = ("air_temperature", "airflow")


def add_probe(scene, custom_map=None):
    """Non-colliding mocap visualization, outside the robot dynamics tree."""
    body = ET.SubElement(scene.find("worldbody"), "body", name="probe_visual", mocap="true")
    ET.SubElement(
        body,
        "geom",
        name="probe_rod",
        type="capsule",
        fromto=f"0 0 0 {LENGTH} 0 0",
        size=".008",
        mass=".03",
        rgba="1 .65 .05 1",
        group="4",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        body,
        "geom",
        name="probe_head",
        type="sphere",
        pos=f"{LENGTH} 0 0",
        size=".018",
        mass=".01",
        rgba=".1 .9 .95 1",
        group="4",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(body, "site", name="probe_tip", pos=f"{LENGTH} 0 0", size=".022", rgba=".1 1 1 1")
    cooler = next((p for p in (custom_map or {}).get("points", []) if p["id"] == "cooler"), None)
    if cooler:
        ports = measurement_ports(cooler["position"])
        x, y, z = ports[0]["source"]
        ET.SubElement(
            scene.find("worldbody"),
            "geom",
            name="probe_outlet_visual",
            type="cylinder",
            pos=f"{x + 0.04} {y} {z}",
            size=".15 .04",
            quat=".70710678 0 .70710678 0",
            rgba=".25 .4 .5 1",
            group="4",
            contype="0",
            conaffinity="0",
        )
        for port, color in zip(ports, ("1 .6 .1 .6", ".1 .8 1 .6"), strict=True):
            angle = port["nominal_probe_angle"]
            ET.SubElement(
                scene.find("worldbody"),
                "geom",
                name=f"window_{port['item']}",
                type="sphere",
                pos=f"{x - 0.26 * math.cos(angle)} {y - 0.26 * math.sin(angle)} {z}",
                size=".04",
                rgba=color,
                group="4",
                contype="0",
                conaffinity="0",
            )


def measurement_ports(service):
    """Known service geometry, independent of fault and sensor values."""
    x, y = service
    source = [x + 0.60, y, 0.53]
    return [
        {
            "equipment": "cooler",
            "item": item,
            "source": source,
            "position": [
                source[0] - 0.1 - (LENGTH + 0.26) * math.cos(angle),
                source[1] - (LENGTH + 0.26) * math.sin(angle),
            ],
            "body_yaw": 0.0,
            "nominal_probe_angle": angle,
        }
        for item, angle in (("air_temperature", 0.6), ("airflow", -0.6))
    ]


def vent_sample(item, source, pose, *, outlet_c, outlet_speed, clear=True):
    """A conical decaying jet and ambient mixing. Invalid geometry yields no value."""
    if item not in ITEMS:
        raise ValueError("Unsupported probe measurement")
    tip, direction = pose["tip"], pose["direction"]
    if not all(math.isfinite(v) for v in [*tip, *direction, *source]):
        raise ValueError("Non-finite sensor geometry")
    delta = [s - p for s, p in zip(source, tip, strict=True)]
    distance = math.sqrt(sum(v * v for v in delta))
    cosine = sum(d * v for d, v in zip(direction, delta, strict=True)) / max(distance, 1e-9)
    # Vent faces west. The two instrument access windows lie on opposite sides of the outlet.
    side = tip[1] - source[1]
    zone = (-0.30 <= side <= -0.04) if item == "air_temperature" else (0.04 <= side <= 0.30)
    valid = (
        0.15 <= distance <= 0.42
        and cosine >= math.cos(math.radians(16))
        and tip[0] < source[0]
        and abs(tip[2] - source[2]) <= 0.07
        and zone
        and clear
        and abs(pose["joint_speed"]) < 0.08
        and abs(pose["angle"]) > 0.2
    )
    geometry = {
        "distance": distance,
        "alignment_cosine": cosine,
        "correct_sampling_zone": zone,
        "line_of_sight": bool(clear),
    }
    if not valid:
        return {
            "valid": False,
            "value": None,
            "unit": "degC" if item == "air_temperature" else "m/s",
            "geometry": geometry,
            "reason": "probe geometry, deployment or settling condition not met",
        }
    axial = source[0] - tip[0]
    radial = math.hypot(tip[1] - source[1], tip[2] - source[2])
    gain = math.exp(-0.4 * axial) * math.exp(-((radial / 0.5) ** 2))
    value = 8.0 + (outlet_c - 8.0) * gain if item == "air_temperature" else outlet_speed * gain
    return {
        "valid": True,
        "value": round(value, 3),
        "unit": "degC" if item == "air_temperature" else "m/s",
        "geometry": geometry,
        "reason": "analytic local vent field; not thermofluid simulation",
    }


def execute_measurement(motion, port, measure, action):
    """Bounded body alignment, kinematic probe motion, then sampling and stowing."""
    from pai_motion.executor import Velocity

    sink = motion.sink
    action["alignment_started"] = sink.observe()["time"]
    for _ in range(70):
        if motion.remaining() <= motion.reserve(sink.observe()) + 10:
            raise RuntimeError("Insufficient return reserve for probe operation")
        state = sink.observe()
        error = math.atan2(math.sin(port["body_yaw"] - state["yaw"]), math.cos(port["body_yaw"] - state["yaw"]))
        if abs(error) < 0.04:
            break
        sink.run_velocity(Velocity(0, 0, max(-0.5, min(0.5, 1.8 * error))), 0.2)
    else:
        raise RuntimeError("Probe body alignment timeout")
    motion.settle()
    action["before_deploy"] = sink.probe_pose()
    try:
        mount = sink.probe_pose()["mount"]
        angle = math.atan2(port["source"][1] - mount[1], port["source"][0] - mount[0]) - sink.observe()["yaw"]
        angle = math.atan2(math.sin(angle), math.cos(angle))
        sink.move_probe(angle)
        motion.settle()
        observation = measure(port["item"], sink)
        action["sampled_at"] = observation["time"]
        action["deployed"] = sink.probe_pose()
        return observation
    finally:
        motion.settle()
        sink.move_probe(0.0)
        action["stowed"] = sink.probe_pose()
