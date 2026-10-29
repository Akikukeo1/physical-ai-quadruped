"""Validation and simulator geometry for hand-authored search maps."""

import math
import re

from pai_motion.field_areas import distance_to_area, validate_areas

WALLS = [(-1.3, 0, 0.1, 2.8), (4.7, 0, 0.1, 2.8), (1.7, -2.8, 3, 0.1), (1.7, 2.8, 3, 0.1)]


def validate_map(value):
    if not isinstance(value, dict) or set(value) != {"start", "obstacles", "targets", "areas", "points"}:
        raise ValueError("Map requires start, obstacles, targets, areas and points")

    def position(point):
        if (
            not isinstance(point, list)
            or len(point) != 2
            or any(type(n) not in (int, float) or not math.isfinite(n) for n in point)
            or not -1.15 <= point[0] <= 4.55
            or not -2.65 <= point[1] <= 2.65
        ):
            raise ValueError("Position must lie inside the field")

    position(value["start"])
    obstacles = value["obstacles"]
    if not isinstance(obstacles, list) or len(obstacles) > 30:
        raise ValueError("At most 30 obstacle rectangles")
    for box in obstacles:
        if not isinstance(box, list) or len(box) != 4:
            raise ValueError("Obstacle requires four bounds")
        position(box[:2])
        position(box[2:])
        if box[0] >= box[2] or box[1] >= box[3]:
            raise ValueError("Obstacle needs positive width and height")
    if not isinstance(value["targets"], dict) or set(value["targets"]) - {"blue", "green"}:
        raise ValueError("Targets must be blue or green")
    for p in value["targets"].values():
        position(p)
    if not isinstance(value["areas"], list):
        raise ValueError("Areas must be a list")
    if value["areas"]:
        validate_areas(value["areas"])
        for area in value["areas"]:
            position(area["bounds"][:2])
            position(area["bounds"][2:])
    if not isinstance(value["points"], list) or len(value["points"]) > 8:
        raise ValueError("At most eight points")
    ids = set()
    for point in value["points"]:
        if not isinstance(point, dict) or set(point) != {"id", "position"}:
            raise ValueError("Point requires id and position")
        name = point["id"]
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", name) or name in ids:
            raise ValueError("Point IDs must be unique letters, numbers, _ or -")
        ids.add(name)
        position(point["position"])
    if any(distance_to_area(value["start"], box) < 0.65 for box in obstacles):
        raise ValueError("Start must be at least 0.65 m from obstacles")
    if not (-0.65 <= value["start"][0] <= 4.05 and -2.15 <= value["start"][1] <= 2.15):
        raise ValueError("Start must have at least 0.5 m clearance from field walls")
    for point in value["targets"].values():
        if math.dist(value["start"], point) < 0.8:
            raise ValueError("Targets must be at least 0.8 m from the start")
    return value


def map_scene(value):
    validate_map(value)
    boxes = [*WALLS, *((((a + c) / 2, (b + d) / 2, (c - a) / 2, (d - b) / 2)) for a, b, c, d in value["obstacles"])]
    scene = [{"box": box, "rgba": [0.65, 0.65, 0.65, 1]} for box in boxes]
    for color, (x, y) in value["targets"].items():
        scene.append({"box": (x, y, 0.14, 0.14), "rgba": [0.1, 0.25, 1, 1] if color == "blue" else [0.1, 1, 0.25, 1]})
    return scene
