"""Named, user-drawn rectangles in simulator world coordinates."""

import math
import re


def validate_areas(value):
    if not isinstance(value, list) or not 1 <= len(value) <= 8:
        raise ValueError("Areas must be a list of 1–8 rectangles")
    ids = set()
    for area in value:
        if not isinstance(area, dict) or set(area) != {"id", "bounds"}:
            raise ValueError("Each area requires id and bounds")
        name, bounds = area["id"], area["bounds"]
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", name) or name in ids:
            raise ValueError("Area IDs must be unique and consist of letters, digits, _ or -")
        if (
            not isinstance(bounds, list)
            or len(bounds) != 4
            or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 5 for v in bounds)
        ):
            raise ValueError("Area bounds must contain four finite coordinates within +/-5m")
        xmin, ymin, xmax, ymax = bounds
        if not xmin < xmax or not ymin < ymax:
            raise ValueError("Area bounds must have positive width and height")
        ids.add(name)
    return value


def distance_to_area(point, bounds):
    x, y = point[:2]
    xmin, ymin, xmax, ymax = bounds
    return math.hypot(max(xmin - x, 0, x - xmax), max(ymin - y, 0, y - ymax))
