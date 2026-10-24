"""Seeded simulator scenes. These answers never enter the exploration interface."""

import math
import random


def search_scene(seed: int, missing_target: bool = False) -> list[dict]:
    rng = random.Random(seed)
    boxes = [(-1.3, 0, 0.1, 2.8), (4.7, 0, 0.1, 2.8), (1.7, -2.8, 3, 0.1), (1.7, 2.8, 3, 0.1)]
    for _ in range(1 + seed % 3):
        for _ in range(100):
            box = (rng.uniform(1.1, 2.7), rng.uniform(-1.1, 1.1), rng.uniform(0.2, 0.4), rng.uniform(0.2, 0.45))
            if all(math.hypot(box[0] - x, box[1] - y) > 1.25 for x, y, _, _ in boxes[4:]):
                boxes.append(box)
                break
    result = [{"box": box, "rgba": [0.65, 0.65, 0.65, 1]} for box in boxes]
    side = rng.choice([-1, 1])
    for color, y in (("blue", side * rng.uniform(0.7, 1.6)), ("green", -side * rng.uniform(0.7, 1.6))):
        if color == "blue" and missing_target:
            continue
        result.append(
            {
                "box": (rng.uniform(3.3, 3.9), y, 0.14, 0.14),
                "rgba": [0.1, 0.25, 1, 1] if color == "blue" else [0.1, 1, 0.25, 1],
            }
        )
    return result
