"""Validate a recorded development run and draw its observed-map route comparison."""

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from pai_motion.field_areas import distance_to_area  # noqa: E402
from pai_motion.search import ObservedMap  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    run = json.loads(args.log.read_text(encoding="utf-8"))
    (revision,) = run["corrections"]
    result, trace = run["result"], run["trace"]
    old, new = revision["old_route"], revision["new_route"]
    area = next(a["bounds"] for a in run["field_areas"] if a["id"] == "area2")
    received, applied = revision["received"]["time"], revision["applied_time"]
    assert run["status"] == "completed" and run["planner"] == "codex"
    assert revision["status"] == "accepted" and revision["blocked_areas"] == ["area2"]
    assert old != new and any(distance_to_area(p, area) == 0 for p in old)
    # Grid edges are at most 0.15 m long; check their interiors as well as their endpoints.
    new_samples = [
        [a[k] + (b[k] - a[k]) * i / 20 for k in (0, 1)] for a, b in zip(new, new[1:], strict=False) for i in range(21)
    ]
    assert new_samples and min(distance_to_area(p, area) for p in new_samples) > 0.15
    after = [s for s in trace if s["time"] >= applied - 1e-6]
    assert after and all(distance_to_area(s["position"], area) > 0.05 for s in after)
    assert all(s["finite"] and s["upright"] > 0.7 and not s["obstacle_contact"] for s in trace)
    wait = [e for e in run["events"] if received - 1e-6 <= e["start"]["time"] < applied - 1e-6]
    assert wait and all(e["vx"] == e["vy"] == e["yaw"] == 0 for e in wait)
    plan = run["search_plan"]
    assert plan["target"] == "blue" and plan["on_found"] == "return_home"
    for key in ("target", "seconds", "on_found"):
        assert revision["original_plan"][key] == revision["updated_plan"][key] == plan[key]
    assert revision["context"]["home"] == revision["home"]
    assert [s["id"] for s in revision["context"]["pending"]] == ["search-1", "home"]
    assert revision["correction"]["steps"] == revision["context"]["pending"]
    assert result["outcome"] == "found" and not result["tasks"]["pending"] and not result["tasks"]["cancelled"]
    assert [s["step"]["id"] for s in result["tasks"]["completed"]] == ["search-1", "home"]
    found = result["found"]["time"]
    scan = next(o for o in run["observations"] if abs(o["time"] - found) < 1e-6)
    assert sum(ray[3] == "blue" for ray in scan["scan"]) >= 2
    assert result["home_error"] <= 0.15 and result["elapsed"] <= plan["seconds"]
    reached = next(s for s in after if math.dist(s["position"][:2], new[-1]) < 0.35)
    assert reached["time"] > applied
    old_goal_reached = next(
        s
        for s in after
        if reached["time"] < s["time"] < found and math.dist(s["position"][:2], revision["old_goal"]) < 0.3
    )
    grid = ObservedMap()
    for observation in run["observations"]:
        if observation["time"] <= received + 1e-6:
            grid.update(observation)

    def xy(p):
        return 70 + (p[0] + 1.3) * 105, 100 + (2.8 - p[1]) * 105

    def poly(points):
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in map(xy, points))

    cells = []
    for group, color in ((grid.free, "#dbe5e8"), (grid.occupied, "#36464e")):
        for cell in group:
            x, y = xy(grid.point(cell))
            cells.append(f'<rect x="{x - 5.25:.1f}" y="{y - 5.25:.1f}" width="10.5" height="10.5" fill="{color}"/>')
    x, y = xy((area[0], area[3]))
    actual = [s["position"] for s in after if s["time"] <= old_goal_reached["time"]]
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="800" viewBox="0 0 1400 800">
<rect width="1400" height="800" fill="#101c24"/><g font-family="sans-serif" fill="white">
<text x="70" y="52" font-size="30" font-weight="bold">制約追加前後の経路と実際の歩行</text>
<text x="70" y="78" font-size="15" fill="#c1d1d7">観測済み地図 / 未観測は暗色 / 位置は真値</text>
{"".join(cells)}
<rect x="{x:.1f}" y="{y:.1f}" width="{(area[2] - area[0]) * 105:.1f}"
height="{(area[3] - area[1]) * 105:.1f}" fill="#ed4d4d" fill-opacity=".8" stroke="#ffb2a4" stroke-width="3"/>
<polyline points="{poly(old)}" fill="none" stroke="#ffc963" stroke-width="5" stroke-dasharray="10 6"/>
<polyline points="{poly(new)}" fill="none" stroke="#42d4f4" stroke-width="5"/>
<polyline points="{poly(actual)}" fill="none" stroke="#f484b4" stroke-width="4"/>
<circle cx="{xy(old[0])[0]:.1f}" cy="{xy(old[0])[1]:.1f}" r="8" fill="white"/>
<g transform="translate(800 145)" font-size="24">
<text y="0" font-size="29" font-weight="bold">青を探して帰還</text>
<text y="60" fill="#ffb2a4">■ エリア2：通行禁止</text>
<text y="105" fill="#ffc963">┄ 受信時の旧経路</text>
<text y="150" fill="#42d4f4">━ 更新後の最初の経路</text>
<text y="195" fill="#f484b4">━ 更新後の実軌跡</text>
<text y="260">{received:.1f}秒　新しい制約を受信</text>
<text y="305">{applied:.1f}秒　制約を適用・再開</text>
<text y="350">{found:.1f}秒　青をセンサーで観測</text>
<text y="395">{result["final"]["time"]:.1f}秒　帰還停止</text>
<text y="465" font-size="18">実軌跡は新たな探索目標を経由し、</text>
<text y="495" font-size="18">旧目標の近くに戻るまでを表示。</text>
<text y="535" font-size="18">判定はロボット中心、0.1秒間隔。</text>
</g></g></svg>'''
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "route-comparison.svg").write_text(svg, encoding="utf-8")
    summary = {
        "log": str(args.log),
        "received": received,
        "applied": applied,
        "found": found,
        "new_goal_reached": reached["time"],
        "old_goal_neared": old_goal_reached["time"],
        "home_error": result["home_error"],
        "minimum_trace_distance": min(distance_to_area(s["position"], area) for s in after),
        "old_route_intersects": True,
        "new_route_clear": True,
        "obstacle_contacts": 0,
    }
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
