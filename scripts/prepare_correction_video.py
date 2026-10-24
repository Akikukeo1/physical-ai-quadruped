"""実MuJoCoの対照録画とログから、判断変更を比較するHyperFramesプレビューを作る。"""

import json
import math
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "outputs/correction-video"
START, RATE, DURATION = 63.0, 1.5, 14.0


def prepare():
    logs = [
        sorted((ROOT / f"outputs/correction-comparison/{name}").glob("plan-*.json"))[-1]
        for name in ("baseline", "corrected")
    ]
    baseline, corrected = [json.loads(path.read_text(encoding="utf-8")) for path in logs]
    for run in (baseline, corrected):
        assert run["status"] == "completed" and run["planner"] == "codex"
        assert run["result"]["outcome"] == "found"
        assert run["scene"] == {"seed": 305, "missing_blue": False}
        assert run["result"]["final"]["time"] >= START + RATE * DURATION
    keys = ("target", "seconds", "on_found")
    assert {k: baseline["search_plan"][k] for k in keys} == {k: corrected["search_plan"][k] for k in keys}
    (revision,) = corrected["corrections"]
    received, applied = revision["received"]["time"], revision["applied_time"]

    def prefix(run):
        return [state for state in run["trace"] if state["time"] < received - 1e-6]

    assert prefix(baseline) == prefix(corrected), "訂正前の物理軌跡が一致しない"
    assert revision["status"] == "accepted" and revision["updated_plan"]["target"] == "green"
    for key in ("seconds", "on_found"):
        assert revision["original_plan"][key] == revision["updated_plan"][key]
    evidence = corrected["result"]["found"]
    assert evidence == corrected["result"]["detections"]["green"]
    assert evidence["time"] < received
    waiting_scans = [
        observation for observation in corrected["observations"] if received <= observation["time"] <= applied
    ]
    assert waiting_scans
    assert all(not any(ray[3] == "green" for ray in observation["scan"]) for observation in waiting_scans), (
        "訂正時に緑が視界内にある"
    )
    position = revision["received"]["position"]
    old = [revision["old_goal"][i] - position[i] for i in (0, 1)]
    home = [revision["home"][i] - position[i] for i in (0, 1)]
    cosine = sum(a * b for a, b in zip(old, home, strict=True)) / (math.hypot(*old) * math.hypot(*home))
    assert cosine < -0.9, "旧目標と帰還先が反対方向になっていない"
    returns = [next(d for d in run["decisions"] if d["phase"] == "return") for run in (baseline, corrected)]
    assert returns[1]["reason"] == "target in observation history"
    assert START < received < applied <= returns[1]["time"] < returns[0]["time"] < START + RATE * DURATION
    assets = PROJECT / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    for source, dest in (("correction-baseline", "baseline"), ("correction-turnaround", "corrected")):
        shutil.copyfile(ROOT / f"outputs/{source}.mp4", assets / f"{dest}.mp4")

    def clip(identifier, text, start, end, style):
        return (
            f'<div id="{identifier}" class="clip label" data-start="{start:.6f}" '
            f'data-duration="{end - start:.6f}" style="{style}">{text}</div>'
        )

    wait = (received - START) / RATE
    switched = (returns[1]["time"] - START) / RATE
    left_return = (returns[0]["time"] - START) / RATE
    labels = "\n".join(
        [
            clip("left-search", "青を探し続ける", 0, left_return, "left:180px;top:216px;color:#173db6"),
            clip("left-return", "青を検出 → 帰還", left_return, DURATION, "left:180px;top:216px;color:#173db6"),
            clip("right-search", "青を探している", 0, wait, "left:1020px;top:216px;color:#173db6"),
            clip("right-wait", "停止して訂正を解釈中", wait, switched, "left:1020px;top:216px;color:#654300"),
            clip("right-return", "緑に変更 → 帰還へ転換", switched, DURATION, "left:1020px;top:216px;color:#096037"),
            clip("instruction-before", "指示：青を探して、最後は出発地点へ戻って", 0, wait, "left:200px;top:790px"),
            clip(
                "instruction",
                f"{received:.1f}秒：人の訂正「青ではなく緑に変更して」",
                wait,
                DURATION,
                "left:200px;top:790px",
            ),
            clip(
                "decision",
                f"{applied:.1f}秒：緑は観測済み → 再探索せず帰還",
                switched,
                DURATION,
                "left:200px;top:842px;color:#096037",
            ),
        ]
    )
    compositions = PROJECT / "compositions"
    compositions.mkdir(exist_ok=True)
    (PROJECT / "index.html").write_text(
        """<!doctype html>
<html lang="ja"><head><meta charset="utf-8">
<script src="https://cdn.jsdelivr.net/npm/gsap@3.14.2/dist/gsap.min.js"></script>
<style>html,body{margin:0;width:1920px;height:1080px;overflow:hidden}#root{width:100%;height:100%}</style>
</head><body><div id="root" data-composition-id="main" data-duration="14" data-width="1920" data-height="1080">
<div id="comparison" class="clip" data-composition-id="comparison" data-composition-src="compositions/comparison.html"
data-start="0" data-duration="14" data-track-index="0" data-track-kind="graphics"
data-width="1920" data-height="1080"></div>
</div><script>
window.__timelines=window.__timelines||{};
window.__timelines.main=gsap.timeline({paused:true});
</script></body></html>
""",
        encoding="utf-8",
    )
    (compositions / "comparison.html").write_text(
        f"""<template>
<style>
#root{{position:absolute;inset:0;background:#f3f1e9;color:#222a30;font-family:"Noto Sans JP";font-weight:400}}
*{{box-sizing:border-box;margin:0}} h1{{position:absolute;left:200px;top:108px;font-size:56px;line-height:1.3}}
.heading{{position:absolute;top:172px;font-size:30px;line-height:1.3}}
.label{{position:absolute;font-size:36px;line-height:1.3;white-space:nowrap}}
video{{position:absolute;top:290px;width:720px;height:480px;object-fit:contain}}
.evidence{{position:absolute;left:200px;top:896px;font-size:32px;line-height:1.4}}
.note{{position:absolute;left:200px;top:946px;font-size:28px;line-height:1.4}}
#clock{{position:absolute;left:1280px;top:118px;font-size:30px;line-height:1.3}}
</style>
<div id="root" data-composition-id="comparison" data-width="1920" data-height="1080">
<h1 id="title">青の探索中に、緑へ訂正する</h1><div id="clock">物理時刻 63.0 秒</div>
<div class="heading" style="left:180px">訂正なし</div><div class="heading" style="left:1020px">訂正あり</div>
<video id="baseline" class="clip" muted playsinline src="assets/baseline.mp4" data-start="0" data-duration="14"
data-media-start="63" data-playback-rate="1.5" data-track-index="1" style="left:180px"></video>
<video id="corrected" class="clip" muted playsinline src="assets/corrected.mp4" data-start="0" data-duration="14"
data-media-start="63" data-playback-rate="1.5" data-track-index="2" style="left:1020px"></video>
{labels}
<div class="evidence">保持：{evidence["time"]:.1f}秒の緑の検出履歴 ／ 出発地点へ帰還 ／ 制限180秒</div>
<div class="note">実MuJoCo・実Codex CLI｜同一配置・訂正前の軌跡は一致｜1.5倍速｜発見＝観測</div>
</div>
<script>
const tl=gsap.timeline({{paused:true}});
const clock={{time:63}};
tl.to(clock,{{time:84,duration:14,ease:"none",onUpdate:()=>{{
document.getElementById("clock").textContent=`物理時刻 ${{clock.time.toFixed(1)}} 秒`;
}}}},0);
window.__timelines.comparison=tl;
</script></template>
""",
        encoding="utf-8",
    )
    summary = {
        "logs": [path.relative_to(ROOT).as_posix() for path in logs],
        "same_physics_before_correction": True,
        "received": received,
        "applied": applied,
        "green_first_seen": evidence["time"],
        "green_visible_during_correction": False,
        "old_goal_vs_home_cosine": cosine,
        "return_started": {"baseline": returns[0]["time"], "corrected": returns[1]["time"]},
        "home_error": {"baseline": baseline["result"]["home_error"], "corrected": corrected["result"]["home_error"]},
        "video": {"source_start": START, "source_end": START + RATE * DURATION, "rate": RATE, "duration": DURATION},
    }
    (PROJECT / "evidence.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    prepare()
