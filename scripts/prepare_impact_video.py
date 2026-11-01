"""Build a log-timed HyperFrames composition after the impact run passes validation."""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from pai_motion.search import ObservedMap  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    subprocess.run(
        [sys.executable, str(Path(__file__).with_name("verify_impact_demo.py")), str(args.log), str(args.output)],
        check=True,
    )
    run = json.loads(args.log.read_text(encoding="utf-8"))
    revision = run["corrections"][0]
    start, end = revision["received"]["time"], run["result"]["final"]["time"]
    duration = end - start
    grid = ObservedMap()
    for observation in run["observations"]:
        if observation["time"] <= start + 1e-6:
            grid.update(observation)
    cells = []
    for group, color in ((grid.free, "#dbe5e8"), (grid.occupied, "#36464e")):
        squares = []
        for cx, cy in group:
            x, y = 70 + (cx * 0.1 + 1.3) * 105 - 5.25, 100 + (2.8 - cy * 0.1) * 105 - 5.25
            squares.append(f"M{x:.1f},{y:.1f}h10.5v10.5h-10.5z")
        cells.append(f'<path d="{" ".join(squares)}" fill="{color}"/>')
    data = {
        "start": start,
        "end": end,
        "applied": revision["applied_time"],
        "found": run["result"]["found"]["time"],
        "home": run["result"]["tasks"]["completed"][-1]["time"],
        "deadline": revision["deadline"],
        "area": next(a["bounds"] for a in run["field_areas"] if a["id"] == "area2"),
        "old": revision["old_route"],
        "new": revision["new_route"],
        "trace": [[s["time"], *s["position"][:2], s["yaw"]] for s in run["trace"] if s["time"] >= start - 0.1],
    }
    assets = args.output / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.video, assets / "run.mp4")
    html = """<!doctype html><html lang="ja"><head><meta charset="utf-8">
<script src="https://cdn.jsdelivr.net/npm/gsap@3.14.2/dist/gsap.min.js"></script>
<style>
html,body{margin:0;background:#101c24;color:#eef7fa;font-family:sans-serif;overflow:hidden}
#root{width:100%;height:100%} h1{position:absolute;left:60px;top:25px;font-size:40px;margin:0}
video{position:absolute;left:60px;top:145px;width:960px;height:640px}
#map{position:absolute;left:1050px;top:120px;width:700px;height:650px}
.sub{position:absolute;left:60px;top:90px;font-size:28px;color:#bed2d9}
#status{position:absolute;left:60px;top:820px;font-size:34px;font-weight:bold;color:#ffe09e}
#details{position:absolute;left:60px;top:880px;font-size:28px}
#clock{position:absolute;right:75px;top:40px;font-size:29px}
.legend{position:absolute;left:1080px;top:780px;font-size:28px;line-height:1.5}
</style></head><body><div id="root" data-composition-id="main" data-duration="DURATION"
data-width="1920" data-height="1080">
<h1>未知環境の探索中に、立入禁止を追加</h1><div id="clock"></div>
<div class="sub">実MuJoCo録画 / 青を探索して出発地点へ帰還 / 受信以降を等速再生</div>
<video id="run" class="clip" muted playsinline src="assets/run.mp4" data-start="0" data-duration="DURATION"
data-media-start="START" data-track-index="1"></video>
<svg id="map" viewBox="0 0 700 760"><rect width="700" height="760" fill="#172a34"/>CELLS
<rect id="area" fill="#ed4d4d" fill-opacity=".8" stroke="#ffb2a4" stroke-width="3"/>
<polyline id="old" fill="none" stroke="#ffc963" stroke-width="5" stroke-dasharray="10 6"/>
<polyline id="new" fill="none" stroke="#42d4f4" stroke-width="5"/>
<polyline id="actual" fill="none" stroke="#f484b4" stroke-width="4"/>
<path id="robot" d="M 11 0 L -8 -7 L -8 7 Z" fill="white" stroke="#182b33" stroke-width="2"/></svg>
<div id="status"></div><div id="details"></div>
<div class="legend">赤：指定エリア2　黄破線：旧経路<br>
水色：更新後の最初の経路　桃色：実軌跡<br>白矢印：位置・向き（真値）<br>地図：受信時までのセンサー観測</div>
</div><script>
const d=DATA, xy=p=>[70+(p[0]+1.3)*105,100+(2.8-p[1])*105];
const points=p=>p.map(xy).map(q=>q.join(',')).join(' '), el=id=>document.getElementById(id);
el('old').setAttribute('points',points(d.old)); el('new').setAttribute('points',points(d.new));
const a=xy([d.area[0],d.area[3]]); el('area').setAttribute('x',a[0]); el('area').setAttribute('y',a[1]);
el('area').setAttribute('width',(d.area[2]-d.area[0])*105); el('area').setAttribute('height',(d.area[3]-d.area[1])*105);
const clock={t:d.start}; function update(){const t=clock.t, active=t>=d.applied, found=t>=d.found, home=t>=d.home;
el('clock').textContent=`物理時刻 ${t.toFixed(1)}秒 / 残り ${(Math.max(0,d.deadline-t)).toFixed(1)}秒`;
el('status').textContent=home?'帰還地点に到着・停止中':found?'青を観測 → 帰還中':
active?'制約を適用 → 探索を再開':'新しい指示を受信 → 停止指令・解釈中';
el('details').textContent=active?'エリア2を避ける / 青の探索と帰還条件を保持':
'「エリア2は危険なので通らないで。元の任務と帰還条件はそのまま」';
el('area').style.opacity=active?1:.25; el('new').style.opacity=active?1:0;
const past=d.trace.filter(s=>s[0]<=t); el('actual').setAttribute('points',points(past.map(s=>[s[1],s[2]])));
if(past.length){const s=past[past.length-1], p=xy([s[1],s[2]]);
el('robot').setAttribute('transform',`translate(${p[0]} ${p[1]}) rotate(${-s[3]*180/Math.PI})`);}}
window.__timelines=window.__timelines||{}; const tl=gsap.timeline({paused:true});
tl.to(clock,{t:d.end,duration:DURATION,ease:'none',onUpdate:update},0); update(); window.__timelines.main=tl;
</script></body></html>"""
    html = html.replace("DURATION", f"{duration:.6f}").replace("START", f"{start:.6f}")
    html = html.replace("CELLS", "".join(cells)).replace("DATA", json.dumps(data, separators=(",", ":")))
    (args.output / "index.html").write_text(html, encoding="utf-8")
    (args.output / "hyperframes.json").write_text('{"paths":{"assets":"assets"}}\n', encoding="utf-8")
    (args.output / "package.json").write_text(
        '{"name":"impact-video","private":true,"scripts":{"check":"npx --yes hyperframes@0.8.220 check",'
        '"render":"npx --yes hyperframes@0.8.220 render"}}\n',
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
