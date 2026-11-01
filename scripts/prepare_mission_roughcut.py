"""Prepare an editable, evidence-labelled rough cut from the recorded restriction trial."""

# ruff: noqa: E501

import csv
import html
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from pai_motion.field_areas import distance_to_area  # noqa: E402
from pai_motion.search import ObservedMap  # noqa: E402

OUT = ROOT / "outputs/mission-roughcut"
LOG = ROOT / "outputs/impact-dev-2/plan-20261030T061825692778Z.json"
VIDEO = ROOT / "outputs/impact-dev-2.mp4"


def svg_panel(title, body, color="#e9f3f5"):
    return (
        '<svg xmlns="http://www.w3.org/2000/svg" width="1920" height="1080" viewBox="0 0 1920 1080">'
        '<rect x="64" y="790" width="1030" height="165" rx="8" fill="#101c24" fill-opacity=".88"/>'
        f'<text x="90" y="850" fill="{color}" font-size="38" font-family="Noto Sans JP, sans-serif" '
        f'font-weight="700">{html.escape(title)}</text>'
        f'<text x="90" y="910" fill="#e9f3f5" font-size="27" font-family="Noto Sans JP, sans-serif">'
        f"{html.escape(body)}</text></svg>"
    )


def map_path(cells):
    return "".join(f"M{x / 10 - 0.05:.2f},{-y / 10 - 0.05:.2f}h.1v.1h-.1z" for x, y in sorted(cells))


def poly(points):
    return " ".join(f"{p[0]:.3f},{-p[1]:.3f}" for p in points)


def main():
    if OUT.exists() and any(OUT.iterdir()):
        raise SystemExit(f"Refusing to overwrite non-empty output: {OUT}")
    data = json.loads(LOG.read_text(encoding="utf-8"))
    revision = data["corrections"][0]
    result = data["result"]
    area = next(a["bounds"] for a in data["field_areas"] if a["id"] == "area2")
    assert data["status"] == "completed" and data["planner"] == "codex" and result["outcome"] == "found"
    assert revision["status"] == "accepted" and revision["blocked_areas"] == ["area2"]
    assert abs(revision["received"]["time"] - 50) < 1e-4 and abs(revision["applied_time"] - 53.8) < 1e-4
    assert any(distance_to_area(point, area) == 0 for point in revision["old_route"])
    assert result["tasks"]["pending"] == [] and result["tasks"]["cancelled"] == []
    assert [c["step"]["id"] for c in result["tasks"]["completed"]] == ["search-1", "home"]
    assert abs(result["detections"]["green"]["time"] - 22) < 1e-4
    assert abs(result["detections"]["blue"]["time"] - 91.8) < 1e-4 and result["home_error"] < 0.15
    assert all(not s["obstacle_contact"] and s["finite"] and s["upright"] > 0.7 for s in data["trace"])
    assert all(
        distance_to_area(s["position"], area) > 0.05
        for s in data["trace"]
        if s["time"] >= revision["applied_time"] - 1e-6
    )
    for directory in (
        "footage",
        "overlays/path",
        "overlays/minimap",
        "overlays/mission-status",
        "overlays/events",
        "audio",
        "subtitles",
        "timeline",
        "logs",
    ):
        (OUT / directory).mkdir(parents=True, exist_ok=True)
    shutil.copyfile(VIDEO, OUT / "footage/overhead.mp4")
    shutil.copyfile(LOG, OUT / "logs/restriction-trial.json")

    grid = ObservedMap()
    maps = []
    for obs in data["observations"]:
        grid.update(obs)
        maps.append([round(obs["time"], 6), map_path(grid.free), map_path(grid.occupied)])
    assert len(maps) > 50 and maps[0][0] <= 1.01 and maps[-1][0] <= result["final"]["time"]
    payload = {
        "maps": maps,
        "area": area,
        "areas": data["field_areas"],
        "old": revision["old_route"],
        "new": revision["new_route"],
        "received": revision["received"]["time"],
        "applied": revision["applied_time"],
        "green": result["detections"]["green"],
        "blue": result["detections"]["blue"],
        "trace": [[round(s["time"], 6), *s["position"][:2], s["yaw"]] for s in data["trace"]],
    }
    (OUT / "map-data.js").write_text(
        "const mapData=" + json.dumps(payload, separators=(",", ":")) + ";\n", encoding="utf-8"
    )
    shutil.copyfile(OUT / "map-data.js", OUT / "overlays/minimap/map-data.js")
    # The standalone vectors are a graphic of recorded grid routes, not a rendered 3-D floor projection.
    x1, y1, x2, y2 = area
    paths = (
        '<svg xmlns="http://www.w3.org/2000/svg" width="720" height="720" viewBox="-1.5 -3 6.4 6.4">'
        f'<rect id="restricted-area" x="{x1}" y="{-y2}" width="{x2 - x1}" height="{y2 - y1}" '
        'fill="#ef5f62" fill-opacity=".3" stroke="#ef5f62" stroke-width=".05"/>'
        f'<polyline id="old-route" points="{poly(revision["old_route"])}" fill="none" stroke="#ffc15a" '
        'stroke-width=".055" stroke-dasharray=".12 .07"/>'
        f'<polyline id="new-first-route" points="{poly(revision["new_route"])}" fill="none" '
        'stroke="#52dbec" stroke-width=".055"/></svg>'
    )
    (OUT / "overlays/path/route-vectors.svg").write_text(paths, encoding="utf-8")
    panels = {
        "overlays/mission-status/mission.svg": ("MISSION", "青を探し、出発地点へ帰還する"),
        "overlays/mission-status/tasks-confirmed.svg": (
            "TASK STATUS",
            "青の観測 → 帰還。緑の再確認はこの試行では未実施",
        ),
        "overlays/mission-status/constraint.svg": ("CONSTRAINT", "エリア2を回避：物理時刻53.8秒から有効"),
        "overlays/mission-status/observation.svg": (
            "OBSERVATION HISTORY",
            "緑を22.0秒に観測。紫は観測時のロボット位置",
        ),
        "overlays/mission-status/clock-template.svg": (
            "SIMULATION TIME / TEMPLATE",
            "実験時刻・残り時間は編集時に設定",
        ),
        "overlays/mission-status/result.svg": ("MISSION RESULT", "青を観測・帰還　誤差 0.115 m　障害物接触 0 件"),
        "overlays/events/green-observed.svg": ("OBSERVATION", "緑をセンサーで観測。位置は観測時のロボット位置"),
        "overlays/events/restriction-received.svg": ("NEW RESTRICTION", "エリア2を避ける。元の青探索と帰還条件は維持"),
        "overlays/events/route-blocked.svg": ("PREVIOUS ROUTE INVALID", "旧格子経路は新しい禁止区域と交差"),
        "overlays/events/revision-placeholder.svg": (
            "PLACEHOLDER / 未実施",
            "緑の再確認と禁止区域を同時に維持する連続実行は未実証",
        ),
    }
    for name, (title, body) in panels.items():
        (OUT / name).write_text(svg_panel(title, body), encoding="utf-8")
    task_stack = """<svg xmlns="http://www.w3.org/2000/svg" width="1920" height="1080" viewBox="0 0 1920 1080">
<rect id="background" x="1100" y="620" width="730" height="260" rx="8" fill="#101c24" fill-opacity=".88"/>
<g font-family="Noto Sans JP, sans-serif" fill="#eef6f7">
<text id="title" x="1130" y="670" font-size="32" font-weight="700">TASK STACK / 53.8秒</text>
<text id="search" x="1130" y="725" font-size="28">1. 青の探索（継続中）</text>
<text id="home" x="1130" y="775" font-size="28">2. 出発地点へ帰還（待機中）</text>
<text id="not-added" x="1130" y="835" font-size="24" fill="#bdcdd0">緑の再確認はこの試行では追加されていない</text>
</g></svg>"""
    (OUT / "overlays/mission-status/task-stack.svg").write_text(task_stack, encoding="utf-8")
    shots = [
        ("A", 0, 8, "実験A / 任務開始", "footage/overhead.mp4", 1, 9, 1, "青探索 / 帰還"),
        ("B", 8, 25, "実験A / 自律探索", "footage/overhead.mp4", 9, 43, 2, "緑観測 t=22"),
        ("B", 25, 38, "実験A / 制約受信と経路変更", "footage/overhead.mp4", 44, 57, 1, "受信50 / 適用53.8"),
        ("C", 38, 53, "PLACEHOLDER / 複合指示", "", "", "", "", "未実施。禁止区域と緑再確認を同時に要求"),
        ("D", 53, 68, "PLACEHOLDER / タスク訂正", "", "", "", "", "未実証。同一実験の成功として表示しない"),
        ("E", 68, 92, "実験A / 経路変更後の歩行", "footage/overhead.mp4", 57, 93, 1.5, "青観測91.8 / 緑再確認なし"),
        ("F", 92, 103, "実験A / 帰還", "footage/overhead.mp4", 93, 113.4, 20.4 / 11, "113.4停止 / 誤差0.115m"),
        ("F", 103, 110, "実験A / 検証結果", "", "", "", "", "青観測・帰還 / 接触0 / 連続複合試行ではない"),
    ]
    with (OUT / "timeline/shots.csv").open("w", encoding="utf-8-sig", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(
            [
                "scene",
                "video_start_s",
                "video_end_s",
                "description",
                "source",
                "source_in_sim_s",
                "source_out_sim_s",
                "speed",
                "notes",
            ]
        )
        writer.writerows(shots)
    events = [
        {"video_s": 14.5, "simulation_s": 22, "event": "green_observed", "source": "restriction-trial"},
        {
            "video_s": 31,
            "simulation_s": 50,
            "event": "restriction_received_old_route_intersects",
            "source": "restriction-trial",
        },
        {"video_s": 34.8, "simulation_s": 53.8, "event": "restriction_applied", "source": "restriction-trial"},
        {"video_s": 38, "simulation_s": None, "event": "placeholder_combined_instruction", "source": None},
        {"video_s": 53, "simulation_s": None, "event": "placeholder_combined_revision", "source": None},
        {"video_s": 91.2, "simulation_s": 91.8, "event": "blue_observed", "source": "restriction-trial"},
        {"video_s": 103, "simulation_s": 113.4, "event": "stopped_at_home", "source": "restriction-trial"},
    ]
    (OUT / "timeline/events.json").write_text(json.dumps(events, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    subtitles = [
        (0, 8, "青い対象を探し、出発地点へ帰還する。"),
        (8, 15, "センサーの観測から未知の環境を探索する。"),
        (15, 25, "緑の対象を観測し、履歴に記録した。"),
        (25, 31, "未知環境の探索を続ける。"),
        (31, 34.8, "人から通行制約を受信し、停止する。"),
        (34.8, 38, "旧経路は指定区域と交差。制約を適用する。"),
        (38, 53, "未実施：禁止区域を維持しながら緑の再確認を追加する指示。"),
        (53, 68, "未実証：上記を同一任務で解釈し、再確認する場面。"),
        (68, 81, "実験Aの続き。禁止区域を避け、別の経路を歩く。"),
        (81, 91.2, "禁止区域を避けて青の探索を続ける。"),
        (91.2, 95, "青をセンサーで観測し、帰還を開始する。"),
        (95, 103, "元の帰還条件を維持し、出発地点へ戻る。"),
        (103, 110, "青を観測して帰還。帰還誤差0.115 m、障害物接触0件。"),
    ]

    def stamp(t):
        ms = round(t * 1000)
        return f"{ms // 3600000:02}:{ms // 60000 % 60:02}:{ms // 1000 % 60:02},{ms % 1000:03}"

    (OUT / "subtitles/subtitles.srt").write_text(
        "\n\n".join(f"{i}\n{stamp(a)} --> {stamp(b)}\n{text}" for i, (a, b, text) in enumerate(subtitles, 1)) + "\n",
        encoding="utf-8-sig",
    )
    (OUT / "audio/cue-sheet.md").write_text(
        """# 音声・音楽キュー（未収録）

仮ナレーションは `subtitles/subtitles.srt` と同文。BGM・効果音を焼き込まない。

| 編集時刻 | 仮キュー |
| --- | --- |
| 00:00 | 控えめなBGM開始、導入フェードイン |
| 00:14.5 | 緑を観測。短い控えめな通知音 |
| 00:31 | 制約受信。短い通知音、BGMを少し下げる |
| 00:34.8 | 制約適用。短い決定音 |
| 00:38–01:08 | 未実施のプレースホルダー。ナレーションで仮シーンと明示 |
| 01:08 | 実験Aに復帰。BGMを戻す |
| 01:31.2 | 青を観測。控えめな通知音 |
| 01:43–01:50 | 結果表示。BGMを徐々にフェードアウト |

音源そのものは含まない。ライセンスが確認された素材を編集時に追加する。
""",
        encoding="utf-8",
    )
    (OUT / "hyperframes.json").write_text('{"paths":{"assets":"footage"}}\n', encoding="utf-8")
    (OUT / "package.json").write_text(
        '{"name":"mission-roughcut","private":true,"scripts":{"check":"npx --yes hyperframes@0.8.223 check","render":"npx --yes hyperframes@0.8.223 render"}}\n',
        encoding="utf-8",
    )
    (OUT / "overlays/minimap/hyperframes.json").write_text("{}\n", encoding="utf-8")
    (OUT / "overlays/minimap/package.json").write_text(
        '{"name":"minimap-overlay","private":true,"scripts":{"check":"npx --yes hyperframes@0.8.223 check","render":"npx --yes hyperframes@0.8.223 render"}}\n',
        encoding="utf-8",
    )
    make_html()
    (OUT / "README.md").write_text(README, encoding="utf-8")
    print(f"Prepared {OUT}; run HyperFrames in the output and overlays/minimap as documented.")


SVG = """<svg id="map" viewBox="-1.5 -3 6.4 6.4" aria-label="観測地図">
<path id="free" data-layout-allow-overflow fill="#dbe5e8"/><path id="occupied" data-layout-allow-overflow fill="#a87462"/>
<rect id="area1" fill="#32b9d0" fill-opacity=".3" stroke="#32b9d0" stroke-width=".05"/>
<rect id="area2" fill="#32b9d0" fill-opacity=".3" stroke="#32b9d0" stroke-width=".05"/>
<polyline id="old" fill="none" stroke="#ffc15a" stroke-width=".055" stroke-dasharray=".12 .07"/>
<polyline id="new" fill="none" stroke="#52dbec" stroke-width=".055"/>
<polyline id="trail" fill="none" stroke="#f484b4" stroke-width=".04"/>
<circle id="green" r=".12" fill="#c295f2" stroke="#49355b" stroke-width=".03"/>
<path id="robot" d="M .1 0 L -.075 -.06 L -.075 .06 Z" fill="white" stroke="#182b33" stroke-width=".02"/>
</svg>"""

JS = """const el=id=>document.getElementById(id), coords=ps=>ps.map(p=>`${p[0]},${-p[1]}`).join(' ');
el('old').setAttribute('points',coords(mapData.old));el('new').setAttribute('points',coords(mapData.new));
mapData.areas.forEach((item,i)=>{const a=item.bounds,b=el(`area${i+1}`);b.setAttribute('x',a[0]);
b.setAttribute('y',-a[3]);b.setAttribute('width',a[2]-a[0]);b.setAttribute('height',a[3]-a[1]);});
let cached=-1;function draw(p){let i=0;while(i+1<mapData.maps.length&&mapData.maps[i+1][0]<=p+1e-7)i++;
if(i!==cached){el('free').setAttribute('d',mapData.maps[i][1]);el('occupied').setAttribute('d',mapData.maps[i][2]);cached=i;}
el('old').setAttribute('opacity',p>=50?'1':'0');el('new').setAttribute('opacity',p>=53.8?'1':'0');
el('area2').setAttribute('fill',p>=53.8?'#ef5f62':'#32b9d0');el('area2').setAttribute('stroke',p>=53.8?'#ef5f62':'#32b9d0');
const past=mapData.trace.filter(s=>s[0]<=p+1e-7);el('trail').setAttribute('points',coords(past.map(s=>[s[1],s[2]])));
if(past.length){const s=past[past.length-1];el('robot').setAttribute('transform',`translate(${s[1]} ${-s[2]}) rotate(${-s[3]*180/Math.PI})`);}
el('green').setAttribute('opacity',p>=mapData.green.time?'1':'0');el('green').setAttribute('cx',mapData.green.position[0]);
el('green').setAttribute('cy',-mapData.green.position[1]);}"""


def make_html():
    video_clips = [
        ("a", 0, 8, 1, 1),
        ("b", 8, 17, 9, 2),
        ("c", 25, 13, 44, 1),
        ("d", 68, 24, 57, 1.5),
        ("e", 92, 11, 93, 20.4 / 11),
    ]
    videos = "\n".join(
        f'<video id="{n}" class="clip" muted playsinline src="footage/overhead.mp4" data-start="{t}" '
        f'data-duration="{d}" data-media-start="{s}" data-playback-rate="{speed}" data-track-index="1"></video>'
        for n, t, d, s, speed in video_clips
    )
    html_text = """<!doctype html><html lang="ja"><head><meta charset="utf-8">
<script src="https://cdn.jsdelivr.net/npm/gsap@3.14.2/dist/gsap.min.js"></script>
<style>html,body{margin:0;background:#101c24;color:#eef6f7;font-family:sans-serif;overflow:hidden}
#root{width:100%;height:100%;position:relative}#head{position:absolute;top:35px;left:65px;font-size:38px;font-weight:700}
#tag{position:absolute;top:45px;right:60px;font-size:25px;color:#98cbd2}#footage{position:absolute;left:55px;top:145px;width:1020px;height:680px;overflow:hidden}
video{position:absolute;width:100%;height:100%;object-fit:contain;background:#101c24}
#mapbox{position:absolute;left:1100px;top:130px;width:720px;height:720px;background:#172a34}#map{width:100%;height:100%}
#status{position:absolute;left:65px;top:855px;font-size:38px;font-weight:700;color:#ffda97}
#detail{position:absolute;left:65px;top:918px;font-size:28px}#legend{position:absolute;left:1100px;top:860px;font-size:25px;line-height:1.4}
#note{position:absolute;left:65px;bottom:24px;font-size:23px;color:#b6cbd0}
#placeholder{position:absolute;inset:0;background:#101c24;padding:160px 115px;box-sizing:border-box}
#ph-title{font-size:60px;font-weight:700;color:#ffe0a1}#ph-body{font-size:38px;line-height:1.6;margin-top:60px}
#ph-note{font-size:29px;color:#b8cbd0;margin-top:60px}#result{position:absolute;inset:0;background:#101c24;padding:130px 120px;box-sizing:border-box}
#result h2{font-size:60px;color:#ffe0a1}#result p{font-size:37px;line-height:1.5}#result .small{font-size:27px;color:#b9cbd0}
</style></head><body><div id="root" data-composition-id="main" data-width="1920" data-height="1080" data-duration="110">
<div id="head">未知環境の探索と、人による制約変更</div><div id="tag"></div><div id="footage">VIDEOS</div>
<div id="mapbox">SVG</div><div id="status"></div><div id="detail"></div>
<div id="legend">黄：旧経路　水色：更新後最初の経路<br>赤：禁止区域　桃：実軌跡<br>紫：緑の観測時のロボット位置</div>
<div id="note">実験A：実MuJoCo / 実Codex CLI ｜ 観測地図と真値オドメトリ ｜ 映像は編集上の仮尺</div>
<div id="placeholder"><div id="ph-title"></div><div id="ph-body"></div><div id="ph-note"></div></div>
<div id="result"><h2>実験A：青を観測し、帰還</h2><p>青探索と帰還条件を保持</p>
<p>帰還誤差 0.115 m ／ 障害物への接触 0件</p><p>禁止適用後の区域からの最短距離 0.614 m</p>
<p class="small">ロボット中心の0.1秒間隔の記録。対象はセンサーで観測。</p>
<p class="small">緑の再確認と禁止区域を組み合わせた任務は、この実行では未実施。</p></div>
</div><script src="map-data.js"></script><script>MAPJS
const c={t:0};function update(){const t=c.t,ph=t>=38&&t<68,end=t>=103;
el('placeholder').style.display=ph?'block':'none';el('result').style.display=end?'block':'none';
for(const id of ['head','tag','status','detail','legend','note'])el(id).style.opacity=ph||end?'0':'1';
if(ph){el('ph-title').textContent=t<53?'PLACEHOLDER / 複合指示は未実施':'PLACEHOLDER / 同一任務での再確認は未実証';
el('ph-body').textContent=t<53?'「緑を再確認し、青の探索を続ける。禁止区域を避けて帰還」':'別試行で緑の再確認は実証済み。ただし、この禁止区域の実行とは別。';
el('ph-note').textContent='ここは企画上の仮シーンです。映像・計画・結果を捏造していません。';return;}
if(end)return;const p=t<8?1+t:t<25?9+(t-8)*2:t<38?44+(t-25):t<92?57+(t-68)*1.5:93+(t-92)*20.4/11;
draw(p);el('tag').textContent=`実験A｜物理時刻 ${p.toFixed(1)} 秒｜${t<8||t>=25&&t<38?'等速':t<25?'2倍速':t<92?'1.5倍速':'約1.85倍速'}`;
el('status').textContent=p<22?'青を探して探索中':p<50?'緑を観測。青の探索を継続':p<53.8?'新しい制約を受信 → 停止・解釈中':p<91.8?'旧経路は使用不可 → 別の探索経路へ':'青を観測 → 帰還中';
el('detail').textContent=p<50?'青探索と帰還が元の任務':p<53.8?'「エリア2は危険なので通らないで」':'元の青探索・帰還条件を保持';}
window.__timelines=window.__timelines||{};const tl=gsap.timeline({paused:true});tl.to(c,{t:110,duration:110,ease:'none',onUpdate:update});update();window.__timelines.main=tl;
</script></body></html>"""
    html_text = html_text.replace("VIDEOS", videos).replace("SVG", SVG).replace("MAPJS", JS)
    (OUT / "index.html").write_text(html_text, encoding="utf-8")
    mini = """<!doctype html><html><head><meta charset="utf-8"><script src="https://cdn.jsdelivr.net/npm/gsap@3.14.2/dist/gsap.min.js"></script>
<style>html,body{margin:0;background:transparent;overflow:hidden}#root,#map{width:100%;height:100%}</style></head><body>
<div id="root" data-composition-id="main" data-width="640" data-height="640" data-duration="13">SVG</div>
<script src="map-data.js"></script><script>MAPJS
const c={t:0};window.__timelines=window.__timelines||{};const tl=gsap.timeline({paused:true});
tl.to(c,{t:13,duration:13,ease:'none',onUpdate:()=>draw(44+c.t)});draw(44);window.__timelines.main=tl;
</script></body></html>""".replace("SVG", SVG).replace("MAPJS", JS)
    (OUT / "overlays/minimap/index.html").write_text(mini, encoding="utf-8")


README = """# Cinematic Mission Demo / Rough Cut

このラフは開発用 seed 305 の**実験A**（`logs/restriction-trial.json`）を使用する。
動画上の00:38–01:08は未実施の複合指示・緑再確認を示すプレースホルダー。
01:08以降は同じ実験Aの続きであり、緑の再確認は行っていない。
別試行の再確認成功については `docs/TASK_REVISION_RESULTS.md`。混ぜて連続実行を装わない。

- `roughcut.mp4`: 無音、1920×1080、30 fps、110秒。字幕は焼き込まずSRTを別添。
- `footage/overhead.mp4`: 実験Aの無加工MuJoCo俯瞰録画（720×480、25 fps、約113.4秒、音声なし）。
- `overlays/mission-status/` と `overlays/events/`: 透過SVG、テキストと背景はSVG要素として分離。
- `overlays/path/route-vectors.svg`: 透過SVGの旧・新格子経路と禁止区域。3Dの床上投影ではない。
- `overlays/minimap/`: 00:25–00:38向け観測地図アニメーションの編集用HTMLとProRes 4444透過MOV。
  白色は観測済み、茶色は検出面、透明は未観測。紫は対象位置ではなく緑を観測したロボット位置。
- `audio/cue-sheet.md`: 仮ナレーション・BGM・効果音のキュー。音源は含まず映像にも焼き込まない。
- `subtitles/subtitles.srt`, `timeline/shots.csv`, `timeline/events.json`: 編集時刻と実験時刻の対応。
- `index.html`, `map-data.js`: 再編集できるHyperFrames構成。`logs/`に元の実験Aログのコピー。

## 未収録・限界

この記録に関節姿勢はなく、当時のFPV・斜め・横・低角度ショットを忠実に後生成できない。
新しい実行で同一ステップに複数カメラを録画する変更は可能だが、別の試行になる。
具体的には、既存の `_advance` の録画時点で同じ MuJoCo `data` を使い、カメラごとに `update_scene` と `render` を呼んで別々の writer に保存する。前方視点にはロボット本体に固定した仮想カメラが必要で、この追加収録は未実装・未検証。
床上の3D経路帯・遠近遮蔽は未実装。SVGと地図はログ上の2D格子経路。
初期目標から別の探索目標を選び直したため、同じ終点の2経路の比較ではない。
黒い未観測部分や物体内部は真値地図から補完しない。観測は理想的な距離・表面色センサーでありカメラ認識ではない。
区域の距離はロボット中心の離散時刻判定で、実機や連続時間の安全を保証しない。
FCPXMLはResolve実機検証ができないため含めない。

## 再現

空の `outputs/mission-roughcut` に対し `.venv/Scripts/python.exe scripts/prepare_mission_roughcut.py` を実行。
既存ファイルを上書きしないため、空でなければ中断する。
各ディレクトリで `npx --yes hyperframes@0.8.223 check` を実行し、次を実行する。

```powershell
cd outputs/mission-roughcut
npx --yes hyperframes@0.8.223 render --quality delivery --fps 30 --output roughcut.mp4
cd overlays/minimap
npx --yes hyperframes@0.8.223 render --format mov --fps 30 --output restriction-map-alpha.mov
```

地図MOVは映像の00:25から重ねる。出力ファイルの有無とalphaは書き出し後に確認すること。
"""


if __name__ == "__main__":
    main()
