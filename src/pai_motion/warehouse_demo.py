"""Display-only warehouse fixtures and synchronized native MuJoCo recordings."""

import bisect
import json
import math
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path


def add_equipment(scene, geometry, *, door_open=False):
    """Visual bodies never change collisions, range scans or equipment observations."""
    world = scene.find("worldbody")
    points = {p["id"]: p["position"] for p in geometry["points"]}

    def box(parent, name, pos, size, color):
        ET.SubElement(
            parent,
            "geom",
            name=name,
            type="box",
            pos=" ".join(map(str, pos)),
            size=" ".join(map(str, size)),
            rgba=color,
            group="4",
            contype="0",
            conaffinity="0",
        )

    x, y = points["cooler"]
    cooler = ET.SubElement(world, "body", name="warehouse_cooler", pos=f"{x + 0.87} {y} 0")
    box(cooler, "cooler_housing", [0, 0, 0.52], [0.19, 0.38, 0.52], ".22 .47 .63 1")
    for z in (0.15, 0.23, 0.31, 0.75, 0.83, 0.91):
        box(cooler, f"cooler_grille_{z}", [-0.20, 0, z], [0.015, 0.29, 0.016], ".07 .13 .17 1")
    ET.SubElement(
        cooler,
        "geom",
        name="cooler_fan",
        type="cylinder",
        pos="-.21 0 .53",
        size=".17 .015",
        quat=".70710678 0 .70710678 0",
        rgba=".04 .1 .14 1",
        group="4",
        contype="0",
        conaffinity="0",
    )
    for angle in (0, math.pi / 3, -math.pi / 3):
        geom = ET.SubElement(
            cooler,
            "geom",
            type="box",
            pos="-.30 0 .53",
            size=".01 .13 .018",
            euler=f"{angle} 0 0",
            rgba=".65 .77 .8 1",
            group="4",
            contype="0",
            conaffinity="0",
        )
        geom.set("name", f"fan_blade_{angle}")

    x, y = points["supply"]
    panel = ET.SubElement(world, "body", name="warehouse_supply", pos=f"{x} {y - 0.64} 0")
    box(panel, "supply_cabinet", [0, 0, 0.48], [0.28, 0.10, 0.48], ".7 .72 .73 1")
    box(panel, "supply_screen", [0, 0.105, 0.70], [0.16, 0.012, 0.10], ".04 .12 .12 1")
    box(panel, "supply_warning", [-0.15, 0.115, 0.43], [0.07, 0.012, 0.08], "1 .75 .05 1")
    for i in range(3):
        box(panel, f"supply_breaker_{i}", [-0.12 + i * 0.12, 0.12, 0.23], [0.035, 0.025, 0.07], ".1 .12 .15 1")

    x, y = points["door"]
    frame = ET.SubElement(world, "body", name="warehouse_door", pos=f"{x} {y + 0.82} 0")
    for side in (-1, 1):
        box(frame, f"door_post_{side}", [side * 0.46, 0, 0.58], [0.05, 0.07, 0.58], ".7 .75 .8 1")
    box(frame, "door_header", [0, 0, 1.14], [0.51, 0.07, 0.05], ".7 .75 .8 1")
    leaf = ET.SubElement(
        frame, "body", name="warehouse_door_leaf", pos="-.4 0 0", euler=f"0 0 {math.pi / 3 if door_open else 0}"
    )
    box(leaf, "door_leaf", [0.4, 0, 0.55], [0.4, 0.035, 0.55], ".85 .88 .83 1")
    box(leaf, "door_window", [0.4, -0.04, 0.80], [0.18, 0.008, 0.16], ".12 .4 .5 1")
    box(leaf, "door_handle", [0.7, -0.07, 0.45], [0.025, 0.035, 0.1], ".2 .22 .23 1")
    box(leaf, "door_window_back", [0.4, 0.04, 0.80], [0.18, 0.008, 0.16], ".12 .4 .5 1")
    box(leaf, "door_handle_back", [0.7, 0.07, 0.45], [0.025, 0.035, 0.1], ".2 .22 .23 1")


def align_events(log, frames):
    """Map exact simulation event times to nearest recorded frame, without altering footage."""
    if not frames:
        raise ValueError("Recording contains no frames")
    times = [f["sim_time"] for f in frames]
    events = []

    def add(kind, value, reference):
        if value is None:
            return
        index = bisect.bisect_left(times, value)
        options = {max(0, min(len(times) - 1, index)), max(0, index - 1)}
        closest = min(options, key=lambda i: (abs(times[i] - value), i))
        events.append(
            {
                "kind": kind,
                "sim_time": value,
                "frame": closest,
                "video_seconds": frames[closest]["video_seconds"],
                "alignment_error_seconds": times[closest] - value,
                "log_reference": reference,
            }
        )

    for i, entry in enumerate(log["decisions"]):
        add("llm_requested", entry["started"], f"/decisions/{i}/context")
        add("llm_result", entry.get("finished"), f"/decisions/{i}/decision")
    for i, entry in enumerate(log["observations"]):
        add("measurement" if "item" in entry else "inspection", entry["time"], f"/observations/{i}")
    for i, entry in enumerate(log["actions"]):
        for key in ("started", "arrived", "alignment_started", "sampled_at", "finished"):
            add("action_" + key, entry.get(key), f"/actions/{i}")
    add("report", log.get("reported_at"), "/report")
    add("return_started", log.get("return_started"), "/return_started")
    return sorted(events, key=lambda e: e["sim_time"])


class WarehouseRecorder:
    """Two synchronized cameras, 25 fps, one frame per 20 real physics steps."""

    def __init__(self, sink, directory):
        import imageio.v2 as imageio

        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.frames = []
        self.closed = False
        self.renderer = sink.mj.Renderer(sink.model, height=480, width=720)
        self.writers = {}
        try:
            for name in ("overview", "follow"):
                self.writers[name] = imageio.get_writer(
                    self.directory / f"{name}.mp4", fps=25, codec="libx264", macro_block_size=16
                )
        except BaseException:
            self.close()
            raise

    def capture(self, sink):
        from PIL import Image, ImageDraw, ImageFont

        status = sink.demo_status
        observation = status.get("last_observation", {})
        lines = [
            f"SIM {sink.data.time:7.2f}s  {status['phase']}  {status.get('equipment') or ''} "
            f"{status.get('measurement') or ''}",
            "Hypotheses: " + " / ".join(f"{h['cause']}:{h['status']}" for h in status.get("hypotheses", [])),
            f"Evidence {observation.get('id', '-')}: "
            + (
                f"{observation['item']} = {observation.get('value')} {observation.get('unit', '')}"
                if "item" in observation
                else str(observation.get("readings", {}))
            ),
            f"Conclusion: {status.get('conclusion', 'pending')} | kinematic probe / simulated sensing",
        ]
        try:
            font = ImageFont.truetype("DejaVuSans.ttf", 13)
        except OSError:
            font = ImageFont.load_default()
        camera = sink.mj.MjvCamera()
        camera.lookat[:] = [1.7, 0, 0.35]
        camera.distance, camera.azimuth, camera.elevation = 8.4, -35, -58
        for name, writer in self.writers.items():
            if name == "follow":
                camera.lookat[:] = [*sink.data.qpos[:2], 0.48]
                camera.distance, camera.azimuth, camera.elevation = 1.9, -35, -28
            self.renderer.update_scene(sink.data, camera=camera, scene_option=sink.visual_options)
            picture = Image.new("RGB", (720, 560), (12, 18, 23))
            picture.paste(Image.fromarray(self.renderer.render()), (0, 80))
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, 719, 76), fill=(12, 18, 23))
            for i, line in enumerate(lines):
                draw.text((8, 3 + i * 18), line, font=font, fill=(235, 243, 245))
            writer.append_data(sink.np.asarray(picture))
        self.frames.append(
            {
                "frame": len(self.frames),
                "video_seconds": len(self.frames) / 25,
                "sim_time": float(sink.data.time),
                "status": deepcopy(status),
            }
        )

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            for writer in self.writers.values():
                writer.close()
        finally:
            self.renderer.close()
            (self.directory / "frames.json").write_text(json.dumps(self.frames, indent=2), encoding="utf-8")

    def finalize(self, log):
        self.close()
        events = align_events(log, self.frames)
        (self.directory / "timeline.json").write_text(
            json.dumps(events, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        metadata = {
            "fps": 25,
            "width": 720,
            "height": 560,
            "frames": len(self.frames),
            "streams": ["overview.mp4", "follow.mp4"],
            "cameras": {
                "overview": {"lookat": [1.7, 0, 0.35], "distance": 8.4, "azimuth": -35, "elevation": -58},
                "follow": {"lookat": "robot x,y; z=0.48", "distance": 1.9, "azimuth": -35, "elevation": -28},
            },
            "execution_log": "execution.json",
            "frame_index": "frames.json",
            "event_timeline": "timeline.json",
            "sim_time_first_frame": self.frames[0]["sim_time"],
            "sim_time_last_frame": self.frames[-1]["sim_time"],
            "max_event_alignment_error": max(abs(e["alignment_error_seconds"]) for e in events),
            "limits": "native unedited simulation; kinematic probe, analytic sensing; visuals are not colliders",
        }
        (self.directory / "manifest.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        return metadata
