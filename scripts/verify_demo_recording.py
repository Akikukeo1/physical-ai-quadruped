"""Decode both native recordings and audit frame/event synchronization and the actual mission."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

from scripts.verify_investigation import verify  # noqa: E402


def check_frame_index(frames):
    assert frames, "No frames"
    for i, frame in enumerate(frames):
        assert frame["frame"] == i
        assert abs(frame["video_seconds"] - i / 25) < 1e-9
        assert abs(frame["sim_time"] - frames[0]["sim_time"] - i / 25) < 1e-6


def audit(directory):
    import imageio.v2 as imageio

    def read(name):
        return json.loads((directory / name).read_text(encoding="utf-8"))

    frames, timeline, manifest, log = (
        read(n) for n in ("frames.json", "timeline.json", "manifest.json", "execution.json")
    )
    check_frame_index(frames)
    assert len(frames) == manifest["frames"]
    assert manifest["fps"] == 25
    assert max(abs(e["alignment_error_seconds"]) for e in timeline) <= 0.020001
    mission = verify(log)
    accepted = [e["decision"]["hypotheses"] for e in log["decisions"] if "decision" in e]
    observations = {o["id"]: o for o in log["observations"]}
    hud_changes = []
    previous_status = None
    for frame in frames:
        status = frame["status"]
        if status != previous_status:
            hud_changes.append(
                {
                    "frame": frame["frame"],
                    "video_seconds": frame["video_seconds"],
                    "sim_time": frame["sim_time"],
                    "status": status,
                }
            )
            previous_status = status
        if "hypotheses" in status:
            assert status["hypotheses"] in accepted
        if "last_observation" in status:
            o = status["last_observation"]
            assert observations[o["id"]] == o and o["time"] <= frame["sim_time"] + 1e-6
    selected = {0, len(frames) - 1}
    selected.update(e["frame"] for e in timeline if e["kind"] in ("measurement", "report", "inspection"))
    selected.update(e["frame"] for e in hud_changes if e["status"]["phase"] == "RETURN_HOME")
    images, videos = [], []
    qa = directory / "qa"
    qa.mkdir(exist_ok=True)
    for stream in manifest["streams"]:
        path = directory / stream
        reader = imageio.get_reader(path)
        try:
            metadata = reader.get_meta_data()
            assert abs(metadata["fps"] - 25) < 1e-6
            count = 0
            for i, picture in enumerate(reader):
                assert picture.shape == (manifest["height"], manifest["width"], 3)
                if i in selected:
                    target = qa / f"{path.stem}-{i:05d}.png"
                    imageio.imwrite(target, picture)
                    images.append(target.relative_to(directory).as_posix())
                count += 1
            assert count == len(frames), (stream, count, len(frames))
        finally:
            reader.close()
        videos.append(
            {
                "file": stream,
                "frames_decoded": count,
                "duration": count / 25,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "bytes": path.stat().st_size,
            }
        )
    result = {
        "mission": mission,
        "recording": manifest,
        "videos": videos,
        "qa_images": images,
        "hud_changes": hud_changes,
        "execution_sha256": hashlib.sha256((directory / "execution.json").read_bytes()).hexdigest(),
        "checks": "full decode of both streams, 25fps, identical counts, 40ms simulation frame steps, "
        "event alignment <=20ms, displayed observations and hypotheses agree with logs, mission audit",
    }
    (directory / "qa.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(
        f"PASS: {directory}; {len(frames)} frames/view; {len(frames) / 25:.2f}s; "
        f"home={log['home_error']:.3f}m; conclusion={log['report']['conclusion']}"
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    audit(parser.parse_args().directory)
