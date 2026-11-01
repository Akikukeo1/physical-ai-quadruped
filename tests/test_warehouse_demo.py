import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.investigation_world import warehouse_scene  # noqa: E402
from pai_motion.warehouse_demo import add_equipment, align_events, configure_fpv  # noqa: E402


class WarehouseDemoTests(unittest.TestCase):
    def test_fpv_reuses_horizontal_150_degree_fov_and_tracks_position_and_yaw(self):
        from types import SimpleNamespace

        camera = SimpleNamespace(lookat=[0, 0, 0])
        vertical = configure_fpv(camera, {"position": [2, 3, 0.36], "yaw": math.pi / 2})
        horizontal = math.degrees(2 * math.atan(math.tan(math.radians(vertical / 2)) * 1.5))
        self.assertAlmostEqual(horizontal, 150)
        self.assertAlmostEqual(camera.lookat[0], 2)
        self.assertAlmostEqual(camera.lookat[1], 3.95)
        self.assertEqual(camera.lookat[2], 0.31)
        self.assertAlmostEqual(camera.azimuth, 90)
        self.assertEqual((camera.distance, camera.elevation), (0.9, -24))

    def test_recording_audit_rejects_dropped_or_mistimed_frames(self):
        from scripts.verify_demo_recording import check_frame_index

        frames = [{"frame": i, "video_seconds": i / 25, "sim_time": 1 + i / 25} for i in range(4)]
        check_frame_index(frames)
        for field in ("frame", "video_seconds", "sim_time"):
            wrong = [dict(f) for f in frames]
            wrong[2][field] += 0.03
            with self.assertRaises(AssertionError):
                check_frame_index(wrong)

    def test_equipment_is_distinct_and_does_not_change_collision_or_sensor_geometry(self):
        scene = ET.fromstring("<mujoco><worldbody/></mujoco>")
        add_equipment(scene, warehouse_scene().geometry, door_open=True)
        self.assertEqual(len(scene.findall("worldbody/body")), 3)
        names = {g.get("name") for g in scene.findall(".//geom")}
        self.assertTrue({"cooler_housing", "supply_cabinet", "door_leaf"} <= names)
        self.assertTrue(
            all(
                g.get("group") == "4" and g.get("contype") == g.get("conaffinity") == "0"
                for g in scene.findall(".//geom")
            )
        )

    def test_events_keep_exact_times_and_map_to_nearest_shared_frame(self):
        frames = [{"frame": i, "video_seconds": i / 25, "sim_time": 1 + i / 25} for i in range(26)]
        log = {
            "decisions": [{"started": 1.11, "finished": 1.5}],
            "observations": [{"time": 1.63}],
            "actions": [{"started": 1.52, "sampled_at": 1.63, "finished": 1.7}],
            "reported_at": 1.8,
            "return_started": 1.84,
        }
        events = align_events(log, frames)
        self.assertTrue(all(abs(e["alignment_error_seconds"]) <= 0.020001 for e in events))
        self.assertEqual(next(e for e in events if e["kind"] == "inspection")["sim_time"], 1.63)
        with self.assertRaises(ValueError):
            align_events(log, [])


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for real MuJoCo")
class WarehouseDemoPhysicsTests(unittest.TestCase):
    def test_visual_equipment_preserves_go2_motion(self):
        import numpy as np

        from pai_motion.executor import Velocity
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        world = warehouse_scene(probe=True)
        plain = MuJoCoGo2Sink(headless=True, search_seed=0, custom_map=world.geometry, probe=True)
        demo = MuJoCoGo2Sink(
            headless=True, search_seed=0, custom_map=world.geometry, probe=True, warehouse_display=True
        )
        try:
            for command in (Velocity(0.18, 0, 0), Velocity(0, 0, 0.35)):
                plain.run_velocity(command, 2)
                demo.run_velocity(command, 2)
                np.testing.assert_allclose(plain.data.qpos, demo.data.qpos, atol=1e-10, rtol=0)
            self.assertEqual(plain.sense()["scan"], demo.sense()["scan"])
        finally:
            plain.close()
            demo.close()


if __name__ == "__main__":
    unittest.main()
