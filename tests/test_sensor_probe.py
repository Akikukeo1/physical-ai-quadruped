import json
import math
import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from pai_motion.investigation import InvestigationMotion, load_knowledge, validate_decision  # noqa: E402
from pai_motion.investigation_world import warehouse_scene  # noqa: E402
from pai_motion.motion import InvalidPlan  # noqa: E402
from pai_motion.sensor_probe import execute_measurement, measurement_ports, vent_sample  # noqa: E402
from tests.test_investigation import decision  # noqa: E402


class ProbeTests(unittest.TestCase):
    def test_manual_sections_are_representable_in_probe_output_schema(self):
        root = Path(__file__).resolve().parents[1]
        schema = json.loads((root / "schema/investigation-probe-decision.schema.json").read_text())
        ids = {s["id"] for s in load_knowledge(True)["sections"]}
        self.assertEqual(set(schema["properties"]["knowledge_ids"]["items"]["enum"]), ids)

    def pose(self, angle=0.6, distance=0.26):
        source = [1.6, 0, 0.53]
        direction = [math.cos(angle), math.sin(angle), 0]
        return source, {
            "tip": [s - distance * d for s, d in zip(source, direction, strict=True)],
            "direction": direction,
            "joint_speed": 0,
            "angle": angle,
        }

    def test_field_requires_distance_heading_height_access_window_stillness_deployment_and_los(self):
        source, pose = self.pose()
        result = vent_sample("air_temperature", source, pose, outlet_c=-12, outlet_speed=2)
        self.assertTrue(result["valid"])
        self.assertLess(result["value"], 0)
        invalid = [
            self.pose(distance=1)[1],
            {**pose, "direction": [-1, 0, 0]},
            {**pose, "tip": [pose["tip"][0], pose["tip"][1], 1.0]},
            self.pose(angle=-0.6)[1],
            {**pose, "joint_speed": 0.1},
            {**pose, "angle": 0},
        ]
        for wrong in invalid:
            sample = vent_sample("air_temperature", source, wrong, outlet_c=-12, outlet_speed=2)
            self.assertFalse(sample["valid"])
            self.assertIsNone(sample["value"])
        self.assertIsNone(
            vent_sample("air_temperature", source, pose, outlet_c=-12, outlet_speed=2, clear=False)["value"]
        )
        _, pose = self.pose(angle=-0.6)
        normal = vent_sample("airflow", source, pose, outlet_c=-12, outlet_speed=2)
        failed = vent_sample("airflow", source, pose, outlet_c=-12, outlet_speed=0.05)
        self.assertGreater(normal["value"], 0.8)
        self.assertLess(failed["value"], 0.2)
        self.assertNotEqual(measurement_ports([1, 0])[0]["position"], measurement_ports([1, 0])[1]["position"])

    def test_same_digital_readings_do_not_leak_air_fault_or_measurement_values(self):
        worlds = [warehouse_scene(fault=f, probe=True) for f in ("cooler_fault", "airflow_fault", "door_open")]
        state = {"time": 0, "position": [1, 0, 0.36], "body_velocity": [0, 0, 0], "yaw_rate": 0}
        readings = [w.inspect("cooler", state)["readings"] for w in worlds]
        self.assertEqual(readings[0], readings[1])
        self.assertEqual(readings[1], readings[2])
        self.assertNotIn("supply_air_c", readings[0])
        self.assertEqual(worlds[0].inventory(), worlds[1].inventory())

    def test_measurement_validator_rejects_hidden_values_unknown_items_duplicates_invalid_evidence(self):
        world = warehouse_scene(probe=True)
        context = {
            "probe": True,
            "inventory": world.inventory(),
            "knowledge": load_knowledge(True),
            "observations": [],
            "unavailable": [],
        }
        value = {**decision(), "action": "measure", "measurement": "airflow"}
        value["hypotheses"].append({"cause": "airflow_fault", "status": "possible", "evidence_ids": []})
        self.assertEqual(validate_decision(value, context), value)
        for change in ({"measurement": "surface_temperature"}, {"equipment": "door"}, {"value": 2.0}):
            with self.assertRaises(InvalidPlan):
                validate_decision({**value, **change}, context)
        context["observations"] = [{"id": "i1", "equipment": "cooler", "item": "airflow", "valid": False}]
        with self.assertRaises(InvalidPlan):
            validate_decision(value, context)
        with self.assertRaises(InvalidPlan):
            validate_decision({**value, "measurement": "air_temperature", "evidence_ids": ["i1"]}, context)


@unittest.skipUnless(os.environ.get("PAI_SIM_TEST") == "1", "Set PAI_SIM_TEST=1 for MuJoCo probe tests")
class ProbePhysicsTests(unittest.TestCase):
    def test_identical_leg_contract_mass_inertia_and_locomotion(self):
        import numpy as np

        from pai_motion.executor import Velocity
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        plain = MuJoCoGo2Sink(headless=True)
        probe = MuJoCoGo2Sink(headless=True, probe=True)
        try:
            self.assertEqual((probe.model.nq, probe.model.nv, probe.model.nu), (19, 18, 12))
            for name in (plain.model.body(i).name for i in range(1, plain.model.nbody)):
                a, b = plain.model.body(name).id, probe.model.body(name).id
                np.testing.assert_array_equal(plain.model.body_mass[a], probe.model.body_mass[b])
                np.testing.assert_array_equal(plain.model.body_inertia[a], probe.model.body_inertia[b])
            for cmd in (Velocity(0.18, 0, 0), Velocity(-0.12, 0, 0), Velocity(0, 0, 0.35), Velocity(0, 0, -0.35)):
                plain.run_velocity(cmd, 2)
                probe.run_velocity(cmd, 2)
                np.testing.assert_allclose(probe.data.qpos, plain.data.qpos, atol=1e-10, rtol=0)
            plain.stop()
            probe.stop()
            self.assertGreater(min(s["upright"] for s in probe.trace), 0.9)
            self.assertLess(math.hypot(*probe.observe()["body_velocity"][:2]), 0.05)
        finally:
            plain.close()
            probe.close()

    def test_walk_align_measure_both_channels_stow_and_return_with_unfixed_base(self):
        from pai_motion.executor import Velocity
        from pai_motion.sim_adapter import MuJoCoGo2Sink

        world = warehouse_scene(fault="airflow_fault", probe=True)
        sink = MuJoCoGo2Sink(headless=True, search_seed=0, custom_map=world.geometry, probe=True)
        log = {"scans": []}
        motion = InvestigationMotion(sink, 180, log)
        try:
            self.assertIsNone(world.measure("air_temperature", sink)["value"])
            self.assertTrue(sink.probe_line_clear([1.5, 0, 0.53]))
            self.assertFalse(sink.probe_line_clear([2.0, 0, 0.53]))
            cooler = next(p for p in world.inventory() if p["id"] == "cooler")
            for port in cooler["measurements"]:
                self.assertEqual(motion.go(port["position"]), "reached")
                action = {}
                observed = execute_measurement(motion, port, world.measure, action)
                self.assertTrue(observed["valid"], observed)
                self.assertGreater(abs(action["deployed"]["angle"]), 0.2)
                self.assertLess(abs(action["stowed"]["angle"]), 0.02)
                if port["item"] == "airflow":
                    self.assertLess(observed["value"], 0.2)
                else:
                    self.assertLess(observed["value"], 0)
            motion.settle()
            sink.move_probe(0.6)
            with self.assertRaises(ValueError):
                sink.run_velocity(Velocity(0.1, 0, 0), 0.2)
            sink.move_probe(0)
            self.assertEqual(motion.go(motion.home, returning=True), "reached")
            sink.stop()
            self.assertLess(math.dist(sink.observe()["position"][:2], motion.home), 0.15)
            self.assertFalse(any(s["obstacle_contact"] for s in sink.trace))
        finally:
            sink.close()


if __name__ == "__main__":
    unittest.main()
