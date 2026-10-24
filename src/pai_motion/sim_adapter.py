"""Go2 MoE policy: 50 Hz inference and 500 Hz MuJoCo torque control.

Observation and velocity compensation adapted from syswonder (Apache-2.0).
See third_party/NOTICE.md for revisions and licenses.
"""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from pai_motion.assets import ensure_assets
from pai_motion.executor import Velocity


class MuJoCoGo2Sink:
    """Own physics and joint control; accept only bounded velocity segments."""

    def __init__(
        self,
        *,
        headless: bool = False,
        video: Path | None = None,
        navigation: bool = False,
        dynamic_obstacle: bool = False,
        search_seed: int | None = None,
        missing_target: bool = False,
    ) -> None:
        if dynamic_obstacle and not navigation:
            raise ValueError("dynamic_obstacle requires navigation")
        try:
            import mujoco
            import numpy as np
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError("Install simulation dependencies with: uv sync --extra sim") from exc
        self.mj, self.np = mujoco, np
        self.events: list[dict] = []
        self.trace: list[dict] = []
        self.viewer = self.renderer = self.writer = None
        self.ticks = 0
        self.stopped = False
        self.navigation = navigation
        self.dynamic_obstacle = dynamic_obstacle
        self.search = search_seed is not None
        if self.search and navigation:
            raise ValueError("Search and known-map navigation are separate scenes")
        self.world_events: list[dict] = []
        assets = ensure_assets()
        scene = ET.parse(assets / "go2.xml").getroot()
        scene.find("compiler").set("meshdir", (assets / "assets").as_posix())
        scene.find("option").set("timestep", "0.002")
        additions = ET.fromstring("""<mujoco>
          <visual><global offwidth="720" offheight="480"/></visual>
          <asset><texture name="grid" type="2d" builtin="checker" width="512" height="512"
            rgb1=".18 .22 .26" rgb2=".28 .32 .36"/>
            <material name="floor" texture="grid" texrepeat="8 8" texuniform="true"/></asset>
          <worldbody><light pos="0 -2 3" dir="0 0 -1" directional="true" castshadow="false"/>
            <geom name="ground" type="plane" size="5 5 .1" material="floor" friction=".8 .02 .01"/>
          </worldbody></mujoco>""")
        scene.append(additions.find("visual"))
        scene.find("asset").extend(additions.find("asset"))
        scene.find("worldbody").extend(additions.find("worldbody"))
        if navigation:
            from pai_motion.navigation import OBSTACLE, WAYPOINTS

            x, y, hx, hy = OBSTACLE
            ET.SubElement(
                scene.find("worldbody"),
                "geom",
                name="task_obstacle",
                type="box",
                pos=f"{x} {y} 0.3",
                size=f"{hx} {hy} 0.3",
                rgba="0.85 0.1 0.1 1",
            )
            if dynamic_obstacle:
                from pai_motion.navigation import BLOCKER

                x, _, hx, hy = BLOCKER
                body = ET.SubElement(
                    scene.find("worldbody"), "body", name="moving_blocker", mocap="true", pos=f"{x} 3.0 .3"
                )
                ET.SubElement(body, "geom", name="task_blocker", type="box", size=f"{hx} {hy} .3", rgba="1 .45 .05 1")
            for name, color in (("blue", "0.1 0.35 1 1"), ("green", "0.1 0.85 0.3 1"), ("home", "1 .8 .1 1")):
                x, y = WAYPOINTS[name]
                ET.SubElement(
                    scene.find("worldbody"),
                    "geom",
                    name=f"marker_{name}",
                    type="cylinder",
                    pos=f"{x} {y} .006",
                    size=".18 .005",
                    rgba=color,
                    contype="0",
                    conaffinity="0",
                )
        search_names = []
        if self.search:
            from pai_motion.search_world import search_scene

            for index, item in enumerate(search_scene(search_seed, missing_target)):
                x, y, hx, hy = item["box"]
                name = f"search_object_{index}"
                search_names.append(name)
                ET.SubElement(
                    scene.find("worldbody"),
                    "geom",
                    name=name,
                    type="box",
                    group="5",
                    pos=f"{x} {y} .3",
                    size=f"{hx} {hy} .3",
                    rgba=" ".join(map(str, item["rgba"])),
                )
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        names = [f"{leg}_{joint}" for leg in ("FL", "FR", "RL", "RR") for joint in ("hip", "thigh", "calf")]
        joints = [self.model.joint(f"{name}_joint").id for name in names]
        self.actuators = np.array([self.model.actuator(name).id for name in names])
        self.qpos = self.model.jnt_qposadr[joints]
        self.dofs = self.model.jnt_dofadr[joints]
        if (
            self.model.nq != 19
            or self.model.nv != 18
            or self.model.nu != 12
            or self.model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE
            or not np.array_equal(self.model.actuator_trnid[self.actuators, 0], joints)
            or not np.allclose(self.model.actuator_gear[self.actuators], [1, 0, 0, 0, 0, 0])
        ):
            raise ValueError("Go2 model does not match the released policy joint/torque contract")
        self.base = self.model.body("base_link").id
        self.obstacle_ids = [self.model.geom("task_obstacle").id] if navigation else []
        self.obstacle_ids.extend(self.model.geom(name).id for name in search_names)
        if dynamic_obstacle:
            self.obstacle_ids.append(self.model.geom("task_blocker").id)
            self.blocker_mocap = self.model.body_mocapid[self.model.body("moving_blocker").id]
        self.home = np.array([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5], dtype=np.float32)
        self.action = np.zeros(12, dtype=np.float32)
        self.history = np.zeros((5, 45), dtype=np.float32)
        self.integral = np.zeros(3)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.data.qpos[2] = 0.35
        self.data.qpos[self.qpos] = self.home
        self.data.ctrl[:] = 0
        mujoco.mj_forward(self.model, self.data)
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.policy = ort.InferenceSession(
            str(assets / "policy/moe_rough/policy.onnx"), sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs, outputs = self.policy.get_inputs(), self.policy.get_outputs()
        if (
            len(inputs) != 1
            or inputs[0].name != "history"
            or inputs[0].shape != [1, 225]
            or inputs[0].type != "tensor(float)"
            or outputs[0].name != "actions"
            or outputs[0].shape != [1, 12]
            or outputs[0].type != "tensor(float)"
        ):
            raise ValueError("Go2 ONNX policy signature mismatch")
        try:
            if video is not None:
                import imageio.v2 as imageio

                video.parent.mkdir(parents=True, exist_ok=True)
                self.renderer = mujoco.Renderer(self.model, height=480, width=720)
                self.writer = imageio.get_writer(str(video), fps=25, codec="libx264", macro_block_size=16)
                self.camera = mujoco.MjvCamera()
                self.camera.lookat[:] = [0.25, 0, 0.2]
                self.camera.distance, self.camera.azimuth, self.camera.elevation = 2.7, 135, -25
                if navigation:
                    self.camera.lookat[:] = [0.9, 0, 0.2]
                    self.camera.distance, self.camera.azimuth, self.camera.elevation = 4.5, 110, -55
                if self.search:
                    self.camera.lookat[:] = [1.7, 0, 0]
                    self.camera.distance, self.camera.azimuth, self.camera.elevation = 8.0, 90, -75
            if not headless:
                import mujoco.viewer

                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                self.viewer.cam.distance = 2.7
                self.viewer.cam.elevation = -25
                if navigation:
                    self.viewer.cam.lookat[:] = [0.9, 0, 0.2]
                    self.viewer.cam.distance, self.viewer.cam.elevation = 4.5, -55
                if self.search:
                    self.viewer.cam.lookat[:] = [1.7, 0, 0]
                    self.viewer.cam.distance, self.viewer.cam.elevation = 8.0, -75
            self._advance(np.zeros(3), 1.0)
        except BaseException:
            self.close()
            raise

    def _state(self) -> dict:
        np = self.np
        rotation = self.data.xmat[self.base].reshape(3, 3)
        state = {
            "time": float(self.data.time),
            "position": self.data.qpos[:3].tolist(),
            "yaw": math.atan2(rotation[1, 0], rotation[0, 0]),
            "upright": float(rotation[2, 2]),
            "body_velocity": (rotation.T @ self.data.qvel[:3]).tolist(),
            "yaw_rate": float(self.data.qvel[5]),
            "contacts": int(self.data.ncon),
            "finite": bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()),
        }
        if self.navigation or self.search:
            from pai_motion.navigation import ROBOT_RADIUS

            state["obstacles"] = [
                self.data.geom_xpos[geom, :2].tolist() + self.model.geom_size[geom, :2].tolist()
                for geom in self.obstacle_ids
            ]
            state["obstacle_clearance"] = min(
                math.hypot(max(abs(state["position"][0] - x) - hx, 0), max(abs(state["position"][1] - y) - hy, 0))
                - ROBOT_RADIUS
                for x, y, hx, hy in state["obstacles"]
            )
            state["obstacle_contact"] = any(
                geom in contact.geom for contact in self.data.contact for geom in self.obstacle_ids
            )
        return state

    def observe(self) -> dict:
        """Search exposes oracle odometry, but never hidden geometry or target coordinates."""
        state = self._state()
        if self.search:
            return {name: state[name] for name in ("time", "position", "yaw", "body_velocity", "yaw_rate")}
        return state

    def sense(self) -> dict:
        """Ideal horizontal range scanner with surface color at the FIRST visible ray hit."""
        if not self.search:
            raise RuntimeError("Range/color sensing requires a search scene")
        from pai_motion.search import SENSOR_RANGE

        state = self.observe()
        origin = self.np.array([*state["position"][:2], 0.3])
        group = self.np.array([0, 0, 0, 0, 0, 1], dtype=self.np.uint8)
        geom = self.np.zeros(1, dtype=self.np.int32)
        scan = []
        for index in range(180):
            angle = index * math.tau / 180
            heading = state["yaw"] + angle
            direction = self.np.array([math.cos(heading), math.sin(heading), 0.0])
            distance = self.mj.mj_ray(self.model, self.data, origin, direction, group, 1, -1, geom)
            hit = 0 <= distance <= SENSOR_RANGE
            color = None
            if hit:
                r, g, b = self.model.geom_rgba[geom[0], :3]
                color = (
                    "blue" if b > 0.8 and r < 0.2 and g < 0.4 else "green" if g > 0.8 and r < 0.2 and b < 0.4 else None
                )
            scan.append([angle, float(distance) if hit else SENSOR_RANGE, hit, color])
        return {**state, "scan": scan}

    def _advance(self, command, seconds: float) -> None:
        np, mj = self.np, self.mj
        # shortcut: upstream low-speed compensation; revalidate for other terrain/policies.
        for _ in range(round(seconds / 0.002)):
            start = time.monotonic()
            if self.dynamic_obstacle:
                from pai_motion.navigation import BLOCKER

                x, y, _, _ = BLOCKER
                fraction = min(1.0, max(0.0, (self.data.time - 4.0) / 2.0))
                self.data.mocap_pos[self.blocker_mocap] = [x, 3.0 + (y - 3.0) * fraction, 0.3]
                if not self.world_events and self.data.time >= 4.0 - 1e-9:
                    self.world_events.append({"type": "blocker_motion_started", "time": float(self.data.time)})
                if len(self.world_events) == 1 and self.data.time >= 6.0 - 1e-9:
                    self.world_events.append({"type": "blocker_motion_finished", "time": float(self.data.time)})
            if self.ticks % 10 == 0:
                rotation = self.data.xmat[self.base].reshape(3, 3)
                measured = np.r_[(rotation.T @ self.data.qvel[:3])[:2], self.data.qvel[5]]
                active = np.abs(command) > 1e-6
                self.integral[~active] = 0
                limits = np.array([0.6, 0.35, 1.0])
                raw = np.array([2.0, 2.0, 2.2]) * command + self.integral
                error = command - measured
                integrate = active & ((np.abs(raw) < limits) | (raw * error < 0))
                self.integral += np.array([2.5, 2.5, 5.0]) * error * 0.02 * integrate
                self.integral[:] = np.clip(self.integral, -limits, limits)
                policy_command = np.where(
                    active, np.clip(np.array([2.0, 2.0, 2.2]) * command + self.integral, -limits, limits), 0
                )
                w, x, y, z = self.data.qpos[3:7]
                gravity = [2 * (w * y - x * z), -2 * (w * x + y * z), 2 * (x * x + y * y) - 1]
                self.history[:-1] = self.history[1:]
                self.history[-1] = np.concatenate(
                    (
                        self.data.qvel[3:6] * 0.25,
                        gravity,
                        policy_command * [2, 2, 0.25],
                        self.data.qpos[self.qpos] - self.home,
                        self.data.qvel[self.dofs] * 0.05,
                        self.action,
                    )
                )
                if not np.isfinite(self.history).all():
                    raise RuntimeError("Non-finite Go2 observation")
                result = self.policy.run(["actions"], {"history": self.history.reshape(1, 225)})[0]
                if result.shape != (1, 12) or result.dtype != np.float32 or not np.isfinite(result).all():
                    raise RuntimeError("Invalid Go2 policy action")
                self.action[:] = result[0]
            torques = 20 * (self.home + 0.25 * self.action - self.data.qpos[self.qpos])
            torques -= 0.5 * self.data.qvel[self.dofs]
            bounds = self.model.actuator_ctrlrange[self.actuators]
            self.data.ctrl[self.actuators] = np.clip(torques, bounds[:, 0], bounds[:, 1])
            mj.mj_step(self.model, self.data)
            self.ticks += 1
            state = self._state()
            if not state["finite"] or state["upright"] < 0.65 or state["position"][2] < 0.17:
                self.data.ctrl[:] = 0
                raise RuntimeError(f"Go2 fell or state became invalid at t={self.data.time:.3f}s")
            if state.get("obstacle_contact") or state.get("obstacle_clearance", 1.0) < 0.05:
                self.data.ctrl[:] = 0
                raise RuntimeError(f"Go2 obstacle safety stop at t={self.data.time:.3f}s")
            if self.ticks % 50 == 0:
                self.trace.append(state)
            if self.writer is not None and self.ticks % 20 == 0:
                self.renderer.update_scene(self.data, camera=self.camera)
                self.writer.append_data(self.renderer.render())
            if self.viewer is not None:
                if not self.viewer.is_running():
                    raise RuntimeError("MuJoCo viewer closed")
                if self.ticks % 10 == 0:
                    self.viewer.sync()
                time.sleep(max(0, 0.002 - (time.monotonic() - start)))

    def run_velocity(self, cmd: Velocity, seconds: float) -> None:
        values = self.np.array([cmd.vx, cmd.vy, cmd.yaw])
        if (
            not self.np.isfinite(values).all()
            or self.np.any(self.np.abs(values) > [0.3, 0.2, 0.6])
            or not math.isfinite(seconds)
            or not 0.2 <= seconds <= 2.0
        ):
            raise ValueError("Velocity segment exceeds Go2 limits (.3 m/s, .2 m/s, .6 rad/s, .2–2 s)")
        if self.stopped:
            raise RuntimeError("Go2 sink has already stopped")
        event = {"vx": cmd.vx, "vy": cmd.vy, "yaw": cmd.yaw, "duration": seconds, "start": self._state()}
        self.events.append(event)
        self._advance(values, seconds)
        event["end"] = self._state()

    def stop(self) -> None:
        if self.stopped:
            return
        self.stopped = True
        state = self._state()
        if (
            not state["finite"]
            or state["upright"] < 0.65
            or state["position"][2] < 0.17
            or state.get("obstacle_contact")
            or state.get("obstacle_clearance", 1.0) < 0.05
        ):
            self.data.ctrl[:] = 0
            return
        event = {"vx": 0.0, "vy": 0.0, "yaw": 0.0, "duration": 2.0, "phase": "stop", "start": state}
        self.events.append(event)
        self._advance(self.np.zeros(3), 2.0)
        event["end"] = self._state()

    def close(self) -> None:
        for resource in (self.writer, self.renderer, self.viewer):
            if resource is not None:
                resource.close()
        self.writer = self.renderer = self.viewer = None
