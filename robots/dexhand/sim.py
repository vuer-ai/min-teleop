"""DexHand MuJoCo simulation robot."""

from pathlib import Path

import mujoco
import numpy as np

from robot import Robot
from .joint_map import JointMapper

_MODEL_PATH = Path(__file__).parent / "models" / "juggle_cube_dex_hands.mjcf.xml"


class SimRobot(Robot):
    """DexHand simulated in MuJoCo with off-screen rendering."""

    def __init__(self, width: int = 640, height: int = 480, camera: str = "front"):
        self.model = mujoco.MjModel.from_xml_path(str(_MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        # Override "front" camera to see both hand (x≈-0.5) and table/boxes (x≈0)
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front")
        if cam_id >= 0:
            self.model.cam_pos[cam_id] = [-0.25, -0.6, 1.3]
            # Look from y=-0.6 toward +y, slightly down (w,x,y,z)
            self.model.cam_quat[cam_id] = [0.92, 0.39, 0.0, 0.0]
            self.model.cam_fovy[cam_id] = 70

        self.renderer = mujoco.Renderer(self.model, width=width, height=height)
        self.camera = camera
        self.joint_mapper = JointMapper(self.model, self.data)

    # -- Robot interface --

    def step(self, mocap_targets: dict) -> None:
        if mocap_targets:
            for body_name, (pos, quat) in mocap_targets.items():
                try:
                    bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                    mid = self.model.body_mocapid[bid]
                    if mid >= 0:
                        self.data.mocap_pos[mid] = pos
                        self.data.mocap_quat[mid] = quat
                except Exception:
                    pass
        mujoco.mj_step(self.model, self.data)

    def render(self) -> np.ndarray:
        self.renderer.update_scene(self.data, camera=self.camera)
        return self.renderer.render()

    def get_joint_angles(self) -> dict:
        return self.joint_mapper.get_joint_angles()
