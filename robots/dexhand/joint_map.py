"""DexHand joint mapping: MuJoCo joint names <-> hardware joint names."""

import mujoco
import numpy as np

# MuJoCo joint name -> hardware joint name (12 DOF)
JOINT_MAP = {
    "dex_hand_right-r_f_joint1_3": "th_dip",
    "dex_hand_right-r_f_joint1_2": "th_mcp",
    "dex_hand_right-r_f_joint1_1": "th_rot",
    "dex_hand_right-r_f_joint2_1": "ff_spr",
    "dex_hand_right-r_f_joint2_3": "ff_dip",
    "dex_hand_right-r_f_joint2_2": "ff_mcp",
    "dex_hand_right-r_f_joint3_3": "mf_dip",
    "dex_hand_right-r_f_joint3_2": "mf_mcp",
    "dex_hand_right-r_f_joint4_3": "rf_dip",
    "dex_hand_right-r_f_joint4_2": "rf_mcp",
    "dex_hand_right-r_f_joint5_3": "lf_dip",
    "dex_hand_right-r_f_joint5_2": "lf_mcp",
}


class JointMapper:
    """Maps MuJoCo simulation joints to hardware joint names and reads angles."""

    def __init__(self, model, data):
        self.model = model
        self.data = data

        # Scan model for hinge joints that appear in JOINT_MAP
        self._indices = []  # (qpos_index, hw_name)
        for i in range(model.njnt):
            if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name in JOINT_MAP:
                qpos_idx = model.jnt_qposadr[i]
                self._indices.append((qpos_idx, JOINT_MAP[name]))

    def get_joint_angles(self) -> dict:
        """Read qpos and return {hw_name: degrees}."""
        return {
            hw: float(np.degrees(self.data.qpos[idx]))
            for idx, hw in self._indices
        }
