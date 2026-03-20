"""G1 retargeting: HAND_MOVE event → arm joint angles + hand joint angles.

Coordinate transforms from xr_teleoperate/teleop/televuer/src/televuer/tv_wrapper.py.
Hand retargeting via dex_retargeting library with Dex3 config.
"""

import logging
import os

import numpy as np
import yaml
from dex_retargeting import RetargetingConfig

from .arm_ik import G1ArmIK

log = logging.getLogger(__name__)

_ASSETS_DIR = os.path.join(os.path.dirname(__file__), "assets")

# ---------------------------------------------------------------------------
# Coordinate transform constants (from tv_wrapper.py:89-137)
# ---------------------------------------------------------------------------

# Basis change: OpenXR (Y-up, Z-back, X-right) ↔ Robot (Z-up, Y-left, X-front)
T_ROBOT_OPENXR = np.array([[ 0, 0,-1, 0],
                            [-1, 0, 0, 0],
                            [ 0, 1, 0, 0],
                            [ 0, 0, 0, 1]], dtype=float)

T_OPENXR_ROBOT = np.array([[ 0,-1, 0, 0],
                            [ 0, 0, 1, 0],
                            [-1, 0, 0, 0],
                            [ 0, 0, 0, 1]], dtype=float)

# Initial pose: OpenXR arm convention → Unitree arm URDF convention
T_TO_UNITREE_LEFT_ARM  = np.array([[1, 0, 0, 0],
                                    [0, 0,-1, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 0, 1]], dtype=float)

T_TO_UNITREE_RIGHT_ARM = np.array([[1, 0, 0, 0],
                                    [0, 0, 1, 0],
                                    [0,-1, 0, 0],
                                    [0, 0, 0, 1]], dtype=float)

# Initial pose: OpenXR hand convention → Unitree hand URDF convention
T_TO_UNITREE_HAND = np.array([[ 0, 0, 1, 0],
                               [-1, 0, 0, 0],
                               [ 0,-1, 0, 0],
                               [ 0, 0, 0, 1]], dtype=float)

# Default head pose when tracking unavailable
CONST_HEAD_POSE = np.array([[1, 0, 0, 0],
                             [0, 1, 0, 1.5],
                             [0, 0, 1,-0.2],
                             [0, 0, 0, 1]], dtype=float)


def _fast_mat_inv(mat):
    """Fast inverse for SE(3) matrices."""
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret


def _extract_wrist_mat(hand_data, side):
    """Extract wrist 4x4 from HAND_MOVE event (landmark index 0, column-major)."""
    poses = hand_data.get(side)
    if poses is None or len(poses) < 25 * 16:
        return None
    return np.array(poses[0:16]).reshape(4, 4).T  # column-major → row-major


def _extract_hand_positions(hand_data, side):
    """Extract 25 hand joint positions (25, 3) from HAND_MOVE event."""
    poses = hand_data.get(side)
    if poses is None or len(poses) < 25 * 16:
        return None
    positions = np.zeros((25, 3))
    for i in range(25):
        mat = np.array(poses[16 * i : 16 * i + 16]).reshape(4, 4).T
        positions[i] = mat[:3, 3]
    return positions


# ---------------------------------------------------------------------------
# Hand Retargeting (Dex3)
# ---------------------------------------------------------------------------
class _Dex3HandRetargeting:
    """Wraps dex_retargeting for Unitree Dex3 hands."""

    def __init__(self, assets_dir=None):
        assets_dir = assets_dir or _ASSETS_DIR
        yml_path = os.path.join(assets_dir, "unitree_hand", "unitree_dex3.yml")

        RetargetingConfig.set_default_urdf_dir(assets_dir)

        with open(yml_path, "r") as f:
            cfg = yaml.safe_load(f)

        left_cfg = RetargetingConfig.from_dict(cfg["left"])
        right_cfg = RetargetingConfig.from_dict(cfg["right"])
        self.left_retargeting = left_cfg.build()
        self.right_retargeting = right_cfg.build()

        self.left_indices = self.left_retargeting.optimizer.target_link_human_indices
        self.right_indices = self.right_retargeting.optimizer.target_link_human_indices

        # Joint reordering: dex_retargeting output → DDS hardware order
        left_joint_names = self.left_retargeting.joint_names
        right_joint_names = self.right_retargeting.joint_names

        # Left hand DDS order (from robot_hand_unitree.py Dex3_1_Left_JointIndex)
        left_hw_names = [
            "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
            "left_hand_middle_0_joint", "left_hand_middle_1_joint",
            "left_hand_index_0_joint", "left_hand_index_1_joint",
        ]
        # Right hand DDS order (from robot_hand_unitree.py Dex3_1_Right_JointIndex)
        right_hw_names = [
            "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
            "right_hand_index_0_joint", "right_hand_index_1_joint",
            "right_hand_middle_0_joint", "right_hand_middle_1_joint",
        ]

        self._left_reorder = [left_joint_names.index(n) for n in left_hw_names]
        self._right_reorder = [right_joint_names.index(n) for n in right_hw_names]

        log.info("Dex3 hand retargeting ready")

    def retarget(self, left_hand_pos, right_hand_pos):
        """Retarget hand positions to Dex3 joint angles.

        Args:
            left_hand_pos:  (25, 3) hand positions in Unitree hand URDF frame.
            right_hand_pos: (25, 3) hand positions in Unitree hand URDF frame.

        Returns:
            (left_q[7], right_q[7]) joint angles in DDS hardware order.
        """
        ref_left = left_hand_pos[self.left_indices[1, :]] - left_hand_pos[self.left_indices[0, :]]
        ref_right = right_hand_pos[self.right_indices[1, :]] - right_hand_pos[self.right_indices[0, :]]

        left_q = self.left_retargeting.retarget(ref_left)[self._left_reorder]
        right_q = self.right_retargeting.retarget(ref_right)[self._right_reorder]

        return left_q, right_q


# ---------------------------------------------------------------------------
# G1Retarget: full pipeline
# ---------------------------------------------------------------------------
class G1Retarget:
    """Complete VR → G1 retargeting pipeline.

    Transforms HAND_MOVE events into arm joint angles (14 DoF) and
    Dex3 hand joint angles (7 DoF × 2).
    """

    def __init__(self, urdf_dir=None, assets_dir=None):
        self.arm_ik = G1ArmIK(urdf_dir=urdf_dir)
        self.hand_retarget = _Dex3HandRetargeting(assets_dir=assets_dir)

    def __call__(self, event_value):
        """Process a HAND_MOVE event.

        Args:
            event_value: dict from event.value with 'left' and 'right' keys,
                each containing 25*16 floats (column-major 4x4 matrices).

        Returns:
            (arm_q[14], arm_tau[14], left_hand_q[7], right_hand_q[7]) or None.
        """
        # 1. Extract wrist poses
        left_wrist = _extract_wrist_mat(event_value, "left")
        right_wrist = _extract_wrist_mat(event_value, "right")
        if left_wrist is None or right_wrist is None:
            return None

        # 2. Extract hand positions
        left_hand_pos = _extract_hand_positions(event_value, "left")
        right_hand_pos = _extract_hand_positions(event_value, "right")
        if left_hand_pos is None or right_hand_pos is None:
            return None

        # ----- Arm transform pipeline (tv_wrapper.py:260-308) -----

        # Basis change: OpenXR → Robot convention (similarity transform)
        left_arm = T_ROBOT_OPENXR @ left_wrist @ T_OPENXR_ROBOT
        right_arm = T_ROBOT_OPENXR @ right_wrist @ T_OPENXR_ROBOT

        # Initial pose change: OpenXR arm → Unitree arm URDF
        left_arm = left_arm @ T_TO_UNITREE_LEFT_ARM
        right_arm = right_arm @ T_TO_UNITREE_RIGHT_ARM

        # Head-relative translation
        head = T_ROBOT_OPENXR @ CONST_HEAD_POSE @ T_OPENXR_ROBOT
        left_arm[:3, 3] -= head[:3, 3]
        right_arm[:3, 3] -= head[:3, 3]

        # Origin offset: head → waist
        left_arm[0, 3] += 0.15
        right_arm[0, 3] += 0.15
        left_arm[2, 3] += 0.45
        right_arm[2, 3] += 0.45

        # 3. Solve arm IK
        arm_q, arm_tau = self.arm_ik.solve_ik(left_arm, right_arm)

        # ----- Hand transform pipeline (tv_wrapper.py:311-344) -----

        # Homogeneous coordinates (4, 25)
        left_h = np.vstack([left_hand_pos.T, np.ones((1, 25))])
        right_h = np.vstack([right_hand_pos.T, np.ones((1, 25))])

        # Basis change: OpenXR → Robot
        left_h = T_ROBOT_OPENXR @ left_h
        right_h = T_ROBOT_OPENXR @ right_h

        # World → arm-local frame
        left_arm_openxr = T_ROBOT_OPENXR @ left_wrist @ T_OPENXR_ROBOT
        right_arm_openxr = T_ROBOT_OPENXR @ right_wrist @ T_OPENXR_ROBOT
        left_h = _fast_mat_inv(left_arm_openxr) @ left_h
        right_h = _fast_mat_inv(right_arm_openxr) @ right_h

        # Initial pose change: OpenXR hand → Unitree hand URDF  →  (25, 3)
        left_hand_local = (T_TO_UNITREE_HAND @ left_h)[:3, :].T
        right_hand_local = (T_TO_UNITREE_HAND @ right_h)[:3, :].T

        # 4. Hand retargeting
        left_hand_q, right_hand_q = self.hand_retarget.retarget(left_hand_local, right_hand_local)

        return arm_q, arm_tau, left_hand_q, right_hand_q
