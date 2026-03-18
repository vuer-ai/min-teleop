"""DexHand retargeting: convert Vuer HAND_MOVE events to MuJoCo mocap targets."""

import numpy as np
from scipy.spatial.transform import Rotation

# 25 VR hand landmarks (Vuer / WebXR ordering)
LANDMARKS = [
    "wrist",
    "thumb-metacarpal", "thumb-phalanx-proximal", "thumb-phalanx-distal", "thumb-tip",
    "index-finger-metacarpal", "index-finger-phalanx-proximal",
    "index-finger-phalanx-intermediate", "index-finger-phalanx-distal", "index-finger-tip",
    "middle-finger-metacarpal", "middle-finger-phalanx-proximal",
    "middle-finger-phalanx-intermediate", "middle-finger-phalanx-distal", "middle-finger-tip",
    "ring-finger-metacarpal", "ring-finger-phalanx-proximal",
    "ring-finger-phalanx-intermediate", "ring-finger-phalanx-distal", "ring-finger-tip",
    "pinky-finger-metacarpal", "pinky-finger-phalanx-proximal",
    "pinky-finger-phalanx-intermediate", "pinky-finger-phalanx-distal", "pinky-finger-tip",
]

# Which landmarks become mocap bodies (10 total)
TRACKING_SITES = [
    "wrist",
    "thumb-tip",
    "index-finger-tip",
    "middle-finger-tip",
    "ring-finger-tip",
    "pinky-finger-tip",
    "index-finger-metacarpal",
    "middle-finger-metacarpal",
    "ring-finger-metacarpal",
    "pinky-finger-metacarpal",
]

# Vuer Y-up -> MuJoCo Z-up rotation matrix
_R_VUER_TO_MUJOCO = np.array([
    [1, 0,  0],
    [0, 0, -1],
    [0, 1,  0],
], dtype=float)

_T_VUER_TO_MUJOCO = np.eye(4)
_T_VUER_TO_MUJOCO[:3, :3] = _R_VUER_TO_MUJOCO


def hand_event_to_mocap_targets(hand_data: dict, hand: str = "right") -> dict | None:
    """Convert a Vuer HAND_MOVE event to mocap targets.

    Args:
        hand_data: event.value from HAND_MOVE.  Expected keys:
            - ``right`` or ``left``: flat list of 25*16 floats (25 column-major 4x4 matrices)
        hand: which hand side to extract ("right" or "left")

    Returns:
        {body_name: (pos[3], quat[4])} or None if hand data unavailable.
        Quaternion is scalar-first (w, x, y, z) as MuJoCo expects.
    """
    poses = hand_data.get(hand)
    if poses is None:
        return None

    targets = {}
    for site in TRACKING_SITES:
        idx = LANDMARKS.index(site)
        # Each landmark is a column-major 4x4 stored as 16 floats
        mat = np.array(poses[16 * idx : 16 * idx + 16]).reshape(4, 4).T

        # Vuer (Y-up) -> MuJoCo (Z-up)
        mat = _T_VUER_TO_MUJOCO @ mat

        pos = mat[:3, 3]
        quat = Rotation.from_matrix(mat[:3, :3]).as_quat(scalar_first=True)

        body_name = f"{hand}-{site}"
        targets[body_name] = (pos, quat)

    return targets
