"""DDS command publishers for G1 arm and Dex3 hands.

Simplified from xr_teleoperate/teleop/robot_control/robot_arm.py and robot_hand_unitree.py.
"""

import logging

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, HandCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.utils.crc import CRC

log = logging.getLogger(__name__)

_dds_initialized = False


def _ensure_dds_init(domain_id=1):
    """Initialize DDS channel factory (must be called exactly once per process)."""
    global _dds_initialized
    if not _dds_initialized:
        ChannelFactoryInitialize(domain_id)
        _dds_initialized = True
        log.info("DDS ChannelFactory initialized (domain_id=%d)", domain_id)


# ---------------------------------------------------------------------------
# Arm DDS sender
# ---------------------------------------------------------------------------

# Motor indices for G1-29 arm joints (robot_arm.py:275-292)
_ARM_MOTOR_INDICES = [
    15, 16, 17, 18, 19, 20, 21,  # Left arm
    22, 23, 24, 25, 26, 27, 28,  # Right arm
]

# Wrist motor indices (for lower kp/kd)
_WRIST_MOTOR_INDICES = {19, 20, 21, 26, 27, 28}

# PD gains
_KP_SHOULDER_ELBOW = 80.0
_KD_SHOULDER_ELBOW = 3.0
_KP_WRIST = 40.0
_KD_WRIST = 1.5


def create_dds_arm_sender(domain_id=1):
    """Create a callable that publishes arm joint commands via DDS.

    Returns:
        send(arm_q_14, arm_tau_14) → None
    """
    _ensure_dds_init(domain_id)

    publisher = ChannelPublisher("rt/lowcmd", hg_LowCmd)
    publisher.Init()

    crc = CRC()
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0

    # Initialize all arm motors with PD gains
    for motor_id in _ARM_MOTOR_INDICES:
        msg.motor_cmd[motor_id].mode = 1
        if motor_id in _WRIST_MOTOR_INDICES:
            msg.motor_cmd[motor_id].kp = _KP_WRIST
            msg.motor_cmd[motor_id].kd = _KD_WRIST
        else:
            msg.motor_cmd[motor_id].kp = _KP_SHOULDER_ELBOW
            msg.motor_cmd[motor_id].kd = _KD_SHOULDER_ELBOW

    log.info("DDS arm sender ready (rt/lowcmd)")

    def send(arm_q, arm_tau):
        for idx, motor_id in enumerate(_ARM_MOTOR_INDICES):
            msg.motor_cmd[motor_id].q = float(arm_q[idx])
            msg.motor_cmd[motor_id].dq = 0.0
            msg.motor_cmd[motor_id].tau = float(arm_tau[idx])
        msg.crc = crc.Crc(msg)
        publisher.Write(msg)

    return send


# ---------------------------------------------------------------------------
# Dex3 hand DDS sender
# ---------------------------------------------------------------------------

class _RISMode:
    """RIS motor mode byte for Dex3 hand."""
    def __init__(self, motor_id=0, status=0x01, timeout=0):
        self.motor_mode = 0
        self.motor_mode |= (motor_id & 0x0F)
        self.motor_mode |= (status & 0x07) << 4
        self.motor_mode |= (timeout & 0x01) << 7

    @property
    def value(self):
        return self.motor_mode


def create_dds_hand_sender(domain_id=None):
    """Create a callable that publishes Dex3 hand commands via DDS.

    Args:
        domain_id: If provided, initialize DDS with this domain. If None,
            assumes DDS is already initialized (e.g. by create_dds_arm_sender).

    Returns:
        send(left_q_7, right_q_7) → None
    """
    if domain_id is not None:
        _ensure_dds_init(domain_id)

    left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
    left_pub.Init()
    right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
    right_pub.Init()

    kp = 1.5
    kd = 0.2

    # Initialize left hand message
    left_msg = unitree_hg_msg_dds__HandCmd_()
    for i in range(7):
        ris = _RISMode(motor_id=i, status=0x01)
        left_msg.motor_cmd[i].mode = ris.value
        left_msg.motor_cmd[i].q = 0.0
        left_msg.motor_cmd[i].dq = 0.0
        left_msg.motor_cmd[i].tau = 0.0
        left_msg.motor_cmd[i].kp = kp
        left_msg.motor_cmd[i].kd = kd

    # Initialize right hand message
    right_msg = unitree_hg_msg_dds__HandCmd_()
    for i in range(7):
        ris = _RISMode(motor_id=i, status=0x01)
        right_msg.motor_cmd[i].mode = ris.value
        right_msg.motor_cmd[i].q = 0.0
        right_msg.motor_cmd[i].dq = 0.0
        right_msg.motor_cmd[i].tau = 0.0
        right_msg.motor_cmd[i].kp = kp
        right_msg.motor_cmd[i].kd = kd

    log.info("DDS hand sender ready (rt/dex3/{left,right}/cmd)")

    def send(left_q, right_q):
        for i in range(7):
            left_msg.motor_cmd[i].q = float(left_q[i])
            right_msg.motor_cmd[i].q = float(right_q[i])
        left_pub.Write(left_msg)
        right_pub.Write(right_msg)

    return send
