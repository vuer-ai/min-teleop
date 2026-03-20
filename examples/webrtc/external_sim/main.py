#!/usr/bin/env python3
"""WebRTC External Sim: G1+Dex3 IsaacLab Teleoperation

Demonstrates Vuer with a completely different backend — IsaacLab + G1 robot
instead of MuJoCo + DexHand. The Vuer frontend pattern (Hands + WebRTCVideoPlane)
is identical to webrtc/simple/; only the backend differs.

Single Vuer process that:
- Receives hand tracking from VR headset
- Transforms coordinates and solves IK for G1 arms (14 DoF)
- Retargets hand poses to Dex3 joints (7 DoF x 2)
- Sends all joint commands to IsaacLab via DDS
- Displays IsaacLab camera feed via WebRTC (from IsaacLab's own WebRTC server,
  NOT Vuer's create_webrtc_stream())

Usage:
    # Terminal 1: Start IsaacLab
    python sim_main.py --device cpu --enable_cameras \\
        --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \\
        --enable_dex3_dds --robot_type g129

    # Terminal 2: Start this client
    python examples/webrtc/external_sim/main.py
"""

import asyncio
import logging
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent.parent))

from vuer import Vuer
from vuer.schemas import Hands, WebRTCVideoPlane

from robots.g1 import G1Retarget, create_dds_arm_sender, create_dds_hand_sender

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
log = logging.getLogger("isaaclab-g1")

if "ISAAC_WEBRTC_URL" not in os.environ:
    raise RuntimeError(
        "ISAAC_WEBRTC_URL environment variable is required.\n"
        "Set it to your IsaacLab WebRTC offer endpoint, e.g.:\n"
        "  export ISAAC_WEBRTC_URL=https://127.0.0.1:60001/offer"
    )
ISAAC_WEBRTC_URL = os.environ["ISAAC_WEBRTC_URL"]

# --- Setup ---
retarget = G1Retarget()
send_arm = create_dds_arm_sender(domain_id=1)
send_hand = create_dds_hand_sender()  # reuses already-initialized DDS

app = Vuer(
    host="0.0.0.0", port=8012, free_port=True,
    cert=Path("~/cert.pem").expanduser(),
    key=Path("~/key.pem").expanduser(),
)


@app.add_handler("HAND_MOVE")
async def on_hand_move(event, _session):
    result = retarget(event.value)
    if result is not None:
        arm_q, arm_tau, left_hand_q, right_hand_q = result
        send_arm(arm_q, arm_tau)
        send_hand(left_hand_q, right_hand_q)


@app.spawn(start=True)
async def main(session):
    session.upsert @ [
        Hands(stream=True, key="hands"),
        WebRTCVideoPlane(
            src=ISAAC_WEBRTC_URL, key="sim-video",
            iceServer=None,
            distanceToCamera=3, height=3, aspect=4 / 3,
        ),
    ]
    await asyncio.sleep(float("inf"))
