#!/usr/bin/env python3
"""Approach 1: VR Hand Tracking + Local MuJoCo Viewer (No WebRTC)

The simplest approach — no video streaming.
- Browser sends hand tracking data via WebSocket
- Python runs MuJoCo simulation + pops up a local desktop viewer
- User sees the simulation on their computer screen

viewer.sync() submits scene data to the GPU pipeline and returns immediately
(~1-2ms), so the asyncio event loop stays responsive.

Usage:
    python examples/1_local_viewer.py          # Linux
    mjpython examples/1_local_viewer.py        # macOS (requires mjpython)
"""

import asyncio
from pathlib import Path
import sys

# Allow imports from project root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco.viewer
from vuer import Vuer
from vuer.schemas import Hands

from robots.dexhand import SimRobot, hand_event_to_mocap_targets

_latest_targets: dict | None = None

robot = SimRobot(width=1, height=1, camera="front")  # renderer unused
viewer = mujoco.viewer.launch_passive(robot.model, robot.data)
app = Vuer(
    host="0.0.0.0", port=8012, free_port=True,
    cert=Path("~/cert.pem").expanduser(),
    key=Path("~/key.pem").expanduser(),
)


@app.add_handler("HAND_MOVE")
async def on_hand_move(event, _session):
    global _latest_targets
    targets = hand_event_to_mocap_targets(event.value, hand="right")
    if targets is not None:
        _latest_targets = targets


@app.spawn(start=True)
async def main(session):
    session.upsert @ Hands(stream=True, key="hands")

    while viewer.is_running():
        robot.step(_latest_targets or {})
        viewer.sync()
        await asyncio.sleep(0.001)
