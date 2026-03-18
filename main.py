#!/usr/bin/env python3
"""min-teleop: Minimal teleoperation with VR hand tracking + MuJoCo + WebRTC."""

import threading
import time

from vuer import Vuer
from vuer.schemas import Hands, WebRTCVideoPlane

from robots.dexhand import SimRobot, hand_event_to_mocap_targets

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
_lock = threading.Lock()
_latest_targets: dict | None = None

# ---------------------------------------------------------------------------
# Vuer app + WebRTC stream
# ---------------------------------------------------------------------------
app = Vuer(host="0.0.0.0", port=8012, free_port=True)
stream = app.create_webrtc_stream("sim")

# ---------------------------------------------------------------------------
# Sim robot
# ---------------------------------------------------------------------------
robot = SimRobot(width=640, height=480, camera="front")


# ---------------------------------------------------------------------------
# Sim + render thread (60 Hz)
# ---------------------------------------------------------------------------
def sim_render_loop():
    stream.wait_ready_sync()
    dt = 1.0 / 60.0

    while True:
        t0 = time.monotonic()

        with _lock:
            targets = _latest_targets

        robot.step(targets or {})
        rgb = robot.render()
        bgr = rgb[:, :, ::-1]
        stream.push_frame(bgr)

        elapsed = time.monotonic() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)


threading.Thread(target=sim_render_loop, daemon=True).start()


# ---------------------------------------------------------------------------
# HAND_MOVE handler
# ---------------------------------------------------------------------------
@app.add_handler("HAND_MOVE")
async def on_hand_move(event, _session):
    global _latest_targets
    targets = hand_event_to_mocap_targets(event.value, hand="right")
    if targets is not None:
        with _lock:
            _latest_targets = targets


# ---------------------------------------------------------------------------
# Session setup
# ---------------------------------------------------------------------------
@app.spawn(start=True)
async def main(session):
    session.upsert(
        Hands(stream=True, key="hands"),
        WebRTCVideoPlane(
            src=stream.endpoint,
            key="sim-video",
            distanceToCamera=3,
            height=3,
            aspect=4 / 3,
        ),
    )
    await session.forever()
