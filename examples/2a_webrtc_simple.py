#!/usr/bin/env python3
"""Approach 2a: Single-Process WebRTC Video Feedback

Renders the simulation with MuJoCo's offscreen Renderer and streams the
frames to the VR headset via WebRTC. Everything runs in one process.

Limitation: Renderer.render() is a synchronous OpenGL call (~25ms) that
blocks the asyncio event loop. We mitigate by capping render FPS and
yielding between frames, but the event loop is still starved during each
render call. For heavy simulations, use the dual-process approach (2b).

Usage:
    python examples/2a_webrtc_simple.py
"""

import asyncio
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from vuer import Vuer
from vuer.schemas import Hands, WebRTCVideoPlane

from robots.dexhand import SimRobot, hand_event_to_mocap_targets

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
log = logging.getLogger("webrtc-simple")

_latest_targets: dict | None = None

SIM_HZ = 60
RENDER_HZ = 24

robot = SimRobot(width=640, height=480, camera="front")

app = Vuer(
    host="0.0.0.0", port=8012, free_port=True,
    cert=Path("~/cert.pem").expanduser(),
    key=Path("~/key.pem").expanduser(),
)
stream = app.create_webrtc_stream("sim")


async def sim_loop():
    """Run simulation and push frames independently of WebSocket sessions."""
    log.info(f"sim loop started: sim={SIM_HZ}Hz, render={RENDER_HZ}Hz")

    frame_count = 0
    t_log = time.monotonic()
    t_render_sum = 0.0

    sim_dt = 1.0 / SIM_HZ
    render_dt = 1.0 / RENDER_HZ
    last_render = 0.0
    last_sim = time.monotonic()

    while True:
        now = time.monotonic()
        targets = _latest_targets

        # Physics steps (catch up to real time)
        steps = 0
        while now - last_sim >= sim_dt:
            robot.step(targets or {})
            last_sim += sim_dt
            steps += 1
            if steps >= 4:
                last_sim = now
                break

        # Render + push (capped at RENDER_HZ)
        if now - last_render >= render_dt:
            t2 = time.monotonic()
            rgb = robot.render()
            t_render_sum += time.monotonic() - t2
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            stream.push_frame(bgr)
            frame_count += 1
            last_render = now

        # Stats
        if now - t_log >= 3.0:
            n = max(frame_count, 1)
            log.info(
                f"{frame_count / (now - t_log):.1f} render-FPS | "
                f"render={1000*t_render_sum/n:.1f}ms"
            )
            frame_count = 0
            t_log = now
            t_render_sum = 0.0

        # Yield to event loop
        sleep_time = max(0.005, last_render + render_dt - time.monotonic())
        await asyncio.sleep(sleep_time)


# Start sim loop when the server starts (event loop is ready), before any client connects.
async def _on_startup(_app):
    asyncio.ensure_future(sim_loop())

app.app.on_startup.append(_on_startup)


@app.add_handler("HAND_MOVE")
async def on_hand_move(event, _session):
    global _latest_targets
    targets = hand_event_to_mocap_targets(event.value, hand="right")
    if targets is not None:
        _latest_targets = targets


@app.spawn(start=True)
async def main(session):
    session.upsert @ [
        Hands(stream=True, key="hands"),
        WebRTCVideoPlane(
            src=stream.endpoint, key="sim-video",
            iceServer=None,
            distanceToCamera=3, height=3, aspect=4 / 3,
        ),
    ]
    await asyncio.sleep(float("inf"))
