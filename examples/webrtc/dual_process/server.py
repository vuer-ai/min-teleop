#!/usr/bin/env python3
"""WebRTC Dual-Process — Simulation Server (Process 2)

Runs MuJoCo simulation + offscreen rendering and serves video via WebRTC.
Receives mocap targets from the Vuer client process via ZMQ.

Usage:
    # Terminal 1:
    python examples/webrtc/dual_process/server.py

    # Terminal 2:
    python examples/webrtc/dual_process/client.py
"""

import asyncio
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent.parent))

import numpy as np
from vuer import Vuer

from robots.dexhand import SimRobot
from transport import create_receiver

logging.basicConfig(level=logging.INFO, format="%(asctime)s [sim-server] %(message)s")
log = logging.getLogger("sim-server")

SIM_HZ = 60
RENDER_HZ = 30

robot = SimRobot(width=640, height=480, camera="front")
recv = create_receiver("tcp://0.0.0.0:5555")
log.info("targets: listening on tcp://0.0.0.0:5555")

app = Vuer(
    host="0.0.0.0", port=8013, free_port=True,
    cert=Path("~/cert.pem").expanduser(),
    key=Path("~/key.pem").expanduser(),
)
stream = app.create_webrtc_stream("sim")


async def sim_loop():
    """Run simulation, render frames, and push to WebRTC stream."""
    log.info(f"sim loop started: sim={SIM_HZ}Hz, render={RENDER_HZ}Hz")

    sim_dt = 1.0 / SIM_HZ
    render_dt = 1.0 / RENDER_HZ
    last_sim = time.monotonic()
    last_render = 0.0
    latest_targets = {}

    frame_count = 0
    t_log = time.monotonic()
    t_render_sum = 0.0

    while True:
        now = time.monotonic()

        # Receive latest targets from client process
        targets = await recv()
        if targets is not None:
            latest_targets = targets

        # Physics steps (catch up to real time)
        steps = 0
        while now - last_sim >= sim_dt:
            robot.step(latest_targets)
            last_sim += sim_dt
            steps += 1
            if steps >= 4:
                last_sim = now
                break

        # Render + push to WebRTC (capped at RENDER_HZ)
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
                f"{frame_count / (now - t_log):.1f} FPS | "
                f"render={1000*t_render_sum/n:.1f}ms"
            )
            frame_count = 0
            t_log = now
            t_render_sum = 0.0

        # Yield
        remaining = last_render + render_dt - time.monotonic()
        await asyncio.sleep(max(0.001, remaining))


async def _on_startup(_app):
    asyncio.ensure_future(sim_loop())

app.app.on_startup.append(_on_startup)
app.start()
