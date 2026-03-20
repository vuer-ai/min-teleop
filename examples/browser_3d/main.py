#!/usr/bin/env python3
"""Browser 3D: Browser-Side 3D Rendering

Instead of streaming video, the browser renders the full 3D scene locally.
Python runs the physics / reads real hardware, then sends qpos (joint
positions) back to the browser via WebSocket. The browser updates the
scene at native refresh rate (72/90/120Hz).

Data sent per frame: ~hundreds of bytes (qpos array)
vs. WebRTC approach: ~tens of KB (compressed video frame)

Vuer's MuJoCo component loads the full MJCF scene (robot + environment +
objects) in the browser using MuJoCo WASM. The Python side sets qpos
to update all body positions — not just robot joints, but also movable
objects like cups, blocks, etc.

For other simulators (Isaac Sim) or real hardware, you need a mapping
from their joint/object state → the MJCF model's qpos indices.

Usage:
    python examples/browser_3d/main.py
"""

import asyncio
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from vuer import Vuer
from vuer.schemas import DefaultScene, Hands, SceneBackground, ContribLoader
from vuer.schemas import MuJoCo as MuJoCoScene

from robots.dexhand import SimRobot, hand_event_to_mocap_targets

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
log = logging.getLogger("browser-scene")

SCENE_DIR = Path(__file__).resolve().parent.parent.parent / "robots" / "dexhand" / "scene"

# Serve the self-contained scene directory (MJCF + meshes + table)
app = Vuer(
    host="0.0.0.0", port=8012, free_port=True,
    cert=str(Path.home() / "cert.pem"), key=str(Path.home() / "key.pem"),
    static_root=str(SCENE_DIR),
)

# Python-side physics (same as all other approaches)
robot = SimRobot(width=1, height=1, camera="front")

_latest_targets: dict | None = None
_is_mujoco_loaded = False

ASSET_PREFIX = "https://localhost:8012/workspace/"
MJCF_URL = ASSET_PREFIX + "juggle_cube_dex_hands.mjcf.xml"

# List all asset URLs (meshes + table models) — same pattern as mocap_control.py
ASSET_LIST = [
    ASSET_PREFIX + str(p.relative_to(SCENE_DIR))
    for p in SCENE_DIR.rglob("*")
    if p.is_file() and p.suffix.lower() in {".stl", ".obj", ".mtl"}
]


@app.add_handler("ON_MUJOCO_LOAD")
async def on_mujoco_load(event, _session):
    global _is_mujoco_loaded
    _is_mujoco_loaded = True
    log.info("browser MuJoCo WASM loaded")


@app.add_handler("HAND_MOVE")
async def on_hand_move(event, _session):
    global _latest_targets
    targets = hand_event_to_mocap_targets(event.value, hand="right")
    if targets is not None:
        _latest_targets = targets


@app.spawn(start=True)
async def main(session):
    # Load full MJCF scene in the browser (robot + environment + objects)
    session.set @ DefaultScene(
        ContribLoader(
            key="contrib-loader-mujoco",
            library="@vuer-ai/mujoco-ts",
            version="0.0.79",
            main="dist/index.umd.js",
            dependencies=["VUER", "LEVA", "FIBER"],
        ),
        Hands(stream=True, key="hands"),
        MuJoCoScene(
            key="dexhand-sim",
            src=MJCF_URL,
            assets=ASSET_LIST,
            workDir="/",     # Avoid double /workspace prefix
            pause=True,      # Don't run physics in browser
            useLights=True,
            useMocap=True,
        ),
    )

    log.info("waiting for browser to load MuJoCo WASM...")
    while not _is_mujoco_loaded:
        await asyncio.sleep(0.5)

    log.info("browser ready — starting control loop")
    frame_count = 0
    t_log = time.monotonic()

    while True:
        # Physics runs on the Python side (same as approach 1 and 2)
        robot.step(_latest_targets or {})

        # Send full qpos to the browser — updates everything:
        # robot joints, free-body positions, movable objects, etc.
        session.upsert @ MuJoCoScene(
            key="dexhand-sim",
            src=MJCF_URL,
            qpos=robot.data.qpos.tolist(),
        )

        frame_count += 1
        now = time.monotonic()
        if now - t_log >= 3.0:
            log.info(f"control loop: {frame_count / (now - t_log):.0f} Hz")
            frame_count = 0
            t_log = now

        await asyncio.sleep(1 / 60)
