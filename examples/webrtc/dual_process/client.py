#!/usr/bin/env python3
"""WebRTC Dual-Process — Vuer Frontend Client (Process 1)

Lightweight Vuer app that:
- Receives HAND_MOVE events from the browser
- Forwards mocap targets to the sim server via ZMQ
- Tells the browser to fetch WebRTC video directly from the sim server

This process does ZERO heavy computation — the event loop is 100% free
for WebSocket and scene graph updates.

Usage:
    # Terminal 1: start sim server first
    python examples/webrtc/dual_process/server.py

    # Terminal 2: start this client
    python examples/webrtc/dual_process/client.py
"""

import logging
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent.parent))

from vuer import Vuer
from vuer.schemas import Hands, WebRTCVideoPlane

from robots.dexhand import hand_event_to_mocap_targets
from transport import create_sender

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
log = logging.getLogger("vuer-client")

app = Vuer(
    host="0.0.0.0", port=8012, free_port=True,
    cert=Path("~/cert.pem").expanduser(),
    key=Path("~/key.pem").expanduser(),
)

SIM_HOST = os.environ.get("SIM_HOST", app.local_ip)
send = create_sender(f"tcp://{SIM_HOST}:5555")
webrtc_url = f"https://{SIM_HOST}:8013/webrtc/offer/sim"
log.info(f"sim server: {SIM_HOST} (ZMQ:5555, WebRTC:8013)")

@app.add_handler("HAND_MOVE")
async def on_hand_move(event, _session):
    targets = hand_event_to_mocap_targets(event.value, hand="right")
    if targets is not None:
        send(targets)


@app.spawn(start=True)
async def main(session):
    session.upsert @ [
        Hands(stream=True, key="hands"),
        WebRTCVideoPlane(
            src=webrtc_url,
            key="sim-video",
            iceServer=None,
            distanceToCamera=3,
            height=3,
            aspect=4 / 3,
        ),
    ]
    await session.forever()
