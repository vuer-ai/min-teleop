#!/usr/bin/env python3
"""Approach 2b — Vuer Frontend Client (Process 1)

Lightweight Vuer app that:
- Receives HAND_MOVE events from the browser
- Forwards mocap targets to the sim server via ZMQ
- Tells the browser to fetch WebRTC video directly from the sim server

This process does ZERO heavy computation — the event loop is 100% free
for WebSocket and scene graph updates.

Usage:
    # Terminal 1: start sim server first
    python examples/2b_sim_server.py

    # Terminal 2: start this client
    python examples/2b_vuer_client.py

    # Remote sim server:
    python examples/2b_vuer_client.py --sim-host 192.168.1.100
"""

import argparse
import logging
import pickle
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import zmq
from vuer import Vuer
from vuer.schemas import Hands, WebRTCVideoPlane

from robots.dexhand import hand_event_to_mocap_targets

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
log = logging.getLogger("vuer-client")


def main():
    parser = argparse.ArgumentParser(description="Vuer frontend (connects to sim_server)")
    parser.add_argument("--sim-host", default="127.0.0.1",
                        help="sim_server IP address")
    parser.add_argument("--sim-port", type=int, default=8013,
                        help="sim_server WebRTC port")
    parser.add_argument("--targets-port", type=int, default=5555)
    parser.add_argument("--vuer-port", type=int, default=8012)
    parser.add_argument("--cert", default="/Users/marvin/cert.pem")
    parser.add_argument("--key", default="/Users/marvin/key.pem")
    args = parser.parse_args()

    # ZMQ: send targets to sim_server
    ctx = zmq.Context()
    sock_targets = ctx.socket(zmq.PUSH)
    sock_targets.setsockopt(zmq.SNDHWM, 1)
    targets_addr = f"tcp://{args.sim_host}:{args.targets_port}"
    sock_targets.connect(targets_addr)
    log.info(f"targets → {targets_addr}")

    # WebRTC offer URL on the sim server
    webrtc_url = f"https://{args.sim_host}:{args.sim_port}/offer"
    log.info(f"WebRTC video ← {webrtc_url}")

    # Vuer app
    app = Vuer(
        host="0.0.0.0", port=args.vuer_port, free_port=True,
        cert=args.cert, key=args.key,
    )

    _count = 0

    @app.add_handler("HAND_MOVE")
    async def on_hand_move(event, _session):
        nonlocal _count
        targets = hand_event_to_mocap_targets(event.value, hand="right")
        if targets is not None:
            sock_targets.send(pickle.dumps(targets), zmq.NOBLOCK)
            _count += 1
            if _count % 100 == 1:
                log.info(f"HAND_MOVE #{_count}: {len(targets)} targets → sim_server")

    @app.spawn(start=True)
    async def session_main(session):
        session.upsert @ [
            Hands(stream=True, key="hands"),
            WebRTCVideoPlane(
                src=webrtc_url,
                key="sim-video",
                distanceToCamera=3,
                height=3,
                aspect=4 / 3,
            ),
        ]
        await session.forever()


if __name__ == "__main__":
    main()
