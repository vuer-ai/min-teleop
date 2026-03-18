#!/usr/bin/env python3
"""Approach 2b — Simulation Server (Process 2)

Runs MuJoCo simulation + offscreen rendering and serves the video
directly to the browser via WebRTC. No image relay through Process 1.

- Receives mocap targets from the Vuer client via ZMQ
- Steps physics, renders frames, pushes to WebRTC track
- Runs its own aiohttp server for WebRTC signaling

The browser connects here for video, and to the Vuer client for
scene graph / hand tracking.

Usage:
    # Terminal 1:
    python examples/2b_sim_server.py

    # Terminal 2:
    python examples/2b_vuer_client.py
"""

import argparse
import asyncio
import fractions
import json
import logging
import pickle
import ssl
import time

import av
import numpy as np
import zmq
import zmq.asyncio
from aiohttp import web
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robots.dexhand import SimRobot

logging.basicConfig(level=logging.INFO, format="%(asctime)s [sim-server] %(message)s")
log = logging.getLogger("sim-server")


# ---------------------------------------------------------------------------
# WebRTC video track (same pattern as Vuer's standalone_server.py)
# ---------------------------------------------------------------------------
class BGRVideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._start_time = None
        self._pts = 0

    async def recv(self):
        try:
            frame = await asyncio.wait_for(self._queue.get(), timeout=1.0)
        except asyncio.TimeoutError:
            # Send black frame if no data
            frame = av.VideoFrame(width=640, height=480, format="bgr24")
            frame.pts = self._pts
            frame.time_base = fractions.Fraction(1, 90000)
        return frame

    def push_frame(self, bgr_numpy):
        video_frame = av.VideoFrame.from_ndarray(bgr_numpy, format="bgr24")
        if self._start_time is None:
            self._start_time = time.time()
            self._pts = 0
        else:
            self._pts = int((time.time() - self._start_time) * 90000)
        video_frame.pts = self._pts
        video_frame.time_base = fractions.Fraction(1, 90000)

        loop = asyncio.get_event_loop()
        def _put():
            try:
                if self._queue.full():
                    self._queue.get_nowait()
                self._queue.put_nowait(video_frame)
            except Exception:
                pass
        loop.call_soon_threadsafe(_put)


# ---------------------------------------------------------------------------
# WebRTC signaling
# ---------------------------------------------------------------------------
pcs: set = set()
track = BGRVideoTrack()
relay = MediaRelay()


async def offer_handler(request):
    params = await request.json()
    desc = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    relayed = relay.subscribe(track)
    transceiver = pc.addTransceiver(relayed, direction="sendonly")

    # Prefer H264
    caps = RTCRtpSender.getCapabilities("video")
    h264 = [c for c in caps.codecs if c.mimeType == "video/H264"]
    if h264:
        transceiver.setCodecPreferences(h264)

    @pc.on("connectionstatechange")
    async def on_state():
        if pc.connectionState in ("failed", "closed"):
            pcs.discard(pc)
            await pc.close()

    await pc.setRemoteDescription(desc)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }),
    )


# ---------------------------------------------------------------------------
# Sim + render loop
# ---------------------------------------------------------------------------
async def sim_render_loop(robot, zmq_sock, sim_hz, render_hz):
    sim_dt = 1.0 / sim_hz
    render_dt = 1.0 / render_hz
    last_sim = time.monotonic()
    last_render = 0.0
    latest_targets = {}

    frame_count = 0
    t_log = time.monotonic()
    t_render_sum = 0.0

    log.info(f"sim loop: sim={sim_hz}Hz render={render_hz}Hz")

    while True:
        now = time.monotonic()

        # Receive latest targets (non-blocking)
        while True:
            try:
                msg = await zmq_sock.recv(zmq.NOBLOCK)
                latest_targets = pickle.loads(msg)
            except zmq.Again:
                break

        # Physics steps
        steps = 0
        while now - last_sim >= sim_dt:
            robot.step(latest_targets)
            last_sim += sim_dt
            steps += 1
            if steps >= 4:
                last_sim = now
                break

        # Render + push to WebRTC track
        if now - last_render >= render_dt:
            t2 = time.monotonic()
            rgb = robot.render()
            t_render_sum += time.monotonic() - t2
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            track.push_frame(bgr)
            frame_count += 1
            last_render = now

        # Stats
        if now - t_log >= 3.0:
            n = max(frame_count, 1)
            fps = frame_count / (now - t_log)
            log.info(f"{fps:.1f} FPS | render={1000*t_render_sum/n:.1f}ms")
            frame_count = 0
            t_log = now
            t_render_sum = 0.0

        # Yield
        remaining = last_render + render_dt - time.monotonic()
        await asyncio.sleep(max(0.001, remaining))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Sim server with WebRTC")
    parser.add_argument("--port", type=int, default=8013)
    parser.add_argument("--bind-targets", default="tcp://127.0.0.1:5555")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--sim-hz", type=int, default=60)
    parser.add_argument("--render-hz", type=int, default=30)
    parser.add_argument("--cert", default="/Users/marvin/cert.pem")
    parser.add_argument("--key", default="/Users/marvin/key.pem")
    args = parser.parse_args()

    # ZMQ (async)
    zmq_ctx = zmq.asyncio.Context()
    sock_targets = zmq_ctx.socket(zmq.PULL)
    sock_targets.setsockopt(zmq.RCVHWM, 1)
    sock_targets.setsockopt(zmq.CONFLATE, 1)
    sock_targets.bind(args.bind_targets)
    log.info(f"targets: listening on {args.bind_targets}")

    # Robot
    robot = SimRobot(width=args.width, height=args.height, camera="front")
    log.info(f"SimRobot ready ({args.width}x{args.height})")

    # aiohttp app
    import aiohttp_cors
    app = web.Application()
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods="*",
        )
    })
    resource = app.router.add_resource("/offer")
    cors.add(resource.add_route("POST", offer_handler))

    async def on_startup(_app):
        asyncio.ensure_future(sim_render_loop(robot, sock_targets, args.sim_hz, args.render_hz))

    async def on_shutdown(_app):
        coros = [pc.close() for pc in pcs]
        await asyncio.gather(*coros)

    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    # SSL
    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(args.cert, args.key)

    log.info(f"WebRTC offer: https://0.0.0.0:{args.port}/offer")
    web.run_app(app, host="0.0.0.0", port=args.port, ssl_context=ssl_ctx)


if __name__ == "__main__":
    main()
