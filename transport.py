"""ZMQ-based transport for inter-process communication.

Provides simple sender/receiver functions for passing mocap targets
between the Vuer client process and the simulation server process.
"""

import pickle

import zmq
import zmq.asyncio


def create_sender(address="tcp://127.0.0.1:5555"):
    """Create a ZMQ PUSH sender. Returns a send(data) function.

    Usage::

        send = create_sender("tcp://sim-host:5555")
        send({"right-wrist": (pos, quat), ...})
    """
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.connect(address)

    def send(data):
        sock.send(pickle.dumps(data), zmq.NOBLOCK)

    return send


def create_receiver(address="tcp://127.0.0.1:5555"):
    """Create a ZMQ PULL receiver. Returns an async recv() function.

    The receiver binds to the address and uses CONFLATE mode to always
    return the latest message, discarding stale ones.

    Usage::

        recv = create_receiver("tcp://127.0.0.1:5555")
        targets = await recv()  # returns dict or None
    """
    ctx = zmq.asyncio.Context()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(address)

    async def recv():
        try:
            msg = await sock.recv(zmq.NOBLOCK)
            return pickle.loads(msg)
        except zmq.Again:
            return None

    return recv
