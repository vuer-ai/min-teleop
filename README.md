# min-teleop

[中文版](README_zh.md) | **English**

Minimal VR teleoperation examples using [Vuer](https://github.com/vuer-ai/vuer). Control simulated (MuJoCo) or real robots from a VR headset in the browser.

---

## Getting Started

### 1. Create the conda environment

```bash
git clone https://github.com/vuer-ai/min-teleop.git
cd min-teleop
conda env create -f environment.yml
conda activate min-teleop
```

Or install with pip directly (see the `pip` section in `environment.yml` for the package list).

### 2. Generate a self-signed SSL certificate

Vuer serves over HTTPS (required by WebRTC and browser security APIs). For local development, generate a self-signed cert:

```bash
openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 \
  -keyout ~/key.pem -out ~/cert.pem -days 365 -nodes \
  -subj "/CN=localhost"
```

When connecting from a VR headset, you must open `https://<your-ip>:8012` in the headset browser and accept the certificate warning before the app will work.

### 3. Run an example

**Local Viewer** — simplest, pops up a desktop MuJoCo window:

```bash
# Linux
python examples/local_viewer/main.py

# macOS (MuJoCo requires main-thread access)
mjpython examples/local_viewer/main.py
```

**Browser 3D** — renders the full 3D scene in the VR headset:

```bash
python examples/browser_3d/main.py
```

**WebRTC Single-Process** — streams rendered video to VR:

```bash
python examples/webrtc/simple/main.py
```

**WebRTC Dual-Process** — separates Vuer frontend from simulation for better performance:

```bash
# Terminal 1: simulation server
python examples/webrtc/dual_process/server.py

# Terminal 2: Vuer frontend
python examples/webrtc/dual_process/client.py
```

Each example prints a URL (e.g. `https://192.168.x.x:8012`). Open it in your VR headset browser or any desktop browser.

### 4. (Optional) G1 + IsaacLab example

The `examples/webrtc/external_sim/` example drives a Unitree G1 robot in IsaacLab. It requires additional dependencies:

```bash
# Install pinocchio via conda-forge
conda install -c conda-forge pinocchio

# Install G1-specific pip packages
pip install casadi pyyaml nlopt anytree pytransform3d trimesh lxml

# Install dex-retargeting from submodule
git submodule update --init
pip install --no-deps -e third_party/dex-retargeting

# unitree_sdk2py must be installed separately (not on PyPI)
# pip install -e <path/to/unitree_sdk2_python>
```

Then run:

```bash
python examples/webrtc/external_sim/main.py
```

---

## Architecture Guide

This section covers the three architectural approaches for VR teleoperation with Vuer, from simplest to most capable.

### Vuer's Role: Backend-Agnostic VR Frontend

**Vuer is a VR frontend framework, not tied to any specific simulator or backend.** It provides three core capabilities:

1. **Bidirectional WebSocket** — receives tracking data from VR headsets (hands, head), and sends scene updates back (qpos, component state, etc.)
2. **Asset Loading** — the VR client loads arbitrary assets (MJCF models, meshes, textures) via HTTP
3. **Video Display** — `WebRTCVideoPlane` accepts any WebRTC-compliant URL, not limited to Vuer's built-in `create_webrtc_stream()`

Your Python code decides what to do with the tracking data — drive MuJoCo, send to IsaacLab, control a real robot, or anything else.

```
                         ┌─── MuJoCo
VR Headset ←──WebSocket──→ Vuer Server ←→ │─── IsaacLab
  ↑                      (Python)       │─── Real Robot
  └── HTTP loads assets                  └─── Any other system
```

The `Robot` base class (`robot.py`) defines the backend abstraction: `step()`, `render()`, `get_joint_angles()`. Simulated robots run physics and offscreen rendering in MuJoCo; real robots send commands to hardware and read from cameras. The control pipeline is identical — only the Robot implementation differs.

Most examples in this repo demonstrate MuJoCo + DexHand, but that's just one backend. The [`external_sim/`](examples/webrtc/external_sim/main.py) example shows a completely different backend (IsaacLab + G1) with the same Vuer frontend pattern.

---

### The Core Problem

VR teleoperation needs two data flows:

```
VR Headset ──hand tracking──→ Python (control robot)
VR Headset ←──some feedback── Python (show results to user)
```

**Input** (VR → Python) is the same across all approaches: Vuer's `Hands` component sends `HAND_MOVE` events over WebSocket.

**Output** (Python → VR) is what differentiates the three approaches.

---

### Approach 1: Local Viewer (No Video Feedback)

> **Example**: [`examples/local_viewer/main.py`](examples/local_viewer/main.py)

```
VR Headset / Browser                    Python
┌─────────────────┐                  ┌──────────────────────┐
│ Hands()          │──HAND_MOVE──→  │ retarget → mj_step() │
│ (hand tracking)  │   WebSocket     │         ↓             │
│                  │                  │ viewer.sync() 1-2ms  │
│ No video         │                  │ [local desktop window]│
└─────────────────┘                  └──────────────────────┘
```

Python uses `mujoco.viewer.launch_passive()` to pop up a local desktop window.

```python
viewer = mujoco.viewer.launch_passive(robot.model, robot.data)

@app.spawn(start=True)
async def main(session):
    while viewer.is_running():
        robot.step(targets or {})
        viewer.sync()           # async GPU submit, ~1-2ms
        await asyncio.sleep(0.001)
```

> **macOS note**: `launch_passive()` requires the main thread for GUI. Use `mjpython` instead of `python`:
> ```bash
> python examples/local_viewer/main.py          # Linux
> mjpython examples/local_viewer/main.py        # macOS
> ```

| Pros | Cons |
|------|------|
| Simplest — a few dozen lines | User must look at computer screen; no image in VR |
| Event loop completely free | Not suitable for remote deployment |
| Lowest latency (WebSocket + a few ms of compute) | |

**Use when**: debugging, verifying the control pipeline, or when the user can look at the real robot directly.

---

### Approach 2: WebRTC Video Feedback

> See [`examples/webrtc/README.md`](examples/webrtc/README.md) for the WebRTC overview.

#### The Problem

Approach 1 requires the user to look at a computer screen. Streaming video directly to the VR headset (from simulation or a real camera) provides a much better experience.

WebRTC is the browser-native real-time video protocol. Vuer provides `WebRTCVideoPlane`.

---

#### 2a: Single-Process (simple/)

> **Example**: [`examples/webrtc/simple/main.py`](examples/webrtc/simple/main.py)

```
VR Headset / Browser                    Python (single process, asyncio)
┌─────────────────┐                  ┌──────────────────────────┐
│ Hands()          │──HAND_MOVE──→  │ retarget → mj_step() 0.1ms│
│                  │   WebSocket     │         ↓                  │
│ WebRTCVideoPlane │←──WebRTC────   │ render()     25ms ← bottleneck│
│ (video feed)     │   H264         │ push_frame() 1ms           │
└─────────────────┘                  └──────────────────────────┘
                                      All in the same asyncio event loop
```

**Core limitation**: to push frames to the browser, we need RGB pixel data from `Renderer.render()`. Unlike `viewer.sync()`, `render()` waits for GPU completion and copies pixels back to CPU — a ~25ms synchronous blocking call.

| | Approach 1 `viewer.sync()` | Approach 2 `Renderer.render()` |
|---|---|---|
| What it does | Submits to GPU, returns immediately | Waits for GPU, copies pixels to CPU |
| Duration | 1-2ms | 25-30ms |
| Returns | Nothing (displays in local window) | RGB numpy array (for WebRTC streaming) |

During `render()`'s 25ms, the asyncio event loop is completely blocked.

**Mitigation**: lower render resolution (320×240 → ~6ms), cap frame rate (24 FPS). But blocking can't be fully eliminated.

> **Why not use a separate thread?** macOS OpenGL (CGL) requires GL context operations on the main thread. Calling `render()` from a background thread will deadlock. Linux (EGL/OSMesa) doesn't have this restriction.

**Use when**: fast frame acquisition (low-res rendering, USB camera ~5ms) or rapid prototyping.

---

#### 2b: Dual-Process (dual_process/)

> **Example**: [`examples/webrtc/dual_process/server.py`](examples/webrtc/dual_process/server.py) + [`examples/webrtc/dual_process/client.py`](examples/webrtc/dual_process/client.py)

Since `render()` blocks the event loop, the solution is to move it to a separate process with its own main thread and event loop.

**Key design**: Process 2 serves WebRTC directly to the browser — no need to pipe frames back through Process 1.

```
Browser
  │
  ├── WebSocket ──→ Process 1 (Vuer, :8012)
  │                   receives HAND_MOVE, forwards targets
  │                   serves scene components (Hands, WebRTCVideoPlane)
  │                   event loop: 100% free
  │
  └── WebRTC ←──── Process 2 (Vuer, :8013)
                     receives targets (ZMQ), mj_step, render
                     serves H264 video directly
                     can block freely — no other I/O

Process 1 ──ZMQ (mocap targets)──→ Process 2
```

Inter-process communication uses ZMQ PUSH/PULL via `transport.py`:

```python
# client
send = create_sender("tcp://sim_host:5555")
send(targets)

# server
recv = create_receiver("tcp://0.0.0.0:5555")
targets = await recv()
```

```bash
# Terminal 1: simulation server (can run on any machine)
python examples/webrtc/dual_process/server.py

# Terminal 2: Vuer frontend (auto-detects LAN IP, or set SIM_HOST)
python examples/webrtc/dual_process/client.py
```

Process 1's `WebRTCVideoPlane(src="https://sim_host:8013/webrtc/offer/sim")` points directly at Process 2. The browser establishes a WebRTC peer connection with Process 2 — video never passes through Process 1.

> **Self-signed certificate warning**: The browser needs direct HTTPS access to Process 2 (`:8013`) for WebRTC. With self-signed certs, the browser silently refuses (black screen, no error). **Before first use, open `https://<sim_host>:8013` in the browser and accept the certificate warning.**

---

#### 2c: External Simulator (external_sim/)

> **Example**: [`examples/webrtc/external_sim/main.py`](examples/webrtc/external_sim/main.py)

This demonstrates **the same Vuer frontend pattern with a completely different backend**. The only difference from `simple/` is the backend implementation — proving Vuer isn't tied to any simulator.

```
VR Headset / Browser                    Python (Vuer)              External System
┌─────────────────┐                  ┌─────────────────┐        ┌──────────────────┐
│ Hands()          │──HAND_MOVE──→  │ retarget (IK)    │──DDS─→│ IsaacLab + G1    │
│                  │   WebSocket     │                  │        │                  │
│ WebRTCVideoPlane │←─────────WebRTC─────────────────────────── │ Built-in WebRTC  │
│ (video feed)     │                  │                  │        │                  │
└─────────────────┘                  └─────────────────┘        └──────────────────┘
```

**Key differences from simple/**:

| | simple/ | external_sim/ |
|---|---|---|
| **Backend** | MuJoCo (local) | IsaacLab + G1 (external) |
| **Control** | Direct `robot.step()` | DDS to G1 robot |
| **WebRTC source** | Vuer's `create_webrtc_stream()` | IsaacLab's built-in WebRTC |
| **Vuer frontend** | `Hands` + `WebRTCVideoPlane` | `Hands` + `WebRTCVideoPlane` (identical) |

**Dependencies**:
- `robots/g1/` — coordinate transforms, IK solving, DDS communication
- [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) — IsaacLab simulation
- `unitree_sdk2py` — DDS communication SDK

`WebRTCVideoPlane` accepts IsaacLab's WebRTC URL (default `https://127.0.0.1:60001/offer`), without using Vuer's `create_webrtc_stream()`. This proves `WebRTCVideoPlane` can connect to any standard WebRTC source.

---

#### Approach 2 Summary

| | 2a Single-Process | 2b Dual-Process | 2c External Sim |
|---|---|---|---|
| Files | 1 | 2 | 1 |
| Event loop blocked | 25ms/frame | 0ms | 0ms |
| Cross-machine | No | Yes | Yes (DDS) |
| Backend | MuJoCo | MuJoCo | IsaacLab + G1 |
| WebRTC source | Vuer built-in | Vuer built-in | IsaacLab built-in |
| Vuer frontend | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane |
| Best for | Lightweight rendering, prototyping | Heavy simulations | Different backend integration |

#### Notes (Multi-Process / Cross-Machine)

**1. IP Address Binding**

Multi-process approaches (2b, 2c) involve multiple network endpoints (ZMQ, WebRTC, WebSocket):

- **Server** ZMQ should bind `0.0.0.0` (not `127.0.0.1`) — otherwise clients on the LAN can't connect
- **Client** defaults to `app.local_ip` (LAN IP). For a remote server, set `SIM_HOST`:
  ```bash
  SIM_HOST=192.168.1.100 python examples/webrtc/dual_process/client.py
  ```
- `WebRTCVideoPlane(src=...)` URL must point to an address reachable by the browser — if running on a VR headset, it can't be `127.0.0.1`

**2. Self-Signed SSL Certificates**

Vuer uses HTTPS (required by WebRTC and browser security APIs). Development uses self-signed certs (`cert.pem` / `key.pem`).

**On the VR headset browser, manually visit each HTTPS endpoint and accept the certificate warning**:

- 2a (single-process): visit `https://<host>:8012`
- 2b (dual-process): accept certs for **both** ports:
  1. `https://<host>:8012` — Vuer WebSocket
  2. `https://<host>:8013` — WebRTC video
- 2c (external sim): also accept the IsaacLab WebRTC server cert

> **Tip**: In Quest browser, open each URL in the address bar, tap "Continue (unsafe)", then navigate back to the Vuer page. Otherwise WebRTC will silently fail (black screen, no error).

#### Pros/Cons (Approach 2 overall)

| Pros | Cons |
|------|------|
| VR user can see the scene | Video codec adds latency (50-200ms) |
| Any image source (sim, camera, Isaac Sim) | Frame rate limited by render speed |
| 2b/2c support cross-machine deployment | 2D plane, no 3D depth |
| Browser only needs hardware H264 decode | |

---

### Approach 3: Browser-Side 3D Rendering

> **Example**: [`examples/browser_3d/main.py`](examples/browser_3d/main.py)

#### The Problem

Approach 2 sends 2D video frames with encoding latency and quality loss, and no stereoscopic 3D. If the scene can be described with 3D models (not real camera feeds), why not render directly on the VR device?

#### Architecture

**Physics/control still runs in Python**, same as Approaches 1 and 2. The difference is that instead of video frames, we send scene state (qpos) to VR.

```
VR Headset / Browser                          Python
┌────────────────────────────┐           ┌────────────────────────────┐
│ Loads full MJCF scene        │           │ mj_step() / real hardware   │
│ (robot + table + objects)    │   qpos    │           ↓                 │
│ Browser-local GPU rendering  │←──WS────│ data.qpos (all joints+objects)│
│ @ device refresh 72/90Hz    │ ~hundreds  │                            │
│ Hands() → HAND_MOVE ──────→│──WS──→   of bytes                      │
└────────────────────────────┘           └────────────────────────────┘
```

Python sends the full qpos array (~hundreds of bytes) over WebSocket. The browser's MuJoCo WASM updates the entire scene — robot joints and free-body objects (cups, blocks, etc.).

```python
# Load full MJCF scene (robot + environment + objects)
session.set @ DefaultScene(
    MuJoCoScene(key="sim", src=MJCF_URL, pause=True, useMocap=True),
    Hands(stream=True, key="hands"),
)

# Control loop: physics in Python, send qpos to browser
while True:
    robot.step(targets or {})

    session.upsert @ MuJoCoScene(
        key="sim", src=MJCF_URL,
        qpos=robot.data.qpos.tolist(),   # update entire scene state
    )
    await asyncio.sleep(1/60)
```

Vuer's `MuJoCo` component loads the full MJCF XML (robot + environment + objects) and renders in the browser with MuJoCo WASM. The frontend component receives `qpos`, `mocap_pos`, `mocap_quat` etc. via keyframes.

Note: Running MuJoCo WASM on VR devices has performance overhead. More complex scenes (mesh count, object count) require more GPU and memory from the device.

For other simulators (Isaac Sim, etc.) or real robots, you need a **state mapping**: source joint/object state → MJCF model qpos indices. This is the same class of problem as `retarget.py` mapping VR hand data to mocap targets.

| Pros | Cons |
|------|------|
| Zero video latency (local rendering) | VR device must have enough power to render |
| Highest quality (native 3D, not compressed video) | Requires MJCF scene + mesh assets served over HTTP |
| Stereoscopic 3D (VR binocular rendering) | Can't show real camera feeds |
| Full scene (robot + environment + objects) | Other sims/real robots need state → qpos mapping |
| Minimal bandwidth (~hundreds of bytes vs ~tens KB/frame) | |
| Event loop friendly (only sends small messages) | |

**Use when**: low-latency, high-quality, stereoscopic 3D VR experience is needed; the scene is describable with MJCF models; VR device has adequate compute (Quest 3, Pico 4, etc.); "digital twin" display for real robots with matching MJCF models.

---

### Comparison Overview

```
Simple ──────────────────────────────────────────────────────→ Complex

Approach 1          2a              2b              2c              3
Local Viewer        Single WebRTC   Dual WebRTC     External Sim    Browser 3D

Desktop screen      VR (2D)         VR (2D)         VR (2D)         VR (3D)
Zero latency        Has latency     Has latency     Has latency     Zero latency
No video            Video stream    Video stream    Video stream    Joint state
Event loop free     Loop blocked    Loop free       Loop free       Loop free
MuJoCo backend      MuJoCo          MuJoCo          IsaacLab        Any backend
```

| | Approach 1 | 2a | 2b | 2c | 3 |
|------|--------|---------|---------|---------|--------|
| **Visible in VR** | No | Yes | Yes | Yes | Yes |
| **Stereoscopic 3D** | No | No | No | No | Yes |
| **Event loop blocked** | 1-2ms | 25ms | 0ms | 0ms | <1ms |
| **Video latency** | Local | 50-200ms | 50-200ms | 50-200ms | <10ms |
| **Real camera feed** | N/A | Yes | Yes | Yes | No |
| **Cross-machine** | No | No | Yes | Yes | No |
| **Backend** | MuJoCo | MuJoCo | MuJoCo | IsaacLab+G1 | Any |
| **WebRTC source** | — | Vuer built-in | Vuer built-in | IsaacLab built-in | — |
| **Vuer frontend** | Hands | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane | Hands + MuJoCoScene |

---

### How to Choose

1. **Start with Approach 1** to verify hand tracking → robot control pipeline
2. **Need to see the scene in VR?**
   - Virtual sim + need 3D depth → **Approach 3**
   - Real camera / heavy simulator → **Approach 2**
3. **2a stuttering?** → Lower resolution first. Still stuttering → **2b**
4. **Non-MuJoCo backend (IsaacLab, etc.)?** → See **2c**

---

## File Index

```
min-teleop/
├── README.md                                    # This document
├── environment.yml                              # Conda environment (all base deps)
├── robot.py                                     # Robot ABC (step/render/get_joint_angles)
├── transport.py                                 # ZMQ IPC wrapper
├── robots/
│   ├── dexhand/                                 # DexHand robot (MuJoCo)
│   │   ├── sim.py                               #   SimRobot (MuJoCo simulation)
│   │   ├── retarget.py                          #   Hand tracking → mocap targets
│   │   └── joint_map.py                         #   MuJoCo ↔ hardware joint mapping
│   └── g1/                                      # G1+Dex3 robot (IsaacLab)
│       ├── retarget.py                          #   Coordinate transforms + IK
│       ├── arm_ik.py                            #   G1 arm IK
│       └── dds_sender.py                        #   DDS communication (arm + finger commands)
├── third_party/
│   └── dex-retargeting/                         # Git submodule (unitree branch)
└── examples/
    ├── local_viewer/main.py                     # Approach 1: local desktop window
    ├── webrtc/
    │   ├── README.md                            # WebRTC approach overview
    │   ├── simple/main.py                       # Approach 2a: single-process WebRTC
    │   ├── dual_process/server.py + client.py   # Approach 2b: dual-process WebRTC
    │   └── external_sim/main.py                 # Approach 2c: external sim (IsaacLab+G1)
    └── browser_3d/main.py                       # Approach 3: browser-side 3D rendering
```
