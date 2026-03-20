# VR 遥操作架构指南

本文档介绍使用 Vuer 构建 VR 遥操作系统的三种架构方案。从最简单的开始，逐步解决更复杂的需求。

---

## Vuer 的角色：后端无关的 VR 前端框架

**Vuer 是 VR 前端框架，不绑定任何特定模拟器或后端。** 它提供三项核心能力：

1. **双向 WebSocket** — 从 VR 头显接收追踪数据（手部、头部），也可以向 VR 发送场景更新消息（qpos、组件状态等）
2. **资源加载** — VR 端可以通过 HTTP 或其他协议加载任意 assets（MJCF 模型、mesh、纹理等）
3. **视频显示** — `WebRTCVideoPlane` 接受任何符合 WebRTC 标准的 URL，不限于 Vuer 内置的 `create_webrtc_stream()`

用户的 Python 代码决定如何处理从 VR 收到的追踪数据——可以驱动 MuJoCo 仿真、发送给 IsaacLab、控制真实机器人、或任何其他系统。

```
                         ┌─── MuJoCo
VR 头显 ←──WebSocket──→ Vuer Server ←→ │─── IsaacLab
  ↑                      (Python)       │─── 真实机器人
  └── HTTP 加载 assets                  └─── 任何其他系统
```

`Robot` 基类（`robot.py`）定义了后端抽象接口：`step()`, `render()`, `get_joint_angles()`。仿真机器人在 MuJoCo 里执行物理计算并离屏渲染，真实机器人则发送控制指令给硬件并从摄像头读取图像。控制链路和数据流完全相同，只是 Robot 实现不同。

本仓库的大多数 example 以 MuJoCo + DexHand 做演示，但这只是一种后端实现。[`external_sim/`](examples/webrtc/external_sim/main.py) 示例展示了对接 IsaacLab + G1 的完全不同的后端，Vuer 前端代码模式不变。

---

## 核心问题

VR 遥操作需要两个方向的数据流：

```
VR 头显 ──手部追踪数据──→ Python (控制机器人)
VR 头显 ←──某种反馈────── Python (让用户看到结果)
```

**输入方向**（VR → Python）三种方案都一样：Vuer 的 `Hands` 组件通过 WebSocket 发送 `HAND_MOVE` 事件。

**输出方向**（Python → VR）是三种方案的根本区别。

---

## 方案一：本地 Viewer（无视频回传）

> **示例**: [`examples/local_viewer/main.py`](examples/local_viewer/main.py)

### 架构

```
VR 头显 / 浏览器                      Python
┌─────────────────┐                  ┌──────────────────────┐
│ Hands()          │──HAND_MOVE──→  │ retarget → mj_step() │
│ (手部追踪)       │   WebSocket     │         ↓             │
│                  │                  │ viewer.sync() 1-2ms  │
│ 无画面回传       │                  │ [本地桌面弹窗]        │
└─────────────────┘                  └──────────────────────┘
```

Python 端用 `mujoco.viewer.launch_passive()` 在桌面弹出本地窗口。

```python
viewer = mujoco.viewer.launch_passive(robot.model, robot.data)

@app.spawn(start=True)
async def main(session):
    while viewer.is_running():
        robot.step(targets or {})
        viewer.sync()           # 异步 GPU 提交，~1-2ms
        await asyncio.sleep(0.001)
```

> **macOS 注意**：`launch_passive()` 要求使用 `mjpython` 启动脚本（macOS GUI 必须在主线程）：
> ```bash
> python examples/local_viewer/main.py          # Linux
> mjpython examples/local_viewer/main.py        # macOS
> ```

### 优缺点

| 优点 | 缺点 |
|------|------|
| 最简单，几十行代码 | 用户必须看电脑屏幕，VR 头显里没有画面 |
| event loop 完全不受影响 | 不适合远程部署 |
| 延迟最低（仅 WebSocket + 几ms 计算） | |

### 适用场景

- 开发调试，验证控制链路
- 控制真实机器人时，用户直接用眼睛看实物
- 不需要在 VR 内看到反馈

---

## 方案二：WebRTC 视频回传

> 详见 [`examples/webrtc/README.md`](examples/webrtc/README.md) 了解 WebRTC 方案的两个维度。

### 要解决的问题

方案一的用户必须看电脑屏幕。如果画面能直接显示在 VR 头显里（无论是仿真渲染还是真实摄像头），体验会好得多。

WebRTC 是浏览器原生的实时视频协议，Vuer 提供了 `WebRTCVideoPlane` 组件。

---

### 方案 2a：单进程（simple/）

> **示例**: [`examples/webrtc/simple/main.py`](examples/webrtc/simple/main.py)

```
VR 头显 / 浏览器                      Python (单进程, asyncio)
┌─────────────────┐                  ┌──────────────────────────┐
│ Hands()          │──HAND_MOVE──→  │ retarget → mj_step() 0.1ms│
│                  │   WebSocket     │         ↓                  │
│ WebRTCVideoPlane │←──WebRTC────   │ render()     25ms ← 瓶颈  │
│ (视频画面)       │   H264         │ push_frame() 1ms           │
└─────────────────┘                  └──────────────────────────┘
                                      全在同一个 asyncio event loop
```

**核心限制**：要把画面推给浏览器，我们需要拿到 RGB 像素数据，必须用 `Renderer.render()` 做离屏渲染。和方案一的 `viewer.sync()` 不同，`render()` 必须等 GPU 渲染完成并把像素拷贝回 CPU，是一个 ~25ms 的同步阻塞调用。

| | 方案一 `viewer.sync()` | 方案二 `Renderer.render()` |
|---|---|---|
| 做什么 | 提交给 GPU，立即返回 | 等 GPU 渲染完，拷贝像素回 CPU |
| 耗时 | 1-2ms | 25-30ms |
| 返回值 | 无（画面显示在本地窗口） | RGB numpy 数组（用于 WebRTC 推流） |

在 `render()` 执行的 25ms 内，asyncio event loop 完全停止，无法处理 WebRTC 和 WebSocket 消息。

```
event loop 时间线:
[render 25ms][yield 17ms][render 25ms][yield 17ms]...
              ↑ 只有这段时间能处理网络 I/O
```

**缓解手段**：降渲染分辨率（320×240 → render ~6ms）、降帧率（24 FPS）。但阻塞无法彻底消除。

> **为什么不用独立线程？** macOS 的 OpenGL (CGL) 要求 GL context 操作必须在主线程。从后台线程调用 `render()` 会死锁。Linux (EGL/OSMesa) 没有此限制。

**适用场景**：画面获取很快的情况（低分辨率渲染、USB 摄像头读帧 ~5ms）或快速原型。

---

### 方案 2b：双进程（dual_process/）

> **示例**: [`examples/webrtc/dual_process/server.py`](examples/webrtc/dual_process/server.py) + [`examples/webrtc/dual_process/client.py`](examples/webrtc/dual_process/client.py)

既然 `render()` 会阻塞 event loop，解决方案是把它移到另一个进程——另一个进程有自己的主线程和 event loop。

**关键设计**：Process 2 直接提供 WebRTC 给浏览器，不需要把图片传回 Process 1 再转发。

```
浏览器
  │
  ├── WebSocket ──→ Process 1 (Vuer, :8012)
  │                   接收 HAND_MOVE, 转发 targets
  │                   提供场景组件 (Hands, WebRTCVideoPlane)
  │                   event loop: 100% 空闲
  │
  └── WebRTC ←──── Process 2 (Vuer, :8013)
                     接收 targets (ZMQ), mj_step, render
                     直接提供 H264 视频流
                     可以随意阻塞，没有其他 I/O 任务

Process 1 ──ZMQ (mocap targets)──→ Process 2
```

进程间通讯通过 `transport.py` 封装的 ZMQ PUSH/PULL 完成，两端各只需一行调用：

```python
# client 端
send = create_sender("tcp://sim_host:5555")
send(targets)

# server 端
recv = create_receiver("tcp://0.0.0.0:5555")
targets = await recv()
```

```bash
# 终端 1: 仿真服务器（可以在任意机器上运行）
python examples/webrtc/dual_process/server.py

# 终端 2: Vuer 前端（自动检测局域网 IP，也可通过 SIM_HOST 环境变量指定）
python examples/webrtc/dual_process/client.py
```

Process 1 的 `WebRTCVideoPlane(src="https://sim_host:8013/webrtc/offer/sim")` 直接指向 Process 2。浏览器和 Process 2 之间建立 WebRTC peer connection，视频帧不经过 Process 1。

> **⚠ 自签名证书**：浏览器需要直接访问 Process 2（`:8013`）的 HTTPS 服务来建立 WebRTC 连接。由于使用自签名证书，浏览器会静默拒绝连接（视频黑屏，无报错）。**首次使用前，需要在浏览器中手动打开 `https://<sim_host>:8013` 并接受证书安全警告**，之后 WebRTC 才能正常工作。

---

### 方案 2c：外部模拟器（external_sim/）

> **示例**: [`examples/webrtc/external_sim/main.py`](examples/webrtc/external_sim/main.py)

这个示例展示了**同样的 Vuer 前端模式 + 完全不同的后端**。与 `simple/` 的区别仅在于后端实现不同——证明 Vuer 不绑定任何特定模拟器。

```
VR 头显 / 浏览器                      Python (Vuer)              外部系统
┌─────────────────┐                  ┌─────────────────┐        ┌──────────────────┐
│ Hands()          │──HAND_MOVE──→  │ retarget (IK)    │──DDS─→│ IsaacLab + G1    │
│                  │   WebSocket     │                  │        │                  │
│ WebRTCVideoPlane │←─────────WebRTC─────────────────────────── │ 自带 WebRTC 服务器│
│ (视频画面)       │                  │                  │        │                  │
└─────────────────┘                  └─────────────────┘        └──────────────────┘
```

**与 simple/ 的关键区别**:

| | simple/ | external_sim/ |
|---|---|---|
| **后端** | MuJoCo（本地） | IsaacLab + G1（外部进程） |
| **控制指令** | 直接调用 `robot.step()` | 通过 DDS 发送到 G1 机器人 |
| **WebRTC 视频源** | Vuer 的 `create_webrtc_stream()` | IsaacLab 自带 WebRTC 服务器 |
| **Vuer 前端代码** | `Hands` + `WebRTCVideoPlane` | `Hands` + `WebRTCVideoPlane`（完全相同） |

**依赖说明**:
- `robots/g1/` — G1 机器人的坐标变换、IK 求解、DDS 通信
- [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) — IsaacLab 端仿真环境
- `unitree_sdk2py` — DDS 通信 SDK

`WebRTCVideoPlane` 接受 IsaacLab 的 WebRTC URL（默认 `https://127.0.0.1:60001/offer`），无需使用 Vuer 的 `create_webrtc_stream()`。这证明了 `WebRTCVideoPlane` 可以对接任何标准 WebRTC 源。

---

### 方案二总结

| | 2a 单进程 | 2b 双进程 | 2c 外部模拟器 |
|---|---|---|---|
| 文件数 | 1 个 | 2 个 | 1 个 |
| event loop 阻塞 | 25ms/帧 | 0ms | 0ms |
| 跨机器部署 | 不支持 | 支持 | 支持（DDS） |
| 后端 | MuJoCo | MuJoCo | IsaacLab + G1 |
| WebRTC 源 | Vuer 内置 | Vuer 内置 | IsaacLab 自带 |
| Vuer 前端模式 | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane |
| 适用 | 轻量渲染、快速原型 | 重型仿真 | 不同后端集成 |

### 注意事项（多进程 / 跨机器部署）

**1. IP 地址绑定**

多进程方案（2b、2c）涉及多个网络端点（ZMQ、WebRTC、WebSocket），需要注意 IP 地址的一致性：

- **server 端**的 ZMQ 应绑定 `0.0.0.0`（而非 `127.0.0.1`），否则 client 通过局域网 IP 连接时消息无法送达
- **client 端**默认使用 `app.local_ip`（局域网 IP）连接 server。如果 server 在其他机器上，通过 `SIM_HOST` 环境变量指定：
  ```bash
  SIM_HOST=192.168.1.100 python examples/webrtc/dual_process/client.py
  ```
- `WebRTCVideoPlane(src=...)` 的 URL 也必须指向**浏览器能访问的地址**——如果浏览器在 VR 头显上运行，这个地址必须是头显网络可达的 IP，不能是 `127.0.0.1`

**2. 自签名 SSL 证书**

Vuer 使用 HTTPS（WebRTC 和部分浏览器 API 要求安全上下文）。开发环境通常使用自签名证书（`cert.pem` / `key.pem`），浏览器会拦截并显示安全警告。

**在 VR 头显的浏览器中，需要手动访问每个 HTTPS 端点并接受证书警告**：

- 方案 2a（单进程）：访问 `https://<host>:8012` 并接受证书
- 方案 2b（双进程）：需要分别接受**两个**端口的证书：
  1. `https://<host>:8012` — Vuer WebSocket 服务
  2. `https://<host>:8013` — WebRTC 视频流服务
- 方案 2c（外部模拟器）：除了 Vuer 的端口，还需要接受 IsaacLab WebRTC 服务器的证书

> **提示**：在 Quest 浏览器中，先在地址栏依次打开上述 URL，点击"继续访问（不安全）"，然后再回到 Vuer 页面。否则 WebRTC 连接会静默失败（视频黑屏，无报错）。

### 优缺点（整个方案二）

| 优点 | 缺点 |
|------|------|
| VR 用户能看到画面 | 视频编解码引入延迟（50-200ms） |
| 支持任意画面源（仿真、摄像头、Isaac Sim） | 帧率受限于渲染速度 |
| 2b/2c 可跨机器部署 | 2D 平面画面，无 3D 深度感 |
| 浏览器只需硬件解码 H264，零负担 | |

---

## 方案三：浏览器端 3D 渲染

> **示例**: [`examples/browser_3d/main.py`](examples/browser_3d/main.py)

### 要解决的问题

方案二传送的是 2D 视频帧，有编码延迟和画质损失，而且没有立体 3D 感。如果场景可以用 3D 模型描述（而不是真实摄像头画面），为什么不直接在 VR 设备上渲染？

### 架构

**物理/控制仍然运行在 Python 端**，和方案一、二完全一样。区别只是回传给 VR 的不是视频帧，而是场景状态（qpos）。

```
VR 头显 / 浏览器                          Python
┌────────────────────────────┐           ┌────────────────────────────┐
│ 加载 MJCF 完整场景           │           │ mj_step() / 真实硬件读取    │
│ (机器人 + 桌子 + 物体)       │   qpos    │           ↓                 │
│ 浏览器本地 GPU 渲染          │←──WS────│ data.qpos (全部关节+物体)   │
│ @ 设备原生刷新率 72/90Hz    │ ~几百字节  │                            │
│ Hands() → HAND_MOVE ──────→│──WS──→  │                            │
└────────────────────────────┘           └────────────────────────────┘
```

Python 通过 WebSocket 发送完整的 qpos 数组（几百字节），浏览器端的 MuJoCo WASM 据此更新整个场景——不仅是机器人关节，也包括自由物体的位置（桌上的杯子、可抓取的方块等）。

```python
# 加载完整 MJCF 场景（机器人 + 环境 + 物体）
session.set @ DefaultScene(
    MuJoCoScene(key="sim", src=MJCF_URL, pause=True, useMocap=True),
    Hands(stream=True, key="hands"),
)

# 控制循环：物理在 Python 跑，只发送 qpos 给浏览器
while True:
    robot.step(targets or {})

    session.upsert @ MuJoCoScene(
        key="sim", src=MJCF_URL,
        qpos=robot.data.qpos.tolist(),   # 更新整个场景状态
    )
    await asyncio.sleep(1/60)
```

Vuer 的 `MuJoCo` 组件加载完整的 MJCF XML（包含机器人、环境、物体），在浏览器端用 MuJoCo WASM 渲染。前端组件通过 keyFrame 机制接收 `qpos`、`mocap_pos`、`mocap_quat` 等状态，Python 端直接传入即可更新整个场景（目前 Python schema 未显式声明这些属性，但 Vuer 的 Element 基类会透传所有 kwargs 到前端）。

注意：在 VR 设备上运行 MuJoCo WASM 会有一定的性能开销。场景越复杂（mesh 数量、物体数量），对设备 GPU 和内存的要求越高。

对于其他仿真器（Isaac Sim 等）或真实机器人，需要一个**状态映射**：从源端的关节/物体状态 → MJCF 模型的 qpos 索引。这和 `retarget.py` 映射 VR 手部数据到 mocap targets 是同一类问题。

### 优缺点

| 优点 | 缺点 |
|------|------|
| 零视频延迟（本地渲染） | VR 设备必须有足够算力渲染场景 |
| 最高画质（原生 3D，非压缩视频） | 需要 MJCF 场景文件 + mesh 资源通过 HTTP 分发 |
| 立体 3D（VR 双目渲染） | 不支持真实摄像头画面 |
| 显示完整场景（机器人 + 环境 + 物体） | 其他仿真器/真机需要状态 → qpos 映射 |
| 带宽极低（几百字节 vs 几十 KB/帧） | |
| event loop 友好（只发小消息） | |

### 适用场景

- 需要低延迟、高画质、3D 立体的 VR 体验
- 场景包含机器人 + 桌面 + 可交互物体等复杂环境
- VR 设备有足够算力（Quest 3、Pico 4 等）
- 真实机器人 + 对应 MJCF 模型 → "数字孪生"显示

---

## 对比总览

```
简单 ──────────────────────────────────────────────────────→ 复杂

方案一              方案 2a          方案 2b          方案 2c          方案三
本地 viewer         单进程 WebRTC    双进程 WebRTC    外部模拟器        浏览器端渲染

电脑屏幕看          VR 里看 (2D)     VR 里看 (2D)     VR 里看 (2D)     VR 里看 (3D)
零延迟              有延迟           有延迟            有延迟            零延迟
不传视频            传视频流         传视频流          传视频流          传关节状态
event loop 空闲     event loop 阻塞  event loop 空闲   event loop 空闲   event loop 空闲
MuJoCo 后端         MuJoCo 后端      MuJoCo 后端      IsaacLab 后端     任意后端
```

| | 方案一 | 方案 2a | 方案 2b | 方案 2c | 方案三 |
|------|--------|---------|---------|---------|--------|
| **VR 内可见** | ❌ | ✅ | ✅ | ✅ | ✅ |
| **立体 3D** | ❌ | ❌ | ❌ | ❌ | ✅ |
| **event loop 阻塞** | 1-2ms | 25ms | 0ms | 0ms | <1ms |
| **画面延迟** | 本地 | 50-200ms | 50-200ms | 50-200ms | <10ms |
| **真实摄像头画面** | N/A | ✅ | ✅ | ✅ | ❌ |
| **跨机器部署** | ❌ | ❌ | ✅ | ✅ | ❌ |
| **后端** | MuJoCo | MuJoCo | MuJoCo | IsaacLab+G1 | 任意 |
| **WebRTC 源** | — | Vuer 内置 | Vuer 内置 | IsaacLab 自带 | — |
| **Vuer 前端模式** | Hands | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane | Hands + WebRTCVideoPlane | Hands + MuJoCoScene |

---

## 如何选择

1. **先用方案一** 验证手部追踪 → 机器人控制链路是否正确
2. **需要在 VR 里看到画面？**
   - 虚拟仿真 + 需要 3D 立体感 → **方案三**
   - 真实摄像头 / 重型仿真器 → **方案二**
3. **方案 2a 卡顿？** → 先降分辨率。还卡 → **方案 2b**
4. **对接非 MuJoCo 后端（IsaacLab 等）？** → 参考 **方案 2c**

---

## 文件索引

```
min-teleop/
├── README.md                                    # 本文档
├── requirements.txt
├── robot.py                                     # Robot ABC 接口 (step/render/get_joint_angles)
├── transport.py                                 # ZMQ 进程间通讯封装
├── robots/
│   ├── dexhand/                                 # DexHand 机器人实现 (MuJoCo)
│   │   ├── sim.py                               #   SimRobot (MuJoCo 仿真)
│   │   ├── retarget.py                          #   手部追踪数据 → mocap targets
│   │   └── joint_map.py                         #   MuJoCo ↔ 硬件关节映射
│   └── g1/                                      # G1+Dex3 机器人实现 (IsaacLab)
│       ├── retarget.py                          #   坐标变换 + IK 求解
│       ├── arm_ik.py                            #   G1 手臂 IK
│       └── dds_sender.py                        #   DDS 通信（手臂 + 手指指令）
└── examples/
    ├── local_viewer/main.py                     # 方案一：本地桌面窗口
    ├── webrtc/
    │   ├── README.md                            # WebRTC 方案总览
    │   ├── simple/main.py                       # 方案 2a：单进程 WebRTC
    │   ├── dual_process/server.py + client.py   # 方案 2b：双进程 WebRTC
    │   └── external_sim/main.py                 # 方案 2c：外部模拟器 (IsaacLab+G1)
    └── browser_3d/main.py                       # 方案三：浏览器端 3D 渲染
```
