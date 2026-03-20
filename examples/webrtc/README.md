# WebRTC 视频回传方案

WebRTC 子目录下的三个示例体现两个维度：

- **性能维度**: `simple/` → `dual_process/` — 解决 `render()` 阻塞 event loop
- **后端维度**: `simple/` vs `external_sim/` — 同样的 Vuer 前端，不同的后端实现（MuJoCo → IsaacLab+G1）

| 目录 | 解决的问题 | 关键特点 |
|------|-----------|---------|
| [`simple/`](simple/main.py) | 最简 WebRTC 集成 | 单进程，本地渲染+推流。`render()` 阻塞 event loop ~25ms/帧 |
| [`dual_process/`](dual_process/) | render 阻塞 event loop | 渲染独立进程，event loop 100% 空闲，支持跨机器部署 |
| [`external_sim/`](external_sim/main.py) | 不同后端示例 | 对接 IsaacLab+G1，WebRTC 视频来自 IsaacLab 自带服务器（非 Vuer 的 `create_webrtc_stream()`），Vuer 前端代码模式不变 |

三个示例的 Vuer 前端代码模式完全一致：`Hands` 接收手部追踪 + `WebRTCVideoPlane` 显示视频。区别仅在于后端如何处理追踪数据和如何提供 WebRTC 视频源。
