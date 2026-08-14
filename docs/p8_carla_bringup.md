# P8 CARLA 上机手册（S5 租机窗口 1 的 checklist）

> 前置：P8-S4 已交付 carla_bridge（本地 dry-run 全绿）。本文是**上机才能做**的
> 事项清单，按顺序执行 —— 每一步的失败模式都标了排查方向。
> 沿用 [p0b_minimal_alignment.md](p0b_minimal_alignment.md) 的流程骨架
> （Vulkan 先行、无头低画质、本地只做瘦客户端），那份手册的 §6「两个坑」照旧适用。

## 0. 租机规格（SPEC §4.1 的硬性筛选）

RTX 4090/5070Ti 级、显存 ≥ 8 GB、**CPU ≥ 8 物理核**（CARLA 物理与 TM 吃 CPU，
矿机改装的便宜实例 GPU 强 CPU 弱 —— 最易踩）、内存 ≥ 32 GB、磁盘 ≥ 80 GB。
镜像自选 + 必须能设 `NVIDIA_DRIVER_CAPABILITIES=all`（`graphics` 能力项默认
不开，没有它 CARLA 一启动就崩而 nvidia-smi 一切正常）。

## 1. 环境自检（照 p0b §2.1–2.3）

1. Vulkan 硬件设备可见（`vkEnumeratePhysicalDevices` 设备类型非 CPU）。
2. CARLA 0.9.16 起服务端：`./CarlaUE4.sh -RenderOffScreen -quality-level=Low --ros2`
   —— **`--ros2` 别忘**，双通道的原生半边全靠它。
3. 容器内 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（Dockerfile 已锁；
   CARLA 内嵌 Fast DDS，CycloneDDS 的症状是本地全好、CARLA 完全收不到）。

## 2. 四件 dry-run 验不了的事（P0b 已知 issue，按序核）

| # | 核什么 | 怎么核 | 失败排查 |
|---|---|---|---|
| 1 | **原生传感器话题名** | 起服务端 + spawn 带 ros_name 的传感器，`ros2 topic list` | 名字带 role_name 前缀 / 双斜杠（issue #2）→ 调 sidecar 里 spawn 时的 ros_name 属性；实在对不上就在 sidecar 加一层 relay（低频方案 B，先量带宽再决定） |
| 2 | **原生 ↔ PythonAPI 坐标一致性**（issue #3） | 静止车：对比 /ego_pose_gt（sidecar，已做 y 翻转）与原生 IMU/GNSS 的位置/朝向 | 差一个 y 翻转 = 原生通道**已经**做了 ENU 转换 → 删 sidecar 侧对应翻转，**只能一处翻** |
| 3 | **/clock 与 sidecar 时间戳同源** | `ros2 topic echo /clock` vs sidecar 消息 stamp | 不同源会让 TF 报 extrapolation —— sidecar 改读 world snapshot 时间戳 |
| 4 | **控制偶发失效**（issue #1） | teleop 长按加速 60 s，观察是否丢指令 | sidecar 已按 tick 重发兜底；仍丢就升频重发 |

## 3. 验收（合同不变，换四个环境变量）

```bash
LAUNCH_PKG=carla_bridge LAUNCH_FILE=carla_sim.launch.py RTF_SOURCE=clock \
BRIDGE_NODES="/lidar_preprocessor /carla_sidecar /robot_state_publisher" \
./scripts/verify_ros_bridge.sh
```

六项检查逐条同 Gazebo（SPEC §4.1 的对外契约）。⚠️ RTF 用 clock 源
（Δ仿真钟/Δ墙钟）—— gz 源的 0.970 基线**不因此作废**，两种测法各测各的。

## 4. 标定（每项都有本地初值，上机只重标常数）

1. **throttle/brake 映射**：`carla_bridge_params.yaml` 的
   `throttle_per_mps2 / brake_per_mps2`（初值 0.4 / 0.33 是保守猜测）。
   阶跃指令 → 量稳态加速度 → 反解系数。
2. **转向响应 τ**：`scripts/probe_steering_response.py` 同一把尺子
   （P0b 实测 0.140 s，citroen.c3 —— 复核用）。
3. **物理对齐**：`scripts/carla_align_vehicle.py`（干跑过一次实机，2026-08-03），
   结果固化进 sidecar 的 `_spawn_ego`。
4. **NDT 卡方地板/门限**：世界与地图不同源，散布分布会变 ——
   拿 `.p8s2c/collect_d2.py` 同一把尺子重量（localization.md §10.6b）。

## 5. S4 遗留的两个决策点（上机前找用户拍板）

1. **红绿灯消息**：S06 需要灯态 + 停止线话题 —— 新增 `ads_msgs/TrafficLight`
   属于「修改 ads_msgs 接口」（CLAUDE.md 先问后做）。最小闭环拍板过深度，
   消息形态没拍。
2. **NPC 编排**：L3-C 的 S03/S05/junction 需要 CARLA 侧 NPC。方案 a：sidecar
   读 dynamic_actors.yaml spawn CARLA actor + set_target_velocity 复刻航点驱动
   （与 Gazebo 同一份 YAML，单一来源）；方案 b：CARLA Traffic Manager 自动驾驶
   （省事但行为不可复刻，判据窗口对不上）。倾向 a。
