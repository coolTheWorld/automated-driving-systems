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

⚠️ **驱动版本硬约束（2026-08-14 实测，窗口 2 的 4090/VM）**：
**580-open 驱动上 CARLA 首个 vkQueueSubmit 即 `VK_ERROR_DEVICE_LOST`**，
内核刷 **Xid 32**（corrupted push buffer），UE4 崩得连日志目录都来不及建。
Vulkan 枚举完全正常（ctypes 三设备）—— 所以「验货时 vkEnumerate 过了」
不等于能跑，**枚举不提交命令流**。`-norhithread` / `-opengl` 都救不了
（CARLA 的 UE4 没编 OpenGL RHI，flag 被静默无视）。
**修法：降到 550 专有版**（`apt remove nvidia-driver-580-open` +
`apt install nvidia-driver-550` + reboot），实测 550.163.01 秒起。
选机时优先驱动 ≤570 的宿主；上机自检第 0 步就看 `nvidia-smi` 驱动版本。
降级坑：重启后 `unattended-upgrades` 抢 dpkg 锁会让 apt 静默失败 ——
**装完必须断言 `dpkg -l` 与内核模块在位才允许 reboot**，否则白折腾一轮。

## 1. 环境自检（照 p0b §2.1–2.3）

> **一键版（窗口 4 起）**：`scripts/cloud_window_open.sh` 把 §0–§1 的全部步骤
> 焊成一条幂等命令（宿主体检 → 掐 dpkg 锁 → carla 用户 → CARLA 下载/解包 →
> compose 构建 → 容器内 pip+wheel → colcon → **服务端冒烟**），五个里程碑
> 逐条打印，任一步 FATAL 指名原因（含 580-open 的降级命令）。用法在脚本头。
> 下面的手工步骤保留作为它的说明书与失败排查依据。

1. Vulkan 硬件设备可见（`vkEnumeratePhysicalDevices` 设备类型非 CPU）。
2. CARLA 服务端：`./CarlaUE4.sh -RenderOffScreen -quality-level=Low`
   —— **不带 `--ros2`**（原生层已弃用，SPEC §9 D5：双斜杠话题 rclcpp
   不可达 / /clock 世界重建后静默死 / 传感器流 segfault，三项实测钉死）。
   全部数据走 carla_sidecar_node 中继，/clock 也由它自发。
3. 容器内 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`（Dockerfile 已锁；
   CARLA 内嵌 Fast DDS，CycloneDDS 的症状是本地全好、CARLA 完全收不到）。
4. **容器里装 carla wheel**（窗口 2 实测新容器漏了这步 —— 症状是 sidecar
   启动即死「No module named carla」，两轮 BRINGUP_TIMEOUT 才追到）：
   `docker exec -u root ads-dev bash -c "apt-get install -y python3-pip &&
   python3 -m pip install carla==0.9.16 --break-system-packages"`
   —— 镜像本体不含 pip（本地开发用不上 carla wheel，dry-run 靠惰性 import）。

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

## 6. 两环境一致性差异表（CP-P8-B ③，2026-08-14 收口）

> 双环境头号风险是「行为漂移」（SPEC §4.1）。下表是窗口 1+2 全部实测差异的
> 台账：**已对齐**的写对齐手段，**保留**的写机理与影响面。判据全部维持
> SPEC/plan 原值 —— 差异靠设计余量吸收或如实入表，不放宽判据。

| # | 差异 | 机理 | 处置 | 证据 |
|---|---|---|---|---|
| 1 | steering_curve 随速衰减转角 | CARLA 蓝图自带高速转向衰减，Gazebo 无此物；P0b「稳态达成率 86.3%」的机制本体 | **已对齐**：`_spawn_ego` 拉平为常值曲线（未拉平时弯道失控出图 → 引擎 Segfault） | 窗口 1 实测 |
| 2 | 转向响应 τ | CARLA 0.140 s vs Gazebo 0.294 s，更快 = 安全方向 | **保留**；k_e=1.0 可通用（P0b 结论，窗口 1/2 两块 GPU 复现一字不差） | probe_steering_response |
| 3 | 轮胎侧偏（执行误差） | 绕行执行误差 CARLA −0.150~−0.164 vs Gazebo −0.127；弯道侧滑外漂比运动学模型宽 ~13% | **设计余量吸收**：margin 0.5→0.7（跳满网格步长）+ **两级准入**（floor=SPEC 0.5 保延续候选，margin 选新轨迹 —— 单级在执行误差>网格富余时数学死锁，实测 27/27 全灭停锥旁 103 s） | avoid 9/9 |
| 4 | throttle→加速度强非线性 | 怠速顶不动风阻、0.54 bias 断崖、20 FPS 粗粒度；中速段增益偏大 | bias+k 映射 + 怠速死区 + **驻车闩锁** + 上升速率限幅。**弯道超调残留**（目标 3.9 实测 4.6）→ 差异 #8 | 标定 + junction tap |
| 5 | brake 低开度即凶 | brake 0.2 → −3.31 m/s²（Gazebo 线性） | 0.185/m/s² 标定；停车翻转期的微冲-停已由孤儿定性消解 | 窗口 1 标定 |
| 6 | 路外语义 | Gazebo 草肩（出线无代价）vs CARLA 生成世界路外即虚空（坠落 = 引擎 Segfault） | wall_height=1.0 护栏（诚实红判据替代硬崩）+ **additional_width 2.0 硬化路肩**（恢复容错语义 —— 0.6 时弯道外漂亲墙被物理逮捕，控制命令 +1.5 实测 −4.6） | junction tap 铁证 |
| 7 | NPC 道具语义 | Gazebo 道具无碰撞无重力；CARLA spawn 做碰撞检查、仿真钟粒度 0.05 限定时器 20 Hz | 高空 spawn+瞬移归位；dt 用实际仿真流逝；pose_gt 补 twist | 行为三连 14/14 |
| 8 | **跟踪质量（S01 四条，差异入表）** | 弯道油门映射增益误差 → 冲宽-急刹循环：max_lateral 0.84（判据 0.3）/ rms 0.17（0.1）/ Δv 0.42（0.2）/ a_lat 3.04（2.0）。安全类判据（碰撞/间距/行为正确性 14/14）全绿不受影响 | **入表放行**（用户拍板先例）；根治需中速段标定戏 —— 「静态准≠闭环好」教训在案，不在 P8 展开 | 窗口 2 S01 ×2 |
| 9 | **感知域差距（P5 复测结论）** | 检测率 0-6%（行人 0%）、3437 帧车道内虚警（6.0×3.2 板块 = 路面被聚成障碍物）—— 地面分割/聚类的参数域绑定 Gazebo 世界（平面地板 z=0、无墙、盒型行人） | **复测条款完成（量化+定性）**；修复 = 独立切片（RANSAC 地面模型/传感器高度核对/墙段过滤/walker 网格），量级为「重调感知前端」 | l3c_p5_round |
| 10 | 驱动栈 | 580-open × UE4.26 = Xid 32 秒崩；550 专有版正常 | 上机自检第 0 步查驱动版本；降级流程入 §0 | 窗口 2 |

**判据与协议差异**：仿真钟由 sidecar 自发（原生层弃用，D5）；IMU 上限 20 Hz
（同步步长 0.05）—— 定位（P4-CARLA）上机时要重估步长与 NDT 门限。
