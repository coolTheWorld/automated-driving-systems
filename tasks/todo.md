# P0a 任务清单

> 详细拆解与理由见 [plan.md](./plan.md)　|　规格见 [SPEC.md](../SPEC.md)
> 状态：**S1 ✓ / S2 ✓，下一步 S3**　|　更新：2026-07-30（已切换到 Jazzy + Ubuntu 24.04）
> 技术栈：**Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**（官方组合）

---

## S1　容器 + GPU + GUI　【检查点 CP1：go / no-go】

- [x] **1.1** 宿主机安装 Docker
      　　✅ docker.io 29.1.3 + compose-v2 2.40.3 + buildx；`hello-world` 通过
      　　⚠️ `usermod -aG docker` 对**已存在的会话不生效** → 需在新终端重启 Claude Code
- [x] **1.2** `docker/Dockerfile`：Ubuntu 24.04 + ROS 2 Jazzy desktop + mesa 工具
      　　✅ `docker compose build` 通过，镜像 `ads-dev:jazzy`
      　　✅ `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 已锁进镜像并实测生效
- [x] **1.3** `docker/docker-compose.local.yml`：映射 GPU 设备、挂 WSLg socket、传 `DISPLAY`
      　　✅ 容器 `ads-dev` 起来；容器内 `uid=1000(dev) groups=44(video),990`
      　　⚠️ 排查 1.4 时**两次修订**：补 `/dev/dxg` 映射、补 `GALLIUM_DRIVER=d3d12`（详见下）
- [x] **1.4** 🚦 **验证容器内硬件加速**
      　　✅ `verify_gpu.sh` 退出码 0：`D3D12 (AMD Radeon 780M Graphics)` /
      　　　 `Accelerated: yes` / OpenGL core **4.6**
- [x] **1.5** 验证容器内 GUI
      　　✅ `rviz2` 在 Windows 桌面弹窗，鼠标拖动视角流畅；日志 `OpenGl version: 4.6 (GLSL 4.6)`

> **CP1 结果：通过 ✓**　环境 A 方案成立，可进入 S2。

### 1.4 排查记录（两个独立缺陷叠加，缺一不可）

第一次跑 `verify_gpu.sh` 得到 `llvmpipe` / `Accelerated: no`，且**全程无任何报错**——
正是计划里预判的最坏情况。定位出两个原因：

| # | 缺陷 | 症状 | 修复 |
|---|------|------|------|
| 1 | 只映射了 `/dev/dri`，漏了 **`/dev/dxg`** | WSL2 的 GPU 通路是 dxgkrnl 的 `/dev/dxg`，`/dev/dri` 只是兼容外观 | compose `devices` 补 `/dev/dxg:/dev/dxg` |
| 2 | 容器 Mesa **25.2.8 不自动选 d3d12** | 宿主 Ubuntu 26.04 / Mesa 26.0.3 能自动探测，容器 Ubuntu 24.04 不能；Jazzy 绑死 24.04 升不了 | compose `environment` 加 `GALLIUM_DRIVER=d3d12` |

决定性对照：同一容器内 `glxinfo -B` → llvmpipe，
`GALLIUM_DRIVER=d3d12 glxinfo -B` → D3D12 硬件加速。
`d3d12_dri.so` 两边都只是 `libdril_dri.so` 的软链，驱动本身无差别，差的是自动选择逻辑。

**两项检查都已加进 `verify_gpu.sh`**，换机器不会再踩。
另外确认 DRI3 是干扰项：宿主同样报 `screen 0 does not appear to be DRI3 capable`，但照样硬件加速。

---

## S2　Gazebo 世界 + 车能动

- [x] **2.1** Dockerfile 加装 `ros-jazzy-ros-gz`（**官方组合**，自动拉 Harmonic）
      　　✅ `gz sim --versions` → **8.11.0**（= Harmonic）
      　　ℹ️ 走 `ros-jazzy-gz-sim-vendor`，`gz` 在 ROS 前缀下，脚本里必须先 source
- [x] **2.2** `worlds/campus_minimal.sdf`：地面 + 100 m 直路 + 3 个交错方块 + 一堵墙
      　　✅ `gz sim` 加载正常，GUI 中肉眼确认
      　　ℹ️ 障碍摆成左右交错，S4 的「绕障一圈」才需要真的打方向
      　　ℹ️ 墙是给 S3 激光雷达用的大面积回波目标
- [x] **2.3** `models/ego_vehicle/`：车体 + 四轮 + `AckermannSteering` 插件
      　　✅ 车出现在世界中，四轮姿态正确
      　　⚠️ `model.sdf` 是**生成物**，由 `scripts/gen_vehicle_model.py` 从 YAML 生成
      　　ℹ️ `base_link` 在后轴中心、地面高度（Autoware 惯例）
- [x] **2.4** `config/vehicle_params.yaml`（**单一来源**，SPEC §4.1 强制）
      　　✅ 轴距 2.7 m / 轮距 1.55 m / 质量 1500 kg / 最大转角 0.6 rad 等齐全，全部带单位后缀
      　　✅ 生成器带 `--check` 模式，S5 的 CI 可用它卡住「改了 YAML 忘了重新生成」
- [x] **2.5** 用 gz 原生话题驱动车辆
      　　✅ 直行：6 s 前进 **15.478 m**（`verify_sim.sh` 步骤 4）
      　　✅ 转向：6 s 侧移 **10.988 m**（步骤 5，驱动与转向是两条独立链路，都要测）
- [x] **2.6** 测量实时率
      　　✅ headless 纯物理 **RTF = 1.000**；带 GUI 渲染 **RTF = 0.999**
      　　ℹ️ 用 `scripts/verify_sim.sh` 从 `/world/*/stats` 取 12 帧均值，不读 GUI 状态栏

> **参数确实传导到了模型里**：指令 3 m/s、`max_accel_mps2: 1.5` →
> 加速到位需 2 s 走 3 m，其后 4 s × 3 m/s = 12 m，理论 15 m，实测 15.478 m。
> 这比「车动了」有信息量得多 —— 它证明的是 YAML 里的限值真的生效，而不只是模型能跑。

---

## S3　ROS 2 桥接 + 规范话题 + RViz2　【检查点 CP2】

- [ ] **3.1** colcon 工作区骨架（SPEC §5 的包，先建空壳）
      　　✅ `colcon build` 返回 0
- [ ] **3.2** `ads_msgs` 最小消息集
      　　✅ `ros2 interface show` 可见
- [ ] **3.3** 车辆加 `gpu_lidar`（32 线 / 水平 1800 / 量程 50 m，依 SPEC §2 ODD）
      　　✅ `gz topic -e` 有点云数据
- [ ] **3.4** `ads_simulation/gazebo_bridge`：桥接 + **重映射到 SPEC §4.1 规范话题名**
      　　✅ `/lidar/points`、`/imu`、`/odom`、`/clock` 均存在
- [ ] **3.5** 点云频率与坐标系
      　　✅ `/lidar/points` **≥ 9 Hz**，`frame_id == base_link`
- [ ] **3.6** 仿真时间
      　　✅ 各节点 `use_sim_time=true`，`/clock` 正常推进
- [ ] **3.7** TF 树 `map` → `odom` → `base_link`
      　　✅ `view_frames` 输出无断裂
- [ ] **3.8** `ads_visualization/rviz/default.rviz`
      　　✅ 点云 + TF + 车模型同时正确显示

> **CP2**：数据流打通、频率达标，是 P1 开始的前提。

---

## S4　键盘 teleop 闭环

- [ ] **4.1** teleop 节点：键盘 → `/vehicle_cmd`（转角 rad + 加速度 m/s²）
      　　✅ `ros2 topic echo /vehicle_cmd` 有对应输出
- [ ] **4.2** `gazebo_bridge` 订阅 `/vehicle_cmd` → 转 Gazebo 指令
      　　✅ 车响应键盘
- [ ] **4.3** 指令限幅（超 `vehicle_params.yaml` 范围则截断并告警）
      　　✅ 手动发超限指令，车不失控
- [ ] **4.4** 🎯 **闭环验证**：键盘开车绕障碍物一圈
      　　✅ 全程 RViz 中点云 / TF / 车姿态同步更新

---

## S5　工程化收尾　（可与 S4 并行）　【检查点 CP3】

- [ ] **5.1** `.clang-format`（ROS 2 风格，SPEC §7）
      　　✅ `clang-format --dry-run` 无差异
- [ ] **5.2** `stack.launch.py`，支持 `sim:=gazebo`
      　　✅ 一条命令起全栈
- [ ] **5.3** L1 单元测试样板（`ads_common` 几何函数 + gtest）
      　　✅ `colcon test` 返回 0
- [ ] **5.4** CI（GitHub Actions）：build + test + lint
      　　✅ CI 绿
- [ ] **5.5** `README.md`：从零复现 P0a
      　　✅ 照着做能跑起来
- [ ] **5.6** `docs/adr/0001-dual-simulation.md`
      　　✅ 与 SPEC §9 D4 一致

> **CP3**：P0a 验收。决定下一步做 P1（继续本地）还是 P0b（先建云端）。

---

## 进度

```
S1 █████  5/5 ✓   S2 ██████  6/6 ✓   S3 ░░░░░░░░  0/8
S4 ░░░░  0/4      S5 ░░░░░░  0/6
                                          总计  11/29
```

## 待办（非当前阶段，记下免得忘）

- **P0b**：`docker-compose.cloud.yml` 里 `network_mode: host` 与 `ports: 6080` 并存，
  Docker 会**静默丢弃**发布的端口 → noVNC 连不上且不报错。落地 P0b 时二选一。
