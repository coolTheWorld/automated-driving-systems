# P0a 任务清单

> 详细拆解与理由见 [plan.md](./plan.md)　|　规格见 [SPEC.md](../SPEC.md)
> 状态：**S1 ✓ / S2 ✓ / S3 7.5-8 项完成（CP2 已过）**　|　更新：2026-07-30
> 技术栈：**Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**（官方组合）

---

## 🔖 下次从这里继续

**当前位置**：S1 ✓、S2 ✓、**S3 的 3.1-3.7 ✓（CP2 验证脚本 6/6 通过）**。
只剩 **3.8 的肉眼确认**（RViz 里点云 + TF + 车模型是否同时正确显示）。
之后进 **S4：键盘 teleop 闭环**（4.1-4.4），S4 要接的是 `/vehicle_cmd`
反向通道 —— 目前桥接表里全是 GZ_TO_ROS，S4 要加第一条 ROS_TO_GZ。

恢复环境（宿主机执行）：

```bash
cd ~/work/automated-driving-systems
export COMPOSE_FILE=docker/docker-compose.local.yml
docker compose up -d                                        # 镜像已构建好，直接起
docker compose exec dev /workspace/scripts/verify_gpu.sh    # GPU 仍是 D3D12（10 项）
docker compose exec dev /workspace/scripts/verify_sim.sh    # S2 仿真基线（6 项）
docker compose exec dev /workspace/scripts/verify_ros_bridge.sh  # S3 桥接契约（6 项）
```

三个脚本都退出码 0 才往下做。**若 `verify_gpu.sh` 挂了，先看它是不是又变回 llvmpipe**
——Windows 侧驱动更新或 WSL 重启后 `/dev/dxg` 的存在性值得重新确认。

改了 `src/` 下的代码后要重新构建（容器内）：

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /workspace/install/setup.bash
```

肉眼看效果（带 Gazebo GUI + RViz）：

```bash
ros2 launch gazebo_bridge gazebo_sim.launch.py
```

### ⚠️ 每次动手前先确认没有残留的仿真进程

```bash
docker compose exec dev bash -c 'ps -eo pid,pgid,args | grep -E "gz sim|ros2 launch" | grep -v grep'
```

**有残留必须先清掉**，否则两套仿真同时发 `/clock` 和 `/tf`，所有测量值都是垃圾
（症状：TF 疯狂刷 `Detected jump back in time` / `TF_OLD_DATA`）。
清理要用**进程组**，且**绝对不能用 `pkill -f "gz sim"`**（见下方陷阱表）：

```bash
docker compose exec dev bash -c 'kill -INT -- -<PGID>'   # PGID 取自上面 ps 的第二列
```

容器内跑任何 `gz` 命令前需 `source /opt/ros/jazzy/setup.bash`
（Gazebo 装在 ROS 前缀下），脚本里 source 时要临时 `set +u`。

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

- [x] **3.1** colcon 工作区骨架
      　　✅ `colcon build` 返回 0，3 个包 18.2 s
      　　⚠️ **偏离计划**：只建 S3 真正需要的 3 个包（`ads_msgs`、
      　　　 `ads_simulation/gazebo_bridge`、`ads_visualization`），**不建 SPEC §5 全部 12 个空壳**。
      　　　 空包各需两个纯样板文件、拖慢构建，还会让 `src/` 看起来已经有一套栈了。
      　　　 模块划分由 SPEC §5 定义，不靠空目录存在。
- [x] **3.2** `ads_msgs` 最小消息集
      　　✅ `ros2 interface list` 三条全可见：`VehicleCmd` / `Obstacle` / `ObstacleArray`
- [x] **3.3** 车辆加 `gpu_lidar`（32 线 / 水平 1800 / 量程 50 m）
      　　✅ 57,600 点/帧，`height=32 width=1800`
      　　✅ 顺带加了 **IMU（100 Hz，装在 base_link 原点）** 和 **NavSat（10 Hz）**
      　　ℹ️ 传感器外参进了 `vehicle_params.yaml` 的 `sensors:` 段（同一来源同时喂 SDF 和 URDF）
      　　ℹ️ 世界文件加了 `<spherical_coordinates>`，否则 navsat 不发数据
- [x] **3.4** `gazebo_bridge`：桥接 + 重映射到 SPEC §4.1 规范话题名
      　　✅ 7 条全在：`/clock` `/lidar/points` `/imu` `/gnss` `/odom` `/tf` `/joint_states`
      　　ℹ️ 契约表在 `src/ads_simulation/gazebo_bridge/config/bridge_topics.yaml`
- [x] **3.5** 点云频率与坐标系
      　　✅ `/lidar/points` **10.00 Hz**（仿真时间，标称值满打满算），`frame_id == base_link`
      　　✅ **Δz = +1.600 m**，与雷达安装高度完全吻合 → 证明点云是**真的做了坐标变换**，
      　　　 不是把 `gz_frame_id` 改成 `base_link` 蒙混过关（详见 1 号排查记录）
- [x] **3.6** 仿真时间
      　　✅ 四个节点 `use_sim_time` 均为 true，`/clock` 正常推进
- [x] **3.7** TF 树
      　　✅ 五段全通，且平移值可核对：
      　　　 `base_link→lidar_link = [1.350, 0, 1.600]`、`base_link→gnss_link = [0.500, 0, 1.600]`
      　　　 —— 与 YAML 里的外参逐位一致，说明 URDF 生成链路正确
- [ ] **3.8** `ads_visualization/rviz/default.rviz`
      　　✅ 程序化确认：RViz 已订阅 `/lidar/points`、`/robot_description`、`/tf`，无报错
      　　⏳ **待肉眼确认**：点云 + TF + 车模型是否同时正确显示

> **CP2 结果：`verify_ros_bridge.sh` 6/6 通过 ✓**　数据流已打通，频率达标。
> 带激光雷达后 RTF = **0.998**（S2 纯物理是 1.000）—— 传感器几乎没有代价。

### 3.5 排查记录 1：点云频率只有标称的 35%，病根不在 GPU

第一次测 `/lidar/points` 只有 **3.58 Hz**（标称 10 Hz）。直觉是「GPU 渲不动」，
于是把水平采样从 1800 砍到 900 —— 频率只从 8.65 涨到 8.97 Hz（**+4%**）。
**射线数减半而几乎无改善，说明瓶颈根本不在射线数上。**

分层测量把问题定位清楚了：

| 测量点 | 1800 采样 | 900 采样 |
|---|---|---|
| Gazebo 原生话题（无 ROS） | 8.65 Hz | 8.97 Hz |
| ROS `/lidar/points_raw` | 6.67 Hz | 9.23 Hz |
| ROS `/lidar/points` | 3.58 Hz | 8.00 Hz |

**决定性证据是帧间隔的分布**：Gazebo 侧的相邻帧间隔要么正好 **100 ms**（32 次），
要么正好 **200 ms**（3 次），**没有中间值**。
传感器一直在准点按 10 Hz 出帧，少掉的那些是被**队列丢掉**的，不是没生成。

真正的原因：**一帧 1.8 MB 的点云走 best-effort QoS，每一跳的深度 5 队列都会溢出**。
链路是 Gazebo → gz-transport → `ros_gz_bridge` → 变换节点 → 下游，逐跳递减。
改成 `reliable` + 深度 10 之后，**1800 采样原规格下 `/lidar/points` 稳定在 10.00 Hz**。

**教训**：
1. 「频率低 = 算力不够」是最容易犯的想当然。**先分层测量再动参数**，
   否则会像这次一样先砍掉一半分辨率去解决一个跟分辨率无关的问题。
2. 丢帧是**静默**的 —— 没有任何日志，只有频率变低。
3. 判据不能为了让测试通过而放宽（SPEC §11）。这次坚持 ≥9 Hz 才逼出了真正的病根；
   若当初把线降到 7 Hz「通过」，这个 QoS 缺陷会一路潜伏到 P5 感知。

### 3.5 排查记录 2：验证脚本自己就是测量误差源

中途一度以为链路更慢，实际是**测量工具在丢帧**：检查脚本用 best-effort 订阅，
而它是 Python、第一帧还要遍历 57,600 个点，跟不上就丢。
**测出来的是测量工具的速度，不是被测链路的速度。**
现已改为 reliable 订阅（`scripts/check_cloud_frames.py`）。

同类问题还有两个，都表现为「命令没输出」的假阴性：
- `ros2 topic hz` / `tf2_echo` 输出到管道时是**全缓冲**的，被 `timeout` 打断时缓冲区连同结果一起丢弃
- `ros2 topic list` 依赖长驻 daemon，daemon 刚起来时节点图是空的，只返回 2 个话题

两者都已绕开：频率和 TF 改用常驻 Python 节点测（`check_cloud_frames.py`、`check_tf_tree.py`），
只付一次 DDS 发现代价，结果也确定。

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
S1 █████  5/5 ✓   S2 ██████  6/6 ✓   S3 ███████░  7/8
S4 ░░░░  0/4      S5 ░░░░░░  0/6
                                          总计  18/29
```

## 待办（非当前阶段，记下免得忘）

- **S4**：桥接表目前全是 `GZ_TO_ROS`。`/vehicle_cmd` 是第一条 `ROS_TO_GZ`，
  要把「转角 rad + 加速度 m/s²」转成 Ackermann 插件吃的 Twist。
- **P1 评测**：`/ego_pose_gt`（SPEC §4.1 列了但本次未实现）。需要先定消息类型 ——
  Gazebo 的 `PosePublisher` 发 `gz.msgs.Pose_V`，桥到 ROS 只有 `TFMessage` 和
  `PoseArray` 两个选项，都不理想。等 P1 真正要用它打分时再定。
- **P2 控制**：`limits.max_steer_rate_rad_s` 目前只进了 URDF 的关节速度限位，
  **SDF 里没有生效**（AckermannSteering 插件自己控制转向速度）。
  真正的转向速率约束应由控制器实现。
- **P2 控制**：SDF 转向关节的 `<effort>1.0e6</effort>` 不是物理力矩，
  是 dartsim 在速度控制下让位置限位生效所需的「足够大」。建执行器模型时换成有依据的值。
- **P4 定位**：接上定位后必须删掉 launch 里的 `map_to_odom_identity` 静态 TF，
  否则会和定位模块抢着发同一段 TF，症状是车在 RViz 里疯狂跳动。
- **P7 实车**：点云 QoS 现为 `reliable`（本机仿真链路的正确选择，见排查记录 1）。
  上实车要改回 `best-effort` —— 真实网络下 reliable 的重传会让延迟不可控。
- **P0b**：`docker-compose.cloud.yml` 里 `network_mode: host` 与 `ports: 6080` 并存，
  Docker 会**静默丢弃**发布的端口 → noVNC 连不上且不报错。落地 P0b 时二选一。
