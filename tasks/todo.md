# P0a 任务清单

> 详细拆解与理由见 [plan.md](./plan.md)　|　规格见 [SPEC.md](../SPEC.md)
> 状态：**P0a 全部完成 29/29，CP1 / CP2 / CP3 均已达成**　|　更新：2026-07-31
> 技术栈：**Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**（官方组合）

---

## 🔖 下次从这里继续

**当前位置**：**P0a 已全部完成**（S1–S5，29/29），CP1 / CP2 / CP3 全部达成，
并已做过一轮**收官复查**（2026-07-31，见下）。

### ✅ 2026-07-31 收官复查：按数据流讲解一遍 + 修掉三处

按 ①车辆参数与世界 → ②`gazebo_bridge` → ③控制链路 → ④装配层/公共库/CI
的顺序通读了全部 P0a 代码，改掉三处：

1. **`sim` 参数的报错分支此前零覆盖** —— CI 只跑 `--show-args`，而选数据源的
   `OpaqueFunction` 是 launch 真正启动时才执行的，`--show-args` 碰不到它。
   讽刺之处在于：那段代码本身就是为了消灭静默失败而写的，自己却没被验证过。
   补了两层：`ads_bringup` 的 7 个 pytest 用例（函数级，0.03 s）+ CI 的端到端
   shell 断言（证明 `ros2 launch` 真的以非零码退出，**且失败原因正确** ——
   只断言退出码非零会让"环境坏了"也算通过，那又是一次假绿）。
   > ⚠️ 其中 `test_registries_do_not_overlap` 是**给 P0b 埋的**：
   > `_resolve_sim_source` 先查 `PLANNED_SOURCES` 再查 `SIM_SOURCES`，
   > 所以往 `SIM_SOURCES` 加 `carla` 时若忘了从 `PLANNED_SOURCES` 删掉，
   > `sim:=carla` 会永远报「尚未实现」。症状是"我明明实现了它说我没实现"，
   > 而所有人第一反应都是去查 `carla_bridge` 装没装上。
2. **teleop 用墙钟、下游看门狗用仿真钟** —— 补注释写清成立条件：
   发布间隔换算成仿真时间是 `(1/20)×RTF`，看门狗阈值 0.5 仿真秒，
   于是 **RTF > 10 时看门狗会持续误触发**。当前 RTF≈1.0 余量 10 倍，
   现状安全，但这是隐含假设。将来若拿它做加速回放的指令注入会撞上，
   正确改法是换成节点时钟，**不是调大看门狗阈值**（那是削弱安全逻辑）。
3. **`/lidar/points_raw` → `/gazebo/lidar/points_raw`** —— 加前缀标明它不是
   SPEC §4.1 的对外契约（与 `/gazebo/cmd_vel` 同一约定），并避免与
   `carla_bridge` 的 `/carla/lidar/points_raw` 串台。
   顺带堵掉一个隐患：这个名字原本在 `bridge_topics.yaml` 和 C++ 默认值里
   **各写一份且无人校验**，漂移的症状是预处理节点订阅了没人发的话题，
   而它只在收到点云时才打日志 → 零日志、零报错、RViz 一片空白。
   现在由 launch 从 `bridge_topics.yaml` 读出来传进去（按"类型+方向"查表，
   不按名字查，否则等于又写死一次）。

复查后实测：`colcon test` **101 tests / 0 failures**（原 91），
`verify_ros_bridge.sh` **6/6**（RTF 0.970、点云 10.00 Hz、自车点 0），
`verify_teleop.sh` **3/3**。

### ⏸️ 下一步仍待用户决定，**不要自行开工**

CP3 的分叉是 **P1（继续本地）** 还是 **P0b（先建云端 CARLA）**。
用户已通读完代码并认可上述三项修改，但**尚未选定下一阶段**。
所以下次开始时，在他明确说做哪个之前，**不要动手写 P1 或 P0b 的代码**。

两个选项的权衡（上次已完整汇报过，这里只留结论）：

| | 支持它的理由 | 代价 |
|---|---|---|
| **P1 地图与路由** | 现在还没有任何算法代码，`carla_bridge` 要对齐的「行为」根本不存在，对齐空栈没意义 | CARLA 侧一直没验证 |
| **P0b 云端 CARLA** | 「行为漂移」是本项目自列的头号风险，越晚接触越难查；且 `gazebo_bridge` 刚做完，接口契约最清晰 | 开始产生 GPU 实例费用 |

**上次给出的建议是 P1，但附带一个条件：不应晚于 P2 结束就接上 CARLA。**
一旦控制参数已经在 Gazebo 上调好才去对齐，漂移就已经发生了 —— 那正是最难查的故障。

### 恢复环境（宿主机执行）

```bash
cd ~/work/automated-driving-systems           # ⚠️ 必须在仓库根目录，COMPOSE_FILE 是相对路径
export COMPOSE_FILE=docker/docker-compose.local.yml
docker compose up -d                          # 镜像已构建好，直接起

docker compose exec dev /workspace/scripts/verify_gpu.sh          # GPU 硬件加速（10 项）
docker compose exec dev /workspace/scripts/verify_sim.sh          # 仿真基线 + RTF（6 项）
docker compose exec dev /workspace/scripts/verify_ros_bridge.sh   # 桥接契约（6 项）
docker compose exec dev /workspace/scripts/verify_teleop.sh       # 控制链路（3 项）
```

四个脚本都退出码 0 才往下做。**若 `verify_gpu.sh` 挂了，先看它是不是又变回 llvmpipe**
—— Windows 侧驱动更新或 WSL 重启后 `/dev/dxg` 的存在性值得重新确认。

**若 `gz sim` 启动几秒就退出、RViz 里只剩个孤零零的车模型**：那是 GPU 驱动瞬态
故障（D3D12 device removal），`verify_gpu.sh` 照样全过、launch 日志还会谎报
`finished cleanly`。**重跑一次即可，不要去改 compose**。详见 CLAUDE.md 陷阱表。

构建与测试（容器内）：

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /workspace/install/setup.bash

colcon test && colcon test-result --all     # 应为 101 tests, 0 errors, 0 failures
./build/ads_common/test_angles              # 只跑 L1，0.003 s
```

肉眼看效果（带 Gazebo GUI + RViz）：

```bash
ros2 launch ads_bringup stack.launch.py                 # 全栈入口
ros2 launch gazebo_bridge gazebo_sim.launch.py          # 只起仿真侧，调链路时更快
docker compose exec dev /workspace/scripts/drive.sh     # 键盘开车（另开终端）
```

### 🧹 跑完 Gazebo 记得清 core

每次 `gz sim` 收 SIGINT 退出都会 segfault 产 **400–600 MB** 的 `core.<pid>`，
落在仓库根目录。`.gitignore` 拦住了提交，但磁盘会被吃掉：

```bash
rm -f core.*
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
- [x] **3.8** `ads_visualization/rviz/default.rviz`
      　　✅ 程序化确认：RViz 已订阅 `/lidar/points`、`/robot_description`、`/tf`，无报错
      　　✅ **肉眼确认通过**：按高度染色的点云、TF 坐标轴、蓝色车模型 + 四个黑轮同时正确显示；
      　　　 `base_link` 的红色（X）箭头指向车头
      　　ℹ️ 两个 RViz 配置上的坑已写进 `default.rviz` 头部注释：
      　　　 `RobotModel` 必须用 `Transient Local` 订阅（否则车是空的且不报错）

> **CP2 结果：`verify_ros_bridge.sh` 6/6 通过 ✓**　数据流已打通，频率达标。
> 带激光雷达后 RTF = **0.998**（S2 纯物理是 1.000）—— 传感器几乎没有代价。

### 3.x 追加：自车反射点滤除（超出原计划，用户确认后做）

CP2 通过后检查点云 z 分布时发现：**34% 的点（9874/29239）打在自己车顶上。**
`z ∈ [1.4, 1.6]` 的点其 `y` 范围是 −0.69 ~ +0.69 m，与车体顶面宽度（±0.675 m）
几乎逐位吻合。原因是雷达装在 z=1.6，而**车顶就在 z=1.5，只高出 10 cm**，
所有向下的光束在够到地面前先打在自己车顶上。

这在物理上是**正确**的（真车的车顶激光雷达同样如此），但不滤掉的话
**P5 感知会把自车车顶识别成一个零距离障碍物**，直接触发急刹且永远甩不掉。
所以每个量产 AD 栈都有 self-filter。

**否决过的方案**：把雷达抬高。没用 —— 车顶长 4.4 m，抬到 2.0 m 后
−25° 的光束照样落在车顶上。真正的解法只能是按自车轮廓裁剪。

实现在 `lidar_preprocessor`（原 `pointcloud_to_base_link`，改名因为它现在做三件事）：

| 步骤 | 说明 |
|---|---|
| 1. 坐标变换 | `lidar_link` → `base_link`，走 `tf2::doTransform`（SPEC §11 禁止手写变换矩阵） |
| 2. 自车滤除 | 剔除落在自车包围盒内的点。盒子由 launch 从 `vehicle_params.yaml` 算出后传参，**C++ 里不写任何车身尺寸** |
| 3. 无效点滤除 | 剔除 ±inf / NaN |

**为什么第 3 步是第 2 步的必然结果**：一旦开始剔除点，点云就从有序（32×1800 网格）
变成无序。而那 28361 个 ±inf 点在有序点云里的唯一作用就是占住网格位置 ——
无序之后它们纯粹是垃圾，下游还得逐点跳过。所以必须一并剔除。

**失去有序性的代价比想象中小**：Gazebo 的 gpu_lidar 输出带 **`ring` 字段**（线束编号），
基于线束的地面分割算法靠它就能恢复线号，不依赖网格排布。

实测（`verify_ros_bridge.sh` 仍 6/6 通过）：

| | 改前 | 改后 |
|---|---|---|
| 有效点/帧 | 29239 | **19365** |
| 自车盒内点 | 9874 | **0** |
| 消息大小 | 1.84 MB | **0.62 MB**（−66%） |
| `z` 上界 | 1.507 m（车顶） | **1.022 m**（1 m 障碍方块） |
| 频率 | 10.00 Hz | 10.00 Hz |

> `z` 上界从 1.507 掉到 1.022 是最能说明问题的一个数：车顶（1.5 m）的点没了，
> 现在最高的东西正是世界里那三个 1 m 高的障碍方块。

判据已固化进 `check_cloud_frames.py`：**自车盒内必须 0 点**，且输出必须是
无序 + `is_dense=true`（滤除若被绕过，这两条会同时报警）。

### 3.5 排查记录 1：点云频率只有标称的 35%，病根不在 GPU

第一次测 `/lidar/points` 只有 **3.58 Hz**（标称 10 Hz）。直觉是「GPU 渲不动」，
于是把水平采样从 1800 砍到 900 —— 频率只从 8.65 涨到 8.97 Hz（**+4%**）。
**射线数减半而几乎无改善，说明瓶颈根本不在射线数上。**

分层测量把问题定位清楚了：

| 测量点 | 1800 采样 | 900 采样 |
|---|---|---|
| Gazebo 原生话题（无 ROS） | 8.65 Hz | 8.97 Hz |
| ROS `/gazebo/lidar/points_raw` | 6.67 Hz | 9.23 Hz |
| ROS `/lidar/points` | 3.58 Hz | 8.00 Hz |

> 注：中间话题当时叫 `/lidar/points_raw`，P0a 收尾复查时改名加了 `gazebo` 前缀
> （标明它不是 SPEC §4.1 的对外契约，且避免与 `carla_bridge` 的同名话题串台）。
> 翻 S3 时期的日志会看到旧名字。

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

- [x] **4.1** teleop 节点：键盘 → `/vehicle_cmd`（转角 rad + 加速度 m/s²）
      　　✅ 新建 **`ads_teleop`** 包（已加进 SPEC §5）。teleop 与仿真器无关，
      　　　 P0b 接 CARLA 时原样复用 —— 塞进 `gazebo_bridge` 就绑死在 Gazebo 上了
      　　✅ 自动化验证（`check_keyboard_teleop.py`，用 **pty** 程序化按键）：
      　　　 启动初值为零、w×5 累加到上限 1.500 且不越限、a 左转为正、
      　　　 d 使转角减小、空格归零、b 下发紧急制动 −5.000
- [x] **4.2** `gazebo_bridge` 订阅 `/vehicle_cmd` → 转 Gazebo 指令
      　　✅ 直行：6 s **Δx = +25.5 m**；转向：**Δy = +7.8 m**
      　　ℹ️ 新增 `vehicle_cmd_bridge` 节点，桥接表加了**第一条 `ROS_TO_GZ`**
- [x] **4.3** 指令限幅（超 `vehicle_params.yaml` 范围则截断并告警）
      　　✅ 下发转角 10 rad / 加速度 100 m/s²，桥接输出被卡在
      　　　 **0.600 rad** 和 **7.44 m/s**（限值 0.600 / 8.333）
      　　✅ **NaN 指令被拒绝**（NaN 参与比较恒为 false，clamp 会放行，
      　　　 一路传到物理引擎会解算出 NaN 位姿，车直接从世界里消失）
      　　✅ **看门狗**：停发指令 6.3 s 后速度 8.333 → 0.000 m/s
- [x] **4.4** 🎯 **闭环验证**：键盘开车绕障碍物一圈
      　　✅ **已肉眼确认**：全程 RViz 中点云 / TF / 车姿态同步更新

> **4.1-4.3 结果：`verify_teleop.sh` 3/3 通过 ✓**
> S3 的 `verify_ros_bridge.sh` 回归重跑仍 6/6 通过。

### 怎么开车（4.4）

需要**两个终端**：

```bash
# 终端 A —— 仿真 + 桥接 + RViz
docker compose exec dev bash -c 'source /opt/ros/jazzy/setup.bash && \
  source /workspace/install/setup.bash && \
  ros2 launch gazebo_bridge gazebo_sim.launch.py'

# 终端 B —— 键盘
docker compose exec dev /workspace/scripts/drive.sh
```

按键：`w/s` 加减速、`a/d` 左右转、`空格` 松油门回正、`b` 紧急制动、`q` 退出。

> ⚠️ **不要用 `ros2 launch ads_teleop keyboard_teleop.launch.py` 开车** ——
> launch 会接管子进程的 stdio，键盘输入到不了节点。详见下面排查记录 2。

### 两个物理事实，不是 bug

1. **静止时打方向车不会转。** 下游把转角换算成横摆角速度 ω = v·tan(δ)/L，
   v=0 时 ω 恒为 0。真车原地打方向车也不动，阿克曼转向就是这样。
2. **不支持倒车。** `ads_msgs/VehicleCmd` 只有转角和加速度、**没有挡位字段**，
   负加速度只能理解成"减速"，无法与"倒车"区分。真实栈（如 Autoware）
   用独立的 GearCommand 解决。

### 4.1 排查记录 1：pty 测试里"前几个按键有效、之后全无反应"

第一版自动化测试跑出来：`w` 和 `a` 正常，之后 `d`、空格、`b` 全部无响应。
看着像节点的按键分发有 bug，其实是**测试脚手架的问题**：

节点每个定时周期往 pty 写一行状态，而测试端从来没读过 pty 的 master。
伪终端缓冲区只有几 KB，填满后节点的 `printf` **阻塞**，定时器回调卡死，
键盘就再也读不到了。收到 238 条 ≈ 12 s × 20 Hz × 60 字符 ≈ 14 KB，正好对上。

两处都改了：测试端加读取线程排空 pty；节点端改成**只在指令变化时才打印** ——
每周期刷屏本来也没必要，在 `ros2 launch` 下还会把别的节点日志淹掉。

### 4.1 排查记录 2：`ros2 launch` 下节点假死（真 bug）

现象：`ros2 run` 起 teleop 一切正常；`ros2 launch` 起同一个节点，
进程活着、`ros2 topic info` 能看到 publisher，**但一条消息都发不出来**，
40 s 一条都收不到。日志里帮助文本正常打印，没有任何报错。

原因：**`termios` 的 `VMIN=0 / VTIME=0` 只对终端生效。**
`ros2 launch` 下子进程的 stdin 是**管道**：`tcgetattr` 失败 → raw 模式没设上 →
`read()` 在管道上**永久阻塞** → 定时器回调卡死 → 发布者存在但永远不发。
进程状态 `Sl`（睡眠）也对得上 —— 它睡在 `read` 里。

修复：给 fd 加 **`O_NONBLOCK`**。它对管道、终端、`/dev/null` 一视同仁，
不依赖 stdin 是什么类型。另外在 `isatty()` 为假时明确告警，
不再是"按键没反应"这种查不出原因的现象。

**教训**：非阻塞 I/O 有两套独立机制 —— `termios` 的 VMIN/VTIME 只管终端，
`O_NONBLOCK` 管所有 fd。只用前者的代码在终端下测试完全正常，
一换到管道就死锁，而且**不报错**。

---

## S5　工程化收尾　（可与 S4 并行）　【检查点 CP3】

- [x] **5.1** `.clang-format`（ROS 2 风格，SPEC §7）
      　　✅ 全量 lint **154 failures + 4 errors → 0**
      　　⚠️ 起点是个意外发现：lint **一直在跑，只是全红没人看**。
      　　　 比"没配 lint"更糟 —— CI 一接上就是满屏红，新问题会淹没在旧噪声里
      　　ℹ️ 选 clang-format 而非 ROS 2 默认的 uncrustify（后者几乎没有编辑器集成）
- [x] **5.2** `stack.launch.py`，支持 `sim:=gazebo`
      　　✅ 新建 `ads_bringup` 包（SPEC §5 已规划）
      　　✅ 用 S3 的 6 项契约检查直接验收：`LAUNCH_PKG=ads_bringup
      　　　 LAUNCH_FILE=stack.launch.py verify_ros_bridge.sh` → **6/6，RTF 1.004**
      　　✅ 非法 `sim` 值报错而非静默空转（两类错误信息分开）
- [x] **5.3** L1 单元测试样板（`ads_common` 几何函数 + gtest）
      　　✅ 12 个用例跑完 **0.003 s**（不链接 ROS 才做得到）
      　　✅ **测试抓出了文档与实现不符** —— 见下面的排查记录 3
- [x] **5.4** CI（GitHub Actions）：build + test + lint
      　　✅ 四步：生成物同步检查 → 构建 → lint+L1 → launch 可加载性
      　　ℹ️ **不跑 Gazebo**：不只因为 runner 没 GPU，更因为现阶段
      　　　 根本没有 L3 场景测试可跑（要等 P2 的闭环）
- [x] **5.5** `README.md`：从零复现 P0a
      　　✅ 补 `LICENSE`（Apache-2.0 官方原文，`package.xml` 早就声明了但文件没有）
- [x] **5.6** `docs/adr/0001-dual-simulation.md`
      　　✅ 记录了 5 个备选方案及各自的否决理由，以及"行为漂移"的 4 条缓解措施

> **CP3**：P0a 验收。决定下一步做 P1（继续本地）还是 P0b（先建云端）。

### 5.1 排查记录 1：lint 不是没配，是全红没人看

开工前以为 5.1 就是"加个 `.clang-format` 文件"。跑了一次 `colcon test` 才发现
**154 failures + 4 errors** —— `ament_lint_auto` 一直在跑，只是从来没人看结果。

**这比"没配 lint"更糟**：CI 一接上就是满屏红，之后新增的真实问题会淹没在既有
噪声里，而人对满屏红的自然反应是不看。

数字吓人但修起来不大 —— **125 个 flake8 失败里 121 个是同一个单双引号问题**。
先分类再动手，不要被总数吓住。

### 5.1 排查记录 2：两个官方工具在官方配置下互相打架

ROS 2 官方的 `.clang-format` 没有设 `IncludeBlocks`，于是继承 Google 风格的
`Regroup` —— 把所有 `#include` 合并成一整块按字母序排。结果：

```cpp
#include <ads_msgs/msg/vehicle_cmd.hpp>   // 第三方
#include <algorithm>                      // C++ 标准库
```

而 cpplint（`ament_lint_common` 成员，也是官方的）要求
「本文件头 → C 标准库 → C++ 标准库 → 其它」。字母序恰好把第三方排到了标准库
前面，两个官方工具互相要求对方改回去，**实测 7 处报错**。

解法是设 `IncludeBlocks: Preserve`（只在空行分隔的块内排序，不跨块合并），
这是本项目唯一刻意偏离官方配置的一项，理由写在 `.clang-format` 末尾。

**教训**：「用官方默认配置」不等于「不会有冲突」。官方那几个工具是各自独立
演进的，组合起来的行为没人验证过。

### 5.3 排查记录 3：L1 测试抓出的是**文档**的错，不是实现的错

写 `normalize_angle` 的测试时断言 `normalize_angle(3π) == +π`，失败了，实际是 **−π**。

原因是 `std::remainder` 按 IEEE 754 的「就近取偶」选商：

```
π  / 2π = 0.5  → 最近的整数 0 和 1，取偶得 0 → π − 0  = +π
3π / 2π = 1.5  → 最近的整数 1 和 2，取偶得 2 → 3π − 4π = −π
```

数学上 +π 和 −π 是同一个方向，**两个答案都对**。错的是我在头文件里写的
「±π 返回 ±π」那句承诺 —— 它只在输入恰好是 ±π 时成立。

**选择改文档而不是改实现**：加特判把符号统一，只是把不确定性从数据挪进代码，
还在热路径上多一个分支，并没有消除这个奇点（±π 意味着"正对反方向"，
左转右转本来就等价）。改为明确声明「边界符号不保证，调用方不应依赖」。

这条值得单独记，因为它展示了 L1 测试真正的价值：**它逼你把模糊的承诺写成
可执行的断言，一写就发现承诺是假的。** 不写测试的话，这个不一致会一直留到
某个控制器在 ±π 附近抖动时才暴露。

---

## 进度

```
S1 █████  5/5 ✓   S2 ██████  6/6 ✓   S3 ████████  8/8 ✓
S4 ████  4/4 ✓    S5 ██████  6/6 ✓
                                          总计  29/29 ✓
```

**已达成的检查点**：CP1（GPU 硬件加速）✓　CP2（数据流打通）✓　CP3（P0a 验收）✓

### P0a 收官实测汇总

| 项 | 实测 | 判据 |
|---|---|---|
| 容器内 OpenGL | 4.6 core / D3D12 (Radeon 780M) / Accelerated: yes | ≥ 3.3 |
| 实时率 RTF（带 32 线雷达） | **1.000** | ≥ 0.8 |
| `/lidar/points` | **10.00 Hz**，`base_link` 系，自车点 0 | ≥ 9.0 Hz |
| TF 树 | 五段连通，外参与 YAML 逐位一致 | 完全一致 |
| 转角 / 速度限幅 | 10 rad → **0.600**；100 m/s² → **7.41 m/s** | 不越限 |
| 看门狗 | 停发 6.3 s 后 8.333 → **0.000 m/s** | 必须刹停 |
| `colcon test`（6 包） | **101 tests, 0 errors, 0 failures** | 全绿 |
| L1 单元测试耗时 | **0.003 s** | 毫秒级 |

四个 verify 脚本：`verify_gpu.sh` 10/10、`verify_sim.sh` 6/6、
`verify_ros_bridge.sh` 6/6、`verify_teleop.sh` 3/3。

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
- **P0b**：`gazebo_bridge/CMakeLists.txt` 伸手到 `../../../config/vehicle_params.yaml`
  才能把车辆参数装进本包 share。功能没问题（`--symlink-install` 下是同一份），
  但这让包不能单独拷出去用。**等 `carla_bridge` 出现、变成两个包都要伸手时**
  再抽独立的 `ads_description` 包 —— 那属于改 SPEC §5 的模块划分，动手前先问。
  （2026-07-31 收官复查时讨论过，判断是现在抽收益不明确。）
- **P2 控制**：`model.sdf` 的惯量张量按**均质**长方体算，而质心被显式下移到
  `com_height_m`(0.55)，两者不严格自洽；`kingpin_width` 直接取了轮距（真车主销距
  略小于轮距，影响内外轮转角差）。低速园区场景下影响可忽略，建执行器模型时
  一并处理。（同上，收官复查记录。）
