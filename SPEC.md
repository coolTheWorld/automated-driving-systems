# SPEC — 园区/工业场景自动驾驶系统

> 状态：**基线已确认，可开工**（P0 起进入实施）
> 创建日期：2026-07-30　|　最后更新：2026-08-12（话题名对账、P5 行、测试体系标注；上一版 2026-07-30）
> 目标读者：项目开发者（无人驾驶初学者）+ 未来接手的工程师/AI Agent

---

## 0. 阅读顺序说明

本文档是整个项目的**唯一事实来源（Single Source of Truth）**。
任何代码、任务拆解、评审都以本文档为准；本文档与代码冲突时，先改本文档。

---

## 1. 目标（Objective）

### 1.1 一句话定义

构建一套**面向园区/工业场地的低速自动驾驶软件栈**，使用 ROS 2 + C++ 实现，
覆盖 **感知、定位、预测、决策、规划、控制** 六大模块。
日常开发在本机 Gazebo 闭环验证，最终验收在云端 CARLA 完成（见 §4.1）。

### 1.2 明确的非目标（Non-Goals）

写清楚"不做什么"和写清楚"做什么"同等重要。

| 不做 | 原因 |
|------|------|
| 车规级功能安全认证（ISO 26262 / SOTIF） | 需要硬件冗余、失效分析、第三方审核，非软件项目可覆盖 |
| 实车部署与路测 | 无车辆平台；且未经充分仿真验证的代码上实车是危险行为 |
| SOTA 深度学习模型训练 | 目标是理解系统架构，不是刷 benchmark；模型用现成的或经典方法替代 |
| 高速公路场景（匝道汇入、高速变道） | 不在 ODD 内，见 §2 |
| 多车协同 / V2X | 独立课题，规模过大 |

> **给初学者的说明**：真实项目里，"非目标"清单往往比"目标"清单更能救命。
> 它是你拒绝需求蔓延（scope creep）的依据。

### 1.3 成功标准（项目级验收）

**最终验收在云端 CARLA 进行**（本机 Gazebo 只是开发环境，不作为验收依据）。
系统在指定园区地图上，需能够：

1. 从 A 点自主导航至 B 点，全程无人工接管
2. 正确检测并跟踪动态障碍物（车辆、行人），无漏检导致的碰撞
3. 遇到前方静止/慢速障碍物时安全减速或绕行
4. 遇到横穿行人时及时刹停，停止后可恢复行驶
5. 定位横向误差 < 0.3 m，航向误差 < 2°
6. 全程无违反交通规则（闯红灯、逆行、超速）
7. 乘坐舒适性：纵向加速度 |a| < 2.0 m/s²，加加速度 |jerk| < 2.0 m/s³

---

## 2. ODD（Operational Design Domain，运行设计域）

**这是整个项目最重要的一节。** ODD 定义系统"在什么条件下保证工作"，
它直接决定了每个模块的算法选型和复杂度。写不清 ODD 的自动驾驶项目一定会失控。

| 维度 | 约束 |
|------|------|
| 场地 | 封闭/半封闭园区、厂区、物流园（地理围栏内） |
| 地图 | **有先验高精地图**（CARLA 提供 OpenDRIVE） |
| 速度 | 设计最高 30 km/h，常用巡航 15-20 km/h |
| 道路 | 结构化 + 半结构化，单/双车道，含路口、环岛、停车区 |
| 交通参与者 | 乘用车、行人、非机动车、工程车辆（叉车/物流车） |
| 天气 | 晴天、多云（**第一阶段不含雨雾夜间**） |
| 时段 | 白天 |
| 排除场景 | 高速公路、无保护左转穿越高速车流、恶劣天气、无地图区域 |

### 为什么低速园区场景是正确的起点

- **速度低 → 制动距离短 → 感知距离要求从 200 m 降到 50 m**，激光雷达方案完全够用
- **有高精地图 → 不需要在线建图/车道线检测**，定位问题从 SLAM 简化为地图匹配
- **地理围栏 → 场景可穷举**，测试用例写得完
- **但它仍然是完整的 L4 系统**，六大模块一个都不能少

---

## 3. 系统架构

### 3.1 数据流

```
                    ┌─────────────────────────────────────┐
                    │   仿真数据源（可插拔，见 §4.1）        │
                    │   Gazebo ／ CARLA ／ rosbag 回放      │
                    │  (车辆动力学 / 传感器 / 交通流)       │
                    └──────────────┬──────────────────────┘
                                   │ ads_simulation 抽象层
        ┌──────────────────────────┼──────────────────────────┐
        │ /lidar  /camera  /imu  /gnss  /odom       /vehicle_cmd
        ▼                                                     ▲
┌───────────────┐    ┌──────────────┐                        │
│  ads_map      │    │ ads_localiz. │                        │
│  OpenDRIVE    │───▶│ NDT + EKF    │                        │
│  车道图/路由   │    │ → /localization/pose │                        │
└───────┬───────┘    └──────┬───────┘                        │
        │                   │                                │
        │            ┌──────▼────────┐                       │
        │            │ ads_perception│                       │
        │            │ 点云滤波→聚类  │                       │
        │            │ →检测→跟踪     │                       │
        │            │ → /perception/obstacles │                       │
        │            └──────┬────────┘                       │
        │                   │                                │
        │            ┌──────▼────────┐                       │
        │            │ ads_prediction│                       │
        │            │ 意图+轨迹预测  │                       │
        │            │ → /prediction/trajectories │                       │
        │            └──────┬────────┘                       │
        │                   │                                │
        └──────────┬────────┘                                │
                   ▼                                          │
           ┌───────────────┐                                 │
           │ ads_planning  │                                 │
           │ 行为决策(BT)   │                                 │
           │  ↓            │                                 │
           │ 参考线生成     │                                 │
           │  ↓            │                                 │
           │ Frenet 轨迹   │                                 │
           │  ↓            │                                 │
           │ 速度规划       │                                 │
           │ → /trajectory │                                 │
           └───────┬───────┘                                 │
                   ▼                                          │
           ┌───────────────┐                                 │
           │  ads_control  │                                 │
           │  横向: Stanley │─────────────────────────────────┘
           │  纵向: PID     │
           └───────────────┘
```

### 3.2 模块职责与算法选型

选型原则：**先经典后现代，先可解释后高精度**。
每个模块都留了升级路径，但第一版必须是你能手推公式的方案。

| 模块 | 第一版方案 | 升级路径 |
|------|-----------|---------|
| **地图 ads_map** | 解析 OpenDRIVE → 车道级有向图；Dijkstra/A* 路由 | 语义地图、动态区域 |
| **定位 ads_localization** | 点云 NDT 配准 + ESKF 融合 IMU/GNSS/轮速 | LIO 紧耦合、多传感器时空标定 |
| **感知 ads_perception** | 地面分割(RANSAC/Patchwork) → 欧式聚类 → L-Shape 拟合朝向 → 匈牙利匹配 + EKF 多目标跟踪 | PointPillars/CenterPoint、相机-激光后融合 |
| **预测 ads_prediction** | 地图约束下的车道跟随 + 恒速模型；行人恒速+不确定椭圆 | 意图分类器、VectorNet 类模型 |
| **决策 ads_planning/behavior** | 行为树（Behavior Tree），显式状态：巡航/跟车/避障/让行/停车 | 有限状态机→POMDP |
| **规划 ads_planning/motion** | Frenet 坐标系下横纵解耦采样（Lattice），代价函数排序 + 碰撞检测 | 数值优化（OSQP）、时空联合规划 |
| **控制 ads_control** | 横向 Stanley，纵向速度环 PI + 加速度前馈（**加速度内环见下方脚注**） | LQR、MPC |

> **关于「纵向双闭环」的脚注**（2026-08-01 拍板，P2-S4 实测确认）
>
> 本表原写「纵向双闭环 PID（速度环 + 加速度环）」。**P2 实现的是单速度环，
> 这不是"实现漏了"，是一个有依据的取舍**：
>
> 加速度内环要闭合，前提是能测到实际加速度、且执行机构与指令之间有可辨识的动态。
> 而当前链路上 `gazebo_bridge` 把加速度**积分**成速度设定值下发
> （`speed_setpoint += a·dt`），那一段几乎是理想积分器 ——
> 内环等于在补偿一个恒等式，加进去只增加一个可以调坏的旋钮，量不出任何改善。
>
> 缺的那一环由**加速度前馈**补上：速度剖面自己知道每一点该有多大加速度
> （`a_ff = ½·d(v²)/ds`），直接加在输出上。P2-S4 实测：补之前终点冲过 4.26 m、
> 最大横向加速度 2.113；补之后 0.257 m 和 1.594，两条判据同时转绿。
>
> **什么时候要把内环加回来**：有了真车或 CARLA 的执行机构模型之后 ——
> 那时"指令加速度 → 实际加速度"之间有真实的滞后与非线性，内环才有可闭合的对象。
> 用 `scripts/probe_steering_response.py` 那类开环探针先把它辨识出来，再决定结构。

> **为什么用 Frenet 而不是直接在笛卡尔系规划？**
> 因为在道路坐标系下，"沿车道走多远(s)"和"偏离中心线多少(d)"是解耦的，
> 横向和纵向可以分别采样再组合，搜索空间从二维降到两个一维问题。
> 这是 Apollo EM Planner 和绝大多数量产规划器的基础思想。

### 3.3 关键设计约束

1. **模块间只通过 ROS 2 话题/服务通信**，不允许直接函数调用跨模块
   → 保证任一模块可被替换、可单独测试、可分布式部署
   > **注解（P6-1 决策二，2026-08-12 用户拍板）**：这条禁的是**运行时数据**
   > 绕过话题在模块间横穿。**静态先验**（地图、车辆参数等单一来源的生成物）
   > 允许经共享库直接读取 —— 两个模块读同一份 `campus.xodr` 与两个模块读
   > 同一份 YAML 配置性质相同（§4.1 本来就要求两环境共用同一份 .xodr）。
   > 当前唯一实例：`ads_prediction` 链接 `libads_map` 做车道跟随预测。
   > 不要据此推广成「模块可以随意互相链接」。
2. **算法核心逻辑与 ROS 解耦**：每个包分 `lib/`（纯 C++，无 ROS 依赖）和 `node/`（ROS 包装）
   → 算法可脱离 ROS 做单元测试，跑得飞快
3. **所有参数外置到 YAML**，不允许硬编码魔数
4. **统一坐标系约定**：`map`（全局，ENU）→ `odom` → `base_link`（车辆后轴中心，x 前 y 左 z 上）
   全部走 TF2，禁止手写坐标变换。
   **每一段有且只有一个发布者**：`map→odom` 归**定位**（P4 前是仿真侧按自车 spawn 位姿发的
   静态 TF，P4 起由 `localization_node` 动态发布，二者互斥）；`odom→base_link` 归**里程计**。
   ⚠️ 两个发布者同时发同一段**不报错，而且不一定看得出来** —— L3-G 故障注入实测：
   多挂一个静态 `map→odom` 之后，末段位置误差只从 0.012 m 变成 0.043 m，全部数值判据仍绿。
   危险恰在这里：**哪一份生效取决于两个发布者的启动顺序**（tf2 在某个 frame 第一次被写入时
   就定死它用静态缓存还是时间缓存），而启动顺序不是你能控制的量。
   所以这条约束必须**机械地查**（`/tf_static` 上不许出现该段），不能靠观察输出验收。
5. **时间戳**：所有消息使用仿真时间（`use_sim_time=true`），禁止用 `now()` 做算法时序

---

## 4. 技术栈与环境

| 项 | 选择 | 说明 |
|----|------|------|
| 语言 | C++17 | ROS 2 主流标准；不上 C++20 是为了库兼容性 |
| 中间件 | ROS 2 **Jazzy** (LTS, 至 2029-05) | **被 Gazebo 官方组合正向约束，见下方**；不用 Humble/Lyrical |
| 容器基础系统 | Ubuntu 24.04 | Jazzy 目标系统 |
| DDS | **Fast DDS**（Jazzy 默认 `rmw_fastrtps_cpp`） | ⚠️ **不可改用 CycloneDDS**，CARLA 原生 ROS 2 只支持 Fast DDS |
| 构建 | colcon + CMake | ROS 2 标准 |
| 仿真（日常开发） | **Gazebo Harmonic** | 跑**本机 WSL2**，硬件加速，可进 CI。**与 Jazzy 是官方组合** |
| 仿真（最终验收） | **CARLA 0.9.16** | 跑**云 GPU（Vast.ai）**，服务端**原生 ROS 2**，无需 ros-bridge |
| 运行环境 | Docker，本地与云端**同一镜像** | 保证代码只编译一套，行为可比对 |
| 远程开发 | SSH + VS Code Remote | 仅云端阶段使用 |
| 远程可视化 | noVNC / VNC | 传画面，不传点云 |
| 数学库 | Eigen3 | 线性代数 |
| 点云 | PCL | 滤波、配准、分割 |
| 优化 | OSQP（后期） | 二次规划 |
| 测试 | GoogleTest + launch_testing | 单元 + 集成 |
| 可视化 | RViz2 + PlotJuggler | 调试必备 |

### 环境现状（2026-07-30 实测）

```
WSL2 侧：
  OS       Ubuntu 26.04 LTS (resolute)
  ROS 2    未安装；resolute 对应 Lyrical，仓库 2026-07-27 上线
  GPU      无 NVIDIA 设备（/usr/lib/wsl/lib 仅有 d3d12/dxcore，无 libcuda）
  内存     15 GB / CPU 8 核 / 磁盘 949 GB 可用
  CMake    未安装    Docker  未安装

Windows 宿主：
  GPU      AMD Radeon 780M —— RDNA3 核显，无独立显存，无 CUDA

WSL2 图形栈实测（glxinfo -B）：
  /dev/dri            card0 + renderD128 均存在
  Mesa 驱动           d3d12_dri.so（WSLg → Windows GPU 直通）
  direct rendering    Yes        Accelerated  yes
  Renderer            D3D12 (AMD Radeon 780M Graphics)
  OpenGL              4.6 core profile
  可用显存            16.5 GB（UMA 共享系统内存）
```

**三条结论**：

- **本地跑不了 CARLA**：Radeon 780M 是核显、无 CUDA，且 CARLA 官方只真正支持 NVIDIA。
  → CARLA 上 **Vast.ai 云 GPU**（见 §4.1 环境 B）。
- **本地跑得了 Gazebo**：OpenGL 4.6 **硬件加速**可用，而 Gazebo 的 OGRE 渲染只要 3.3+。
  → 日常开发用**本机 Gazebo**（见 §4.1 环境 A）。这是省钱和提速的关键。
- **宿主机的 ROS 版本与本项目无关**：全部工作在容器内进行，宿主机不装 ROS。

> 注意这里的 OpenGL 是经 D3D12 转译层实现的，性能低于原生，但对园区场景这种
> 几何简单的世界完全够用。**Vulkan 在 WSL2 下不保证可用**，所以 Gazebo 要用
> OpenGL 后端（默认即是），不要去折腾 Vulkan。

#### 版本约束链（重要，不要自行改动）

```
Gazebo Harmonic ──官方组合──▶ ROS 2 Jazzy ──目标系统──▶ Ubuntu 24.04
   (LTS 至 2028)              (LTS 至 2029-05)          (Python 3.12)
                                    │                        │
                                    │  Fast DDS              │  CARLA 0.9.16
                                    ▼  (默认 RMW，不可改)     ▼  官方提供 3.12 wheel
                              CARLA 0.9.16 原生 ROS 2 ◀───────┘
                              (--ros2，内置 Fast DDS)
```

**这条链的方向是从 Gazebo 往外推。** 理由是**工作量分布**：路线图 P1-P4、P6-P7
全部在 Gazebo 上完成，只有 P5 和最终验收碰 CARLA。
**把官方支持放在你天天用的那一侧**，比放在一个月碰一次的那一侧划算得多。

> **⚠️ 这与本文档早期版本的结论相反，理由变更记录见 §9 D3。**
> 简言之：早期约束链基于 CARLA **0.9.15**（PythonAPI 仅保证 Python 3.7/3.8，
> 必须靠 Ubuntu 22.04），而 **0.9.16 官方提供 3.10/3.11/3.12 wheel**，
> Ubuntu 24.04 的限制随之消失，约束链的瓶颈也就从 CARLA 转移到了 Gazebo。

**不可自行改动的三条**：

1. **不要升到 Lyrical**：发布于 2026-05，Gazebo 与 CARLA 两侧生态均为空白。
2. **不要退回 Humble**：Humble 的官方 Gazebo 搭档是 Fortress（**2026-09 EOL**），
   配 Harmonic 属非官方组合；且 Humble 自身 2027-05 EOL，剩余支持期不足项目周期。
3. **不要把 RMW 换成 CycloneDDS**：CARLA 原生 ROS 2 目前**只支持 Fast DDS**。

---

### 4.1 双仿真环境（已确定 2026-07-30）

| | **环境 A：本地 Gazebo** | **环境 B：云端 CARLA** |
|---|---|---|
| 位置 | 本机 WSL2 + Docker | Vast.ai GPU 实例 + Docker |
| 定位 | **日常开发主力** | **最终验收 + 高保真验证** |
| 覆盖阶段 | P1-P4 全部，P5-P8 部分 | P5 起的感知验证、各阶段验收 |
| 传感器真实度 | 中（几何正确，无材质/噪声效应） | 高 |
| 成本 | **零** | 按小时计费 |
| 能否进 CI | **能** | 不能 |
| ROS 集成 | `ros_gz`，**Jazzy 官方组合** | 服务端原生 ROS 2 + 自研 sidecar，见下 |

**这个组合的核心收益**：把"改一行代码看效果"的高频动作放在**零成本、零延迟、
无 bridge 版本枷锁**的本地环境；把"证明它在真实传感器下也成立"的低频动作放在云端。

**能这么做的前提**是 SPEC 早就写进架构的一条约束——**仿真数据源必须可插拔**
（本节末尾）。算法节点只认 ROS 话题，不认数据从哪来，所以两套环境切换时
**算法代码一行都不用改**。这就是那条约束的变现。

---

#### 环境 A：本地 Gazebo（日常开发）

| 项 | 选择 | 说明 |
|----|------|------|
| 仿真器 | **Gazebo Harmonic**（LTS 至 2028） | Fortress 2026-09 EOL，不选 |
| ROS 集成 | `ros-jazzy-ros-gz` | **Jazzy + Harmonic 是官方组合**，无版本冲突问题 |
| 渲染 | OGRE，**OpenGL 后端** | 已实测硬件加速可用；不碰 Vulkan |
| GPU 直通 | 容器挂载 `/dev/dri` | WSLg → D3D12 → Radeon 780M |
| 地图 | 与 CARLA 共用 **OpenDRIVE** | 关键：保证两环境地图一致 |
| 车辆模型 | 自行车模型，参数与 CARLA 车辆对齐 | 见下方"一致性"要求 |

**为什么本地也要用 Docker 而不是直接装**：本机是 Ubuntu 26.04（对应 ROS 2 Lyrical），
而算法栈锁定 Ubuntu 24.04 + Jazzy。**本地和云端必须用同一个镜像**，
否则代码要编译两套、行为无法比对，双环境的意义就没了。

---

#### 环境 B：云端 CARLA（最终验收）

**CARLA 与全部算法节点都跑在云 GPU 实例上；本地只是瘦客户端。**

```
┌──────────────────────┐            ┌────────────────────────────────────────┐
│   本地 (WSL2)         │            │        云 GPU 实例 (NVIDIA)             │
│                      │            │  ┌──────────────────────────────────┐  │
│  VS Code Remote ─────┼──SSH:22────┼─▶│ CARLA 0.9.16 --ros2              │  │
│  （只做代码编辑）      │            │  │  -RenderOffScreen                │  │
│                      │            │  │    ▲ Fast DDS（本机环回）          │  │
│                      │            │  │    │ ▲ TCP 2000（PythonAPI）      │  │
│  浏览器 ─────────────┼──noVNC─────┼─▶│    ▼ │                           │  │
│  （看 RViz2 画面）    │  :6080     │  │  Ubuntu 24.04 + ROS 2 Jazzy      │  │
│                      │            │  │   carla_bridge sidecar + ads_*   │  │
│  rosbag 下载 ◀───────┼──scp───────┼──┤   RViz2 / PlotJuggler            │  │
└──────────────────────┘            │  └──────────────────────────────────┘  │
                                    └────────────────────────────────────────┘
```

#### CARLA 侧的接入方式：PythonAPI sidecar 全中继（**已修订** 2026-08-14，见 §9 D5）

> **⚠️ 本节原方案是「原生 `--ros2` + sidecar 双通道」，P8-S5 上机实测后弃用
> 原生通道，全部数据改走 sidecar 中继。** 原文的双通道设想与弃用证据链
> 记录在 §9 D5，此处只写现行方案。

CARLA 0.9.16 的服务端内置 ROS 2（`--ros2`，内嵌 Fast DDS）**实测是半成品中的
半成品**，三项硬伤每一项都单独致命（复现记录见 D5）：

1. **话题名双斜杠**（`/carla//ego/...`）——rclcpp 直接判非法名，**订阅端够不着**；
2. **`/clock` 在世界重建后静默死**——`load_world()` 之后不再发布且无任何报错，
   而生成式世界（`generate_opendrive_world()`）必然要重建世界；
3. **传感器流 segfault**——原生 stream 在我们的传感器组合下崩服务端进程。

**现行方案：sidecar 全中继（单通道）**：

```
CARLA 服务端 ──listen 回调（PythonAPI）──▶ carla_bridge sidecar ──▶ SPEC §4.1 规范话题
             ◀── apply_control ─────────  （翻译 + y 翻转 + /clock 自发）
```

- sidecar 用 `sensor.listen()` 拿原始数据，numpy 做左手系→ENU（y 翻转），
  以规范话题名发布 —— 上游算法节点与 Gazebo 侧**逐字节同一契约**；
- **`/clock` 由 sidecar 自发**（同步模式 + 墙钟节拍线程，RTF=1.000）——
  时钟主权在我们手里，世界怎么重建都不受影响；
- 控制反向通道走 `apply_control()`，含标定映射（throttle=bias+k·a）、
  看门狗与**驻车闩锁**（静止小开度锁不住车，实测蠕动 0.11 m/s）。

**带宽账没有变坏**：点云 16 MB/s 走的是**同机内存**（sidecar 与服务端同实例），
Python 中继单帧实测毫秒级，10 Hz 下绰绰有余。原生通道省的那次拷贝，
在「话题够不着」面前没有意义。

**P0b 清单四项已知问题的实测结论**（2026-08-13/14，RTX 3090）：

1. 车辆控制偶发失效 → **确认存在**，sidecar 每 tick 重发指令兜底；
2. 话题名双斜杠 → **确认，不可绕过**（rclcpp 拒收），弃用原生层的直接原因之一;
3. 坐标系不一致 → **确认**（左手系），sidecar 中继时统一 y 翻转；
4. 内嵌 Fast DDS 互操作 → 能配对，但**发现期长达十几秒**，
   所有探针/发布器必须等 `subscription_count≥1` 再发（盲发全落空）。

#### 为什么不能"CARLA 上云 + 算法留本地"

这是最容易犯的错，算一笔账就清楚：

> 32 线激光雷达 ≈ 10 万点/帧 × 16 字节 × 10 Hz ≈ **16 MB/s ≈ 128 Mbps**

家用宽带带宽勉强够，但**延迟抖动会直接破坏闭环控制**——控制指令回到云端时，
车早已不在你计算时假设的位置上。**跨公网传感器流 + 闭环控制 = 必然失败。**

正确做法：**传感器数据在云端内部环回（localhost），跨公网只传画面和代码。**

#### 平台：Vast.ai（已选定）

**选它的核心理由**：Vast.ai 允许**自选 Docker 镜像并自定义 docker 启动参数**，
所以我们能显式设置 `NVIDIA_DRIVER_CAPABILITIES=all`。

这一点是决定性的。CARLA 是 Unreal Engine 应用，需要 **Vulkan 图形栈**；
而多数算力平台为深度学习优化，容器只暴露 `compute,utility` ——
**只给 CUDA，不给图形渲染**。症状极具迷惑性：`nvidia-smi` 一切正常、
CUDA 跑得飞快，但 CARLA 一启动就崩。NVIDIA 容器工具链的能力项里，
`graphics` 才是 OpenGL/Vulkan 对应的那一个，且它**默认不开启**。

#### 选机器的硬性筛选条件

Vast.ai 是算力市场，机器质量参差极大，**便宜的往往是矿机改装：GPU 多但 CPU 弱**。
而 CARLA 的物理仿真和 Traffic Manager 是**重 CPU** 负载，选错了 GPU 再强也没用。

| 项 | 要求 | 原因 |
|----|------|------|
| GPU | RTX 3080/3090/4090，**显存 ≥ 8 GB** | CARLA 要 6-8 GB；不需要 24 GB，别为显存多花钱 |
| CPU | **≥ 8 物理核** | 物理仿真 + Traffic Manager 吃 CPU，这是最易踩的坑 |
| 内存 | ≥ 32 GB | CARLA + ROS 2 全栈 |
| 磁盘 | **≥ 80 GB** | CARLA ~20 GB + 镜像 + 编译产物 + rosbag |
| 网速 | 下行越高越好 | 首次要拉 20-30 GB |
| 实例类型 | **On-Demand，不要 Interruptible** | 可抢占实例会在开发中途被杀 |
| 主机信誉 | 优先 verified / 高 reliability | 便宜的不可靠主机会浪费更多时间 |

#### 实例创建时必须设置的 Docker 参数

在 Vast.ai 的 **Docker Create/Run Options** 字段填入：

```
-e NVIDIA_DRIVER_CAPABILITIES=all
-e NVIDIA_VISIBLE_DEVICES=all
-p 6080:6080          # noVNC 网页可视化
```

#### P0b 第一件事：先花几块钱验证 Vulkan，再投入

租最便宜的合格机器 1 小时，SSH 进去跑：

```bash
echo $NVIDIA_DRIVER_CAPABILITIES     # 应包含 graphics 或为 all
nvidia-smi                           # 驱动是否正常
vulkaninfo --summary                 # 决定性测试：能否列出 NVIDIA 设备
```

- `vulkaninfo` 列出 NVIDIA 设备 → 通过，继续装 CARLA
- 报 `no devices found` → 换一台机器重试（是主机配置问题，不是平台问题）

**验证通过后再拉 20 GB 的 CARLA。** 顺序反了会浪费下载时间和租金。

#### Vast.ai 特有的计费陷阱

1. **停机（stopped）状态仍然计磁盘费**。不用时要么接受磁盘费，要么彻底销毁（destroy）。
2. **销毁即数据全失**。所以"代码 git 推远端、数据 scp 拉回"
   （见下方"其他工程约束"第 4 条）不是建议，是**每次收尾的强制流程**。
3. 建议**按量付费 + 随手关机**，摸清每天实际用几小时后再考虑长租。

#### 成本控制：用 rosbag 把云端时间砍一半

**不是所有开发都需要实时仿真。** 用 `ros2 bag record -a` 录下 CARLA 传感器数据，
下载到本地：**感知与定位是单向数据流，完全可以在本地用回放数据开发调试**，
只有规划、控制和整车联调才必须开着 CARLA。

> 这不是省钱的歪招，而是行业标准做法：真实车企的感知算法工程师，
> 99% 的时间对着的是路采数据包，不是实车。

#### 其他工程约束

1. **CARLA 以 `--ros2 -RenderOffScreen -quality-level=Low` 启动**：我们不需要它的画面，
   视觉调试全在 RViz2 做，省显存和帧时间。
2. **版本严格匹配**：CARLA 服务端与 PythonAPI wheel 版本必须完全一致，写进 Docker 镜像锁定。
3. **同步模式 + 固定步长**：见 §3.3 与 §12。
4. **实例销毁前必须保存工作**：容器平台的实例是易失的，代码用 git 推远端，
   数据用 scp 拉回本地。**这一条写进每次开发的收尾流程。**

#### 关键架构约束：仿真数据源必须可插拔

`ads_simulation` 定义统一的传感器数据接口，`gazebo_bridge` / `carla_bridge` / `bag_replay`
都只是它的实现。**上游算法节点只认 ROS 话题**，不关心数据从哪来：

```
/lidar/points      sensor_msgs/PointCloud2     base_link 系
/imu               sensor_msgs/Imu             base_link 系
/gnss              sensor_msgs/NavSatFix
/odom              nav_msgs/Odometry           odom → base_link
/ego_pose_gt       nav_msgs/Odometry           真值位姿+速度，map→base_link
                   **仅用于评测打分，算法节点禁止订阅**（P4-S1 实现）
/vehicle_cmd       控制指令（转角 rad + 加速度 m/s²）
```

**切换仿真源 = 换一个 launch 参数，不改任何算法代码。** 这是双环境方案成立的技术前提。

> 真实车企里，同一套算法必须能跑在仿真、路采回放、实车三种数据源上，靠的就是这层抽象。
> 对本项目的直接价值：**云实例断了、余额没了，你依然能在本地继续干活。**

---

#### 双环境的头号风险：行为漂移

两套仿真如果车辆动力学不一致，你会遇到最难查的一类 bug——
**本地调好的控制参数一上 CARLA 就震荡**，而你无法判断是算法错了还是环境不同。

**强制要求**：

1. **车辆参数单一来源**：轴距、最大转角、质量、最大加减速度等写在
   `config/vehicle_params.yaml`，Gazebo 模型和 CARLA 车辆配置**都从这一份读**，
   禁止各写一份。
2. **地图单一来源**：两环境共用同一份 OpenDRIVE 文件。
3. **一致性测试**（P0b 完成后建立）：给两个环境完全相同的开环控制指令序列，
   对比 30 秒后的轨迹。**横向偏差 > 0.5 m 即视为环境不一致，必须先修环境再继续开发。**

> 这条测试看起来是额外工作，但它把"环境差异"从一个随时可能爆炸的隐藏变量，
> 变成一个有量化判据的显式检查项。**在双环境方案里这是必需品，不是可选项。**

---

## 5. 项目结构

```
automated-driving-systems/
├── SPEC.md                     # 本文档
├── CLAUDE.md                   # 给 AI Agent 的项目指引
├── docker/
│   ├── Dockerfile              # Ubuntu 24.04 + Jazzy + Gazebo Harmonic + CARLA PythonAPI
│   ├── docker-compose.local.yml    #   环境 A：挂载 /dev/dxg，本机 Gazebo
│   └── docker-compose.cloud.yml    #   环境 B：NVIDIA runtime，云端 CARLA
├── docs/
│   ├── adr/                    # 架构决策记录
│   └── modules/                # 每模块的算法推导文档
├── config/                     # ★ 手写的**单一来源**，见 §4.1
│   ├── vehicle_params.yaml     #   车辆参数（轴距、限值、传感器外参）
│   └── campus_map.yaml         #   园区地图（路段、路口、车道）
├── scripts/                    # 生成器与验证脚本
│   ├── gen_vehicle_model.py    #   vehicle_params.yaml → SDF + URDF
│   ├── gen_map.py              #   campus_map.yaml → xodr + 路面 SDF + 采样基准
│   └── verify_*.sh             #   各阶段的可复跑量化验收
├── maps/                       # ★ 生成物：两环境共用的 OpenDRIVE 地图
├── models/                     # ★ 生成物：Gazebo 模型（自车、路面）
├── worlds/                     # 手写的 Gazebo 世界（引用 models/）
├── src/
│   ├── ads_msgs/               # 自定义消息接口
│   ├── ads_common/             # 几何、坐标变换、时间、配置工具
│   ├── ads_map/                # OpenDRIVE 解析、车道图、路由
│   ├── ads_localization/       # NDT + ESKF
│   ├── ads_perception/         # 点云处理、检测、跟踪
│   ├── ads_prediction/         # 轨迹预测
│   ├── ads_planning/           # 行为决策 + 运动规划
│   ├── ads_control/            # 横纵向控制
│   ├── ads_simulation/         # 仿真数据源抽象层
│   │   ├── gazebo_bridge/      #   环境 A：Gazebo（本地开发）
│   │   ├── carla_bridge/       #   环境 B：CARLA（云端验收）
│   │   ├── bag_replay/         #   离线回放
│   │   └── scenario_runner/    #   场景描述与自动判定
│   ├── ads_teleop/             # 手动驾驶输入（键盘/手柄）→ /vehicle_cmd
│   ├── ads_bringup/            # launch 文件、参数配置
│   └── ads_visualization/      # RViz 配置、调试可视化
└── test/
    ├── integration/            # 跨模块集成测试
    └── scenarios/              # 场景化测试用例
```

**每个功能包内部统一结构**（以 ads_control 为例）：

```
ads_control/
├── include/ads_control/
│   ├── stanley_controller.hpp      # 纯算法，不含 ROS
│   └── pid_controller.hpp
├── src/
│   ├── stanley_controller.cpp
│   ├── pid_controller.cpp
│   └── control_node.cpp            # ROS 包装层
├── config/
│   └── control_params.yaml
├── test/
│   └── test_stanley.cpp
├── CMakeLists.txt
└── package.xml
```

---

## 6. 常用命令（Commands）

```bash
# ---------- 环境（本地 Gazebo / 云端 CARLA 用同一个 Dockerfile） ----------
export COMPOSE_FILE=docker/docker-compose.local.yml    # 本机开发时
# export COMPOSE_FILE=docker/docker-compose.cloud.yml  # 云端验收时

docker compose build                                   # 构建镜像
docker compose up -d                                   # 启动容器
docker compose exec dev bash                           # 进入开发容器

# ---------- 构建（容器内） ----------
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select ads_control             # 只构建单个包
colcon build --packages-up-to ads_planning             # 构建到某包及其依赖
source install/setup.bash

# ---------- 测试 ----------
colcon test                                            # 全部测试
colcon test --packages-select ads_control              # 单个包
colcon test-result --verbose                           # 查看失败详情
./build/ads_control/test_stanley                       # 直接跑单个测试可执行文件
./build/ads_control/test_stanley --gtest_filter='*ZeroError*'   # 跑单个测试用例

# ---------- 运行 ----------
ros2 launch ads_bringup stack.launch.py sim:=gazebo    # 全栈 + 本地 Gazebo（日常）
ros2 launch ads_bringup stack.launch.py sim:=carla     # 全栈 + 云端 CARLA（验收）
ros2 launch ads_bringup stack.launch.py sim:=bag bag:=logs/xxx   # 离线回放
# 注意：切换仿真源只改 sim 参数，算法节点配置完全不变

# ---------- 场景测试 ----------
# ⚠️ 以下三条是 P8 才建的场景测试体系，**当前不存在**（2026-08-12 对账标注——
#    此前没有任何未实现标记，读者会以为能跑）。当前的闭环验收走
#    scripts/record_*_run.py 一族，见 CLAUDE.md「闭环实测」。
ros2 launch ads_bringup scenario.launch.py sim:=gazebo scenario:=S05   # 【P8】跑单个场景
./scripts/run_all_scenarios.sh gazebo                  # 【P8】L3-G 全场景（CI 用）
./scripts/run_all_scenarios.sh carla                   # 【P8】L3-C 全场景（验收用）
./scripts/check_sim_consistency.sh                     # 两环境一致性测试

# ---------- 调试 ----------
ros2 topic list / ros2 topic hz /perception/obstacles / ros2 topic echo /localization/pose
ros2 node info /planning_node
ros2 run rqt_graph rqt_graph                           # 看节点连接图
rviz2 -d src/ads_visualization/rviz/default.rviz
ros2 bag record -a -o logs/run_$(date +%s)             # 录包
```

> **给初学者**：`colcon test --packages-select X` 和直接跑 gtest 可执行文件的区别是，
> 前者走完整 CTest 流程（含 lint），后者快 10 倍且能用 `--gtest_filter` 精确定位。
> 日常改代码用后者，提交前用前者。

---

## 7. 代码规范（Code Style）

遵循 **ROS 2 官方风格指南**（基于 Google C++ Style），关键约定：

| 项 | 约定 | 示例 |
|----|------|------|
| 文件名 | snake_case | `stanley_controller.cpp` |
| 类名 | PascalCase | `class StanleyController` |
| 函数/变量 | snake_case | `compute_steering()` |
| 成员变量 | 后缀下划线 | `double wheelbase_;` |
| 常量 | kPascalCase | `constexpr double kGravity = 9.81;` |
| 命名空间 | 与包名一致 | `namespace ads_control { ... }` |
| 缩进 | 2 空格，行宽 100 | |

**强制要求：**

1. **物理量必须带单位后缀**：`speed_mps`、`angle_rad`、`dist_m`
   → 单位混淆是自动驾驶最高频的 bug 来源，火星气候探测者号就是这么丢的
2. **不允许裸 `new`/`delete`**，用 `std::unique_ptr`/`shared_ptr`
3. **不允许在回调中做重计算**，超过 10 ms 的工作放到独立线程/定时器
   > 注解（P9-S5a，2026-08-16 拍板）：以**目标硬件**上的实测为准。感知流水线整帧跑在
   > 点云回调里，云机（15 核）p50/p95 = 7.5/8.2 ms 在线内；WSL2 开发笔记本 p95 24 ms
   > 超线但那不是目标硬件（10 Hz 周期 100 ms 下功能无碍）。暂不改线程模型 —— worker
   > 线程会给刚稳定的模块引入竞态面；目标机哪天超 10 ms 再做（worker + 最新帧丢旧）。
   > 每拍耗时的仪器：`scripts/p9_timing_probe.py`（四模块 diagnostics，两环境同一把尺子）。
4. **所有角度统一用弧度**，输出时才转角度
5. **禁止 `using namespace std;`**
6. 公开接口必须有 Doxygen 注释，说明**单位、坐标系、有效范围**

```cpp
/**
 * @brief 计算 Stanley 横向控制的前轮转角
 * @param cross_track_error_m  前轴中心到参考路径的横向误差 [m]，左正右负
 * @param heading_error_rad    车辆航向与路径切向夹角 [rad]，范围 [-pi, pi]
 * @param speed_mps            车辆纵向速度 [m/s]，必须 >= 0
 * @return 前轮转角 [rad]，已限幅至 [-max_steer, +max_steer]
 */
double compute_steering(double cross_track_error_m,
                        double heading_error_rad,
                        double speed_mps) const;
```

工具链：`clang-format`（配置随仓库）、`ament_lint` 在 `colcon test` 中自动执行。

---

## 8. 测试策略（Testing Strategy）

采用**四层测试金字塔**。越往下越快越多，越往上越慢越少。

### L1 单元测试（GoogleTest，无 ROS，毫秒级）

测试纯算法函数。这是为什么架构要求算法与 ROS 解耦。

- 坐标变换：笛卡尔 ↔ Frenet 往返一致性
- 控制器：零误差输入 → 零输出；误差符号 → 转向符号正确
- 滤波器：EKF 在恒定观测下收敛
- 几何：多边形碰撞检测的边界情况（相切、包含、退化）

**覆盖率目标：算法库（`lib/`）行覆盖 > 80%**
（⚠️ 2026-08-12 对账标注：这是**目标值，当前没有任何机制在量它** ——
仓库里无 gcov/lcov 配置，CI 无覆盖率步骤。按本项目自己的纪律，
一条没人量的判据比没有判据更危险，所以在此明示；机制建于 P8。）

### L2 模块测试（launch_testing，秒级）

单个 ROS 节点，喂入构造消息，断言输出话题。

- 感知节点：输入合成点云（已知位置的立方体）→ 输出检测框位置误差 < 0.2 m
- 规划节点：输入直道 + 前方静止障碍 → 输出轨迹绕行或减速

### L3 场景测试（闭环，分钟级）—— 分两套跑

同一份场景定义、同一套判据，在两个环境各跑一遍：

| | **L3-G（Gazebo）** | **L3-C（CARLA）** |
|---|---|---|
| 频率 | **每次合并自动跑（CI）** | 阶段验收时手动跑 |
| 成本 | 零 | 云端计费 |
| 作用 | 抓回归 | 证明真实传感器下也成立 |

**判据完全相同**（见下表）。L3-G 过了 L3-C 没过 → 问题出在传感器真实度上
（噪声、遮挡、反射率），这本身就是有价值的诊断信息。

在仿真中跑完整栈，每个场景一个测试用例，**自动判定通过/失败**。

必测场景清单（第一版）：

| ID | 场景 | 通过判据 |
|----|------|---------|
| S01 | 直道巡航 | 横向偏差 < 0.3 m，速度误差 < 1 km/h |
| S02 | 路口右转 | 不压线，横向加速度 < 2 m/s² |
| S03 | 前车减速跟停 | 保持 TTC > 2 s，不碰撞 |
| S04 | 静止障碍物绕行 | 侧向间距 > 0.5 m |
| S05 | 行人横穿 | 完全停止，最近距离 > 1 m |
| S06 | 红绿灯 | 红灯停在停止线前 0-2 m。⚠️ **仅 L3-C（P8）**：环境 A 无灯态激励源（地图/世界/规范话题皆无灯，灯态数据源设计在 CARLA sidecar —— §4.1），P7 拍板将其整体挂 L3-C；「L3-G 与 L3-C 判据完全相同」对本行例外（2026-08-13，与 P6 静态先验注解同一形式） |
| S07 | 全局导航 A→B | 到达终点，零接管，零碰撞 |

### L4 回归测试（CI）

每次提交跑 L1+L2；每次合并跑 **L3-G** 全场景；阶段验收跑 **L3-C**。
记录关键指标随时间的变化曲线。

**异常注入清单**（P9-S5b 起）见 [docs/fault_injection.md](docs/fault_injection.md)：
故障 × 期望行为 × 守卫（哪个测试）× 缺口，逐行可验红验绿；「异常处理完备」以它为准。

> **本地 Gazebo 能进 CI，是双环境方案最被低估的收益。** CARLA 永远进不了 CI
> （要 GPU、要 20 GB 镜像、跑不确定），意味着如果只有 CARLA，你的 L3 层
> 就只能靠人工手动跑——而人工跑的测试，三个月后一定没人跑。

> **给初学者的核心提醒**：自动驾驶的测试和普通软件最大的不同是——
> **"没崩溃"不等于"对了"**。你必须为每个场景定义**可量化的通过判据**，
> 否则你永远在用眼睛看 RViz 猜"好像还行"。这是业余和专业的分水岭。

---

## 9. 决策记录与待确认事项

### ✅ D1 CARLA 的部署方式（已决定 2026-07-30）

**CARLA 环境全栈上云**：CARLA 与算法节点都跑在云 GPU 实例上，
本地仅作瘦客户端（VS Code Remote 编辑 + noVNC 看 RViz + scp 拉 rosbag）。拓扑见 §4.1 环境 B。

**Why**：**CARLA** 本地跑不了 —— WSL2 无 NVIDIA GPU，Windows 宿主是 AMD Radeon 780M 核显，
而 CARLA 官方只真正支持 NVIDIA。
（注意本条只约束 CARLA；Gazebo 本地跑得动，见 D4。）

**平台选定：Vast.ai**。理由是它允许自选镜像与 docker 启动参数，
可显式设置 `NVIDIA_DRIVER_CAPABILITIES=all`，这是跑通 CARLA 的必要条件。
备选：TensorDock（KVM 全虚拟机，有 root）、RunPod（贵但稳）、
阿里云/腾讯云抢占式（境外延迟影响 VNC 交互时用）。

**演进过的方案**（记录以免重复讨论）：
~~CARLA 跑 Windows 宿主~~ → 显卡查明为 AMD 核显，作废。
~~自研轻量仿真器替代 CARLA~~ → 作废，改为用成熟的 Gazebo，见 D4。
~~AutoDL / 恒源云~~ → 改用 Vast.ai。
~~AWSIM~~ → 绑定 Autoware 生态，不选。

### ✅ D2 协作方式（已决定 2026-07-30）

**由 AI 编写代码并逐行讲解原理，开发者读懂后自行改进与调参。**

对交付物的约束：

- 每个模块交付时同步产出 `docs/modules/<模块>.md`，讲清**数学推导、参数物理含义、推荐取值范围**
- 代码注释密度高于常规工程项目，重点解释**"为什么这么做"**而非"这行做了什么"
- 每个关键参数必须说明：调大会怎样、调小会怎样、什么现象说明该调它
- 每阶段结束提供一组"动手实验"：改哪个参数、预期看到什么变化

### ✅ D3 ROS 与 CARLA 版本（**已修订** 2026-07-30）

**ROS 2 Jazzy + Ubuntu 24.04 + Gazebo Harmonic + CARLA 0.9.16。**
不用 Humble，不用 Lyrical，不用 CARLA 0.10.x。

#### 修订记录：本条曾定为 Humble，已推翻

| | 旧结论（同日早些时候） | **新结论** |
|---|---|---|
| ROS | Humble | **Jazzy** |
| Ubuntu | 22.04 | **24.04** |
| CARLA | 0.9.15 | **0.9.16** |
| 约束方向 | 从 CARLA 往外推 | **从 Gazebo 往外推** |

**推翻的直接原因**：旧结论建立在「CARLA PythonAPI 只保证 Python 3.7/3.8，
因此必须用 Ubuntu 22.04」这个事实上。但该事实**只对 0.9.15 成立**——
CARLA **0.9.16**（2025-09 发布）官方提供 **Python 3.10 / 3.11 / 3.12 wheel**，
并且服务端**内置原生 ROS 2**，不再依赖 ros-bridge。
Ubuntu 版本的枷锁一解开，约束链的瓶颈就从 CARLA 转移到了 Gazebo。

#### 选 Jazzy 的理由（按重要性）

1. **Jazzy + Gazebo Harmonic 是官方组合。** 路线图 P1-P4、P6-P7 全在 Gazebo 上，
   只有 P5 和验收碰 CARLA。**官方支持应该放在你天天用的那一侧。**
   Humble 配 Harmonic 是非官方组合（Humble 官方搭档 Fortress 已于 2026-09 EOL）。
2. **支持期**：Jazzy 至 2029-05；Humble 只到 2027-05，剩余不足项目周期。
3. **CARLA 侧的差距被架构吸收**：原生 ROS 2 缺 TF 树与 ego 状态，
   但我们本来就要写 `ads_simulation` 适配层，见 §4.1。

#### 承认的代价

**CARLA + Jazzy 未经官方点名验证**（官方表述为「Foxy、Galactic、Humble 及更多」）。
这是本决策唯一的真实风险。

- **影响范围有限**：只影响 P0b 与验收，**不影响 P0a → P4 的任何工作**
- **退路**：CARLA 服务端是独立进程。万一 Jazzy 侧接不通，可单独用一个 Humble
  容器只跑 `carla_bridge`，算法栈保持 Jazzy（标准消息类型跨发行版 DDS 互通）

#### 不选 CARLA 0.10.x

理由是**生态不成熟**（非硬件原因——上云后 RTX 4090 能跑 UE5）：
0.10.x 的 ROS 集成与社区资料远不如 0.9.16，对初学者意味着遇到问题搜不到答案。

> **P0b 落地前仍需联网核实**：CARLA 0.9.16 下载地址、`--ros2` 在 Jazzy 下的
> 实际互通性、Python 3.12 wheel 安装方式。**不凭记忆下结论。**

### ✅ D4 双仿真环境（已决定 2026-07-30）

**日常开发用本机 Gazebo，最终验收用云端 CARLA。** 详见 §4.1。

**Why**：实测 WSL2 下 OpenGL 4.6 硬件加速可用（D3D12 → Radeon 780M），
Gazebo 的 OGRE 渲染只需 3.3+，因此本地完全跑得动 Gazebo。
收益是 P1-P4 零云端成本、零网络延迟、**且 L3 场景测试能进 CI**——
CARLA 永远做不到最后这点。

**代价与对策**：引入双环境行为漂移风险 → 车辆参数与地图强制单一来源，
并建立量化的一致性测试（§4.1 末尾）。

### ✅ D5 CARLA 原生 `--ros2` 层弃用，改 sidecar 全中继（已决定 2026-08-14）

**弃用 CARLA 0.9.16 的原生 ROS 2 通道，全部数据（含传感器流）走 PythonAPI
sidecar 中继。** 原方案（§4.1 旧版）是「高频走原生、低频走 sidecar」双通道。

**Why —— 三项上机实测（P8-S5，RTX 3090），每项单独致命**：

1. **话题名双斜杠**：原生层发布 `/carla//<role>/...`（role_name 为空段），
   rclcpp 按 ROS 命名规范判非法，**订阅端连节点图里都看不见它**。
   服务端无参数可改写话题名 —— 不是「难用」，是「够不着」。
2. **`/clock` 静默死**：`load_world()` / `generate_opendrive_world()` 重建世界后
   原生 `/clock` 停止发布且无任何报错。本项目**必然**用生成式世界
   （两环境共用同一份 .xodr 是 §4.1 的硬要求），所以这条必踩。
   `use_sim_time=true` 下全栈的时序都挂在 /clock 上 —— 它死 = 一切冻结，
   且症状是「什么都不动、什么都不报」（陷阱表「仿真钟停走」同款）。
3. **传感器 stream segfault**：启用原生流后服务端在我们的传感器组合下
   直接段错误崩溃（复现 3/3）。

**代价评估**：sidecar 中继多一次进程间拷贝，但同机内存带宽下点云 16 MB/s
单帧毫秒级，10 Hz 无压力 —— 原生通道的性能优势在「话题够不着」面前无意义。

**收益**：时钟主权（sidecar 自发 /clock，墙钟节拍，RTF=1.000）、
话题命名完全归我们（与 Gazebo 侧逐字节同一契约）、坐标系翻转集中一处。

**反向条件**（哪天可以重新评估原生层）：上述 1、2 在 CARLA 上游修复
且有 release note 背书时。届时也只值得迁传感器流，/clock 主权不还。

### ✅ D6 CARLA 侧 S01 跟踪差异不根治，维持入表放行（已决定 2026-08-16）

S01 的四条跟踪类判据在 CARLA 上稳定红（横向 max 0.6–0.84 / rms 0.13–0.17 /
Δv 0.42 / a_lat 3.0–3.3 vs 0.3 / 0.1 / 0.2 / 2.0），安全类判据全绿。P9-S5d 评估
（云窗口 6）量清了机理 —— PhysX 发动机零油门阻尼 2.0 vs 全油门 0.15 + 扭矩曲线峰
在 1 挡 4.8 m/s，使 4–5 m/s 速段增益比 2–3 m/s 高 78%、油门偏置随速度 0.48–0.55；
拉平传动系（同 steering_curve 拉平的法理）后静态图变直，但闭环首轮只赢 Δv、
蠕动死区让车停在目标前 3.96 m —— 「静态准≠闭环好」第二次应验。**用户拍板：不根治**，
维持 P8 的入表放行（`docs/p8_carla_bringup.md` §6 #8），机制留档
（sidecar `ego.align_drivetrain` 默认关 + `scripts/carla_throttle_map_probe.py`）。
前因后果、被否决的方案与反向条件见 **`docs/adr/0002-carla-s01-tracking-gap-not-fixed.md`**。

### ✅ Q1 本地显卡（已查明，问题关闭）

Windows 宿主为 **AMD Radeon 780M**（RDNA3 核显，UMA 共享内存，无 CUDA）。
本地无任何可用于 CARLA 的 GPU —— 这是全栈上云的根本原因。

### 当前无阻塞性未决问题

所有 P0a 之前必须回答的问题均已关闭。剩余不确定项都属于**实施中验证**，不阻塞开工：

| 待验证项 | 验证时机 | 若失败 |
|---|---|---|
| 容器内能否拿到 OpenGL 硬件加速 | P0a S1 | **环境 A 方案重议**（唯一的 go/no-go） |
| Gazebo 实时率 RTF | P0a S2 | 简化世界几何 |
| ~~CARLA `--ros2` 与 Jazzy 互通性~~ | ~~P0b~~ | **已了结（2026-08-14）**：原生层整体弃用，见 D5 |
| ~~CARLA 原生 ROS 2 的三个已知 bug~~ | ~~P0b~~ | **已了结**：三 bug 全部实测确认，走了「若失败」分支（sidecar），见 D5 |

---

## 10. 分阶段路线图

**核心原则：每个阶段结束都必须有一个"能跑起来看得见"的东西。**
绝不允许连续两周只写代码不出效果——那是初学者最容易放弃的方式。

| 阶段 | 环境 | 内容 | 阶段性成果（可演示） |
|------|:---:|------|---------------------|
| **P0a** | A | 建 Docker 镜像（24.04 + Jazzy）→ 装 Gazebo Harmonic → GPU 直通 → 车辆模型 + 最小世界 | 本机 RViz2 里看到车和合成点云，键盘能开动 |
| **P0b** | B | Vast.ai 选机 + Vulkan 验证 → 装 CARLA 0.9.16 → 验证 `--ros2` 与 Jazzy 互通 → 写 PythonAPI sidecar → SSH/noVNC → **建立一致性测试** | 云端 RViz2 看到 CARLA 点云；两环境轨迹偏差 < 0.5 m |
| **P1** | A | `ads_common` + `ads_map`：坐标变换、OpenDRIVE 解析、路由 | RViz 中显示车道图和 A→B 全局路径 |
| **P2** ✅ | A | `ads_control`：Stanley + 速度剖面 + 速度环 PI（**已完成** 2026-08-03） | 车辆沿预设路径自动行驶：**560 m 含路口弯的路线，最大横向误差 0.080 m、终点停车误差 0.327 m**（CP-P2-B 8/8） |
| **P3** ✅ | A | `ads_planning` 运动规划：Frenet 采样（**已完成** 2026-08-04） | 遇静态障碍物自动绕行**或停住**：贴边锥桶侧向间距 **0.532 m**（判据 > 0.5）、车道中心锥桶几何无解时停住且报不可行、无障碍物回归 CP-P2-B **8/8**（CP-P3-B 3 场景） |
| **P4** ✅ | A | `ads_localization`：ESKF + NDT（**已完成** 2026-08-10） | 关闭真值定位，用传感器自主定位行驶：**横向误差 0.129 / 0.087 m**（判据 0.30）、航向 0.46° / 0.68°（判据 2.0）、NDT 单帧 37.5 / 44.3 ms（判据 100），且估计位姿下 CP-P2-B 仍 **8/8**（CP-P4-B 连跑两轮 7/7；2026-08-12 复检修复失锁恢复死代码后**六轮全过**，横向 0.094–0.111 m） |
| **P5** ✅ | A→B | `ads_perception`：点云检测 + 跟踪（**Gazebo 半已完成** 2026-08-12） | 关闭真值障碍物，规划器吃真感知仍绕/停达标（CP-P5-B 九组 + 复检后扩窗协议：检测率 100%、近边误差 0.115–0.131 m（判据 0.5）、遮挡后 ID 保持实测通过、单帧 5 ms）。⚠️ CARLA 那一半随 P0b 并入 P8；车道内幻影已于 P6-S0 修复（机理见 perception.md §6.1，与「欠分割」的旧假设不同） |
| **P6** ✅ | A | `ads_prediction`：轨迹预测（**已完成** 2026-08-12） | RViz 显示他车 3 s 预测轨迹（线 + 2σ 椭圆）；CP-P6-B 双层验收：真值层 10/10、感知层连续两轮全过 —— 弯道车道跟随 FDE@3s p95 **0.39–0.49 m**（判据 2.0）、与恒速基线的区分力对照 **13.6–16.5×**（判据 ≥2）、单帧 p95 1.4–3.3 ms |
| **P7** ✅ | A | `ads_planning` 行为决策：行为树（**已完成** 2026-08-13） | 跟车、让行、无信号路口通行（S06 红绿灯挂 P8/L3-C，见 §8 注解）。CP-P7-A 7/7；CP-P7-B 双层验收：真值层三场景两轮全过 —— S03 TTC 2.58–3.27 s（判据 2）/跟停稳态 **4.93–5.90 m**（判据 [4,10]，解析 5.5）/驶离后恢复 1.12–1.28 s（判据 3）；S05 停止时真值距 4.11–4.34 m（判据 3）/全程最近 1.62–1.71 m（判据 1）；路口对车流真值最小间距 4.46–7.26 m（判据 1.5）；感知层安全类判据四轮全过、回归零劣化（CP-P2-B 8/8、CP-P3-B、CP-P6-B 抽轮 9/9）。机制：冲突窗 → 停车点/逐点限速注入现有速度剖面；自研微型行为树只选标签、约束树外取 min |
| **P8** ✅ | A+B | 场景测试体系 + CI + CARLA 半区（**已完成** 2026-08-14） | CP-P8-A：`run_all_scenarios.sh` 一键全场景连续两轮 9/9、metrics 98 行全 PASS、CI 九守卫。CP-P8-B（云端 CARLA，RTX 3090/4090 两窗口）：S04 avoid **9/9**（margin 0.7 + **两级准入**：floor=SPEC 0.5 保延续候选，单级准入在执行误差>网格富余时数学死锁 —— 实测 27/27 全灭）、block 4/4 ×3、行为三场 truth 层全绿（follow 5/5 跟停 4.87、crossing 5/5 恢复 1.15 s、junction 4/4 让行-通过）、S06 红绿灯 2/2（停线前 1.5–1.8 m、绿灯 2.5 s 恢复）；接入方式修订为 sidecar 全中继（原生 --ros2 弃用，§9 D5）；**两环境一致性差异表十行**（p8_carla_bringup §6）：轮胎侧偏/油门非线性/路外语义等已对齐或入表。S01 四条跟踪判据以实测值入表放行（弯道执行超调，安全类判据全域全绿）。P5-CARLA 复测：检测率 0-6%、3437 帧虚警 —— 地面分割域绑定 Gazebo，**感知域移植挂 P9** |
| **P9** 🔶 | A+B | 感知域移植（P5-CARLA）+ 性能优化与鲁棒性（**感知域移植 S1–S4 已完成** 2026-08-16；S5 性能/鲁棒待拆） | CP-P9-A：CP-P5-B 判据表**原值原样**在 CARLA 连续两轮 9/9（检测率车/行人五档 100%、近边 p95 0.24–0.39、横向 0.15–0.18、速度 0.28–0.30、ID 切换 0、虚警 0）；CP-P9-B：行为三场景真值层 + 感知层 6/6（follow 跟停 5.8–6.2 m / crossing 停距 4.7–5.9、最近 1.66 / junction 让行-通过、对车流 4.0–5.5 m）。**感知算法只动了一处**（聚类竖向容差 1.0，Gazebo 全表 9/9 ×2 零劣化）；其余全在传感器/真值/道具链路：sidecar raw_data 内存别名（每帧只有半个世界）、y 反号、actor 原点≠后轴（点云前移 1.41 m）、行人埋地、护栏墙 L 形 OBB、雷达挂高 1.6→2.2（c3 车顶 1.571，自反射 60%→0）、道具几何贴 ODD（micra→leon）、车队天上停车场。教训与仪器：perception.md §8.3、bringup §6 #11–14、`p9_lidar_probe.py` |

> **环境列**：A = 本地 Gazebo，B = 云端 CARLA。
> **P0b 可以推迟**——P1-P4 完全不需要 CARLA。但建议早做，因为一致性测试建立得越晚，
> 积累的行为漂移越难排查。
> **P5 是唯一必须两边都做的阶段**：先在 Gazebo 的干净点云上把聚类和跟踪逻辑写对，
> 再上 CARLA 面对真实噪声、遮挡和地面起伏。**顺序反了会让你分不清是算法错还是数据脏。**

> **为什么控制排在感知前面？**
> 反直觉，但这是正确的工程顺序。先用仿真器提供的**真值**（ground truth）定位和障碍物，
> 打通"规划→控制→车辆动起来"的闭环。有了闭环，后面每个模块都能立刻看到效果。
> 如果先做感知，你会对着一堆点云调三个月，车一步都没动过。
> 真实项目里这叫 **"先打通主干，再替换支路"**。

---

## 11. 边界（Boundaries）

### ✅ 始终执行（Always）

- 新增算法前先在 `docs/modules/` 写清数学推导和参数含义
- 所有物理量标注单位与坐标系
- 每个模块交付时必须附带对应的 L1 单元测试
- 参数改动走 YAML，不改代码
- 每个阶段结束提交一次可运行的完整版本
- 涉及安全逻辑（碰撞检测、急停）的代码必须有对应测试用例

### ⚠️ 先问后做（Ask First）

- 引入新的第三方依赖（尤其是重型库）
- 修改已定义的消息接口（`ads_msgs`）—— 会影响所有下游模块
- 改变模块划分或数据流
- 引入深度学习模型（涉及训练数据、部署方式、算力）
- 任何超过 500 行的单次改动

### ❌ 禁止（Never）

- 跳过 ODD 直接写算法（"这个场景应该也能用吧"是事故起点）
- 在代码里硬编码魔数（`if (dist < 5.0)` 这个 5.0 必须来自配置且有注释说明依据）
- 跨模块直接调用函数，绕过 ROS 接口
- 手写坐标变换矩阵而不用 TF2
- 在没有量化判据的情况下声称某功能"做完了"
- 为了让测试通过而放宽判据
- 把安全相关逻辑（碰撞检查）放在可被配置关闭的分支里

---

## 12. 关键风险登记

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| **两环境行为漂移** | **最难查的一类 bug**：本地调好的参数上 CARLA 就震荡 | 车辆参数与地图强制单一来源；建立量化一致性测试，偏差 > 0.5 m 即停工修环境（§4.1） |
| **CARLA + Jazzy 未经官方验证** | **阻塞 P0b**（不阻塞 P0a-P4） | 官方只点名 Foxy/Galactic/Humble。P0b 实测；退路是单开 Humble 容器只跑 `carla_bridge` |
| CARLA 原生 ROS 2 功能不全 | 缺 TF 树 / ego 状态 / 红绿灯 / 地图 | 已纳入设计：走 PythonAPI sidecar 补齐（§4.1 双通道方案） |
| CARLA 原生 ROS 2 已知 bug | 控制失效、话题名双斜杠、坐标系不一致 | P0b 逐条实测；问题清单见 §4.1 |
| 误用 CycloneDDS | CARLA 原生 ROS 2 连不上 | 锁定 Jazzy 默认 `rmw_fastrtps_cpp`，写进 Dockerfile 环境变量 |
| WSL2 OpenGL 经 D3D12 转译，性能受限 | Gazebo 帧率不足 | 已实测硬件加速可用；场景保持几何简单；不启用高级光照 |
| Vast.ai 机器不支持 Vulkan | **阻塞 P0b**（不阻塞 P1-P4） | 建实例时设 `NVIDIA_DRIVER_CAPABILITIES=all`；先验证再拉 CARLA；不行就换机器 |
| 选到 CPU 弱的矿机 | CARLA 帧率低，且难以定位 | 按 §4.1 硬性条件筛选：≥8 物理核、≥32 GB 内存 |
| 实例销毁导致代码/数据丢失 | 工作丢失 | 代码 git 推远端、数据 scp 拉回；**强制收尾流程** |
| 停机状态仍计磁盘费 | 隐性超支 | 长期不用则彻底 destroy；用前确认账单 |
| 云 GPU 计费累积超支 | 项目中断 | rosbag 本地回放开发；按量付费；随手关机 |
| 网络不稳导致远程开发中断 | 效率损失 | tmux 保持会话；代码本地也有 git 副本 |
| 擅自改动 ROS / Gazebo 版本 | 官方组合被破坏 | 版本约束链见 §4；三条禁止项已明列 |
| ~~CARLA PythonAPI 与 Python 3.10 不兼容~~ | **已排除** | 0.9.16 官方提供 3.10/3.11/3.12 wheel |
| ~~用已停维护的 ros-bridge~~ | **已规避** | 官方 ros-bridge 停在 0.9.13，改用服务端原生 ROS 2 |
| CARLA 服务端/客户端版本不匹配 | 连接失败或行为异常 | 版本号写入 Docker 镜像并锁定 |
| 仿真时间与 ROS 时间不同步 | 算法时序错乱、控制震荡 | 全局 `use_sim_time=true`，CARLA 开同步模式（synchronous mode）+ 固定步长 |
| 范围蔓延，做不完 | 项目烂尾 | 严守 §1.2 非目标 + 阶段性可演示成果 |
| 初学者卡在调参 | 挫败感 | 每个参数在文档中给出物理含义和推荐范围 |
