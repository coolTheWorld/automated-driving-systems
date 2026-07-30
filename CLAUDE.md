# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 事实来源与工作流

**[SPEC.md](SPEC.md) 是唯一事实来源。** 本文件只做导航和补充实测细节，两者冲突时以 SPEC 为准。
SPEC 的关键小节：§2 ODD、§3.3 设计约束、§4.1 双仿真环境、§7 代码规范、§8 测试策略、§11 边界。

任务拆解在 `tasks/`：`plan.md` 说明**怎么拆、为什么这么拆**，`todo.md` 是可执行清单和**当前进度**。
**开工前先读 `tasks/todo.md` 顶部的「🔖 下次从这里继续」** —— 它记录当前位置、恢复命令和已知陷阱。

计划按**垂直切片**组织（每个切片结束时系统多具备一项可亲眼验证的能力），切片之间有**检查点**。
**检查点处停下来汇报实测数据，不自行决定继续。**

### 修错之后必须同步长期记忆

**只要一次修复推翻了某个事实性判断，就要回头检查长期记忆里有没有同样或相关的错误说法，
有则一并改掉或删掉。** 记忆文件在 Claude Code 的项目记忆目录
（`~/.claude/projects/<项目路径 slug>/memory/`，索引是 `MEMORY.md`）。

**为什么这条值得单列**：长期记忆每个会话都会被加载进上下文，一条过时或错误的记忆
不是「没帮上忙」，而是**会主动把下一个会话带偏** —— 比没有记忆更糟。代码里的错误
下次跑测试就会暴露，记忆里的错误不会，它只会让人以一个错误的前提开始排查。

**本项目已经发生过一次**：记忆里原本写着「`/dev/dri/card0`+`renderD128` 存在，
所以容器能跑 Gazebo」。那是**宿主视角**的观察，容器里根本不成立 —— 真正的通路是
`/dev/dxg`，还得配 `GALLIUM_DRIVER=d3d12`。修好 compose 之后同步改了记忆；
如果没改，下一个会话会带着「GPU 直通靠的是 `/dev/dri`」这个错误前提去排查新问题。

检查范围不止记忆：`SPEC.md`、`tasks/plan.md`、`tasks/todo.md` 和代码注释里若有同一处
错误说法，一并修。**同一个错误在几个地方各躺一份，比只错一处更难清理。**

用中文交流。用户是自动驾驶初学者，要求写代码 + 逐行讲原理，注释密度高于常规工程项目，
重点解释「为什么这么做」而非「这行做了什么」，关键参数要说明调大/调小的后果。

---

## 环境

本地与云端**共用同一个 Dockerfile**——这是双环境方案成立的前提，改动时不要分叉。

```bash
export COMPOSE_FILE=docker/docker-compose.local.yml    # 本机 Gazebo（日常）
# export COMPOSE_FILE=docker/docker-compose.cloud.yml  # 云端 CARLA（P0b 起，骨架）

./scripts/setup_env.sh          # 生成 .env（宿主 UID/GID + GPU 设备组 GID），换机器必跑
docker compose build
docker compose up -d
docker compose exec dev bash
```

### 环境自检（改动 docker/ 或换机器后先跑这两个）

```bash
docker compose exec dev /workspace/scripts/verify_gpu.sh   # 10 项：GPU 硬件加速
docker compose exec dev /workspace/scripts/verify_sim.sh   # 6 项：Gazebo 仿真基线 + RTF
```

退出码 0 才继续。`verify_gpu.sh` 是整个本地环境方案的 go/no-go —— **失败时不要用软件渲染硬撑**。

### 已实测的环境陷阱

| 陷阱 | 症状 | 处理 |
|---|---|---|
| 漏映射 `/dev/dxg` | 静默退化 llvmpipe，**不报错** | compose `devices` 必须有 `/dev/dxg`；`/dev/dri` 只是 WSL 的兼容外观 |
| 未设 `GALLIUM_DRIVER=d3d12` | 同上 | 容器 Ubuntu 24.04 的 Mesa 不自动探测 d3d12（宿主 26.04 会）；Jazzy 绑死 24.04 升不了 Mesa |
| 脚本里直接调 `gz` | command not found | Gazebo 装在 ROS 前缀下（`ros-jazzy-gz-sim-vendor`），需先 `source /opt/ros/jazzy/setup.bash` |
| `set -u` 下 source ROS | `AMENT_TRACE_SETUP_FILES: unbound variable` | source 期间临时 `set +u` |
| `docker exec bash -lc` | `~/.bashrc` 不生效，ROS 环境没加载 | 用 `bash -c` 并显式 source |

`screen 0 does not appear to be DRI3 capable` 是**干扰项**，宿主也报，与硬件加速无关。

---

## 常用命令

`src/` 下的 ROS 包**目前尚未创建**（S3 才开始建）。以下构建/运行命令是规划中的形态，
在包存在之前不可用 —— 不要假设它们能跑。

```bash
# ---------- 构建（容器内） ----------
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select ads_control          # 单个包
colcon build --packages-up-to ads_planning          # 到某包及其依赖

# ---------- 测试 ----------
colcon test --packages-select ads_control           # 走完整 CTest（含 lint），提交前用
./build/ads_control/test_stanley                    # 直接跑 gtest，快 10 倍，日常改代码用
./build/ads_control/test_stanley --gtest_filter='*ZeroError*'   # 单个用例
colcon test-result --verbose                        # 失败详情

# ---------- 运行 ----------
ros2 launch ads_bringup stack.launch.py sim:=gazebo # 切换仿真源只改 sim 参数
```

### 已经可用的

```bash
python3 scripts/gen_vehicle_model.py            # 从 YAML 重新生成车辆 SDF
python3 scripts/gen_vehicle_model.py --check    # 校验生成物是否与 YAML 同步（CI 用）

# 容器内，需先 source /opt/ros/jazzy/setup.bash
gz sim -r /workspace/worlds/campus_minimal.sdf                    # 带 GUI
gz sim -s -r /workspace/worlds/campus_minimal.sdf                 # headless
gz topic -t /model/ego_vehicle/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 3.0}'
gz topic -e -t /world/campus_minimal/stats -n 12                  # 读 RTF
```

---

## 架构要点

以下几条需要读多个文件才能拼出全貌，是这个仓库最容易踩错的地方。

### 1. 仿真数据源必须可插拔（最重要）

`ads_simulation` 定义统一接口，`gazebo_bridge` / `carla_bridge` / `bag_replay` 都只是它的实现。
**上游算法节点只认 ROS 话题，不关心数据从哪来。** 规范话题名（SPEC §4.1）：

```
/lidar/points      sensor_msgs/PointCloud2     base_link 系
/imu               sensor_msgs/Imu             base_link 系
/gnss              sensor_msgs/NavSatFix
/odom              nav_msgs/Odometry           odom → base_link
/ego_pose_gt       真值位姿，仅早期阶段和评测打分用
/vehicle_cmd       控制指令（转角 rad + 加速度 m/s²）
```

**bridge 层的职责就是把仿真器原生话题翻译成上面这组名字。** Gazebo 侧和 CARLA 侧做完全相同的翻译。
翻译层做对了，上游算法就永远不需要知道数据来自哪个仿真器 —— 云实例断了也能在本地继续干活。

### 2. 版本约束链的方向是从 Gazebo 往外推

```
Gazebo Harmonic ──官方组合──▶ ROS 2 Jazzy ──▶ Ubuntu 24.04 (Python 3.12)
```

不是从 CARLA 推。理由：路线图 P1-P4、P6-P7 全在 Gazebo 上，官方支持要放在天天用的那一侧。

**三条硬禁止**（SPEC §4，改动前必须先问）：

1. 不升 ROS 2 Lyrical —— 两侧生态空白
2. 不退 Humble —— Humble 配 Harmonic 是非官方组合；旧结论基于 CARLA 0.9.15 的 Python 限制，
   0.9.16 已提供 3.10/3.11/3.12 wheel，该限制不复存在
3. **不换 CycloneDDS** —— CARLA 原生 ROS 2 只支持 Fast DDS。症状是本地 Gazebo 一切正常，
   一上 CARLA 完全收不到数据。Dockerfile 已锁 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

### 3. 车辆参数单一来源，SDF 是生成物

`config/vehicle_params.yaml` 是**唯一**允许定义轴距/转角/质量/加减速限值的地方。
`models/ego_vehicle/model.sdf` 由 `scripts/gen_vehicle_model.py` 从它生成 —— **不要手改生成物**。
P0b 的 `carla_bridge` 用 PythonAPI 的 `apply_physics_control()` 把 CARLA 车辆调到与它一致。

这条是 SPEC §4.1 的强制要求，针对的是**双环境头号风险「行为漂移」**：两套仿真动力学一旦不一致，
症状是本地调好的控制参数一上 CARLA 就震荡，而你无法判断是算法错了还是环境不同。

`base_link` 定在**后轴中心、地面高度**（Autoware 惯例）—— 自行车模型、Stanley、纯追踪都以
后轴为参考点推导，原点选这里能让控制算法直接用位姿，不必到处做偏移换算。

### 4. 算法与 ROS 强制解耦

每个包分 `lib/`（纯 C++17，无 ROS 依赖）和 `node/`（ROS 包装层）。
这不是风格偏好 —— L1 单元测试要保持毫秒级，必须能脱离 ROS 跑。

模块间**只通过 ROS 话题/服务通信**，不允许跨模块直接函数调用。

### 5. 坐标系与时间

- `map`（全局 ENU）→ `odom` → `base_link`（x 前 y 左 z 上），**全部走 TF2，禁止手写变换矩阵**
- 所有节点 `use_sim_time=true`，**禁止用 `now()` 做算法时序**

### 6. 路线图顺序：控制排在感知前面

P2 控制 → P3 规划 → P4 定位 → P5 感知。反直觉但正确：先用仿真真值打通
「规划→控制→车动起来」的闭环，之后每个模块都能立刻看到效果。
先做感知的话，你会对着点云调三个月而车一步没动。

---

## 代码规范里非显然的强制项

完整规范见 SPEC §7（ROS 2 官方风格，snake_case 文件/函数、PascalCase 类、成员变量后缀下划线、
2 空格缩进、行宽 100）。以下几条最容易被忽略：

1. **物理量必须带单位后缀**：`speed_mps`、`angle_rad`、`dist_m`。单位混淆是本领域最高频 bug 源。
2. **所有角度用弧度**，只在输出时转角度。
3. **不允许硬编码魔数**：`if (dist < 5.0)` 里的 5.0 必须来自配置且有注释说明依据。
4. **不允许在回调中做重计算**，超过 10 ms 的工作放独立线程/定时器。
5. **公开接口必须有 Doxygen 注释，说明单位、坐标系、有效范围。**
6. 不用裸 `new`/`delete`；禁止 `using namespace std;`。

---

## 测试

四层金字塔（SPEC §8）：L1 单元（gtest，无 ROS，毫秒级，`lib/` 行覆盖 > 80%）→
L2 模块（launch_testing，秒级）→ L3 场景（闭环，分钟级，**L3-G 进 CI / L3-C 云端验收**）→
L4 回归。

**本地 Gazebo 能进 CI 是双环境方案最被低估的收益** —— CARLA 永远进不了 CI（要 GPU、
20 GB 镜像、跑不确定），只有 CARLA 的话 L3 层就只能人工跑，而人工跑的测试三个月后一定没人跑。

场景测试必须有**可量化的通过判据**。「没崩溃」不等于「对了」。

---

## 边界

### ⚠️ 先问后做

- 引入新的第三方依赖（尤其重型库）
- 修改 `ads_msgs` 已定义的消息接口 —— 影响所有下游模块
- 改变模块划分或数据流
- 引入深度学习模型
- 任何超过 500 行的单次改动
- 改动版本栈（见上文三条硬禁止）

### ❌ 禁止

- 跳过 ODD 直接写算法（「这个场景应该也能用吧」是事故起点）
- 跨模块直接调用函数，绕过 ROS 接口
- 手写坐标变换矩阵而不用 TF2
- 在没有量化判据的情况下声称某功能「做完了」
- 为了让测试通过而放宽判据
- 把安全相关逻辑（碰撞检查）放在可被配置关闭的分支里
