# automated-driving-systems

园区 / 厂区低速自动驾驶栈，从零手写。ROS 2 Jazzy + C++17。

目标是**工业级架构**而非车规量产：模块划分、接口契约、测试分层都按真实项目的
做法来，但不追求功能覆盖度。

- **[SPEC.md](SPEC.md)** —— 唯一事实来源。ODD、架构、接口契约、测试策略都在这里。
- **[CLAUDE.md](CLAUDE.md)** —— 导航 + 实测踩坑记录。**遇到诡异现象先翻它的陷阱表**。
- **[tasks/](tasks/)** —— `plan.md` 说明怎么拆任务，`todo.md` 是当前进度。

当前阶段：**P3 运动规划已完成**（CP-P3-B 三场景达成，2026-08-04）。
车遇到贴边障碍物会自动绕过去（实测侧向间距 0.532 m，判据 > 0.5），
遇到车道正中、几何上绕不过去的障碍物会停住并报「不可行」——
**两个能力都要有**，因为 3.5 m 车道装 1.8 m 的车再留 0.5 m 间距，
可绕的余地本来就只剩 1.2 m。

此前已完成：P0a 本地环境、P1 地图与路由、P2 控制（CP-P2-B 8/8）、
P0b 方案 B（在云 GPU 上实测出 CARLA 与 Gazebo 的转向执行机构差异）。

---

## 它现在能做什么

一条命令起全栈，键盘开车，RViz 里实时看点云和 TF。

```
Gazebo Harmonic ──┐
                  ├─ gazebo_bridge ─→ /lidar/points  /imu  /gnss  /odom  /tf
键盘 teleop ──────→ /vehicle_cmd ──→ vehicle_cmd_bridge ─→ Gazebo
```

实测基线（`verify_*.sh` 的输出，不是估计值）：

| 指标 | 实测 |
|---|---|
| 实时率 RTF | 1.000 |
| `/lidar/points` 频率 | 10.00 Hz |
| 点云坐标系 | `base_link`（自车反射点已滤除） |
| TF 树 | `map → odom → base_link → {lidar,gnss}_link` 五段连通 |
| 转角限幅 | 下发 10 rad → 输出 0.600 rad |
| 看门狗 | 停发指令 6.3 s 后 8.333 → 0.000 m/s |

---

## 环境要求

| 项 | 要求 | 说明 |
|---|---|---|
| 宿主 | Windows 11 + WSL2 | Linux 原生也行，但 GPU 直通那几步不同 |
| GPU | 支持 D3D12 的显卡 | 开发机实测 AMD Radeon 780M 核显足够 |
| Docker | 装在 WSL2 内 | Docker Desktop 也可 |
| 磁盘 | ≥ 25 GB | 镜像含 ros-jazzy-desktop 和 Gazebo |

**不需要**在宿主装 ROS 或 Gazebo，全在容器里。

---

## 从零跑起来

### 1. 克隆并生成环境配置

```bash
git clone git@github.com:coolTheWorld/automated-driving-systems.git
cd automated-driving-systems

export COMPOSE_FILE=docker/docker-compose.local.yml
./scripts/setup_env.sh          # 生成 .env（宿主 UID/GID + GPU 设备组 GID）
```

> ⚠️ **所有 `docker compose` 命令都要在仓库根目录执行。** `COMPOSE_FILE` 里是相对
> 路径，`cd docker/` 之后就找不到了，会报 `no configuration file provided`。
>
> `.env` 是机器相关的，已在 `.gitignore` 里。**换机器必须重跑 `setup_env.sh`。**

### 2. 构建并进入容器

```bash
docker compose build            # 首次约 10-20 分钟
docker compose up -d
docker compose exec dev bash
```

### 3. 环境自检（**这一步不能跳**）

```bash
docker compose exec dev /workspace/scripts/verify_gpu.sh
```

这是整个本地方案的 **go/no-go**。关键是那项渲染器必须是硬件设备：

```
Device: D3D12 (AMD Radeon 780M Graphics)
Accelerated: yes
```

如果显示 `llvmpipe` 就是掉进软件渲染了 —— **不要用软件渲染硬撑**，Gazebo 会慢到
没法用。先看 CLAUDE.md 的环境陷阱表，八成是 `/dev/dxg` 没映射，或者
`GALLIUM_DRIVER=d3d12` 没设。

（`screen 0 does not appear to be DRI3 capable` 是**干扰项**，宿主也报，与硬件加速无关。）

### 4. 编译

```bash
docker compose exec dev bash -c '
  source /opt/ros/jazzy/setup.bash
  cd /workspace
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
'
```

### 5. 验证全链路

```bash
docker compose exec dev /workspace/scripts/verify_sim.sh         # 6 项：仿真基线 + RTF
docker compose exec dev /workspace/scripts/verify_ros_bridge.sh  # 6 项：ROS 话题契约
docker compose exec dev /workspace/scripts/verify_teleop.sh      # 3 项：控制链路 + 限幅 + 看门狗
```

三个都要**退出码 0**。

> ⚠️ 跑任何 verify 脚本前先确认没有残留的仿真进程，否则两套仿真同时发 `/clock`
> 和 `/tf`，所有测量值都是垃圾：
>
> ```bash
> docker compose exec dev bash -c 'ps -eo pid,pgid,args | grep -E "gz sim|ros2 launch" | grep -v grep'
> docker compose exec dev bash -c 'kill -INT -- -<PGID>'   # 按进程组收
> ```
>
> **不要用 `pkill -f`** —— 它匹配完整命令行，而执行它的 shell 命令行里就含那个
> 模式，结果是清理命令把自己杀了。

---

## 开车

需要**两个终端**。

终端 A —— 起全栈（Gazebo + 桥接 + TF + RViz）：

```bash
docker compose exec dev bash -c '
  source /opt/ros/jazzy/setup.bash
  source /workspace/install/setup.bash
  ros2 launch ads_bringup stack.launch.py
'
```

终端 B —— 键盘：

```bash
docker compose exec dev /workspace/scripts/drive.sh
```

| 键 | 作用 |
|---|---|
| `w` / `s` | 加速 / 减速 |
| `a` / `d` | 左转 / 右转 |
| `空格` | 松油门 + 方向回正 |
| `b` | 紧急制动 |
| `q` | 退出 |

**先按几下 `w` 让车动起来再打方向。** 静止时打方向车不会转 —— 横摆角速度
ω = v·tan(δ)/L，v=0 时恒为 0，真车原地打方向也不动。

也**不支持倒车**：`ads_msgs/VehicleCmd` 只有转角和加速度、没有挡位字段，负加速度
只能理解成"减速"，无法与"倒车"区分。真实栈（如 Autoware）用独立的 GearCommand 解决。

> ⚠️ 键盘驾驶必须走 `drive.sh`（它内部用 `ros2 run`），**不能用 `ros2 launch`** ——
> launch 会接管子进程的 stdio，键盘输入到不了节点。

---

## 开发

```bash
# 单个包
colcon build --packages-select gazebo_bridge

# 测试 + lint（提交前跑）
colcon test && colcon test-result --all

# 只跑 L1 单元测试，快一个数量级
./build/ads_common/test_angles

# 改了 config/vehicle_params.yaml 之后**必须**重新生成 SDF 和 URDF
python3 scripts/gen_vehicle_model.py
python3 scripts/gen_vehicle_model.py --check    # CI 用这个卡住"忘了重新生成"
```

代码风格由根目录的 `.clang-format` 定义，编辑器保存即格式化，CI 会卡。

### 几条不显然的约束

1. **`config/vehicle_params.yaml` 是车辆参数的唯一来源**，SDF 和 URDF 都是生成物，
   不要手改。手改的症状是 RViz 里点云和车模型对不上，或更隐蔽的：TF 报的外参与
   Gazebo 里实际安装位置差几厘米，下游全部带着这个偏差却没有任何报错。
2. **算法与 ROS 解耦**：每个包分 `lib/`（纯 C++17，无 ROS）和 `node/`（ROS 包装层）。
   这不是风格偏好 —— L1 测试要保持毫秒级，必须能脱离 ROS 跑。
3. **所有节点 `use_sim_time=true`**，禁止用 `now()` 做算法时序。
4. **坐标变换一律走 TF2**，禁止手写变换矩阵。
5. **物理量带单位后缀**：`speed_mps`、`angle_rad`、`dist_m`。单位混淆是本领域
   最高频的 bug 源。

完整规范见 SPEC.md §7。

---

## 目录

```
├── SPEC.md                      # 唯一事实来源
├── CLAUDE.md                    # 导航 + 实测踩坑记录
├── config/                      # **全部手写源头**，生成物一律不手改
│   ├── vehicle_params.yaml      #   车辆参数（SDF/URDF 都从它生成）
│   ├── campus_map.yaml          #   地图（.xodr / 路面 SDF / 对账基准 都从它生成）
│   ├── obstacles.yaml           #   P3 验收场景的障碍物（Gazebo 模型从它生成）
│   ├── control_params.yaml      #   控制器调参
│   └── planning_params.yaml     #   规划器调参
├── docker/                      # 本地与云端共用同一个 Dockerfile
├── models/  worlds/             # Gazebo 模型与世界（model.sdf 是生成物）
├── scripts/                     # setup / verify / 单项检查
├── src/
│   ├── ads_msgs/                # 消息接口（模块间契约）
│   ├── ads_common/              # 纯算法工具，**不依赖 ROS**（角度、参考线几何、入参校验）
│   ├── ads_map/                 # OpenDRIVE 解析 + 车道图 + 路由
│   ├── ads_planning/            # Frenet 采样 + 碰撞检测 + 速度剖面
│   ├── ads_control/             # 横向 Stanley + 纵向速度环 PI
│   ├── ads_bringup/             # 全栈 launch 入口
│   ├── ads_simulation/
│   │   └── gazebo_bridge/       # 环境 A：Gazebo → 规范话题（含障碍物真值发布器）
│   ├── ads_teleop/              # 键盘 / 手柄 → /vehicle_cmd
│   └── ads_visualization/       # URDF（生成物）+ RViz 配置
└── tasks/                       # 任务拆解与进度
```

数据流：`ads_map → /route/path → ads_planning → /planning/trajectory → ads_control → /vehicle_cmd`。

SPEC §5 里还列了 `ads_perception`、`ads_localization`、`ads_prediction` 等包，**尚未创建**。

---

## 路线图

P0a 本地环境 ✅ → P0b 云端 CARLA（方案 B 已实测）→ P1 地图与路由 ✅ →
P2 控制 ✅ → P3 规划 ✅ → **P4 定位** → P5 感知 → P6 预测 → P7 实车。

控制排在感知前面是有意的：先用仿真真值打通「规划→控制→车动起来」的闭环，
之后每个模块都能立刻看到效果。先做感知的话，你会对着点云调三个月而车一步没动。

架构决策记录见 [docs/adr/](docs/adr/)。

---

## 许可

[Apache-2.0](LICENSE)
