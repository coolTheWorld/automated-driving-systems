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

**所有 `docker compose` 命令都必须在仓库根目录执行** —— `COMPOSE_FILE` 里是相对路径，
`cd docker/` 之后就找不到了，而那个目录里又没有 compose 默认认的 `docker-compose.yml`，
于是报 `no configuration file provided: not found`。

```bash
export COMPOSE_FILE=docker/docker-compose.local.yml    # 本机 Gazebo（日常）
# export COMPOSE_FILE=docker/docker-compose.cloud.yml  # 云端 CARLA（P0b 起，骨架）

./scripts/setup_env.sh          # 生成 .env（宿主 UID/GID + GPU 设备组 GID），换机器必跑
docker compose build
docker compose up -d
docker compose exec dev bash
```

### 环境自检（改动 docker/ 或换机器后先跑前四个；改了地图或路由跑第五个）

```bash
docker compose exec dev /workspace/scripts/verify_gpu.sh        # 10 项：GPU 硬件加速
docker compose exec dev /workspace/scripts/verify_sim.sh        # 6 项：Gazebo 仿真基线 + RTF
docker compose exec dev /workspace/scripts/verify_ros_bridge.sh # 6 项：ROS 桥接契约（CP2）
docker compose exec dev /workspace/scripts/verify_teleop.sh     # 3 项：控制指令链路 + 限幅 + 看门狗
docker compose exec dev /workspace/scripts/verify_map.sh        # 3 组：地图与路由（**不需要仿真器**）
```

`verify_map.sh` 是唯一**能进 CI** 的端到端验收 —— `map_node` 只要一段
`map→base_link` 的 TF 就能工作，用 `static_transform_publisher` 把自车钉在
已知位姿上即可，不要 GPU、十几秒跑完、结果完全确定。
其余三个要真仿真器，进不了 CI，只能靠人记得跑。

**跑任何 verify 脚本前先确认没有残留的仿真进程**，否则两套仿真同时发
`/clock` 和 `/tf`，所有测量值都是垃圾（`verify_ros_bridge.sh` / `verify_teleop.sh`
已内建拦截）：

```bash
docker compose exec dev bash -c 'ps -eo pid,pgid,args | grep -E "gz sim|ros2 launch" | grep -v grep'
docker compose exec dev bash -c 'kill -INT -- -<PGID>'   # 按进程组收，不要用 pkill -f
```

退出码 0 才继续。`verify_gpu.sh` 是整个本地环境方案的 go/no-go —— **失败时不要用软件渲染硬撑**。

`verify_ros_bridge.sh` 检查的全是 SPEC §4.1 的**对外契约**，不含 Gazebo 特有的东西 ——
P0b 的 `carla_bridge` 要用它**原样验收**，只换 `LAUNCH_PKG` / `LAUNCH_FILE` 两个变量。

### 已实测的环境陷阱

| 陷阱 | 症状 | 处理 |
|---|---|---|
| 漏映射 `/dev/dxg` | 静默退化 llvmpipe，**不报错** | compose `devices` 必须有 `/dev/dxg`；`/dev/dri` 只是 WSL 的兼容外观 |
| 未设 `GALLIUM_DRIVER=d3d12` | 同上 | 容器 Ubuntu 24.04 的 Mesa 不自动探测 d3d12（宿主 26.04 会）；Jazzy 绑死 24.04 升不了 Mesa |
| 脚本里直接调 `gz` | command not found | Gazebo 装在 ROS 前缀下（`ros-jazzy-gz-sim-vendor`），需先 `source /opt/ros/jazzy/setup.bash` |
| `set -u` 下 source ROS | `AMENT_TRACE_SETUP_FILES: unbound variable` | source 期间临时 `set +u` |
| `docker exec bash -lc` | `~/.bashrc` 不生效，ROS 环境没加载 | 用 `bash -c` 并显式 source |

`screen 0 does not appear to be DRI3 capable` 是**干扰项**，宿主也报，与硬件加速无关。

### 仿真进程与测量的陷阱（S3 实测，会重复踩）

| 陷阱 | 症状 | 处理 |
|---|---|---|
| **`pkill -f "gz sim"`** | 清理命令自己先被杀，后面一行没执行 | `pkill -f` 匹配**完整命令行**，而执行它的 shell 命令行里就含这串。**一律按 PID / 进程组收** |
| 只 kill `ros2 launch` 的 PID | 留下孤儿 `gz sim`（PPID=1），下次跑就有**两套仿真** | 用 `setsid` 起 launch，收尾时 `kill -INT -- -<PGID>` 收整组 |
| 两套仿真并存 | TF 刷 `Detected jump back in time` / `TF_OLD_DATA`，**所有测量值作废** | 动手前先 `ps` 查残留；`verify_ros_bridge.sh` 已内建拦截 |
| 点云用 best-effort QoS | **静默丢帧**，频率只有标称的 35%，无任何日志 | 本机仿真链路用 `reliable` + 深度 10。判据：Gazebo 侧帧间隔只有 100/200 ms 两种值 = 传感器没问题，是队列在丢 |
| `ros2 topic hz` / `tf2_echo` 接管道 | 被 `timeout` 打断后**完全没输出**，看着像没数据 | 输出是全缓冲的。用常驻 Python 节点测（见 `scripts/check_*.py`），或 `stdbuf -oL` |
| `ros2 topic list` 紧接 daemon 启动 | 只返回 `/rosout` `/parameter_events` | daemon 的节点图还没建好。先 `ros2 daemon start` 再等几秒 |
| `gpu_lidar` 的无回波射线 | `skip_nans` 滤不掉，`min/max` 变 `±inf` | 返回的是 **±inf 不是 NaN**。求极值/质心/体素前必须显式 `isfinite` 过滤 |
| **用比较去拦非有限值** | 校验代码看着写了、实际**一条都没拦住**，脏数据一路往下传 | `NaN` 参与**任何**比较都返回 `false`，所以 `if (x < limit) reject;` 对 NaN 恒为假，`clamp` 也会原样放行。**必须单独 `isfinite`，而且要判 ±inf 不只是 NaN**（上一行）。已经咬过两次：`vehicle_cmd_bridge` 的指令限幅、`ads_control` 的路径点距校验。后果不是崩溃而是**误诊** —— NaN 传到转角后被下游 isfinite 挡下、看门狗刹停，现场是「车自己停了」，于是人去查控制器，而错在上游 |
| 雷达打到自车车顶 | 34% 的点落在自己车顶上，感知会当成零距离障碍物 | 雷达 z=1.6 而车顶 z=1.5，只高 10 cm。**抬高雷达没用**（车顶长 4.4 m），只能按自车轮廓裁剪。已在 `lidar_preprocessor` 实现 |
| 读 stdin 的节点在 `ros2 launch` 下假死 | 进程活着、`topic info` 有 publisher，**一条消息都不发**，无报错 | `termios` 的 `VMIN/VTIME` **只对终端生效**；launch 下 stdin 是管道，`read()` 永久阻塞、卡死回调。必须另加 **`O_NONBLOCK`**（对所有 fd 都管用） |
| 用 `ros2 launch` 起交互式节点 | 键盘输入到不了节点 | launch 接管子进程 stdio。键盘开车用 `scripts/drive.sh`（走 `ros2 run`） |
| pty 驱动测试不排空 master | 被测进程"前几个按键有效、之后全无反应" | 伪终端缓冲区只有几 KB，填满后子进程 `printf` 阻塞。测试端必须起线程持续读 |
| SDF 转向关节缺 `<effort>` | Gazebo 报错，**前轮能转过机械极限** | 速度控制下 dartsim 需要力矩上限才检查位置限位 |
| `<gz_frame_id>` 报 SDF 警告 | `XML Element[gz_frame_id] ... not defined in SDF` | **正常**，功能有效（frame_id 确实被改写）。它不在 SDF 规范里，Gazebo 自己解析 |
| **`gz sim` 崩溃被 launch 报成"干净退出"** | RViz 里点云/TF 全空，只剩个孤零零的车模型，**launch 日志无任何错误** | launch.log 写 `[gz-1]: process has finished cleanly`，但看**存活时长**：几秒 = 崩了。铁证是仓库根目录冒出 `core.<pid>`。别去查 RViz 配置，查 gz 还在不在 |
| WSL 下 D3D12 设备丢失 | `Removing Device` → `OGRE EXCEPTION: Out of GPU memory`，请求区区 128 MB 却失败 | **因果是反的**：设备先丢，之后所有 GL 调用一律返回 `GL_OUT_OF_MEMORY`。16 GB 显存不可能不够。宿主 GPU 驱动重置（锁屏/休眠/更新）触发，**瞬态**。⚠️ **连挂两次也仍然是瞬态** —— 2026-08-03 实测 08:53 与 08:54 连崩，几分钟后同一条命令一次过。驱动复位的窗口没那么短，别挂两次就去改配置。诊断阶梯（四步全过就是它）：`verify_gpu.sh` → headless → 只开 gz GUI → GUI+RViz 同开。**通路没坏就别动 compose** |
| `gz sim` 退出时 segfault | 每次收 SIGINT 退出都产 400–600 MB `core.*` | Gazebo 已知的退出清理问题，**不影响功能**。但 core 落在 cwd（= 挂载的仓库根），跑几次就是 GB 级 —— 上面那次 D3D12 崩溃两下就攒了 1.4 GB。已在 `.gitignore`，仍需定期 `rm -f core.*` |
| **`setsid cmd &` 之后用 `$!` 取进程组** | 清理命令**静默地什么都没做**，仿真进程留下来继续跑 | `$!` 是 **setsid 自己**的 PID，它 fork 出新进程组后立刻退出，于是 `ps -o pgid= -p $!` 返回**空**，`kill -INT -- -` 变成空操作、还不报错。要按**进程名**查：`ps -eo pgid,args \| grep "gz sim"`。上面那条「用 `kill -INT -- -<PGID>`」是对的，坑在 PGID 怎么取到 |
| headless `gz sim -s` 收 SIGINT 不退 | 发了 INT、等 3 s 仍在 | 实测需升级到 TERM/KILL。**也可能只是它退得比 3 s 慢，两种解释没分辨开 —— 别当定论** |
| **脚本里 `pgrep -f <名字>` 查残留** | 报「已经有 1 个在跑」然后拒绝启动，实际一个都没有 | 与 `pkill -f "gz sim"` 同源：`-f` 匹配**完整命令行**，而执行脚本的那条命令行里只要出现过这几个字（比如 `clang-format -i src/ads_map/node/map_node.cpp && verify_map.sh`）就会自己匹配自己。用 **`pgrep -x <进程名>`** 按进程名精确匹配 |
| 杀掉 `static_transform_publisher` 想测「TF 没了」 | `lookupTransform` 照样成功，什么都测不出来 | 静态变换走 `/tf_static`，**tf2 的 buffer 对它永不过期**。要测「拿不到 TF」只能**一开始就不发**（这也正是真实场景：节点起来了、仿真还没起） |
| **Gazebo 的转向执行机构慢得离谱**（已修） | 控制器在 L1 上完美收敛（1.6 cm），一上 Gazebo 弯道横向误差就在 ±0.8 m 之间震荡 | 开环实测：转角阶跃的 **63% 上升时间 1.20 s**，比 Stanley 自己的闭环时间常数 `1/k_e`=1.0 s 还大 —— 被控对象比控制器慢，这是震荡配方。稳态是**准的**（99.7–100.3%），只是慢，所以**定转角测转弯半径完全看不出问题，只有阶跃响应才量得到**。转向**关节**与横摆角速度的上升时间只差 0.04 s → 滞后全部来自执行机构，不是车身动力学。根子是 `AckermannSteering` 的转向 P 增益 SDF 里从没设过。修法：`vehicle_params.yaml` 加 `actuator.steer_response_time_s`，`gen_vehicle_model.py` 推导出 `<steer_p_gain> = 1/τ`（**不让人直接填增益** —— 增益换个仿真器没有对应物）。实测 1.198 → **0.294 s**，横向误差 0.801 → **0.063 m**。用 `scripts/probe_steering_response.py` 量，**P0b 对齐 CARLA 时要拿同一把尺子再量一遍** |
| **插件的转向响应有约 0.24 s 的地板** | 把 `steer_response_time_s` 从 0.050 调到 0.020（增益 20→50），实测只从 0.294 s 到 0.255 s | 关节速度环自身的滞后，P 增益动不了它。所以**那个参数不等于实测响应时间**，要知道实际多快只能跑脚本量。取 0.050 是因为它拿到了可改进量的绝大部分而增益还只有 20 —— 再往上是在跟地板较劲，代价是大误差时向关节下发上百 rad/s 的速度指令 |
| **仿真钟停走时控制器"冻住"而不是降级** | 杀掉 `parameter_bridge` 想测里程计超时的降级路径，结果**什么日志都没有** | 那个进程同时也是 `/clock` 的来源。`use_sim_time=true` 下 `now()` 不再前进 → `dt = 0` → 控制回调直接 return，"里程计过期"永远判不出来；bridge 的看门狗同样用仿真钟，也冻住 → **车保持最后一条指令**。真车上墙钟不会停，所以这是仿真特有的。要补得用**墙钟做健康检查**（那不算算法时序，不违反 SPEC §5） |
| **横向加速度超标 ≠ 横向跟得不好** | 修好转向执行机构后横向误差降了 12.7 倍，**最大横向加速度一个数都没变**（2.113） | `a_lat = v²·κ` 里有**两个**因子，横向误差不影响 κ，**只有速度会**。实测峰值处横向误差只有 −0.027 m，而车速比剖面高 +0.85 m/s；按剖面速度过那个弯 `a_lat` 恰好是 1.5002 = 限值。**看到横向加速度超标就去调横向增益，方向从一开始就是错的** |
| **拿 `path_remaining_m` 判「停得准不准」** | 冲过终点 4 m 和恰好停住给出**一模一样**的数 | 投影越过路径末点后被**夹到端点**，`s_m` 和剩余距离都不再变化。这条 `path_tracking.hpp` 早就写明并交给 S4 了，**照样踩了**。要判到达/冲过终点必须另算前轴到路径末点的**直线距离**（`control_node` 的 `goal_distance_m`） |
| **纯 P 速度环跟不上剖面的斜坡** | 车总是"入弯偏快、终点冲过头"，而稳态巡航时速度跟得很准 | 「纯 P 对常值目标无稳态误差」只对**常值**成立。速度剖面在入弯前和终点前是**斜坡**，一阶系统跟踪斜坡的稳态误差 = 斜率/`K_p` = 3.0/1.0 = **3.0 m/s**。解法是加速度前馈 `a = v·dv/ds + K_p·(v_ref − v)`，**不是调大 K_p** |
| 采样时 `ceil(span/step)` + 末点夹到端点 | 路径最后**两个点重合**，RViz 里完全看不出来，下游按弧长参数化时除以零 | `span/step` 恰好是整数时，浮点上它可能是 `80.00000000000001`，ceil 多算一步。改成**等分**：`offset = span·i/count`，两端精确、无需夹取。触发它的是 `nearest_lane` 的三分法把 s 细化成 `40.000000000000007` |

**「频率低 = 算力不够」是最容易犯的想当然。先分层测量再动参数** ——
S3 时据此砍掉一半激光雷达分辨率，结果只快了 4%，因为病根是 QoS 不是 GPU。

**"某个显示项没数据" 先怀疑数据源死了，而不是显示端配错了。** 上面头两条就是同一次
故障的两层：表象在 RViz，根因在 GPU 驱动，中间还隔着一层"launch 谎报平安"。
排查顺序永远是**沿数据流往上游走**：RViz → 话题有没有 → 发布者在不在 → 仿真器在不在。

---

## 常用命令

`src/` 下**目前有八个包**：`ads_msgs`、`ads_common`、`ads_map`、`ads_control`、
`ads_bringup`、`ads_simulation/gazebo_bridge`、`ads_teleop`、`ads_visualization`。
SPEC §5 列出的其余包（`ads_planning`、`ads_perception`、`ads_localization` 等）**尚未创建**，
涉及它们的命令是规划中的形态，不要假设能跑。

`ads_map` 已有完整两层：`lib/`（OpenDRIVE 解析 + 几何求值 + 车道级有向图 +
Dijkstra 路由，纯 C++ 无 ROS）和 `node/map_node.cpp`（ROS 包装层，P1-S4）。
`libads_map.so` 只链接 `libtinyxml2` 和 `libads_common`，**零 ROS 依赖** ——
这条由 `scripts/verify_map.sh` 的 `ldd` 检查机械保证，不靠纪律。

`ads_control`（P2，**建设中**）目前有四个 lib：**`path_tracking`**（弧长参数化、
逐点曲率、最近点局部搜索、横向/航向误差）、**`stanley`**（前轴换算 + 控制律 +
转角/转向速率双重限幅）、**`speed_profile`**（曲率限速 + 前后向扫描）、
**`speed_controller`**（速度环 PI + 条件积分抗饱和）。
`node/control_node.cpp`（S4）已跑通闭环，**CP-P2-B 8/8 达标**（2026-08-03，两遍复现）。`libads_control.so` 同样零 ROS 依赖，且**不依赖 `ads_map`** ——
两者是 SPEC §3.3 意义上的两个模块，只通过 ROS 话题通信。
推导与参数见 [docs/modules/control.md](docs/modules/control.md)，**改这个模块前先读它**：
里面有三条「做错了不会报错，只会给出一个看起来能开的车」的结论
（前轴换算、按弧长而非点数、**不要加曲率前馈**），每一条都有专门的**闭环反例用例**
守着（用后轴 → 外偏 1.182 m；加前馈 → 稳态 1.400 m）。
另有一条 S2 实测出来的跨模块耦合：**转向速率限幅起作用的临界车速是 `v* = R·rate`**
（R=8 上 = 4.0 m/s），而 `a_lat_max` 调到 2.0 时曲率限速恰好就是 4.0 —— 见 §3.7。

```bash
# ---------- 构建（容器内） ----------
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /workspace/install/setup.bash            # 每次新 shell 都要
colcon build --packages-select gazebo_bridge    # 单个包

# ---------- 运行（已可用） ----------
ros2 launch ads_bringup stack.launch.py                             # 全栈入口，默认 sim:=gazebo
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false      # headless
ros2 launch gazebo_bridge gazebo_sim.launch.py                      # 只起仿真侧，调链路时更快
ros2 run ads_map map_node                                           # 只起地图节点（不需要仿真器）

# ⚠️ stack.launch.py 的默认世界是 **campus_loop.sdf**（P1 起），
#    而 gazebo_sim.launch.py 的默认仍是 campus_minimal.sdf —— 这不是笔误：
#    三个 verify 脚本都不传 world、直接吃后者的默认值，它们的实测基线
#    （RTF 0.970、点云 10.00 Hz…）全部建立在 campus_minimal 上。

# 键盘开车（**另开一个终端**，必须走 drive.sh 而不是 ros2 launch，见陷阱表）
docker compose exec dev /workspace/scripts/drive.sh

# ---------- 测试与 lint（提交前跑） ----------
colcon test && colcon test-result --all      # 全量：lint + L1 + L3-G 闭环（当前 414 tests，约 20 s）
colcon test --packages-select ads_common     # 单个包
./build/ads_common/test_angles               # 直接跑 gtest，快一个数量级，日常改代码用
./build/ads_map/test_geometry                # 参考线几何 vs 解析解
./build/ads_map/test_opendrive_parser        # 解析 + **CP-P1-A 双实现逐点对账**
./build/ads_map/test_lane_graph              # 车道图：18 节点 / 24 边 + 每条边的几何连续性
./build/ads_map/test_routing                 # Dijkstra：路径与长度 vs 穷举脚本
./build/ads_control/test_path_tracking       # 弧长/曲率/最近点，全部 vs 解析解（P2-S1）
./build/ads_control/test_stanley             # Stanley + **CP-P2-A 闭环收敛判据**（P2-S2）
./build/ads_control/test_speed_profile       # 曲率限速 + 前后向扫描，全部 vs 闭式解（P2-S3）
./build/ads_control/test_speed_controller    # 速度环 + 抗饱和 + bridge 饱和环节（P2-S3）

# L3-G：**不需要 GPU** 的端到端闭环（假车 + map_node + control_node），已进 CI。
# 验的是节点接线（话题/QoS/TF/参数/时序），**不是控制律也不是真物理**。
# 它抓得住 /route/path 那个 QoS bug —— 退回 volatile 立刻红。
colcon test --packages-select ads_control --ctest-args -R test_closed_loop

# ---------- 闭环实测（P2-S4，需要真仿真器，进不了 CI）----------
# 先起 headless 全栈，再跑记录脚本；判据来自 plan.md 的 CP-P2-B 表，脚本里不重新发明
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false
python3 scripts/record_control_run.py --goal 91.75 20.0 --out /tmp/run.csv
# 被控对象辨识（开环，控制器不参与）—— 车必须停在**路面上**再跑
python3 scripts/probe_steering_response.py --step 0.30 --speed 4.0

# ⚠️ 判定成败**必须**看 colcon test-result，不能只看 colcon test 的退出码 ——
#    后者反映的是"测试有没有跑起来"，测试失败它照样可能返回 0。

# ---------- 尚不可用（还没做到那一步） ----------
ros2 launch ads_bringup stack.launch.py sim:=carla  # P0b 才有（现在会明确报错，不会静默空转）
ros2 launch ads_bringup stack.launch.py sim:=carla  # P0b 才有（现在会明确报错，不会静默空转）
```

### 模型生成与单项检查

```bash
# 从 YAML 重新生成 SDF(Gazebo) + URDF(ROS)。改了 vehicle_params.yaml 必跑
python3 scripts/gen_vehicle_model.py
python3 scripts/gen_vehicle_model.py --check    # 校验生成物是否与 YAML 同步（CI 用）

# 从 YAML 重新生成地图。改了 campus_map.yaml 必跑 —— 它有**三个**生成物：
#   maps/campus.xodr（两环境共用）、models/campus_road/model.sdf（Gazebo 可视）、
#   src/ads_map/test/data/reference_samples.csv（给 C++ 做逐点对账的基准）
python3 scripts/gen_map.py
python3 scripts/gen_map.py --check              # 三个生成物逐字节比对
# ⚠️ 采样基准漏了重新生成的话，CP-P1-A 的对账用的是旧基准 → **虚假的通过**。
#    所以它进了 OUTPUTS，也进了 colcon test（本地就能拦，不用等 CI）。

# 容器内，需先 source ROS + install
python3 scripts/check_cloud_frames.py    # 点云 frame_id / 频率 / 是否真的做了变换
python3 scripts/check_tf_tree.py         # TF 树逐段连通性

# 直接玩 Gazebo（需先 source /opt/ros/jazzy/setup.bash）
gz sim -r campus_loop.sdf                                         # P1 起用这个（环线 + 2 个 T 路口）
gz sim -r campus_minimal.sdf                                      # P0a 的回归基线，**冻结不动**
gz topic -t /model/ego_vehicle/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 3.0}'
gz topic -e -t /world/campus_loop/stats -n 12                     # 读 RTF（世界名要跟着换）
```

**两个世界并存，不是替换。** `campus_minimal.sdf` 是 `verify_sim.sh` /
`verify_ros_bridge.sh` / `verify_teleop.sh` 三个脚本的回归基线（RTF 0.970、
点云 10.00 Hz 等实测值都基于它），改动它那些数字全部作废 ——
而它们是判断「环境有没有退化」的唯一依据。三个 verify 脚本都用
`WORLD_FILE` / `WORLD_NAME` 环境变量参数化过，可以原样验收新世界：

```bash
docker compose exec -e WORLD_FILE=/workspace/worlds/campus_loop.sdf \
                    -e WORLD_NAME=campus_loop dev /workspace/scripts/verify_sim.sh
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

### 3. 车辆参数单一来源，SDF 和 URDF 都是生成物

`config/vehicle_params.yaml` 是**唯一**允许定义轴距/转角/质量/加减速限值
**以及传感器安装外参**的地方。`scripts/gen_vehicle_model.py` 从它生成**两份**产物，
**都不要手改**：

| 生成物 | 消费者 | 决定什么 |
|---|---|---|
| `models/ego_vehicle/model.sdf` | Gazebo 物理引擎 | 车**怎么动**（质量、惯量、摩擦、关节驱动、传感器） |
| `src/ads_visualization/urdf/ego_vehicle.urdf` | `robot_state_publisher` / RViz | **TF 树**长什么样、RViz 画什么 |

ROS 不认 SDF，Gazebo 建物理也不用 URDF，所以两份都得有。手写 URDF = 轴距和外参
又抄一遍，症状是 **RViz 里点云和车模型对不上**，或更隐蔽的：TF 报的
`base_link→lidar_link` 与 Gazebo 里雷达实际安装位置差几厘米，下游全部配准带着这个
偏差却没有任何报错。两份共用同一个 `derive()` 推导函数，就是为了堵这个。

P0b 的 `carla_bridge` 用 PythonAPI 的 `apply_physics_control()` 把 CARLA 车辆调到与它一致，
传感器也按同一份外参 spawn。

这条是 SPEC §4.1 的强制要求，针对的是**双环境头号风险「行为漂移」**：两套仿真动力学一旦不一致，
症状是本地调好的控制参数一上 CARLA 就震荡，而你无法判断是算法错了还是环境不同。

`base_link` 定在**后轴中心、地面高度**（Autoware 惯例）—— 自行车模型、Stanley、纯追踪都以
后轴为参考点推导，原点选这里能让控制算法直接用位姿，不必到处做偏移换算。

### 3b. 地图单一来源，`.xodr` 和 Gazebo 道路都是生成物

与上一条完全同构，只是一个防车、一个防路。`config/campus_map.yaml` 是**唯一**手写的源头，
`scripts/gen_map.py` 从它生成**三份**产物，**都不要手改**：

| 生成物 | 消费者 | 决定什么 |
|---|---|---|
| `maps/campus.xodr` | `ads_map` 解析 → 车道图 → 路由；CARLA 的 `generate_opendrive_world()` | **车道语义**：哪里能走、往哪走 |
| `models/campus_road/model.sdf` | Gazebo 渲染 | 你**看到的**路面长什么样 |
| `src/ads_map/test/data/reference_samples.csv` | `test_opendrive_parser` 的逐点对账 | C++ 实现与生成器**是否理解一致** |

**为什么 Gazebo 的路也必须是生成物**：Gazebo 不认识 OpenDRIVE，它需要三维路面几何。
于是「车道语义」和「路面几何」天然是两份东西，各写各的就会漂移。漂移的症状是
**路由算得出来、RViz 画得出来、车也开得动，但车沿着一条肉眼看不见的车道压着绿化带走** ——
全程没有任何模块报错。

**为什么路口不能手写**：OpenDRIVE 的路口是一块区域，内部由若干条「连接道路」填充，
一个 T 型路口就有 6 条，每条都要手算圆弧切点、曲率、起始朝向再手填 `laneLink`。
一个数填错的症状是车经过**那一个**路口时拐进绿化带，且看起来像控制问题。
让程序算，错就每次都错 —— 而每次都错的东西，测试抓得住。

**路口退让距离 `cutback` 是推导量而非配置项**：它必须 ≥ 转弯半径 + 半车道，
否则转弯圆弧的切点会落到路口区域外面、连接道路接不上。把这种约束交给人填
等于给一个填错就崩的机会；做成推导量，约束永远成立。

### 3c. 车道图是**有向**的，路由代价是**车道中心线**长度

两条都不是精细化，都是「做错了不会报错、只会给出一条看起来正常的错路径」。

**有向**：OpenDRIVE 的 lane −1 沿 s 行驶、+1 逆 s 行驶，同一条物理道路的两个方向
是**两个节点，之间没有边**。建成无向图的话，Dijkstra 会算出「原地掉头」是通往对向
车道的最短路 —— 而它在 RViz 里是一条平滑的短线，长度也合理。要到 P2 车真开上去才发现。
`test_lane_graph.cpp` 的 `EveryEdgeIsGeometricallyContinuous` 是这条的守卫：
它不看任何 OpenDRIVE 约定，只问「上一条车道的出口点是不是下一条的入口点、
**行驶朝向**接不接得上」，所以约定理解错了它一定红。

**代价用车道中心线长度**（`Road::lane_arc_length()`），不能用道路参考线长度。
R=12 m 的弯上右侧车道半径 13.75 m，差 14.6%。在本项目这张地图上后果是具体的：
从东侧顺行车道到它对向车道的两条候选路线，用车道长度算是 674.73 vs 685.73（赢家唯一），
用参考线长度算**恰好并列 680.23**（两者用到的参考线长度是同一个多重集）。
**并列意味着返回哪条取决于堆的遍历顺序**，换个标准库实现就可能变。

闭式解：等距偏移曲线 p(s) = r(s) + t·N(s)，Frenet 下 dp/ds = (1 − t·k)·T。
曲率 k 在一个几何段内是常数、横向偏移 t 在一个车道段内是常数，
所以在「几何段 ∩ 车道段」的每一小块上长度 = Δs·(1 − t·k)，是**精确值不是数值积分**。
变宽车道（width 的 b/c/d 非零）会**抛异常**而不是按常宽给个偏小的值。

**最近车道查询必须传朝向**（`LaneGraph::nearest_lane` 的 `heading_rad`）。
不传的话自车偏左一点就被判到对向车道，路由第一步就要求掉头 —— 路径本身依然平滑正常。

完整推导（圆弧闭式解、横向偏移、弧长因子、建边规则、Dijkstra 的虚拟源点）
以及**全部参数的物理含义与调大调小的后果**见
[docs/modules/map_and_routing.md](docs/modules/map_and_routing.md)。
改这个模块前先读它 —— 那里也列了**已实现与未实现的边界**（spiral 几何、
多 laneSection、变宽车道都会显式抛异常，不要假设它们能用）。

### 4. 算法与 ROS 强制解耦

每个包分 `lib/`（纯 C++17，无 ROS 依赖）和 `node/`（ROS 包装层）。
这不是风格偏好 —— L1 单元测试要保持毫秒级，必须能脱离 ROS 跑。

模块间**只通过 ROS 话题/服务通信**，不允许跨模块直接函数调用。

### 5. 坐标系与时间

- `map`（全局 ENU）→ `odom` → `base_link`（x 前 y 左 z 上），**全部走 TF2，禁止手写变换矩阵**
- 所有节点 `use_sim_time=true`，**禁止用 `now()` 做算法时序**

**`map → odom` 不是单位变换**（P1-S4 实测修正）。Gazebo 的 `AckermannSteering`
把 `odom` 原点放在**自车 spawn 的位置**，所以这一段的正确取值是「自车 spawn 位姿
在世界里的坐标」。发单位变换等于宣称「地图原点 = 自车出生点」。

P0a 一直发的是单位变换，而 `campus_minimal.sdf` 里自车 spawn 在 `(0, −1.75)`
几乎就是原点，所以从没露过马脚 —— 那个世界里没有任何东西依赖世界坐标。
换到 `campus_loop.sdf`（自车在 `(30, −51.75)`）症状立刻出来：**RViz 里车画在
园区正中央的草地上，Gazebo 里它好端端停在南边那条路上，两边差 60 m，
而没有任何一层报错**。全局路径于是从一个错误的起点出发，看起来完全正常。

现在 `gazebo_sim.launch.py` 从**世界文件里读** spawn 位姿再发这段静态 TF
（节点名 `map_to_odom_static`）。不要在 launch 里写死那几个数 ——
写死就是给「换个世界忘了改」留一个必然会踩的坑。P4 接上定位后删掉这个节点。

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

### lint 工具链的坑（S5 实测）

格式由仓库根 `.clang-format` 定义，`colcon test` 会卡。几条不显然的：

| 坑 | 症状 | 处理 |
|---|---|---|
| `#include` 不分段 | clang-format 把三段合并按字母序排，cpplint 立刻报 include_order | **C 标准库 → C++ 标准库 → 第三方，段间留空行**。`.clang-format` 设了 `IncludeBlocks: Preserve`，只在段内排序。这是本项目唯一刻意偏离 ROS 2 官方配置的一项 —— 官方没设它，于是两个官方 linter 互相打架 |
| gtest 的 include 位置 | cpplint 报 "C system header after C++ system header" | `<gtest/gtest.h>` 按 `.h` 后缀被归成 C 系统头，**必须放在 C++ 标准库之前** |
| `struct termios saved_{}` | clang-format 拆成三行，看着像在定义结构体 | 去掉 `struct` 前缀写 `termios saved_{}`。C++ 不需要这个 elaborated type specifier |
| tf2_ros 用 `.h` | cpplint 误判成 C 系统头 | 改用 `.hpp`（Jazzy 里 `.h` 已是待淘汰的兼容 shim） |
| 手动调 `ament_clang_format()` | `add_test given test NAME "clang_format" which already exists` | package.xml 声明 test_depend 后 lint hook 已自动注册。只能用 `set(ament_cmake_clang_format_CONFIG_FILE ...)` 指配置 |
| `AMENT_LINT_AUTO_EXCLUDE` 设晚了 | 排除不生效且**不报错** | 必须在 `ament_lint_auto_find_test_dependencies()` **之前** —— 那个函数在调用时就把 linter 列表定死了 |
| 中文句号 / docstring | pep257 报 D400 | pydocstyle 只认 ASCII 的 `.`。docstring 首行用英文句点，正文照常中文 |
| cppcheck 显示 skipped | 看着有检查其实没跑 | 上游主动拒用 2.13.0（已知性能问题）。**有意保留跳过**，不要设 `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` 去覆盖 |
| 手写 C++ 的排版 | `colcon test-result` 一次报 91 处 clang_format 失败 | 别靠手写对齐。写完直接跑 `clang-format --style=file:/workspace/.clang-format -i <文件>` |
| `<tinyxml2.h>` 的 include 位置 | cpplint 报 include_order | 与 `<gtest/gtest.h>` 同理：按 `.h` 后缀被归成 C 系统头，**必须放在 C++ 标准库之前** |
| 按路径加载含 `@dataclass` 的模块 | `AttributeError: 'NoneType' object has no attribute '__dict__'`，报错**完全不提根因** | 必须先 `sys.modules[spec.name] = module` 再 `exec_module`。因为 `from __future__ import annotations` 让注解变成字符串，dataclass 要回 `sys.modules` 里查模块才能解析它们。`test_sim_source.py` 没踩到只是因为 launch 文件里没有 dataclass |

### 数值精度：同一个格式串套不同量纲是想当然（S2 实测）

`gen_map.py` 生成 `.xodr` 时一度对所有浮点用同一个 `%.6f`。坐标用 6 位小数
（微米级）绰绰有余，但**角度和曲率不是坐标**，两者各有各的机理：

- **曲率是小数值** —— R=12 m 的曲率 0.0833333… 只剩 5 位有效数字，
  相对误差 4e-6；乘上 18.85 m 弧长，终点朝向偏 6.3e-6 rad。这是**相对精度**不足。
- **朝向是力臂** —— 5e-7 rad 舍入 × 76 m 直线段 = 38 µm 位置偏差，
  且这一项**随地图变大线性增长**。这是**绝对精度**不足。

后果不是"不准"，而是**判据变脆**：C++ 与 Python 的逐点对账余量一度只剩 1.5 倍，
随时可能因为改地图而误报。改用 `%.12g`（见 `gen_map.py` 的 `precise()`）后
余量回到两三位数倍，且不再随地图规模缩水。

**这个洞是靠「打印实测最大值」而不是「只断言通过」暴露的。**
只看绿灯的话，它会一直躺到某次改地图时突然变红，而那时没人记得判据是怎么来的。

**`colcon test` 的退出码不可信** —— 它反映"测试有没有跑起来"，测试失败照样可能
返回 0。判定成败一律用 `colcon test-result --all`。只看前者的话，CI 会在测试
全红时显示绿灯，**比没有 CI 更危险**。

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
