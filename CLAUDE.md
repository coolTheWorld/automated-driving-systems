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

**教学工作区在 [`docs/learning/`](docs/learning/README.md)**（`/teach` 技能维护，跨会话有状态）。
它**不是项目文档**，是学习这套栈的课程、速查卡与学习记录；与 SPEC 冲突时以 SPEC 为准。
用户要求讲解时先读它的 `MISSION.md`（学习目标）和 `NOTES.md`（教学偏好与已知坑）。
⚠️ **若某一课发现仓库文档写错了，不要只在课里纠正** —— 按上面「修错之后必须同步」那条，
回去改 SPEC / 本文件 / 代码注释 / 长期记忆，否则同一个错误会在几个地方各躺一份。

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

### ⚠️ 本机有硬件 OpenGL，但**没有硬件 Vulkan**（2026-08-03 实测）

这两件事在 WSL 下走**完全不同的路**，极易混为一谈：

| | 通路 | 本机状态 |
|---|---|---|
| **OpenGL** | Mesa 的 **d3d12** Gallium 驱动 → `/dev/dxg` | ✅ 硬件加速（`verify_gpu.sh` 全过，Radeon 780M，16.5 GB） |
| **Vulkan** | 需要 Mesa 的 **`dzn`**（Vulkan-on-D3D12） | ❌ **只有 `llvmpipe`（CPU 软件光栅化）** |

`/usr/share/vulkan/icd.d/` 里有 `radeon_icd.json`，但 RADV 绑不上 —— WSL 不暴露
`amdgpu` 内核驱动，GPU 通路是 `dxg`。而 `dzn_icd.json` 没有。

**后果**：**CARLA 强制要求 Vulkan，所以它在本机跑不了**（llvmpipe 下是秒级每帧）。
这正是 SPEC §4.1 把环境 B 放在云端的直接原因 —— 现在这条有实测依据，不再是假设。

> **别看到 `verify_gpu.sh` 全过就以为 Vulkan 也行。** 那个脚本只验 OpenGL。
> 零安装的验法（不往宿主装 `vulkan-tools`）：用 ctypes 调 `libvulkan.so.1` 的
> `vkCreateInstance` + `vkEnumeratePhysicalDevices`，看设备类型是不是 `CPU`。

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
| **`pgrep -x gz` 查 Gazebo 残留** | **恒返回 0**，残留检查形同虚设，于是带着两套仿真跑测量 | `gz sim` 的真实进程名是 **`ruby`**（`gz` 是个 Ruby 包装脚本，`exec` 之后 comm 就变了）。查法：`ps -eo comm \| grep -cx -E 'ruby\|control_node\|planning_node'`。⚠️ 还要排掉**僵尸**：`gz` 退出后常留一个 `[ruby] <defunct>`（PPID=1，容器里 PID 1 不 reap），`comm` 照样显示 `ruby`，于是"残留 1"是假的。加 `-o stat` 滤掉 `Z` |
| **`pgrep -xc <名字> \|\| echo 0`** | 输出是 `"0\n0"`，之后任何数值比较都是语法错或恒假 | 没匹配到时 `pgrep -c` **既打印 `0` 又返回非零退出码**，于是 `\|\| echo 0` 又追一个 0。用 `$(... \|\| true)` 而不是 `\|\| echo 0` |
| **「实测超限」先怀疑分母** | 转向速率判据连报 FAIL（0.5556 / 0.5750），去查限幅器却查不出毛病 | 同一个量换三个分母给三个数：① 诊断消息**时间戳之差**（发布抖动，`cycle_time_ms` 是变的）→ 0.5556；② 标称周期中位数 → 0.5750（控制器本来就按**那一拍实际的 dt** 缩放限幅，晚到的一拍允许转更多）；③ 控制器**自己那一拍的 `dt`**（现在诊断里直接发 `dt_s`）→ **0.5000 恰好贴限**。判据要量的是"控制器有没有守住自己的预算"，**只有第三个分母在回答这个问题** |
| **判据量的是「路」不是「车」** | 修好一个问题后横向误差改善 **12.7 倍**，而「最大横向加速度」**一个数都没变** | `a_lat` 有三种算法，量的不是一回事：`v²·κ_path`（**参考路径**的，2.687）、`v²·tanδ/L`（按转角推的，2.376）、`\|v·ω\|`（**车实际受的**，横摆角速度来自 `/odom`，2.329）。原来算的是第一种 —— 路的曲率跟控制器好不好完全无关。现在 `control_node` 直接发布 `lateral_accel_mps2 = \|v·ω\|`。**这比"代码写错了"更危险：代码错了会红，判据错了红绿都不可信** |
| **`a_lat_max` 当成实测横向加速度填** | 规划限 1.5，实测 `\|v·ω\|` 峰值 **2.329** —— 判据（2.0）直接超 | 车受的横向加速度 = 路径贡献 + **跟踪修正贡献**，实测放大 **1.55–1.66 倍**。而且这个比例**随限值下降反而上升**（Stanley 的横向修正与车速无关，路径项却按 `v²` 降）。所以「除以超调比」只是一次迭代的起点，不是解析解。现值 **1.15**（实测 1.910，余量 4.5%）；1.2 时余量只有 0.5%，两次跑 1.954/1.991，**判据会 flake** |
| **纵向量用了前轴投影** | 「入弯偏快」和「停车早 2.7 m」两个看似无关的症状，跨两个模块，根因是**同一个** | Stanley 必须用**前轴**（那是它的推导前提），但轨迹点本身是"**后轴**该在哪"，所以速度目标、加速度前馈、剩余距离、到达判定全部要用**后轴投影**。用前轴等于一直读**一个轴距之后**的剖面值：减速段读到更快的目标 → 冲；末点提前一个轴距进入到达半径 → 早停。`control_node` 现在两个投影并存，各司其职 |
| 杀掉 `static_transform_publisher` 想测「TF 没了」 | `lookupTransform` 照样成功，什么都测不出来 | 静态变换走 `/tf_static`，**tf2 的 buffer 对它永不过期**。要测「拿不到 TF」只能**一开始就不发**（这也正是真实场景：节点起来了、仿真还没起） |
| **Gazebo 的转向执行机构慢得离谱**（已修） | 控制器在 L1 上完美收敛（1.6 cm），一上 Gazebo 弯道横向误差就在 ±0.8 m 之间震荡 | 开环实测：转角阶跃的 **63% 上升时间 1.20 s**，比 Stanley 自己的闭环时间常数 `1/k_e`=1.0 s 还大 —— 被控对象比控制器慢，这是震荡配方。稳态是**准的**（99.7–100.3%），只是慢，所以**定转角测转弯半径完全看不出问题，只有阶跃响应才量得到**。转向**关节**与横摆角速度的上升时间只差 0.04 s → 滞后全部来自执行机构，不是车身动力学。根子是 `AckermannSteering` 的转向 P 增益 SDF 里从没设过。修法：`vehicle_params.yaml` 加 `actuator.steer_response_time_s`，`gen_vehicle_model.py` 推导出 `<steer_p_gain> = 1/τ`（**不让人直接填增益** —— 增益换个仿真器没有对应物）。实测 1.198 → **0.294 s**，横向误差 0.801 → **0.063 m**。用 `scripts/probe_steering_response.py` 量，**P0b 对齐 CARLA 时要拿同一把尺子再量一遍** |
| **插件的转向响应有约 0.24 s 的地板** | 把 `steer_response_time_s` 从 0.050 调到 0.020（增益 20→50），实测只从 0.294 s 到 0.255 s | 关节速度环自身的滞后，P 增益动不了它。所以**那个参数不等于实测响应时间**，要知道实际多快只能跑脚本量。取 0.050 是因为它拿到了可改进量的绝大部分而增益还只有 20 —— 再往上是在跟地板较劲，代价是大误差时向关节下发上百 rad/s 的速度指令 |
| **仿真钟停走时控制器"冻住"而不是降级** | 杀掉 `parameter_bridge` 想测里程计超时的降级路径，结果**什么日志都没有** | 那个进程同时也是 `/clock` 的来源。`use_sim_time=true` 下 `now()` 不再前进 → `dt = 0` → 控制回调直接 return，"里程计过期"永远判不出来；bridge 的看门狗同样用仿真钟，也冻住 → **车保持最后一条指令**。真车上墙钟不会停，所以这是仿真特有的。要补得用**墙钟做健康检查**（那不算算法时序，不违反 SPEC §5） |
| **横向加速度超标 ≠ 横向跟得不好** | 修好转向执行机构后横向误差降了 12.7 倍，**最大横向加速度一个数都没变**（2.113） | `a_lat = v²·κ` 里有**两个**因子，横向误差不影响 κ，**只有速度会**。实测峰值处横向误差只有 −0.027 m，而车速比剖面高 +0.85 m/s；按剖面速度过那个弯 `a_lat` 恰好是 1.5002 = 限值。**看到横向加速度超标就去调横向增益，方向从一开始就是错的**。⚠️ **P3-S5 补充**：当时那个「一个数都没变」还有第二层原因 —— 判据算的是 `v²·κ_path`，量的是**路**不是车（见上面「判据量的是「路」不是「车」」那条）。结论方向没变，但根因比这里写的更深一层 |
| **拿 `path_remaining_m` 判「停得准不准」** | 冲过终点 4 m 和恰好停住给出**一模一样**的数 | 投影越过路径末点后被**夹到端点**，`s_m` 和剩余距离都不再变化。这条 `reference_line.hpp`（P3-S1 前叫 `ads_control/path_tracking.hpp`）早就写明并交给 S4 了，**照样踩了**。要判到达/冲过终点必须另算前轴到路径末点的**直线距离**（`control_node` 的 `goal_distance_m`） |
| **纯 P 速度环跟不上剖面的斜坡** | 车总是"入弯偏快、终点冲过头"，而稳态巡航时速度跟得很准 | 「纯 P 对常值目标无稳态误差」只对**常值**成立。速度剖面在入弯前和终点前是**斜坡**，一阶系统跟踪斜坡的稳态误差 = 斜率/`K_p` = 3.0/1.0 = **3.0 m/s**。解法是加速度前馈 `a = v·dv/ds + K_p·(v_ref − v)`，**不是调大 K_p** |
| **判据的适用范围没圈准** | 判据报 FAIL，而**系统其实是对的**；两者在现场长得一模一样 | CP-P3-B 一次跑里踩了两回：① 实测脚本直接拿 `/odom` 的位姿当 map 系用 —— 真实栈里 `map→odom` 是自车 spawn 位姿（本项目 (30,−51.75)），于是轨迹从 (0,0) 起、"车离障碍物 52 m"，看着像车根本没经过障碍物；② 「不越车道边线」用 `y − 车道中心y` 算，而验收路线绕过环线弯角，出了直道那个式子毫无意义，算出 −122 m 和 −3 m 这种荒谬值。**修法都是把范围读回唯一来源**（位姿走 TF；直道范围读 `obstacles.yaml` 的 `valid_from_x_m`/`valid_to_x_m`），不要自己发明过滤条件 |
| **拿 SAT 的最大间隙当"保守估计"去做判据** | 判据**假失败**，而系统其实是对的 | 分离轴的最大间隙只是真实距离的**下界**：顶点对顶点的位形下最近点连线不是任何一条边的法向。P3-S5 实测：车刚越过障碍物时真实间距 0.656 m，SAT 下界报 0.457 m，**低估 0.199 m**，判据（0.5）随之转红。「保守估计不会造成假通过」是对的，但**它会造成假失败**，而假失败同样让人去查一个不存在的问题。要精确就遍历所有（顶点, 边）对 —— C++ 侧 `distance_m()` 早就是这么做的，`test_collision.cpp` 里还有一条专门的用例说明这件事，结果我转头在 Python 测试里用了下界 |
| **末态判据在过滤后的样本里取「最后一拍」** | 「终速 0.950 FAIL」，而车其实稳稳停在障碍物前 1.9 m | block 场景的**正确**结局是：停车轨迹执行完 → 规划器发零长度轨迹 → `control_node` 判无效落到 **`NO_PATH`** 降级分支刹停并保持。实测末段 **449/799 拍是 `NO_PATH`**，车正是在这段里从 0.95 刹到 0.000。而 `score()` 把样本过滤成 `TRACKING`/`GOAL_REACHED` 再取 `[-1]`，量到的是**刹车中途**。**「跟踪质量」类的量才该过滤（车没在跟轨迹时问"它偏了多少"没有定义），「安全与末态」类的量必须在整段里算** —— 车在非跟踪状态下照样在动、照样会撞 |
| **合成地图的采样步长与 NDT 体素尺寸可通约** | NDT 配准**一步都不动**且报 converged。现象像「收敛域太小」，会把人引到调阻尼/步长上 | 点云地图杆件按 0.1 m、墙面按 0.5 m 采样，而体素 2.0 m —— **整除**。于是大量点的 z 恰好在体素边界上，而绕 z 的旋转与水平平移都不改变 z，**只有一动不动的那个位姿**能保住这些 inlier。实测位移 3e-11 m 时 inlier 从 4950 掉到 4349、得分从 −1266 跳到 −933。修法：给扫描加真实的雷达噪声（σ=1 cm）。**与下面那条同源 —— 凡是「离散结构 × 规则采样」，先查可通约性** |
| **测试的采样步长与被测结构的周期可通约** | 用例**全绿，但什么都没测** —— 故障注入进去也不红 | P3-S2 实测：验「朝向跨 ±π 不能裸插值」的用例按「总长的 1/20」= 0.6 m 步进，而参考线段长恰好 0.5 m，两者在 s=6.0 重合，**而跨 ±π 的那一段偏偏就从 6.0 开始** → 只采到 ratio=0 的端点，端点上朴素插值与正确插值给出**同一个值**。改成与段长不整除的步长（0.037）后注入立刻红。**凡是「沿某个离散结构扫一遍」的用例，步长都不能与它的周期整除** |
| 采样时 `ceil(span/step)` + 末点夹到端点 | 路径最后**两个点重合**，RViz 里完全看不出来，下游按弧长参数化时除以零 | `span/step` 恰好是整数时，浮点上它可能是 `80.00000000000001`，ceil 多算一步。改成**等分**：`offset = span·i/count`，两端精确、无需夹取。触发它的是 `nearest_lane` 的三分法把 s 细化成 `40.000000000000007` |
| **`<noise>` 的 `type` 在 IMU/NavSat 与雷达上写法相反** | IMU 写成子元素是**硬解析错误**；雷达写成属性只是警告后被静默归一化 | `lidar.sdf` **自带**一份 noise 定义（`<type>gaussian</type>` 是子元素），而 `imu.sdf`/`navsat.sdf` include 的是共享的 `noise.sdf`（`<noise type="gaussian">` 是属性）。**同一个仓库里两种写法都对，各自的地方**。查法：`gz sdf -p` 看有没有 `not defined in SDF` 警告 |
| **⚠️ Gazebo 的 GNSS 水平噪声按「度」施加，不是米** | 填 `stddev=2.0`（意为 2 m），实测静止车辆纬度在 31.67/35.15 之间跳 —— 散布 2.0 **度** ≈ 217 km | SDF 规范（`navsat.sdf`）白纸黑字写 "in units of **meters**"，gz-sensors 却直接加在 `math::Angle` 上。**垂直位置和测速都是对的，只有水平位置这一项错**。`gen_vehicle_model.py` 除以 111320 绕过。⚠️ 反向风险：Gazebo 哪天修好，噪声就缩小 11 万倍、GNSS 悄悄变完美而判据全绿 —— 用 `scripts/check_sensor_noise.py` 实测拦 |
| **`ps ... \| grep -v "Z "` 排僵尸** | 残留检查**恒报有残留**，或反过来把僵尸当活的 | `Zs`（会话首进程的僵尸）里没有 `"Z "` 这个子串。要按 stat 字段**首字母**判：`ps -eo stat=,comm= \| awk '$1 !~ /^Z/'`。与上面 `pgrep -x gz` 那条同源 —— 残留检查本身写错，比没有检查更糟 |
| **用 `pkill -f "ros2 launch ..."` 收 launch** | 两整组仿真同时在跑，`/ego_pose_gt` 上出现**两种消息类型**，话题频率 87 Hz（实际应 50） | 又是 `-f` 匹配完整命令行那条。实测这样收之后 `ps` 里还剩两个完整进程组（各自带 gz + parameter_bridge + 一堆节点）。一律 `setsid` 起、按 **PGID** 收 |
| **两个 launch 测试共用默认 ROS_DOMAIN_ID** | 两个包的闭环测试**同时**变红，而两边的代码都没问题 | colcon 是**按包并行**跑测试的（包内 ctest 才串行）。P4-S5 加了 `ads_localization` 的定位闭环之后，它与 `ads_control` 的两个闭环同时在跑，默认 domain 0 下**话题和 TF 全共享**：定位侧的「`/tf_static` 上不许有 `map→odom`」读到的是**对方**起的 `static_transform_publisher`，控制侧的 `/odom` 与 TF 同时被本方污染。修法是每个 `add_launch_test` 用 `ENV ROS_DOMAIN_ID=` 独占一个域（现分配 41/42/43）。⚠️ 加**第二个**带 launch 测试的包时才会暴露，此前一直是绿的 |
| **收尾函数按进程名算 PGID，把自己也收了** | 多场景脚本**跑完第一个就没了下文**，退出码还是 0 | 清理函数匹配了 `python3` 去算 PGID，而 `record_*.py` 正是脚本**自己进程组里的前台子进程** —— 于是 `kill -TERM -- -<自己的PGID>`。与 `pkill -f "gz sim"` 同源：**清理命令自己先被杀**。修法：`MYPGID=$(ps -o pgid= -p $$)`，收尾时跳过它。⚠️ 它还会留下上一场的整组仿真，所以**起跑前的残留守卫必须有** —— 实测正是那道守卫拦下了随后两次会被污染的测量 |
| **`<gravity>false</gravity>` 止血，止出一颗地雷** | CP-P5-B 三条判据（检测率/分类/位置）同时红，**指向感知，而错在道具** | 关重力是为了让脚本道具不下沉/不倾倒，但它**同时关掉了受扰后回到地面的唯一机制**：任何一次接触给的角速度**永不衰减**，而 `VelocityControl` 沿**车体 x 轴**推模型 ⟹ 俯仰一旦不为零，道具就斜着飞（实测 `dz/dt = v·sin(pitch)`，20 s 飞到 20 m 高，飞出雷达垂直视场上沿 +10°）。**关重力必须同时删 `<collision>`**，两件事一起做才安全。雷达不受影响 —— gpu_lidar 走**渲染**管线，看的是 `<visual>`（删完实测确认过） |
| **只对准航点的路径跟随（跟「点」不跟「线」）** | NPC 车沿直道跑，肉眼看毫无问题，几十秒后却漂进**隔壁车道** | 掉头甩出 ~1.5 m 横向偏差，而下一个航点在 84 m 外 —— 那点偏差只对应 `atan(1.5/84)=1.0°` 的朝向误差，横向收敛 0.07 m/s，**21 s 才回得来**，而那时它已经又到端点又甩一次。改成「投影到线段 + 前视 4 m」后修正 21°、收敛 1.4 m/s，**快 20 倍**。⚠️ 判据是**前视距离必须远小于航段长度**，否则退化回原样 |
| **传感器自遮挡：`range_min` 小于自车顶面半长** | 32 线雷达**整整 5 根线全方位角消失**，而任何日志都不报错；下游表现是「所有车被判 STATIC」，人会去查分类器 | 雷达装在 z=1.600、车顶顶面 z=1.500 —— 只高 **0.10 m**，而顶面前后各伸 **2.20 m**。射线在 `d = Δz/\|tanθ\|` 处打到车顶；gz 的 `range_min` 是**近裁剪面**，于是 `d < range_min` 的射线**穿过去**（好），`d ≥ range_min` 的返回一个车顶点、被自车轮廓裁掉、**整条射线作废**（坏）。被吃掉的角度带 = `[atan(Δz/L), atan(Δz/range_min)]`，**为空的充要条件是 `range_min ≥ L`**。填 0.5 时丢 5 根线、车高量成 0.4 m（真值 1.5）；改成 2.2 后 **32/32 线全回来、点数 +75%、车高 1.35**。⚠️ **抬高雷达是反效果**（Δz 变大 → 带更宽）；调 `range_min` 到 0.9 只是把带收窄，治标 |
| **判据只量「被测对象」，不量「刺激物」** | 判据表上「目标没检测到」与「目标根本不在视场里」**长得一模一样**，而排查方向截然相反 | CP-P5-B 实测：我拿着那张表查了两轮感知代码，**两次给出根因判断、两次被自己的数据推翻**，真凶是道具飞上了天。修法：打分**之前**先查刺激物（真值目标是否贴地 ≤0.10 m、直立 ≤2.0°），违反就报**「本次运行无效」并拒绝打分** —— 不能只是多印一行警告，**那张表印出来本身就会把人带偏** |
| **诊断脚本用 TF 求 map→base_link** | 真值被放到错误的位置，于是「在错误的地方找簇」，得出一个**看起来自洽**的错误结论 | `localization:=false` 下这条链路是「静态 `map→odom` + **轮速推算** `odom→base_link`」，会漂。评测/诊断脚本应当用 **`/ego_pose_gt`**（SPEC §4.1 禁的是**算法节点**订阅真值，评测脚本正是例外）。症状极隐蔽：两个目标算出来的横向坐标**互相矛盾**（同一时刻推出的自车 z 差 0.5 m），但每一个单独看都合理 |
| **参数声明得再讲究，也证明不了消费它的对象存在** | 开机 bootstrap 与失锁恢复整条死代码 **10 个月没人发现**：L1 全绿（用例自己构造粗网格）、闭环全绿（没断言它执行过） | `ndt_coarse_map_` 的参数推导、声明、消费代码全在，唯独**构造那一行缺失**——空指针短路，恢复"实测 0 次触发"被归因成「粗网格同样退化」。修法之外的守卫：L3-G 机械断言 `ndt_recovery_attempts ≥ 1`。**凡是"参数 → 对象 → 消费"三段式，断言要打在最末端的可观测行为上** |
| **先喂狗后校验** | 上游持续发 NaN 时看门狗**永不触发**，车以闩存旧指令一直开——恰是看门狗要防的后果 | `last_cmd_time_ = now()` 曾是 on_cmd 第一行，被丢弃的坏指令照样喂狗。**持续 NaN 流在语义上等价于失联**（没有任何有效指令到达），喂狗必须在校验之后。check_vehicle_cmd.py 抓不到它（只断言输出无 NaN，闩存值恒有限）——判据要断言语义不是值域，现由 `test_vehicle_cmd_bridge_watchdog.py` 守着 |
| **判据只有外界没有内界** | 扩窗实测：自车过弯贴行人 2.2–2.8 m 经过，连续 11 帧"漏检"——按判据算 FAIL，而那是 `range_min=2.2` 的**近场盲区**（74d12ad 修复时文档写明的代价） | 与「>30 m 物理上不可检」同理：判据范围两头都要由物理定。现内界 3 m。同批教训：符号判据要过跟踪器自己的 `heading_min_speed` 门限（刚建的航迹速度是噪声）；ID 连续性只在中断 ≤ 删除+滑行设计窗口（3.5 s）内判——**系统设计上保不住的中断换 ID 是正确行为不是缺陷** |
| **改了 `ads_msgs` 只重建了直接相关的包** | planning_node **段错误**（exit −11），现场是「自车不动」——目标点、路由、感知全正常，人会去查规划 | 消息布局变了，没重建的下游带着**旧类型支持**反序列化新消息 → 内存错位。改 msg 后一律 `colcon build --packages-above ads_msgs`（P6-S4 实测，Obstacle.msg 加一个 bool 就炸） |
| **QoS 只设 durability 不设 reliability** | 发布端 `get_subscription_count()=1`（配上了！）却**静默不投递**，连发 45 次都收不到 | reliability 落默认值与订阅端 RELIABLE 不兼容——DDS 配对成功≠投递兼容。发布 `/goal_pose` 这类单次指令必须显式 `RELIABLE + TRANSIENT_LOCAL`（照抄 record_control_run） |
| **评测的段归属按瞬间判，不按评测窗判** | 直行段判据的尾巴全是「窗口里其实在过弯」的帧（FDE 5–13 m），人会去查直线外推 | 凡是「t 时刻的预测对 t+h 的真值」类评测，样本属于哪个工况要看 **[t, t+h] 整个窗**（三点采样）。同族：U 转排除窗要用 1 s 滑窗，整窗 45° 会把合法过弯（22°/s×3s=67°）也排除掉 |
| **`declare_parameter<std::vector<double>>(name, {})` 的 `{}`** | launch 不传该参数时节点 FATAL「must be initialized」——而**传了参数的场景全绿** | `{}` 被 rclcpp 解释成 `ParameterValue{}`（NOT_SET），不是空数组。要写 **`std::vector<double>{}`**。P7-S4 实测：npc_controller 的 dwell_s 这么写后，P5/P6 场景（不传）道具全瘫、P7 场景（传）一切正常 —— **「从没吃过默认值的默认值」等于没测过**，改共享节点的参数接口时，不传参数的场景也要跑一个 |
| **模块输出的病理在没有消费者时是隐形的** | P6 双层验收全过；P7 行为层（第一个消费者）上线**当天**，预测里的建筑片段假 CV（滑移锚点的**真实**位移骗过位移一致性闸）让车每根柱子让一次行 | FDE 判据只打分目标车/行人，路侧航迹的假预测**不进任何判据** —— 验收「全过」只覆盖被打分的那部分输出。接新消费者的第一轮实测要当成对上游的**再验收**；P6 由此补了 ODD 尺寸闸（length>5.5→STATIC） |
| **拿 ODD 包络给 L-Shape 剖面设门（云窗口只在一个环境里验）** | 三个行为场景**感知层**同日全红（跟停 −5.19 m 撞车 / 横穿 0.01 m / 路口 0.00 m），真值层全绿；而这刀在 CARLA 上绿了两天 | P9-S2 剃刀门 `min(l,w) < 0.1` 剃 CARLA 接缝残留条，注释写「Gazebo 合法目标远在门上」——错在 L-Shape 量的是**可见剖面**不是物体：正对盒状目标深度只剩雷达噪声，Gazebo 车尾面 `min(l,w)` 实测 p50 0.047，**每帧被吞**；CARLA 网格有曲面所以没露马脚。修法是把先验说对（剃刀条**一维** = 薄且矮，正对目标薄但高 → 二条件门），不是调门。**过程教训**：plan 风险表早写了「每刀带两环境回归」，云窗口里十刀一刀没回归 —— 每一刀在本地都是 2 分钟的事，攒到一起是一天的三场全红 + 一次追凶。见 perception.md §3 |
| **清理按节点名枚举，名单挂一漏万** | 孤儿 `obstacle_truth` 带着旧场景世界观存活数小时，幻影约束与真规划**秒级交替发布**——车微冲-停爬行、在停止线前 3.8 m 外「自己停了」，五轮排查全在查灯链路/规划/控制，而它们都没病 | P8-S6 实测（CARLA 云端）：轮次脚本 cleanup 列了 map/planning/control 却漏了 obstacle_truth，`pkill ros2 launch` 留下它成孤儿（PPID=1），下一轮的 `/perception/obstacles` 就有两个发布者。与「两套仿真并存」同族：这是 ROS 层的**两套世界观并存**，所有行为测量作废。修法：① 清理按**安装路径前缀**杀（`install/ads_[a-z]*/li[b]` 等三条），不按节点名枚举；② 清完必须有**残留守卫**（活进程非零就拒跑，僵尸除外 —— `ps -eo stat=` 首字母 Z，容器 PID 1 不 reap，defunct 无 DDS 存在无害）。症状特征：诊断在「正常/N 点」与「绕不过去/1 点」间交替 = 先查发布者数量，别查算法 |

**「频率低 = 算力不够」是最容易犯的想当然。先分层测量再动参数** ——
S3 时据此砍掉一半激光雷达分辨率，结果只快了 4%，因为病根是 QoS 不是 GPU。

**"某个显示项没数据" 先怀疑数据源死了，而不是显示端配错了。** 上面头两条就是同一次
故障的两层：表象在 RViz，根因在 GPU 驱动，中间还隔着一层"launch 谎报平安"。
排查顺序永远是**沿数据流往上游走**：RViz → 话题有没有 → 发布者在不在 → 仿真器在不在。

---

## 常用命令

`src/` 下**目前有十三个包**：`ads_msgs`、`ads_common`、`ads_map`、`ads_control`、
`ads_planning`、`ads_localization`、`ads_perception`、**`ads_prediction`**、
`ads_bringup`、`ads_simulation/gazebo_bridge`、**`ads_simulation/carla_bridge`**
（P8-S4，仓库第一个 ament_python 包）、`ads_teleop`、`ads_visualization`。

`ads_prediction`（**P6，CP-P6-B 已达成** 2026-08-12）有三个 lib：`motion_model`
（恒速 + 静态 + 不确定椭圆，全解析）、`lane_follow`（nearest_lane 归属 →
successors 枚举链 → ReferenceLine 弧长参数化，路口分叉多假设、末端如实截断）、
`model_selector`（速度/ODD 上限/**位移一致性**/尺寸档位/**ODD 尺寸上闸
length>5.5→STATIC（P7-S4 补：建筑片段的滑移锚点有「真实」位移，位移一致性
闸原理上拦不住 —— 第一个消费者上线当天暴露）** —— **不用 classification**，
TargetSnapshot 结构上没有那个字段）。视界 **6 s**（P7-S4 从 3 延长：让行
可见时界必须 ≥ 自车清空冲突区时长；FDE@3s 判据照旧在 t=3 采样，抽轮零劣化）。推导、参数与 S4 仪器五课见
[docs/modules/prediction.md](docs/modules/prediction.md)，**改这个模块前先读它**。
三条硬规则：运动方向**永远取速度矢量不取 yaw**（180° 二义）；声称的速度必须有
**净位移背书**（结构物假速度占 24.5% 的实测病理）；**决策二仅此一例** ——
prediction 链接 libads_map 读静态先验（SPEC §3.3 注解），不要据此推广。
CP-P6-B 是**双层协议**：`--layer truth` 全过再跑 `--layer perception` 两轮
（层 2 红层 1 绿 = 感知输入的传播，去查感知不改预测）。

`ads_perception`（**P5，CP-P5-B 已达成**）有六个 lib：`ground_segmentation`（RANSAC +
坡度校验）、`euclidean_cluster`（体素哈希 + BFS）、`lshape_fit`（closeness 准则，
给的是**轴向**不是朝向）、`size_classifier`、`hungarian`（O(n³)，**禁用配对要用
有限大数不能用 inf** —— `inf−inf=NaN` 会死循环）、`tracker`（恒速 KF + 匈牙利关联 +
航迹生命周期 + **包围盒补全/重锚/去重**）。
推导、参数与边界见 [docs/modules/perception.md](docs/modules/perception.md)，**改这个模块前先读它**。
其中 §0 那张表最要紧：**P5 的五个根因没有一个在最初怀疑的地方**
（第五个是 P6-S0 钉死的车道内幻影 —— 台账写的「欠分割污染尺寸记忆」也错了，
真身是「补全/重锚虫洞焊出假速度航迹 × 遮挡滑行」，见 perception.md §6.1）。
⚠️ `tracker` 里三条实测结论极反直觉，改它之前先读 `tracker.hpp` 的文件头：
**滤包围盒中心是错的**（中心不是目标身上的固定点，随观测几何漂移 2.2 m，
卡尔曼会把它读成 8.8 m/s 的假速度）、**L-Shape 的「长/宽」命名与目标无关**
（正对时"长轴"指的是车宽，轴向翻 90°，要换说法而不是放弃）、
**重复航迹是独立于 ID 跳变的缺陷**（规划会把一个目标当成两个障碍物）。

`ads_planning`（P3 + **P7 行为决策，CP-P7-B 已达成** 2026-08-13）有九个 lib：
P3 的六个 —— **`frenet`**、**`quintic`**、**`collision`**（OBB 判交 + 精确间距）、
**`lattice`**、**`speed_profile`**（曲率限速 + 前后向扫描 + **P7 的可选
`speed_caps_mps` 逐点上限**）、**`trajectory`**（装配 + 停车剖面 + **P7 的可选
纵向约束注入** —— stop_at 走「几何截断 + terminal=0」的同一条停车路径）；
P7 的三个 —— **`conflict`**（ego 路径时间标注 + 两类冲突：自车道类用**感知
近边 + 阻挡判据**（与 planning.md §6 可行性不等式同一个不等式），横穿类用
预测轨迹 + min(2σ, 3.5) 膨胀 + 时间窗重叠）、**`behavior_tree`**（四节点微型
树，显式写在代码里）、**`longitudinal`**（约束合成取 min + 仲裁滞回）。
推导与 S4 实测改掉的六处见 [docs/modules/behavior.md](docs/modules/behavior.md)，
**改行为层前先读它**。三条结构性红线：树只选标签不裁决约束；约束合成在
树外取最保守；静态准入检查原样保留 —— 树上**不存在**关掉碰撞检查的分支。
⚠️ 两条最反直觉：**FOLLOW 判「阻挡」不判「在车道里」**（否则 P3 的可绕
锥桶被当前车，车停住不绕）；**STATIC 预测不进横穿判定**（起步律椭圆会把
路侧静物变成永久让行 —— STATIC 的威胁是位置性的，归阻挡/准入管）。
`node/planning_node.cpp` 订阅 `/prediction/trajectories`（P7-S3 起 prediction
开关**不再是纯旁路**），`expect_perception`/`expect_prediction` 由 launch
随开关置位 —— 链路该在而从未到达时指名报错不发轨迹。

⚠️ **P4-S4 起 `map→odom` 由 `localization_node` 动态发布**（`localization:=true` 时），
仿真侧那条来自 spawn 位姿的静态 TF 同时关掉 —— **两者不能同时发**。
⚠️ 同时发**不报错，而且数值上不一定看得出来**（L3-G 故障注入实测：多挂一个静态的之后
末段位置误差只从 0.012 m 变成 0.043 m，判据全绿）。危险在于**哪一份生效取决于启动顺序**：
tf2 在某个 frame 第一次被写入时就定死它用静态缓存还是时间缓存。
所以 `test_closed_loop_localization.py` **机械地查 `/tf_static` 上有没有 `map→odom`**。
默认仍是 `false`（CP-P2-B/CP-P3-B 的回归基线建立在真值 TF 上）。

⚠️ **P3-S4 起数据流变了**：`map_node → /route/path → planning_node →
/planning/trajectory → control_node`。`control_node` **不再订阅 `/route/path`、
也不再自己算速度剖面**。改动涉及它们时别照旧图施工。
推导、参数与边界见 [docs/modules/planning.md](docs/modules/planning.md)，**改这个模块前先读它**。
其中 §6 那个可行性不等式最要紧：**车道 3.5 m 装 1.8 m 的车再留 0.5 m 间距，
障碍物左缘必须 ≤ −0.75 m 才绕得过去**（P8-S6 起 margin=0.7；0.5 时代是 −0.55
—— CARLA 轮胎侧偏把实测间距打到 0.486，margin 加厚且必须跳满一个网格步长 0.2
才起作用，见 planning.md 翻案记录）—— 随手把障碍物放在车道中间是几何上无解的，
所以 P3 交付「可行则绕、不可行则停」**两个**能力。

`ads_common` 是**唯一允许被多个模块共用的包**，目前三样东西：`angles`（角度归一化/差）、
**`reference_line`**（折线的弧长参数化、逐点曲率、最近点投影；P3-S1 从
`ads_control/path_tracking` 下沉而来）、**`numeric_checks`**（`RequireFinite` 三兄弟；
P3-S4 从 `ads_control/src` 下沉 —— 它自己的注释写着"本包里有三处要做同一件事"，
而到 S3 已经变成跨两个包**五处**）。
放这儿的判据不是「感觉通用」，而是**有没有第二个消费者、且它有逻辑**：
控制侧把投影当横向/航向误差，规划侧把同一个投影当 Frenet 的 (s, d)。
反例是 `Pose2D` —— 它在 `ads_map` 和 `ads_common` 里**有意各留一份**，
因为它没有逻辑，不存在「改出分歧」的可能。**没有逻辑的类型可以复制，有逻辑的必须共用。**

`ads_map` 已有完整两层：`lib/`（OpenDRIVE 解析 + 几何求值 + 车道级有向图 +
Dijkstra 路由，纯 C++ 无 ROS）和 `node/map_node.cpp`（ROS 包装层，P1-S4）。
`libads_map.so` 只链接 `libtinyxml2` 和 `libads_common`，**零 ROS 依赖** ——
这条由 `scripts/verify_map.sh` 的 `ldd` 检查机械保证，不靠纪律。

`ads_localization`（**P4 已验收**）有四个 lib：**`eskf`**（15 维误差状态卡尔曼滤波，
中值积分 + GNSS 定位/测速 + 轮速 + 位姿观测，**CP-P4-A 达成**）、
**`geodetic`**（经纬高 → map 系，公式与 gz-math 逐点对账）、**`ndt`**（体素高斯地图）、
**`ndt_align`**（6 自由度配准 + 退化检测）；`node/localization_node.cpp` 发布动态
`map→odom`，CP-P4-B 七条判据连跑两轮全过（横向 0.129/0.087 m，判据 0.30）。
⚠️ **2026-08-12 复检改写了这一段的历史结论**（此处原来写「失锁恢复无能为力
（粗网格同样退化）」「结构稀疏残余」—— **两个归因都错了**）：真相是
`ndt_coarse_map_` **从未被构造**（参数推导、声明、消费代码全在，唯独构造
那一行缺失，空指针短路），开机 bootstrap 与失锁恢复整条是**死代码**，
`coarse.degenerate` 根本没被观测过。修复后六轮实测全过（横向 0.094–0.111 m，
判据 0.30），每轮 bootstrap 实测把 0.2–0.98 m 的开机误差拉回。
2026-08-11 前的 14.3% 间歇失败（14 轮 2 失败）与开机锁错机理一致；
六轮全过尚不构成统计学结论（P(0/6|p=0.143)≈0.40），长程统计确认挂 P8。
教训已提炼进陷阱表：**参数声明得再讲究，也证明不了消费它的对象存在** ——
L3-G 现在机械断言 `ndt_recovery_attempts ≥ 1` 守着这条。
⚠️ **`ndt.max_innovation_m`（3.0）不是普通调参项，是安全阀。**
NDT 到迭代上限仍会返回位姿，而它的协方差说的是「代价函数在这里有多陡」，
**不是「离真解有多远」** —— 半收敛的位姿带着毫米级协方差会把滤波器拽跑。
阈值来自三轮实测（新息中位数 0.027 m、峰值 1.18/0.61/0.56，而已观测的锁错是 28 m 和 462 m）。
⚠️ 它**防灾难不保精度**：3 m 的锁错早就违反 SPEC 的 0.3 m 了，它拦的是随后那个正反馈。
0.5 m 量级的锁错原计划由严格卡方门拦（P8-S2c）—— **逐帧拦截三版全部被实测否决**
（硬拒 0.19/0.20、步长封顶 0.24，基线 0.09–0.13）：0.4–0.6 m 那段住的全是合法居民
（NDT 退化段后的纠偏帧、弯道段的帧间散布），**0.5 m 尺度上好帧与坏帧没有空档**，
单看新息分不开「NDT 跳了」和「滤波器漂了」。且逐帧拦截对防跑飞并非必要 ——
0.5 m 单帧偏差远小于体素 2 m 盆地尺度，下一帧自己散掉。最终形态：**d² 只做监视
（`ndt_chi2_exceed`）+ 连续 ≥10 帧超门强制粗网格重锚仲裁**（持续性锁错可防，注入
实测 32 次触发）。⚠️ 监视器被误配时闭环判据**照样全绿**，唯一可观测症状是计数器 ——
L3-G 断言 ③c 机械地查它。CARLA 上（世界与地图不同源）地板/门限要重量。
见 localization.md §10.6b —— 那一节按时间记录了否决过程，**比结论更有用**。

⚠️ NDT 有两条实测结论极反直觉：**必须求和 3×3×3 邻域体素**（只用包含点的那一个时
代价函数是锯齿状的，牛顿步方向完全正确却一步走不动）、**退化检测必须看法向散布
而不是信息阵条件数**（纯地面上信息阵报出 σ_x = 8.7 mm，条件数只差 3 倍；
法向散布差 6 个数量级）。见 docs/modules/localization.md §9。`libads_localization.so` 只链接 Eigen（头文件库，
不产生 DT_NEEDED），**零 ROS 依赖**，由 CTest 机械保证。
推导与参数见 [docs/modules/localization.md](docs/modules/localization.md)，**改这个模块前先读它**。
其中 §6「零偏可观性」最反直觉：**零偏能不能估出来取决于车怎么开**，
匀速圆周上跑 1000 s 相对误差反而涨到 2.07，而八字形上 400 s 就收敛到 0.078 ——
看到「零偏估不出来」先确认那条轨迹上它可不可观，不要去调 Q。
§4.3 另有一条：轮速观测的姿态雅可比在无侧滑时**恒为零**，删掉它漂移反而更小。

`ads_control`（P2 已完成）目前有**两个** lib（`speed_profile` 已于 P3-S4 搬走）：**`stanley`**（前轴换算 + 控制律 +
转角/转向速率双重限幅）、**`speed_controller`**（速度环 PI + 条件积分抗饱和）。
`node/control_node.cpp`（S4）已跑通闭环，**CP-P2-B 8/8 达标**（2026-08-03 两遍复现；
P3-S5 修完前轴/后轴投影与 `a_lat` 判据后**再次 8/8**，2026-08-04）。`libads_control.so` 同样零 ROS 依赖，且**不依赖 `ads_map`** ——
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
# P3 验收场景的静态障碍物。**默认 none** —— 那是 CP-P2-B 的回归基线，
# 世界必须与 P2 时一模一样，所以障碍物是**运行时注入**的，不进 campus_loop.sdf。
ros2 launch ads_bringup stack.launch.py obstacles:=avoid   # 贴边锥桶，应当绕过去
ros2 launch ads_bringup stack.launch.py obstacles:=block   # 车道中心锥桶，应当停住
# P7 行为场景（S1 起）：dynamic:=follow / crossing / junction（前车跟停 /
# 行人横穿 / 无信号路口车流让行）。P5/P6 的 oncoming/cross/both/curve 照旧。
ros2 launch gazebo_bridge gazebo_sim.launch.py                      # 只起仿真侧，调链路时更快
ros2 run ads_map map_node                                           # 只起地图节点（不需要仿真器）

# ⚠️ stack.launch.py 的默认世界是 **campus_loop.sdf**（P1 起），
#    而 gazebo_sim.launch.py 的默认仍是 campus_minimal.sdf —— 这不是笔误：
#    三个 verify 脚本都不传 world、直接吃后者的默认值，它们的实测基线
#    （RTF 0.970、点云 10.00 Hz…）全部建立在 campus_minimal 上。

# 键盘开车（**另开一个终端**，必须走 drive.sh 而不是 ros2 launch，见陷阱表）
docker compose exec dev /workspace/scripts/drive.sh

# ---------- 测试与 lint（提交前跑） ----------
colcon test && colcon test-result --all      # 全量：lint + L1 + L3-G 闭环（当前 732 tests，约 35 s）
colcon test --packages-select ads_common     # 单个包
./build/ads_common/test_angles               # 直接跑 gtest，快一个数量级，日常改代码用
./build/ads_map/test_geometry                # 参考线几何 vs 解析解
./build/ads_map/test_opendrive_parser        # 解析 + **CP-P1-A 双实现逐点对账**
./build/ads_map/test_lane_graph              # 车道图：18 节点 / 24 边 + 每条边的几何连续性
./build/ads_map/test_routing                 # Dijkstra：路径与长度 vs 穷举脚本
./build/ads_common/test_reference_line       # 弧长/曲率/最近点，全部 vs 解析解（P2-S1 写，P3-S1 下沉至此）
./build/ads_control/test_stanley             # Stanley + **CP-P2-A 闭环收敛判据**（P2-S2）
./build/ads_planning/test_speed_profile      # 曲率限速 + 前后向扫描（P2-S3 写，P3-S4 搬来）
./build/ads_control/test_speed_controller    # 速度环 + 抗饱和 + bridge 饱和环节（P2-S3）
./build/ads_planning/test_frenet             # Frenet ↔ 笛卡尔，闭式解 + 往返一致性（**CP-P3-A**）
./build/ads_planning/test_quintic            # 五次多项式：六个边界条件逐项复现（P3-S3）
./build/ads_planning/test_collision          # OBB 判交 + 间距，含相切/包含/退化（P3-S3）
./build/ads_planning/test_lattice            # 横向采样 + **§6 可行性不等式的两侧**（P3-S3）
./build/ads_planning/test_trajectory         # 轨迹装配 + **绕不过去时的停车剖面**（P3-S4）
./build/ads_localization/test_eskf           # ESKF：闭式解对账 + **CP-P4-A 四条判据**（P4-S2）
# ⚠️ 零偏可观性取决于**车怎么开**，不取决于滤波器写得好不好。
#    看到「零偏估不出来」时先跑这个默认不跑的对照，别上来就调 Q：
./build/ads_localization/test_eskf --gtest_also_run_disabled_tests \
    --gtest_filter='*BiasObservability*'     # 匀速圆周 vs 八字形，实测相对误差差 20 倍

# L3-G：**不需要 GPU** 的端到端闭环（假车 + map_node + control_node），已进 CI。
# 验的是节点接线（话题/QoS/TF/参数/时序），**不是控制律也不是真物理**。
# 它抓得住 /route/path 那个 QoS bug —— 退回 volatile 立刻红。
colcon test --packages-select ads_control --ctest-args -R test_closed_loop
# L3-G 定位：**不需要 GPU** 的定位闭环（假传感器 + localization_node），已进 CI。
# 假传感器从先验点云 campus_cloud.pcd 里抠扫描，所以世界与地图**同源** ——
# 没有模型失配、没有遮挡、没有运动畸变。它验的是接线，**不是精度**。
# 判据只到「链路通、末态 NDT_AIDED、误差没发散（0.60 m）」，收紧到 CP-P4-B
# 的 0.30 m 得到的不是更强的保证，而是一个在 CI 上随机变红的测试。
colcon test --packages-select ads_localization --ctest-args -R test_closed_loop_localization
# ⚠️ 它有两个**实测抓不到**的东西，写在文件头的故障注入表里：
#    ① 杆臂补偿（NDT 一锁上 GNSS 就没权重了，0.5 m 偏差被压进噪声）
#    ② 多一个 map→odom 发布者的**数值症状**（误差只从 0.012 → 0.043 m）
#    —— 所以后者是**机械地查 /tf_static**，不是看误差。

# 其中 test_closed_loop_obstacle 是 **P3 唯一能进 CI 的绕障验收**：
# obstacle_truth 只从参数发布、不需要 Gazebo，所以整条
# 「假车 + map_node + obstacle_truth + planning_node + control_node」链路能在 CI 里跑。
# 判据是 SPEC §8 S04 的「侧向间距 > 0.5 m」，量的是**车体外廓**（不是轨迹点）。

# ---------- 闭环实测（需要真仿真器，进不了 CI）----------
# 先起 headless 全栈，再跑记录脚本；判据来自 plan.md 的 CP-P2-B 表，脚本里不重新发明
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false
python3 scripts/record_control_run.py --goal 91.75 20.0 --out /tmp/run.csv
# CP-P3-B 的两个障碍物场景（判据见 plan.md「P3-5 检查点」）。
# ⚠️ 两个脚本都要跑：record_control_run 验**跟踪质量**（CP-P2-B 那 8 条），
#    record_obstacle_run 验**避障行为**。绕对了但把跟踪弄坏了，同样不算通过。
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false obstacles:=avoid
python3 scripts/record_obstacle_run.py --scenario avoid --out /tmp/p3_avoid.csv
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false obstacles:=block
python3 scripts/record_obstacle_run.py --scenario block --out /tmp/p3_block.csv
# CP-P4-B：**关掉真值 TF**，全程用估计位姿。两个记录器同时跑 ——
#   record_control_run   驱动（发目标点）+ CP-P2-B 的 8 条
#   record_localization_run  只监听，记定位误差（对 /ego_pose_gt）
# ⚠️ **必须连跑至少两轮** —— 实测有一轮拿到 0.0664 m，下一轮同配置得到 78 m。
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false localization:=true
python3 scripts/record_localization_run.py --duration-s 55 --out /tmp/loc.csv &
python3 scripts/record_control_run.py --goal 91.75 20.0 --out /tmp/ctrl.csv
# ⚠️ record_control_run 在 localization:=true 下量的是「**估计位姿**相对路径」——
#    定位整体偏移时控制看起来完美而车实际偏了。它单独不成立，必须配定位判据看。
# CP-P5-B：感知验收。⚠️ **记录器必须早起**（仿真 t≈13 起、时长 72 s）——
#   域内遮挡发生在 t≈15–17（车第一趟经过挡住行人），晚起 40 s 的旧协议整个错过，
#   遮挡判据只会报「窗口 0 个：场景没激励」。目标点在记录开始后 ~28 s 再发。
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false perception:=true dynamic:=both
python3 scripts/record_perception_run.py --duration-s 72 --out /tmp/p5.csv   # 早起！
python3 scripts/record_control_run.py --goal 91.75 20.0 --out /tmp/p5_ctrl.csv
# CP-P6-B：预测验收，**双层协议**（决策七）。层 1（真值输入）全过再跑层 2 两轮。
#   记录器等仿真钟 ≥10 起；目标点要在 sim≈37–41 生效 —— ⚠️ record_control_run
#   的启动链在高负载下会把目标点拖后 ~20 s（rclpy 冷启动 + 内部延迟），
#   自动编排要用**预热的 goal 发布器**（RELIABLE+TRANSIENT_LOCAL，见陷阱表）。
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false prediction:=true dynamic:=curve
python3 scripts/record_prediction_run.py --duration-s 72 --layer truth --out /tmp/p6.csv
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false perception:=true prediction:=true dynamic:=curve
python3 scripts/record_prediction_run.py --duration-s 72 --layer perception --out /tmp/p6p.csv

# CP-P7-B：行为决策验收，双层协议（真值层全过 → 感知层；⑨ 有感知层照印条款）。
#   goal 由预热发布器在绝对仿真钟 37.0 发；junction 的 goal 是 (-1.75,-30)
#   （横穿路南行车道 —— 路由唯一解经 j_north 左转，让行几何由地图保证）。
ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false prediction:=true dynamic:=follow
python3 scripts/record_behavior_run.py --scenario follow --layer truth --duration-s 90 --out /tmp/b.csv
#   场景 = follow / crossing / junction；层 truth = perception:=false，perception 层 = true。
#   回归 ⑩（CP-P2-B/P3-B/P6-B 抽轮）用各自已有的记录脚本，别重新发明。

# 被控对象辨识（开环，控制器不参与）—— 车必须停在**路面上**再跑
python3 scripts/probe_steering_response.py --step 0.30 --speed 4.0

# ---------- P0b 方案 B：CARLA 最小对齐验证（需要云 GPU，本机跑不了）----------
# 上机手册见 docs/p0b_minimal_alignment.md。**本机只有软件 Vulkan**，见环境陷阱表。
python3 scripts/carla_align_vehicle.py --dry-run        # 验单位换算，不需要 CARLA
python3 scripts/carla_align_vehicle.py --host 127.0.0.1 --blueprint vehicle.citroen.c3
# ⚠️ 蓝图用 **citroen.c3**（轴距 2.684，差 0.6%），不要用 tesla.model3（3.005，差 11.3%）——
#    轴距 apply_physics_control 改不了，只能换蓝图，而它直接进 Stanley 的前轴换算。
# 2026-08-03 实测：CARLA τ=0.140 s vs Gazebo 0.294 s（更快 = 安全方向），
#    稳态达成率 86.3% vs 100.7%（轮胎侧偏，本质差异）→ k_e=1.0 可以搬。

# ⚠️ 判定成败**必须**看 colcon test-result，不能只看 colcon test 的退出码 ——
#    后者反映的是"测试有没有跑起来"，测试失败它照样可能返回 0。

# ---------- 尚不可用（还没做到那一步） ----------
ros2 launch ads_bringup stack.launch.py sim:=carla  # P8-S4 起有：起 carla_bridge 侧
#   （sidecar + lidar_preprocessor 复用 + RSP）。⚠️ 本机没有 Vulkan 跑不了
#   CARLA 服务端 —— sidecar 连不上 :2000 会指名报错。上机手册 docs/p8_carla_bringup.md
```

### 模型生成与单项检查

```bash
# 从 YAML 重新生成 SDF(Gazebo) + URDF(ROS)。改了 vehicle_params.yaml 必跑
python3 scripts/gen_vehicle_model.py
python3 scripts/gen_vehicle_model.py --check    # 校验生成物是否与 YAML 同步（CI 用）

# 从 YAML 重新生成地图。改了 campus_map.yaml 必跑 —— 它有**五个**生成物：
#   maps/campus.xodr（两环境共用）、models/campus_road/model.sdf（Gazebo 可视）、
#   models/campus_structures/model.sdf（路侧杆件与建筑，**P4 定位的前提**）、
#   maps/campus_cloud.pcd（NDT 的先验点云地图，与上一个同源）、
#   src/ads_map/test/data/reference_samples.csv（给 C++ 做逐点对账的基准）
python3 scripts/gen_map.py
python3 scripts/gen_map.py --check              # 五个生成物逐字节比对

# ⚠️ 后两个生成物为什么必须存在（P4-S1 挖出来的）：
#    加它们之前，campus_loop 里所有点云的法向都是 (0,0,1)（ground_plane +
#    0.01 m 厚的路面板，而板厚恰好等于雷达噪声 stddev）。点云对位姿的约束
#    来自表面法向，一个平面只约束沿它法向的那一个自由度 —— 于是 NDT 只能
#    定住 z/roll/pitch，**x/y/yaw 完全不可观**，配准会「收敛」到任意位姿。
#    这与 P3「障碍物放在车道中间几何无解」同类：**是场景设定问题，不是代码问题**，
#    而症状会是「NDT 怎么调都飘」→ 所有人去查配准代码。
#    杆件与点云是**同源**的（同一个 build_structures()）—— 两份漂移的症状是
#    NDT **稳定地收敛到一个错误的位姿**：残差小、协方差紧、毫无报警，
#    比不收敛危险得多。

# 实测 IMU/GNSS 的噪声是否真的生效（**需要一个正在跑的 Gazebo，车必须静止**）
python3 scripts/check_sensor_noise.py
# ⚠️ 它守着一个具体的坑：Gazebo 把 GNSS 水平噪声按「度」施加（见陷阱表）。
#    gen_vehicle_model.py 为此除以 111320；哪天 Gazebo 修好，那次换算会把
#    噪声缩小 11 万倍 —— GNSS 悄悄变回完美真值，而 P4 判据会全部变绿。

# 从 YAML 重新生成 P3 验收场景的障碍物。改了 config/obstacles.yaml 必跑。
# ⚠️ 它不只是生成：会**机械校验 planning.md §6 的可行性不等式**（按规划器的
#    采样网格判，不只是连续不等式）并与 YAML 里的 expect 对账，对不上拒绝生成。
#    那个不等式的余量只有零点几米，挪 0.3 m 就可能让「可绕」变成实际无解，
#    而症状是实测时车停住、所有人去查规划器 —— 错却在场景设定。
python3 scripts/gen_obstacles.py
python3 scripts/gen_obstacles.py --check       # 已进 CI

# 从 YAML 重新生成 P5 动态目标（NPC 车 + 行人）模型。改了 config/dynamic_actors.yaml 必跑。
# 三条机械校验（航点在直道范围内 / 外廓不侵入自车车道 / 场景引用存在），各注入验红过。
# ⚠️ 行人起点 x=30 是实测几何倒推的（域内遮挡要发生在可跟踪窗口里），不要随手挪。
python3 scripts/gen_dynamic_actors.py
python3 scripts/gen_dynamic_actors.py --check  # 已进 CI（2026-08-12 补接——此前自称"CI 用"却没接）
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
/ego_pose_gt       nav_msgs/Odometry           真值位姿+速度，map→base_link（P4-S1 实现）
/vehicle_cmd       控制指令（转角 rad + 加速度 m/s²）
```

⚠️ **`/ego_pose_gt` 仅供评测打分与实测脚本使用，算法节点一律禁止订阅。**
P4 的全部定位判据都拿它当基准，所以它必须存在；但只要有一个算法节点偷偷订阅了它，
整个定位模块的验收就变成**自己跟自己比** —— 而它看起来仍然是绿的。
这与 `reference_samples.csv` 那条警告同类：一份「标准答案」流进被考核的一方，
考试就失去意义。它来自 `OdometryPublisher` 插件（直接读物理引擎的真实位姿），
**不要与 `/odom` 搞混** —— 后者是 AckermannSteering 的轮速推算，会漂移，
正是 P4 要修正的对象，而两者都是 `nav_msgs/Odometry` 类型。

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
| 多行 docstring 摘要写在第一行 | pep257 报 **D213** | 本仓库启用的是 D213（摘要在**第二行**），与常见的 D212 相反。多行的写成 `"""` 换行 → 摘要 → 空行 → 正文；单行的不受影响 |
| cppcheck 显示 skipped | 看着有检查其实没跑 | 上游主动拒用 2.13.0（已知性能问题）。**有意保留跳过**，不要设 `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` 去覆盖 |
| 手写 C++ 的排版 | `colcon test-result` 一次报 91 处 clang_format 失败 | 别靠手写对齐。写完直接跑 `clang-format --style=file:/workspace/.clang-format -i <文件>` |
| `<tinyxml2.h>` 的 include 位置 | cpplint 报 include_order | 与 `<gtest/gtest.h>` 同理：按 `.h` 后缀被归成 C 系统头，**必须放在 C++ 标准库之前** |
| **`<Eigen/Core>` 的 include 位置** | 同上，cpplint 报 5 处 include_order | 同一个坑的**第三个实例**：Eigen 的头**没有扩展名**，cpplint 于是也归成 C 系统头。规律是「凡是不长得像 C++ 标准库的，都排在 C++ 标准库之前」 |
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
