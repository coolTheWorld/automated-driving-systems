# 任务清单

> 详细拆解与理由见 [plan.md](./plan.md)　|　规格见 [SPEC.md](../SPEC.md)
> 状态：**P0a 完成 29/29**（CP1/CP2/CP3 达成）　|　**当前阶段：P1 地图与路由**
> 更新：2026-07-31　|　技术栈：**Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**（官方组合）
>
> 本文件按阶段分段：[P1 清单](#p1-地图与路由当前阶段)（当前）→ [P0a 清单](#s1容器--gpu--gui检查点-cp1go--no-go)（已完成，保留作记录）

---

## 🔖 下次从这里继续

**当前位置**：**P1 进行中 —— S1–S4 的代码全部完成并量化验收通过，CP-P1-A 已通过。⏳ 卡在 CP-P1-B 的肉眼验收上（需要你打开 RViz 看一眼）。**
剩余：S5 的 5.1（`docs/modules/map_and_routing.md` 数学推导）尚未写；5.2/5.3/5.4 已完成。
用户于 2026-07-31 选定 P1，不做 P0b。P0a 已全部完成（S1–S5，29/29）+ 一轮收官复查（见下）。

### 恢复步骤

```bash
# 宿主机
cd ~/work/automated-driving-systems           # ⚠️ 必须在仓库根，COMPOSE_FILE 是相对路径
export COMPOSE_FILE=docker/docker-compose.local.yml   # 每个新终端都要，漏了报 "no configuration file provided"
docker compose up -d

# 先确认没有残留仿真进程（有残留则所有测量作废）
docker compose exec dev bash -c 'ps -eo pid,pgid,args | grep -E "gz sim|ros2 launch" | grep -v grep'

# 容器内确认基线
docker compose exec dev bash -c '
  set +u; source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; set -u
  cd /workspace
  colcon test-result --all                 # 应为 248 tests, 0 errors, 0 failures
  ./build/ads_map/test_opendrive_parser    # 13 个用例，含 CP-P1-A 对账
  ./build/ads_map/test_lane_graph          # 17 个用例，含每条边的几何连续性
  ./build/ads_map/test_routing'            # 10 个用例，含「对向车道必须绕行」
python3 scripts/gen_map.py --check         # 三个生成物都同步（宿主机跑即可，无需 ROS）
```

两条应当打印出来的数字：

- CP-P1-A 对账 `最大位置偏差 0.000728035 mm，最大朝向偏差 4.99745e-07 rad`
- 车道图连续性 `检查 24 条边，最大位置断裂 4.04211e-07 mm，最大朝向断裂 9.79306e-12 rad`

**第一条变了就先查是不是有人改了地图却没重新生成采样基准** —— 那会让检查点
用旧基准对账，给出虚假的通过。

### ⏳ CP-P1-B 怎么验（这是 P1 的验收，需要你亲眼看）

```bash
cd ~/work/automated-driving-systems
export COMPOSE_FILE=docker/docker-compose.local.yml
docker compose up -d
docker compose exec dev bash -c 'ps -eo pid,pgid,args | grep -E "gz sim|ros2 launch|map_node" | grep -v grep'   # 先确认无残留
docker compose exec dev bash -c '
  set +u; source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; set -u
  ros2 launch ads_bringup stack.launch.py'
```

看四件事：

1. **车道图铺在路上**：青蓝 = 常规道路，橙 = 路口连接道路。
   每个 T 型路口应当有 **6 条橙线** —— 少了就是 laneLink 漏了。
2. **白色箭头指向行驶方向**：双向道路的两条车道箭头**相反**。
   这是「车道图是有向图」的唯一肉眼证据，没有箭头的话有向无向长得一模一样。
3. **车停在南边那条路上**，不是园区正中央的草地上。
   （在草地上说明 `map→odom` 又变回单位变换了，见下面那条 P0a 遗留 bug。）
4. **点 RViz 工具栏的 "2D Goal Pose"**，在任意一条路上点一下 →
   应当立刻出现一条绿色路径，贴着车道中心线，**不掉头、不逆行**。
   终端里 `map_node` 会打一行「路径已发布：N 段车道、M 个点、X m、耗时 …」。

跑完记得清 `core.*`（Gazebo 退出时的已知 segfault，一次 400–600 MB）。

**点不出路径时按这个顺序查**（四种失败的表象都是「RViz 里没有路径」，
所以 `map_node` 对每一种都单独打日志，看它说的是哪一环）：
TF 拿不到 → 自车不在路上（> 5 m）→ 目标点太远（> 10 m）→ 图上不可达。

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

### ✅ CP3 的分叉已定：走 P1，不做 P0b

用户于 2026-07-31 选定 **P1 地图与路由**。同时定下 P1 的两个架构选项：

- **地图拓扑**：环线 180×100 m（四角 R=12 m）+ 一条横穿路 → **2 个 T 型路口**
- **地图来源**：`config/campus_map.yaml` → `scripts/gen_map.py` → `.xodr` + Gazebo 世界
  （与 `vehicle_params.yaml → SDF + URDF` 同构）

详细理由、几何数值、切片依据见 [plan.md 第二部分](./plan.md#第二部分p1-地图与路由)。

> ⚠️ **仍然背着一笔账：CARLA 不应晚于 P2 结束才接上。**
> 一旦控制参数已经在 Gazebo 上调好才去对齐，行为漂移就已经发生了 ——
> 那正是本项目自列的头号风险，也是最难查的一类故障。
> P2 结束时要重新把 P0b 摆上桌面，不要让它一路滑到 P5。

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

# P1 地图与路由（当前阶段）

> 拆解理由与几何数值见 [plan.md 第二部分](./plan.md#第二部分p1-地图与路由)。
> 阶段成果（SPEC §10）：**RViz 中显示车道图和 A→B 全局路径**。

**生成链**（YAML 是唯一手写的源头）：

```
config/campus_map.yaml ──gen_map.py──┬──▶ maps/campus.xodr            两环境共用
                                     └──▶ models/campus_road/model.sdf Gazebo 可视
                                              └──▶ worlds/campus_loop.sdf（手写骨架）
```

**`campus_minimal.sdf` 不许改** —— 它是三个 verify 脚本的回归基线。P1 另建世界。

## P1-S1　地图生成器 + 园区地图【纯 Python】

| # | 任务 | 验收标准 | 状态 |
|---|------|---------|:---:|
| 1.1 | `config/campus_map.yaml`：路段 + 路口的紧凑描述 | 人能读懂、能改数 | ✅ |
| 1.2 | `gen_map.py` 几何内核：line / arc 的 s→(x,y,θ) 求值 | 与解析解一致 | ✅ |
| 1.3 | 生成 `maps/campus.xodr`：3 常规路 + 2 路口 + 12 连接路 | XML 合法、结构完整 | ✅ |
| 1.4 | 生成 `models/campus_road/model.sdf` + `model.config` | Gazebo 显示出环线 | ✅ 肉眼已确认 |
| 1.5 | `worlds/campus_loop.sdf`：手写骨架 + include | 世界能起、车在路上 | ✅ 数值 + 肉眼 |
| 1.6 | `--check` 模式：校验生成物与 YAML 同步 | 不同步则非零退出 | ✅ |
| 1.7 | L1 pytest：几何、周长自洽 539.398 m、路口连接数各 6 | 全绿 | ✅ 18 用例 |

**S1 实测（2026-07-31）**

| 项 | 实测值 |
|---|---|
| 环线周长 | **539.398 m**（= 247.699×2 + 22×2，两条独立算法一致） |
| 道路数 | 3 常规 + 12 连接 = 15；路口 2 个，各 6 条连接道路 |
| 路口退让 | 11.000 m（推导量 = 8 + 1.75 + 1.25） |
| 连接道路长 | 直行 22.000、左转 19.317、右转 17.815 m |
| 自车世界位姿 | (30.000, **−51.75**, ~0)，正是顺行车道中心 |
| RTF | **1.000**（`verify_sim.sh` 6/6 全过，P0a 基线是 0.970） |
| 测试 | `colcon test-result --all` → **130 tests / 0 failures**（基线 101） |

> **地图变大没有代价**：RTF 1.000 与 P0a 持平。原因是道路只有 `<visual>`
> 没有 `<collision>` —— 物理引擎根本不知道路的存在。

**测试有效性已验证**（不是只看绿灯）。往生成器里注入 5 处错误，逐一确认被**预期的**用例抓住：

| 注入的错误 | 抓住它的用例 |
|---|---|
| 参考线半径换算的加减号写反（左右转搞混） | `test_turning_lane_radius_...` |
| `inbound_lane` 正负号写反 | `test_lane_centres_are_continuous_...` |
| 连接道路起点偏 5 cm | 同上 |
| 世界的纬度改掉最后一位小数 | `test_world_geo_origin_matches_the_map` |
| 自车摆到车道线上 | `test_ego_spawn_pose_sits_on_a_lane_centre` |

> ⚠️ 第一条注入尤其值得记：**连续性用例没抓到它。** 参考线半径的加减号写反后，
> 圆心是按错误半径求交点算出来的，端点照样精确落在路口边界上，几何**处处连续**。
> 唯一的变化是车实际转弯半径从 8 m 变成 4.5 m 或 11.5 m。
> 症状要到 P2 才出现：过路口时转角需求突变，而你会以为是控制器的问题。
> `test_turning_lane_radius_...` 就是为堵这个洞补的。

> ⚠️ `test_lane_links_point_at_the_lane_that_actually_connects` 是**自证的** ——
> 两边都取 `leg.inbound_lane`，等于拿生成器和自己比。它的 docstring 已写明
> 这一点，别高估它。车道号本身的正确性由连续性用例独立保证。

**S1 新踩到的两个坑**（S5 的 5.4 要同步进 CLAUDE.md 陷阱表）

| 坑 | 症状 | 处理 |
|---|---|---|
| `setsid cmd &` 之后用 `$!` 取进程组 | 清理命令**静默地什么都没做**，仿真进程留下来 | `$!` 是 **setsid 自己**的 PID，它 fork 出新进程组后立刻退出，于是 `ps -o pgid= -p $!` 返回空，`kill -INT -- -` 变成空操作。要按**进程名**查 pgid：`ps -eo pgid,args \| grep "gz sim"`。CLAUDE.md 现有的陷阱表说了「用 `kill -INT -- -<PGID>`」，但没说 PGID 怎么正确取到 |
| headless `gz sim -s` 收 SIGINT 不退 | 发了 INT、等 3 s 仍在 | 实测需要升级到 TERM/KILL。（也可能是它退得比 3 s 慢，两种解释我没分辨开，**别把这条当定论**） |
| 按路径加载含 `@dataclass` 的模块 | `AttributeError: 'NoneType' object has no attribute '__dict__'`，报错完全不提根因 | 必须先 `sys.modules[spec.name] = module` 再 `exec_module`。因为 `from __future__ import annotations` 让注解变成字符串，dataclass 要回 `sys.modules` 里查模块来解析它们。`test_sim_source.py` 没踩到是因为 launch 文件里没有 dataclass |

## P1-S2　OpenDRIVE 解析 + 参考线求值【纯 C++】【CP-P1-A ✅ 已通过】

| # | 任务 | 验收标准 | 状态 |
|---|------|---------|:---:|
| 2.1 | `ads_map` 包骨架：`lib/`（无 ROS）+ `node/` | 结构同 SPEC §5 | ✅ S1 提前建，S2 补 lib/ |
| 2.2 | tinyxml2 解析 header / road / planView / lanes / junction | 字段全部读出 | ✅ |
| 2.3 | 参考线求值 s → (x, y, θ)，支持 line + arc | 与解析解一致 | ✅ |
| 2.4 | 车道中心线求值 (road, lane, s) → (x, y, θ) | 横向偏移正确 | ✅ |
| 2.5 | 不支持的几何类型**显式抛异常** | 遇到即报错 | ✅ |
| 2.6 | **与 Python 生成器逐点交叉比对** | **偏差 < 1 mm** | ✅ **0.00073 mm** |

**CP-P1-A 实测（2026-07-31）**

| 项 | 实测值 | 判据 | 余量 |
|---|---|---|---|
| 逐点位置偏差 | **0.000728 mm** | < 1 mm | ≈1400× |
| 逐点朝向偏差 | **5.0e-7 rad** | < 1e-5 rad | ≈20× |
| 采样点数 | 2820（15 条路 × 各车道 × 每 0.5 m + 全部几何接缝） | ≥ 2000 | — |
| 测试总数 | `colcon test-result --all` → **188 / 0 failures**（S1 后是 130） | — | — |

> **残差已经到了比对本身的分辨极限**：0.00073 mm 不是「两套实现的分歧」，
> 而是 `reference_samples.csv` 自己的文本精度（6 位小数，舍入 5e-7，
> 两个方向合成 ≈7.3e-7 m）。**别再靠调紧判据来「提高严格度」**，
> 那只会撞上格式下限而误报。

**对账的有效性已验证**：往 C++ 侧注入两处错误，偏差量正好印证了报错信息里的诊断提示。

| 注入 | 最大偏差 | 说明 |
|---|---|---|
| 车道横向偏移的正负号写反 | **3500 mm** | 正好一个车道宽，与「差半个车道说明横向偏移方向反了」对上 |
| 圆弧积分 y 分量符号写反 | **23997 mm** | 弯道整个镜像 |

**S2 新踩到的坑**（S5 的 5.4 要同步进 CLAUDE.md）

| 坑 | 症状 | 处理 |
|---|---|---|
| **同一个浮点格式串套不同量纲** | 对账余量只剩 1.5 倍，是个随时会因为改地图而误报的脆弱判据 | `.xodr` 里坐标用 6 位小数（微米）绰绰有余，但**曲率**是小数值（1/12 只剩 5 位有效数字，相对误差 4e-6），**朝向**是力臂（5e-7 rad × 76 m = 38 µm，且随地图变大线性增长）。两者改用 `%.12g` 后余量回到两三位数倍。见 `gen_map.py` 的 `precise()` |
| 手写 C++ 不过 clang-format | `colcon test-result` 报 91 处失败 | 写完直接跑 `clang-format --style=file:/workspace/.clang-format -i`，别靠手写对齐。**`colcon test` 的退出码照旧不可信** |

## P1-S3　车道图 + Dijkstra 路由【纯 C++】✅ 完成

| # | 任务 | 验收标准 | 状态 |
|---|------|---------|:---:|
| 3.1 | 车道图：节点 = 车道段，边 = 前后继 + 路口连接 | 节点/边数与手数一致 | ✅ **18 节点 / 24 边** |
| 3.2 | Dijkstra 最短路（代价 = 车道长度） | 与手算一致 | ✅ 4 条路线逐条对上 |
| 3.3 | 最近车道查询：世界坐标 → (road, lane, s) | 已知点定位正确 | ✅ 残差 < 1e-6 m |
| 3.4 | **方向性**：不允许逆行 | 逆行请求返回失败 | ✅ 见下面的说明 |
| 3.5 | 不可达返回明确失败，不返回空路径 | 有区分 | ✅ `nullopt` vs 单段零长路径 |

**产出**：`lib/` 加了 `lane_graph.{hpp,cpp}` 与 `routing.{hpp,cpp}`，`road_map` 加了
`lane_offset_at` / `lane_arc_length` / `geometry_at` 三个查询。仍然**无 ROS**
（`ldd libads_map.so` 只有 `libtinyxml2` 和 `libads_common`，后者自身也禁 ROS）。

**S3 实测（2026-08-01）**

| 项 | 实测值 | 判据 |
|---|---|---|
| 节点 / 边 | **18 / 24** | 手数一致（3 路 × 双向 + 12 连接路；6 × 2 + 12 × 1） |
| 每条边的几何断裂 | **4.0e-7 mm / 9.8e-12 rad** | < 1 µm / 1e-8 rad |
| 转弯车道中心线半径 | **8.000000 m**（8 条全部） | = `campus_map.yaml` 的 `turn_radius_m` |
| 掉头边 | **0 条** | 必须为 0 |
| 测试总数 | `colcon test-result --all` → **241 / 0 failures**（S2 后是 188） | — |
| 新增用例耗时 | 27 个用例共 **5 ms** | L1 要求毫秒级 |

**3.4 的验收标准需要说明**：计划写的是「逆行请求返回失败」，但**本地图上不存在
会失败的逆行请求** —— 两个 T 型路口把 6 条常规车道连成了强连通图，任何一对起终点
都可达，只是要绕。所以这条用它的**两种形态**验收：

1. 合成的断头路地图上，「目标在身后」确实返回 `nullopt`（`GoalBehindOnADeadEndLaneIsUnreachable`）。
2. 真实地图上，从东侧顺行车道到它**对向**车道（相距 3.5 m）的最短路是 **674.73 m**
   而不是 ≈0 m（`OppositeDirectionOfTheSameRoadRequiresGoingAround`）。
   这条比第一条更有分量：无向图给出的那条 3.5 m「路径」在 RViz 里是一条平滑的短线，
   长度也合理，肉眼永远看不出问题。

**一个不显然的发现：路由代价用参考线长度会让两条候选路线恰好并列。**

从 (road 1, lane −1) 到 (road 1, lane +1) 有两条候选：借横穿路兜回来、或绕整圈。
用**车道中心线**长度算是 674.73 vs 685.73，差 11 m，赢家唯一；
用**参考线**长度算两条**都是 680.23 m** —— 因为它们用到的参考线长度是同一个多重集
（一条 22 m 直行、一条左转、一条右转、loop_west 全长、cross 全长），只是分配不同。

并列的后果不是「算错」，而是**返回哪条取决于堆的遍历顺序** —— 换个标准库实现就可能变，
而两条路径看起来都完全正常。这就是 `Road::lane_arc_length()` 存在的理由：
弯道上车道中心线与参考线最多差 14.6%（R=12 的弯，右侧车道半径 13.75）。

**测试有效性已用故障注入验证**（5 处注入全部被抓）

| 注入 | 被哪些用例抓住 |
|---|---|
| 正/负编号车道的出口端搞反 | `add_edge` 的符号自洽检查直接让**建图抛异常**，两个 fixture 全红 |
| 代价改用参考线长度 | `LaneCostIsTheLaneCentreLength…` + 3 条路由长度用例 |
| 弧长因子写成 `1 + t·k` | `TurningLanesHaveTheConfiguredTurnRadius`（半径变成 11.5 m）等 6 条 |
| 建成无向图（每条边加反向边） | `EdgeCount…`、`EveryEdgeIsGeometricallyContinuous`、`Opposite…` 等 6 条 |
| Dijkstra 松弛时漏加边权 | `RoutesAcrossTheJunction…`、`SameLaneWithTheGoalBehind…` |

> ⚠️ 注入「代价改用参考线长度」时，`LaneGraphRejects.VaryingLaneWidth` **也跟着红了**。
> 原因是变宽车道的守卫在 `lane_arc_length()` 里，而它只在建图算车道长度时被调用。
> 这个耦合是真实存在的：哪天 `build_nodes()` 不再算车道长度，那个守卫就没人测了。
> 好在注入把它自己报了出来 —— 记在这里，将来改建图时留意。

**S3 新踩到的坑**（S5 的 5.4 要同步进 CLAUDE.md）

| 坑 | 症状 | 处理 |
|---|---|---|
| 路由代价用参考线长度 | 两条候选路线**并列**，返回哪条随实现变，且两条看起来都正常 | 代价必须是车道中心线长度。闭式解：等距偏移曲线 `dp/ds = (1 − t·k)·T`，在「几何段 ∩ 车道段」内 `k` 与 `t` 都是常数，逐块乘起来是**精确值**不是数值积分 |
| 最近车道查询不给朝向 | 自车偏左一点就被判到对向车道，路由第一步就要求掉头 —— 而路径本身平滑正常 | `nearest_lane()` 的 `heading_rad` 参数会把行驶方向夹角 > 90° 的车道整条排除。S4 从 TF 拿到的位姿带朝向，**必须传进去** |
| Dijkstra 里把起点 `dist` 预置成 0 | 「起点终点同车道且目标在后方」时前驱链绕成环，回溯**死循环/越界** | 引入一个虚拟源点（下标 = `node_count()`），起点节点就成了普通节点，可被重新访问。这类 bug 只在这一种输入下触发，很容易漏测 |

## P1-S4　ROS 节点 + RViz【CP-P1-B = P1 验收】

| # | 任务 | 验收标准 | 状态 |
|---|------|---------|:---:|
| 4.1 | `map_node` 发布 `/map/lane_graph`（MarkerArray，`transient_local`） | RViz 看到车道图 | ✅ 18+18 marker |
| 4.2 | 订阅 `/goal_pose`，起点取 TF `map→base_link` | 点击有响应 | ✅ |
| 4.3 | 发布 `/route/path`（`nav_msgs/Path`，map 系） | 贴合车道中心线 | ✅ 1119 点 / 571.451 m |
| 4.4 | 接入 `stack.launch.py` + RViz 配置 | 一条命令起全栈 | ✅ 全栈 headless 实测 |
| 4.5 | ~~L2 launch_testing~~ → **pytest × 3 + CI 跑 verify_map.sh** | 全绿 | ✅ 见下 |
| 4.6 | `scripts/verify_map.sh` 可复跑量化验收 | 退出码 0 | ✅ |
| — | **CP-P1-B 肉眼验收：RViz 里看车道图 + 点目标出路径** | 用户确认 | ⏳ **待你确认** |

**S4 实测（2026-08-01）**

| 项 | 实测值 | 判据 |
|---|---|---|
| 车道图 marker | **18 条中心线 + 18 个方向箭头** | = 车道数 |
| 路径点数 / 长度 | **1119 点 / 571.451 m** | vs 穷举脚本 571.460 m，**误差 0.002%** |
| 端点 | 首点离自车 **0.000 m**，末点离目标 **0.000 m** | ≤ 1 m |
| 相邻点距 | 最大 **0.634 m** / 最小 **0.494 m** | ≤ 0.75 m，无重复点 |
| 朝向自洽 | 最大偏差 **2.27°** | ≤ 15° |
| 路由耗时 | **0.7 – 2.0 ms** | 回调内 < 10 ms |
| `libads_map.so` | **零 ROS 依赖**（ldd） | SPEC §3.3 |
| 全量测试 | **248 / 0 failures**（S3 后 241） | — |
| 回归 | `verify_ros_bridge.sh` **6/6**（RTF 1.002）、`verify_teleop.sh` **3/3** | 无回归 |

**4.5 换了做法，理由**：计划写 L2 launch_testing，实际做成 `ads_bringup` 的
**三条 pytest**（0.15 s）+ **CI 直接跑 `verify_map.sh`**。launch_testing 能验的
（节点起得来、话题发得出）是 `verify_map.sh` 的真子集，再写一遍只是第三套机制；
而那三条 pytest 补的是 `verify_map.sh` **验不到**的一层 —— `stack.launch.py`
有没有把 `map_node` 正确装配进去。三条都做了注入验证：

| 注入 | 被哪条抓住 |
|---|---|
| 可执行文件名拼成 `map_nodee` | `test_every_launched_executable_actually_exists` |
| 漏设 `use_sim_time` | `test_every_launched_node_uses_sim_time` |
| 整个 `map_node` 块被删掉 | `test_map_node_is_assembled_into_the_stack` |

> `--show-args` 挡不住第一条 —— 它只执行 `generate_launch_description()`，
> 不检查 package/executable 存不存在。症状是全栈起来了、话题少一条、
> RViz 里没有车道图，而第一反应会去查 QoS。

### ⚠️ S4 挖出一个 P0a 遗留的真 bug：`map → odom` 不该是单位变换

Gazebo 的 `AckermannSteering` 把 `odom` 原点放在**自车 spawn 的位置**，
所以这一段的正确取值是「自车 spawn 位姿在世界里的坐标」。P0a 一直发单位变换，
而 `campus_minimal.sdf` 里自车在 `(0, −1.75)` 几乎就是原点，从没露过马脚。

换到 `campus_loop.sdf`（自车在 `(30, −51.75)`）立刻现形：**TF 报自车在 `map` 系
的 `(0, 0)`，而它 physically 在 `(30, −51.75)`，差 60 m。** 全局路径于是从园区
正中央的草地上出发 —— 而它在 RViz 里是一条平滑正常的线，没有任何一层报错。

修法：`gazebo_sim.launch.py` **从世界文件里读** spawn 位姿（不写死，否则换个世界
忘了改就又中招），节点名同步改成 `map_to_odom_static`
（`verify_ros_bridge.sh` 里硬编码过旧名字，一并改了）。

> 这个 bug 的形状值得记住：**一个错误的默认值，被一个恰好让它成立的场景
> 掩盖了整个 P0a。** 它不是被测试抓到的，是被「换了一个真实的地图」抓到的。

**S4 新踩到的坑**

| 坑 | 症状 | 处理 |
|---|---|---|
| 脚本里 `pgrep -f <名字>` 查残留 | 报「已经有 1 个在跑」然后拒绝启动，实际一个都没有 | 与 `pkill -f "gz sim"` 同源：`-f` 匹配完整命令行，执行脚本的那条命令行里出现过这几个字就会自己匹配自己。用 **`pgrep -x`** 按进程名精确匹配 |
| 杀掉 `static_transform_publisher` 想测「TF 没了」 | `lookupTransform` 照样成功 | 静态变换走 `/tf_static`，**tf2 buffer 对它永不过期**。要测只能一开始就不发 |
| 采样 `ceil(span/step)` + 末点夹到端点 | 路径最后**两个点重合**，RViz 看不出，下游弧长参数化除以零 | `span/step` 恰为整数时浮点上可能是 `80.00000000000001`，ceil 多算一步。改成**等分** `offset = span·i/count` |
| `setsid` 起进程后查 PGID 用错关键字 | 收尾时**杀了别人的进程组，自己的留下来** | `start_node /log ros2 run ads_map map_node` 里 `shift` 后的 `$2` 是 `run`，grep "run" 命中一堆无关进程。关键字必须显式传 |
| 私有函数的 docstring 摘要写在第一行 | pep257 报 D213 | 本仓库启用了 D213：**多行 docstring 的摘要必须从第二行开始**，且首行句点用 ASCII |

## P1-S5　文档 + CI + 收尾（可与 S4 并行）

| # | 任务 | 验收标准 | 状态 |
|---|------|---------|:---:|
| 5.1 | `docs/modules/map_and_routing.md` 数学推导 | SPEC §11 要求 | ☐ **未做** |
| 5.2 | CI 加 `gen_map.py --check` | 改 YAML 不重生成则 CI 红 | ✅ 顺带把 `verify_map.sh` 也加进 CI |
| 5.3 | 修 SPEC §10 的 P0a 行（删「OpenDRIVE 地图」）、§5 加 `maps/` | 与事实一致 | ✅ 另修了 `/dev/dri` → `/dev/dxg` |
| 5.4 | 同步 `CLAUDE.md` 新包/新命令/新陷阱 | 无过时说法 | ✅ |

> **5.4 为什么提前做了一半**：`CLAUDE.md` 原本写着「`src/` 下目前有六个包」，
> 加了 `ads_map` 之后这句话就是错的。而这个文件**每个会话都会被加载进上下文** ——
> 一条过时的说法不是「没帮上忙」，是会主动把下一个会话带偏。
> 代码里的错误下次跑测试就暴露，文档里的不会。所以它不能攒到 S5 一起改。

**5.2 加进 CI 的两步**：
- `gen_map.py --check` —— 与 `gen_vehicle_model.py --check` 并列。
- **`verify_map.sh`** —— 它是唯一能进 CI 的端到端验收，因为 `map_node`
  不需要 Gazebo。其余三个 verify 脚本要真仿真器（要 GPU、跑不确定），
  只能靠人记得跑，而人记得跑的测试三个月后一定没人跑。

**5.3 顺带修的第三处**：SPEC §5 的目录树里 compose 注释写着「挂载 `/dev/dri`」。
真正的通路是 `/dev/dxg`，`/dev/dri` 只是 WSL 的兼容外观 —— 这个错误说法
CLAUDE.md 里早就改过了，SPEC 里还躺着一份。**同一个错误在几个地方各躺一份，
比只错一处更难清理。**

**检查点**：**CP-P1-A**（S2 结束，双实现逐点 < 1 mm，不过不进 S3）、
**CP-P1-B**（S4 结束，P1 验收）。**检查点处停下来汇报实测数据。**

---

# ── 以下为 P0a 清单（已完成，保留作记录）──

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
