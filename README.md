# automated-driving-systems

[![CI](https://github.com/coolTheWorld/automated-driving-systems/actions/workflows/ci.yml/badge.svg)](https://github.com/coolTheWorld/automated-driving-systems/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![CARLA 0.9.16](https://img.shields.io/badge/CARLA-0.9.16-black)

园区 / 厂区**低速自动驾驶栈**，从零手写：地图与路由、定位、感知、预测、行为决策、
规划、控制，加上一套双仿真环境（本地 Gazebo + 云端 CARLA）和四层测试体系。
ROS 2 Jazzy + C++17，算法层与 ROS 强制解耦。

目标是**工业级架构**而非车规量产：模块划分、接口契约、判据体系、测试分层都按真实项目
的做法来，功能覆盖只到 ODD（≤ 30 km/h、结构化园区道路、白天、无信号路口）为止。

> **v1.0.0（2026-08-16）**：路线图 P0a → P9 全部完成。同一套栈在两个仿真环境里过了同一张
> 判据表，全部数字是实测值，不是估计。下面每张表都能用仓库里的脚本复现。

---

## 它能做什么（实测）

一条命令起全栈：自车在园区环线上接收目标点，路由 → 规划 → 控制闭环行驶；感知给出
车辆/行人的框、速度与 ID，预测给出 6 s 轨迹与不确定椭圆，行为层在跟车、行人横穿、
无信号路口三种情境下选择跟停 / 让行 / 通行；定位在 NDT + ESKF 上给出 map→odom。

**L3 场景表（SPEC §8，`run_all_scenarios.sh` 一键，Gazebo 9/9 进 CI；CARLA 云端同表）**

| 场景 | 判据（节选） | Gazebo（L3-G） | CARLA（L3-C） |
|---|---|---|---|
| S01/S02 直道巡航 + 弯道 | 横向 < 0.3 m、a_lat < 2 m/s²、到达 | 8/8 | 安全类 4/4；**跟踪类 4 条入表差异**（见下） |
| S03 前车减速跟停 | 不碰撞、稳态跟停距 [4,10] m、驶离恢复 | 5/5 ×2 层 | 5/5 ×2 层 |
| S04 静止障碍物绕行 / 停车 | 侧向间距 > 0.5 m；不可绕则停 | 9/9 + 4/4 | 9/9 + 4/4 |
| S05 行人横穿 | 完全停止、最近距离 > 1 m、恢复 | 5/5 ×2 层 | 5/5 ×2 层 |
| S06 红绿灯（仅 CARLA） | 红灯停止线前 0–2 m、绿灯起步 | — | 2/2 |
| junction 无信号路口让行 | 对车流间距 > 1.5 m、让行后通过 | 4/4 ×2 层 | 4/4 ×2 层 |
| S07 全局导航 | 到达、零接管、零碰撞 | ✅ | ✅ |

**模块级验收（同一张判据表两个环境）**

| 模块 | 判据（节选） | Gazebo | CARLA |
|---|---|---|---|
| 感知 CP-P5-B（9 条） | 车/行人检测率、近边 p95 < 0.5 m、横向 p95 < 0.5 m、速度 p95 < 1 m/s、ID 切换 ≤ 2、车道内虚警 = 0 | **9/9 ×3**（横向 0.15，虚警 0，ID 1） | **9/9 ×2**（近边 0.19–0.21，横向 0.12，ID 0，虚警 0） |
| 定位 CP-P4-B（7 条） | 横向 < 0.30 m、NDT 锁定、失锁自举 | 六轮全过（横向 0.09–0.11 m） | 待上机（世界与地图不同源，门限要重量） |
| 预测 CP-P6-B（双层） | FDE@3s、椭圆覆盖率 ≥ 95%、路口多假设 | 真值层 + 感知层全过 | — |
| 控制 CP-P2-B（8 条） | 横向 / 速度 / a_lat / 转向速率 / 到达 | 8/8 | 4/8（跟踪类差异入表，机理见 ADR-0002） |
| 异常注入清单 | 15 种故障 × 期望行为 × 自动化红绿 | 15/15 | sidecar 分支代码路径 |

CARLA 侧唯一的已知差距是 S01 的四条**跟踪质量**判据（横向 0.6–0.84 vs 0.3、Δv 0.42 vs 0.2 …）：
根源是 CARLA 车辆的发动机模型（零油门阻尼 + 扭矩曲线峰）让油门→加速度在中速段增益偏大 78%，
安全类判据不受影响，评估后决定**入表放行**，机理、实验与被否决的方案在
[ADR-0002](docs/adr/0002-carla-s01-tracking-gap-not-fixed.md)。

---

## 架构

```
                 config/*.yaml（唯一手写源头）──gen_*.py──▶ SDF / URDF / .xodr / 点云地图（生成物）
                                                                        │
  Gazebo Harmonic ─ gazebo_bridge ─┐                                    ▼
                                    ├─▶ SPEC §4.1 规范话题 ─▶ ads_localization ─▶ map→odom
  CARLA 0.9.16 ── carla_bridge ────┘   /lidar/points /imu /gnss         │
     (sidecar 全中继)                    /odom /ego_pose_gt(仅评测)       ▼
                                          ads_perception ─▶ ads_prediction ─▶ ads_planning ─▶ ads_control ─▶ /vehicle_cmd
                                                ▲                (行为树 + Frenet 采样 + 速度剖面)  (Stanley + PI)      │
                                            ads_map ─▶ /route/path ─────────────────────────────────────────────────────┘
```

- **仿真数据源可插拔**：上游只认话题名，不知道数据来自 Gazebo 还是 CARLA；桥接层做完全相同的翻译。
- **算法与 ROS 解耦**：每个包 `lib/`（纯 C++17，L1 毫秒级）+ `node/`（ROS 包装）；`libads_map.so` 等零 ROS 依赖由 CTest 机械保证。
- **单一来源**：车辆参数、地图、障碍物、动态目标全部只手写一份 YAML，其余是生成物，`--check` 进 CI。
- **判据即契约**：每个模块交付时带可量化判据（plan.md 各检查点），判据不放宽；每条守卫先注入验红再算数。

十三个包：`ads_msgs` `ads_common` `ads_map` `ads_localization` `ads_perception` `ads_prediction`
`ads_planning` `ads_control` `ads_bringup` `ads_teleop` `ads_visualization`
`ads_simulation/{gazebo_bridge, carla_bridge}`。每个模块的推导、参数含义与实测边界在
[docs/modules/](docs/modules/)（改模块前先读它）。

---

## 从零跑起来（本地 Gazebo）

**环境要求**：Windows 11 + WSL2（Linux 原生也可，GPU 直通那几步不同）、支持 D3D12 的显卡
（开发机 AMD Radeon 780M 核显足够）、WSL2 内的 Docker、≥ 25 GB 磁盘。**宿主不需要装 ROS 或 Gazebo。**

```bash
git clone git@github.com:coolTheWorld/automated-driving-systems.git
cd automated-driving-systems
export COMPOSE_FILE=docker/docker-compose.local.yml     # ⚠️ 所有 docker compose 命令都在仓库根目录执行
./scripts/setup_env.sh                                  # 生成 .env（宿主 UID/GID + GPU 设备组），换机器必跑
docker compose build && docker compose up -d

docker compose exec dev /workspace/scripts/verify_gpu.sh   # go/no-go：必须是 D3D12 硬件加速，llvmpipe 就别硬撑
docker compose exec dev bash -c 'source /opt/ros/jazzy/setup.bash && cd /workspace && \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
docker compose exec dev /workspace/scripts/verify_sim.sh          # 仿真基线 + RTF
docker compose exec dev /workspace/scripts/verify_ros_bridge.sh   # SPEC §4.1 话题契约
docker compose exec dev /workspace/scripts/verify_teleop.sh       # 控制链路 + 限幅 + 看门狗
```

**自动驾驶一次**（终端 A 起栈，终端 B 发目标点并记录判据）：

```bash
docker compose exec dev bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
  ros2 launch ads_bringup stack.launch.py perception:=true prediction:=true dynamic:=follow'
docker compose exec dev bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && cd /workspace && \
  python3 scripts/record_behavior_run.py --scenario follow --layer perception --duration-s 90 --out /tmp/follow.csv'
# 或者一键全表：
docker compose exec dev /workspace/scripts/run_all_scenarios.sh gazebo
```

键盘开车：`docker compose exec dev /workspace/scripts/drive.sh`（`w/s` 加减速、`a/d` 转向、`空格` 回正、`b` 急刹）。

> 遇到诡异现象先翻 [CLAUDE.md](CLAUDE.md) 的**陷阱表** —— 七十来条全是实测踩出来的
> （残留仿真进程、`pkill -f` 自杀、QoS 静默丢帧、NaN 比较恒假、`ros2 topic hz` 管道缓冲……）。

---

## 云端 CARLA（验收环境）

本机没有硬件 Vulkan，CARLA 跑不了 —— 这正是双环境方案的由来。租一台 NVIDIA 实例（实测
3090 / 4090 / 5090 都跑过），一键开场：

```bash
scp -P <port> scripts/cloud_window_open.sh root@<host>:/root/ && rsync -az ... ./ root@<host>:/root/ads/
ssh -p <port> root@<host> 'setsid nohup bash /root/cloud_window_open.sh > /root/open.out 2>&1 &'
# 五个里程碑过后：docker exec ads-dev bash /workspace/scripts/l3c_p5_round.sh both   （CP-P9-A）
#                 docker exec ads-dev bash /workspace/scripts/l3c_behavior_round.sh follow perception 90 91.75 20.0
```

上机手册 [docs/p8_carla_bringup.md](docs/p8_carla_bringup.md)（含两环境一致性差异表）；
亲眼看用 `scripts/carla_view.py`（MJPEG 直播，ssh 隧道到本机浏览器）。

---

## 测试

四层金字塔（SPEC §8）：L1 单元（gtest/pytest，无 ROS，毫秒级）→ L2 模块 → L3-G 场景闭环
（**不需要 GPU，进 CI**：假车/假传感器 + 真节点接线，判的是接线不是精度）→ L4 回归
（`metrics/history.csv`）。L3-C（CARLA）与真仿真器的 L3-G 场景表靠人跑。

```bash
colcon test && colcon test-result --all      # 1202 tests，约 3 min（判定成败只看 test-result）
./build/ads_perception/test_tracker          # 单个 L1，快一个数量级
```

CI（GitHub Actions）：构建开发镜像 → 四份生成物 `--check`（车辆 / 地图 / 障碍物 / 动态目标）→
`colcon build` + `colcon test`（含 L3-G）→ launch 可加载 → `verify_map.sh`（唯一进 CI 的端到端验收）。
故障注入清单在 [docs/fault_injection.md](docs/fault_injection.md)：改任何守卫前先查它，补守卫先注入验红。

---

## 文档地图

| 文件 | 内容 |
|---|---|
| [SPEC.md](SPEC.md) | **唯一事实来源**：ODD、架构、接口契约、代码规范、测试策略、决策记录（D1–D6）、边界 |
| [CLAUDE.md](CLAUDE.md) | 导航 + 实测陷阱表 + 常用命令 |
| [tasks/plan.md](tasks/plan.md) / [todo.md](tasks/todo.md) | 拆片理由、检查点判据与实测数据 / 进度书签 |
| [docs/modules/](docs/modules/) | 七个模块的推导、参数与边界（map_and_routing / control / planning / localization / perception / prediction / behavior） |
| [docs/adr/](docs/adr/) | 架构决策记录（双仿真环境；CARLA S01 差异不根治） |
| [docs/fault_injection.md](docs/fault_injection.md) | 异常注入清单：15 种故障 × 期望 × 守卫 |
| [docs/p8_carla_bringup.md](docs/p8_carla_bringup.md) | 云端 CARLA 上机手册与一致性差异表 |

---

## 已知边界

- **仿真栈**，没有上过实车；ODD 之外的场景（高速、雨雾、无结构化道路）不在范围。
- **无倒车**：`VehicleCmd` 只有转角 + 加速度，没有挡位；无相机感知（激光雷达单传感器）。
- CARLA 侧 S01 四条跟踪判据入表放行（ADR-0002）；定位模块尚未在 CARLA 上验收（世界与地图不同源）。
- 本地 Gazebo 的感知耗时 p95 24 ms 超 SPEC §7 的 10 ms 线，目标硬件（云机）8.9 ms 在线内 —— 拍板暂不改线程模型（SPEC §7 注解）。

## 路线图（已全部完成）

P0a 本地环境 → P0b 云端 CARLA 最小对齐 → P1 地图与路由 → P2 控制 → P3 规划 → P4 定位 →
P5 感知 → P6 预测 → P7 行为决策 → P8 场景测试体系 + CI + CARLA 半区 → P9 感知域移植 + 性能鲁棒。
控制排在感知前面是有意的：先用仿真真值打通「规划→控制→车动起来」的闭环，之后每个模块都能
立刻看到效果。2026-08-12 做过一次全栈对抗性复检（37 个审查/验证子代理），修复 3 high + 11 medium。

## 许可

[Apache-2.0](LICENSE)
