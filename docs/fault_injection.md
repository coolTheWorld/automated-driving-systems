# 异常注入清单（P9-S5b，2026-08-16）

> SPEC §10 给 P9 后半的要求是「异常处理完备」。这张表把它变成可以逐行验红验绿的东西：
> **故障 × 期望行为 × 现有守卫（哪个测试）× 缺口**。原则与 SPEC §8 一致 ——
> 「没崩溃」不等于「对了」，每一行要么有自动化红绿，要么写明抓不到 + 为什么。
> 各测试文件头的「故障注入实测」小表是这张表的原始出处；本表只汇总，不复制推导。

## 1. 现状表

| # | 故障（注入方式） | 期望行为 | 守卫 | 状态 |
|---|---|---|---|---|
| 1 | `/odom` 含 NaN（发一条坏样本） | 控制不死、丢样，超时降级刹停 | `ads_control/test/test_odom_robustness.py`（L3-G，CI） | ✅ |
| 2 | 规划静默（planner 死/卡） | 控制 `TRAJECTORY_STALE`（0.5 s）刹停并保持 | `test_trajectory_timeout.py`（L3-G，CI） | ✅ 实测 0.52 s |
| 3 | `/vehicle_cmd` 持续 NaN 流 | 不喂狗 ⟹ 看门狗刹停（NaN 流 = 失联） | Gazebo：`gazebo_bridge/test/test_vehicle_cmd_bridge_watchdog.py`（CI）；CARLA：sidecar `_on_cmd` 同序（校验先于喂狗），`ControlMapping.is_valid/full_brake` L1 | ✅ / CARLA 侧半守（节拍逻辑无自动化） |
| 4 | **感知/真值链路中途断流**（障碍物流戛然而止） | 规划 `obstacle_timeout_s`（1.0）报 ERROR 不发轨迹 ⟹ 控制 STALE 刹停 | **`test_perception_silence.py`（L3-G，CI，P9-S5b 新增）**：规划 ERROR 1.06 s、控制 STALE 1.48 s、末拍 v=0；注入 `obstacle_timeout_s=1e9` 验红（车一路开到头） | ✅ |
| 5 | 感知从未到达（`expect_perception=true` 而话题空） | 规划指名报错不发轨迹 | 同一测试里被走到（前 4 s 有流之后才发 goal；planning_node 代码路径 P7-S3） | ✅（顺带覆盖，无单独用例） |
| 6 | 预测断流 / 从未到达 | 规划 `prediction_timeout_s`（1.0）/ `expect_prediction` 同 4/5 | **`test_prediction_silence.py`（L3-G，CI，P9-S5b 新增，domain 52）**：停发 `/prediction/trajectories` 后规划 ERROR「预测列表…没有更新」1.0+0.5 s 内、控制 STALE 刹停；注入 `prediction_timeout_s=1e9` 红 | ✅ |
| 7 | 点云陈旧（stamp 落后 >0.15 s） | 感知丢弃并计数 `dropped_stale_clouds`，**期间零输出**，新鲜帧一到即恢复 | perception_node `max_cloud_age_s`；**`test_closed_loop_perception.py` 第 7 段（P9-S5b 新增）**：10 帧落后 1.0 s ⟹ 输出 0 条、计数 +10，再喂 5 帧好的 ⟹ 恢复；注入 `max_cloud_age_s=1e9` 红（计数不涨 —— 陈旧帧还被 dt≤0 守卫二次拦下，所以「输出 0 条」那一半注入后仍绿，两道守卫叠着） | ✅ |
| 8 | 点云含 ±inf/NaN（无回波射线） | 预处理过滤，感知的 `isfinite` 显式过滤 | `verify_ros_bridge.sh`「滤净」（要 GPU，不进 CI）；`lidar_preprocessor` + perception `RequireFinite`（L1 `ThrowsOnNonFiniteInput`） | ✅ 分层各守（CI 只有 L1 那半） |
| 9 | 控制静默 / 控制节点崩 | bridge 看门狗（0.5 s 无指令 ⟹ 全刹） | Gazebo：**`test_vehicle_cmd_bridge_watchdog.py` 第 9 案「停发」（P9-S5b 新增，CI）** + `verify_teleop.sh`（6.3 s 后 8.33→0，要 GPU）；CARLA：sidecar `_apply_control` expired ⟹ `full_brake`（每 tick 重发） | ✅ |
| 10 | 两套仿真并存（`/clock` 双源，dt ≤ 0 或巨 dt） | 感知丢帧 + 跟踪器按首帧重来；脚本层残留守卫拒跑 | perception_node dt 守卫（先守卫后更新 last_stamp，2026-08-12 修）；`run_all_scenarios.sh`/`l3c_*_round.sh` 残留守卫 | ✅ |
| 11 | 定位失锁 / 雷达断流 / GNSS 断流 | NDT `max_innovation` 安全阀；连续 ≥10 帧超门强制粗网格重锚；**NDT 1.0 s 无成功帧降出 NDT_AIDED；GNSS 2.0 s 超时进 DEAD_RECKONING；降级不冻结（map→odom 继续发）** | L3-G 定位闭环 ③c（`ndt_chi2_exceed` 计数）+ 注入实测 32 次重锚；**`test_localization_sensor_timeout.py`（P9-S5b 新增，domain 54）**：假传感器 `fault.lidar_stop_after_s=12`/`fault.gnss_stop_after_s=17` ⟹ 14.0 s 首个 GNSS_ONLY、19.0 s 首个 DEAD_RECKONING、降级期间位姿 400+500 条；注入 NDT 超时 1e9（复原 2026-08-12 前的锁存缺陷）红 | ✅ |
| 12 | 真值道具飞天 / 埋地 / 倾倒 | 记录器判「本次运行无效」拒绝打分 | `record_perception_run.py` 刺激物校验（现在读**物理** z）；`p9_lidar_probe` 落定高度 | ✅（评测层） |
| 13 | TF 缺失（`map→base_link` 从未到达） | 感知跳帧 warn；规划不发；控制 NO_PATH；TF 到位后重发目标即恢复 | 静态 TF 永不过期 ⟹ 只能「一开始就不发」测（陷阱表）；**`ads_control/test/test_route_faults.py::test_a`（P9-S5b 新增，domain 53）**：launch 无 map→odom ⟹ 4 s 恒 NO_PATH v=0；测试补发静态 TF + 重发目标 ⟹ TRACKING；注入「launch 里加静态 TF」红。L3-G 定位闭环另机械查 `/tf_static` 上不许有 `map→odom` | ✅ |
| 14 | `/clock` 停走（仿真钟冻结，物理若还在跑） | 桥用**墙钟**发现仿真钟没走 ⟹ 发零速并保持，钟恢复后放开 | `vehicle_cmd_bridge` `clock_stall_s`（1.0 s 墙钟，wall timer 200 ms）+ **`gazebo_bridge/test/test_vehicle_cmd_bridge_clock_stall.py`（CI，P9-S5b 新增）**：停钟 1.38 s 后零速、钟恢复后 1.0 m/s 回来；注入「不建 stall_timer」验红（停钟后桥再无输出）。CARLA 侧无此洞：/clock 与世界 tick 同一个线程，钟停世界也停 | ✅ |
| 15 | 目标点无路由 | map_node WARN + 空 Path 清屏；规划忽略 <2 点、保留上一条路线；控制不动 | `test_routing`（L1 无路可达）；**`test_route_faults.py::test_b`（P9-S5b 新增）**：(500,500) ⟹ 4 s 恒 GOAL_REACHED v=0；注入换成路上合法目标红。⚠️ 第一次注入用的 (51.75,20) 在草地上、本身也无路由，注入后照样绿 —— 注入没红先查注入是不是有效刺激 | ✅ |

**读法**：✅ 有自动化红绿；⚠️ 代码路径在、无专门注入（写明理由或列为缺口）；❌ 已知未处理。

## 2. 补缺顺序（按风险 × 成本）—— **2026-08-16 全部补齐**

1. ~~#14 `/clock` 停走~~ ✅（桥侧墙钟守卫 + L3-G）。
2. ~~#7 点云陈旧、#6 预测断流~~ ✅（test_closed_loop_perception 第 7 段 / test_prediction_silence）。
3. ~~#11 定位超时降级、#13 TF 一开始就不发、#15 无路由~~ ✅（test_localization_sensor_timeout /
   test_route_faults 两个方法）。
4. ~~#9 控制静默的 CI 用例~~ ✅（test_vehicle_cmd_bridge_watchdog 第 9 案）。

表里现在 15 行全是 ✅（#8 的「滤净」那一半仍要 GPU，L1 那一半进 CI；#10 的守卫在脚本层）。
仍**只在 CARLA 侧靠代码路径**、没有自动化红绿的：#3/#9 的 sidecar 分支（本机没有 Vulkan，
CARLA 进不了 CI —— 只能在云窗口用 `p9_timing_probe`/手工停发验一次，见 p8_carla_bringup.md §6）。
新增用例的 ROS_DOMAIN_ID：50/52/53（ads_control）、51（gazebo_bridge）、54（ads_localization）。

## 3. 三条纪律（从 P5–P9 的注入实测里提炼，写在这里免得下次忘）

- **写完判据立刻注入验红**：一条从来没红过的守卫不知道自己守没守住（`ads-fault-injection-discipline`）。
- **判语义不判值域**：NaN 流期间 bridge 发的是闩存的有限值，「输出无 NaN」恒过；要断言
  「持续坏输入 = 失联 = 刹停」。
- **链路存在但断了 ≠ 链路本来不存在**：过期检查只对「到达过」的流生效，「从未到达」要靠 launch
  层的 `expect_*` 声明 —— 两条都要有，本表 #4/#5 各守一半。
