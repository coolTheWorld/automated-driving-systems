# 任务清单

> 详细拆解与理由见 [plan.md](./plan.md)　|　规格见 [SPEC.md](../SPEC.md)
> 状态：**P0a 29/29**、**P1 28/28**、**P2 28/28**、**P3 已验收（plan 全表 9/9+4/4）**、
> 　　**P0b 方案 B 已实测**、**P4 已验收（复检修复后六轮全过）**、**P5 已验收（Gazebo 半）**、
> 　　**P6 预测已验收（双层全过）**、**P7 行为决策已验收（CP-P7-A 7/7 + CP-P7-B，2026-08-13）**
> 　　**P8 已完成（CP-P8-A + CP-P8-B，2026-08-14）—— 感知域移植挂 P9 首片**
> 　　**P9 感知域移植 S1–S4 完成（2026-08-16：CP-P9-A 9/9 ×2、CP-P9-B 6/6）；S5 拆四片，S5b 异常注入 ✅【CP-P9-D】**
> 更新：2026-08-16　|　技术栈：**Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**（官方组合）
>
> 本文件按阶段分段：**P3/P4/P5 与 2026-08-12 复检的进度都记在「🔖 下次从这里继续」里**
> （它们没有单独的清单小节 —— 任务详拆在 [plan.md](./plan.md) 第四至六部分）；
> 再往下是 [P2 清单](#p2-s1数学推导文档--包骨架--路径预处理纯-c无-ros)、
> [P1 清单](#p1-s1地图生成器--园区地图纯-python)、P0a 清单，**均已完成，保留作记录**。

---

## 🔖 下次从这里继续

**当前位置**：**P9-S5 进行中（2026-08-16 本地）—— S5 拆成 S5a–S5d（plan P9-5）；S5a RANSAC 100 A/B
落地（Gazebo 零劣化、ground_ms 8.1→3.1）、耗时表待拍板「WSL2 感知 p95 24 ms 要不要移出回调」；
S5b **✅ 完成【CP-P9-D 达成】**（15 行全有自动化红绿，八条新守卫各注入验红，1164 tests 绿）。
云实例已销毁；S5a-② 的 CARLA 复核与 S5c/S5d 需要下一个云窗口。** 下面是按时间顺序的窗口战报（不重排）：
P8 租机窗口 1 已收官（2026-08-14，RTX 3090）。
S1–S4 ✅；S5 ✅（桥六项全绿、τ=0.140 复现、油门/刹车标定、七项会话加固）；
**S6 大半达成**：S01_S02_S07 功能通过 + 4 条跟踪差异入表（拍板），
S04 block **4/4 全绿**（停距 4.498），S04 avoid 8/9（间距 0.501/0.499 贴线往复），
**S06 红绿灯全绿 PASS**（停 1.50–1.58 m 进带、vmax 0.068、绿灯 2.5 s 起步过线）。
**窗口末的大案**：三个 block 轮孤儿 `obstacle_truth`（清理名单漏杀）带着
锥桶世界观污染后续所有轮 —— 幻影 stop_at=22.2 与真规划交替发布，控制被
两股轨迹拉扯成微冲-停。S06 前五轮失败、S01 退化轮、avoid 复跑截断
**全部由它解释**；「停车翻转双稳态」高度疑似同源（未做对照复跑，下窗口
用带守卫的脚本跑 block×2 钉死）。修复已入库：`l3c_round.sh`/`l3c_s06_round.sh`
按**安装路径前缀**杀 + **残留守卫拒跑**；执行器层驻车闩锁（control_mapping）。
**窗口 2 战果（2026-08-14）**：block 3/3 稳（孤儿污染定性成立）；margin 0.7
翻案 + 判据拆耦 + **两级准入**（avoid 9/9）；行为×3 truth 全绿（follow 5/5、
crossing 5/5、junction 4/4）；S06 2/2；一致性差异表十行（bringup §6）；
P5 复测条款完成（域差距量化：检测率 0-6%、3437 虚警 = 地面分割失效）。
途中钉死：孤儿发布者（云+本地各一回）、dynamic 接线断、NPC 三连修、
弯道亲墙（路肩 2.0）、Xid 32 降驱 550、准入死锁（两级）。
**P9 已开工（2026-08-14）**：拆片入 plan.md 第九部分（S1 诊断 → S2 修复 →
S3【CP-P9-A】→ S4【CP-P9-B】，三嫌疑待数据裁决：传感器挂载基准 / 生成
mesh 非平面 / walker 点稀）。**S1 本地半区已完**：地面分割诊断计数器
（pool/slope_rejected + L1 断言）、node 诊断五键（ground_height_m =
挂载基准差直读）、p9_capture_frame.py 一键采样（z 分位/众数桶 + 诊断
聚合 + 真值邻域点数 + 原始帧落盘）。**窗口 3 战报（2026-08-15 凌晨，RTX 5090 —— 5090+580-open 能跑，
Xid 32 案定性为宿主特异）**：S1 完成（五案裁决）+ S2 十刀（镜像 y /
丢点 45% / 坡度门 0.12 / 一致性门 / 剃刀门 0.15 / blob / 墙退 6 m /
真车闭环 / 副本航点 / RANSAC 300）。表：0-6%+3437 虚警 → 近场活+
虚警 11+ID 稳。**S2 收官谜**：10 m 外双类全零（嫌疑=跟踪确认层，
诊断法已写进 plan）。**黄金线索（收窗时抓获）**：栈起 40 s 后 npc 车 **actor 从
get_actors 消失**（spawn 日志在）——不是隐形是生命周期异常，此前
「0 点」轮全部要按此重读；裸环境九连测全可见，瞬移隐形理论作废。
**2026-08-15 本地日（零租金）战果**：④ Gazebo 回归跑出 P9-S2 第一个
回归事故 —— 三个行为场景感知层全撞车，根因剃刀门吞正对目标（L-Shape 剖面
≠ ODD 包络，perception.md §3 同坑第二次），修为二条件门（薄且矮）后行为×3
双层 6/6、CP-P5-B 五轮核心判据全过（ID 切换 2–6 贴边摆与门无关，P5 已知
形态列 S5）；全仓 1135 tests 绿。**云窗口弹药全部备好**：`cloud_window_open.sh`
一键开场（五里程碑）、actor 消失案仪器（sidecar `_npc_census` +
`p9_actor_watch.py`）、l3c_p5_round / l3c_behavior_round 照旧。
**窗口 4 战报（2026-08-15，同一台 5090，实例 82.68.63.172:41794 —— 收窗前记得销毁）**：
①② 完成 —— **三案定谳**：actor 消失 = 新客户端首查为空的读数幻影（两条线全程在）；
「10 m 外双类全零」= sidecar `np.frombuffer(raw_data)` 悬空视图 → 每帧半个世界；
「不翻 y」= 坏输入上的实验，raw 是 UE 右正要翻。**四刀落袋（全在 sidecar/launch，
感知没动）**：copy+翻 y / 撤墙（弯角 L 形墙 OBB 侵道）/ 后轴 base_link（轮位读 1.410）+
传感器挂点 / 行人落地 + micra 真值 + 真值 z 报物理底面。**表：0% → round 4/5 车 100%
行人 100%（15 m 内）、横向 0.26、虚警 0、符号/遮挡 ID 过；余近边 0.68-0.71、速度
1.03-1.07、ID 切换 4-6（三项同源：≥25 m micra 前脸断两簇 + 车顶自反射 60%）**。
仪器入库：`p9_lidar_probe.py`（裸测尺子）、`/perception/detections`、
`P9_INSTRUMENTS=1` 随轮起。台账：plan P9-S2/S3、bringup §6 #11-14、perception.md §8.3/§11。
**已拍板（2026-08-15）：两个根因都修。顺序 S3b → S3a → CP-P9-A ×2 → S4。**
**S3b/S3a 本地半区已完成（2026-08-15 下午，零租金）**：雷达 2.2 m（SDF/URDF 重生成、
sidecar 量程改读 yaml）+ `cluster.vertical_tolerance_m` 1.0（L1 +2 注入验红）；
新脚本 `l3g_p5_round.sh`（Gazebo CP-P5-B 一轮一命令）；Gazebo 三轮：车/行人 25–30 m 档
→ 100%、近边 0.14–0.17、横向 0.23–0.33、速度 0.39–0.91、虚警 2–7 帧（U 转鬼影 = §6.5
遮挡滑行 × 重锚跳变，机理与候选修法在 plan P9-4 附记，判据不放宽，待拍板是否列 P9-S5）、
ID 切换 3–6（近场贴边摆，P5 已知形态）；`run_all_scenarios.sh gazebo` 两遍 9/9；
全仓 1137 tests 绿（顺手把 L3-G test_closed_loop_obstacle 的判据从 margin 0.7 拆耦回
SPEC 0.5 —— 4 跑 3 红全是 0.001 m 浮点噪声）。
**窗口 5 战报（2026-08-16，新实例 120.88.119.115:30869，4090 + 580-open —— 冒烟过、全程没崩，
Xid 32 案再添一票「宿主特异」）**：开场一键 37 min（CARLA 下载 17 min + 镜像 18 min）；
裸测自反射 0（1.6 时 11k）；CP-P9-A 第一对 8/9（近边 0.60：micra 3.63 比先验 4.4 短、框两头各垫
0.4–0.5）→ 道具换 seat.leon（33 蓝图普查最近 4.19×1.82×1.47）→ **round 3/4 连续 9/9**
（检测率五档 100%、近边 0.39/0.24、横向 0.18/0.15、速度 0.30/0.28、ID 切换 0、虚警 0）；
CP-P9-B 六轮 6/6（junction truth 首轮红 —— 车队 b/c 高空回退带重力摔在 a 身上，修为
「天上停车场」等出发相位再落地）。判据一条没动。产物 `.p9w4/w5*`。
**下一步**：P9-S5 拆片（先跑两环境全表拿耗时/异常基线，再拆），不需要云机；云机销毁。
**2026-08-16 本地续**：文档补齐（README 阶段、bringup §5 了结、perception §1/§11、CLAUDE 命令表）；
plan P9-5 拆片；S5a-② RANSAC 100（Gazebo ×3 零劣化，ground_ms 3.1）；S5b `docs/fault_injection.md`
+ `test_perception_silence.py` + 桥墙钟守卫 `test_vehicle_cmd_bridge_clock_stall.py`（各注入验红）；
**S5b 余六条缺口同日补齐**（#7 陈旧点云段 / #6 `test_prediction_silence` / #9 看门狗停发案 /
#11 `test_localization_sensor_timeout`（假传感器 `fault.*_stop_after_s` 开关）/ #13+#15 `test_route_faults`），
每条注入验红；**1164 tests 绿，CP-P9-D 达成**。**待拍板**：S5a-③（WSL2 感知 p95 24 ms，SPEC §7 字面要求挪线程
vs 10 Hz 下功能无碍）。
**接着做**：S5c 跟踪鲁棒（U 转鬼影/贴边摆，本地 Gazebo 先，L1 注入 + CP-P5-B ×3）→ 下个云窗口：
S5a-② CARLA 复核 + CP-P9-A 复测【CP-P9-E】+ S5d 评估。
交接全文在 plan.md **P9-4**（数据、推导、候选、守卫、影响面逐条），现场产物在 `.p9w4/`
（不入库，README 有读法）。原冷启动命令（已执行 ①②，留作记录）：
  ① S3b：改 `config/vehicle_params.yaml` `sensors.lidar.mount_z_m` 1.6→拟 2.2 →
    `python3 scripts/gen_vehicle_model.py`（+`--check`）→ 本地 `verify_ros_bridge.sh`
    → CP-P5-B（`perception:=true dynamic:=both` + `record_perception_run.py --duration-s 72`，
    对照 2026-08-15 Gazebo 基线：行人 100%/车 98.1%、近边 0.182、横向 0.305、虚警 0、
    ID 切换 3）→ `run_all_scenarios.sh gazebo` → 同步改 vehicle_params 注释 /
    CLAUDE 陷阱表两行 / perception.md §5 §8.2 → 云上 `p9_lidar_probe.py`（自反射≈0）。
  ② S3a：`euclidean_cluster` 竖向各向异性 `cluster.z_scale`（拟 0.5，推导见 P9-4）+
    L1 两条注入验红 → `run_all_scenarios.sh gazebo S03` → 全表 + CP-P5-B 抽轮 →
    云上 `l3c_p5_round.sh both` ×2 全绿 =【CP-P9-A】。
  ③ 云实例 82.68.63.172:41794（5090）收窗时仍在跑、容器无残留；重开窗口先
    `cloud_window_open.sh`（幂等）+ rsync（用法在脚本头）。改共享感知/车辆参数的每一刀
    **先本地 Gazebo 回归再上云**（记忆「每刀两环境回归」）。

