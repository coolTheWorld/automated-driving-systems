#!/usr/bin/env python3
# Copyright 2026 孙帅
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Score a perception run against ground truth — CP-P5-B.

判据来自 tasks/plan.md 的 CP-P5-B 表，**这里不重新发明**。

⚠️ **检测率按「命中率」定，不是按点数** —— S1 体检实测目标在连续帧之间
**闪烁**（同一锥桶连续三帧 8 点→8 点→0 点），命中率只有 33–74%。
拿"平均点数"或"中位点数"去定判据都会误导（前者说稳定有 4 个点、
后者说完全看不见，而真相是时有时无）。

⚠️ **路侧杆件不是虚警。** 感知会把路灯杆检测成静态障碍物 —— 那是**对的**，
它们真实存在，只是不在真值列表里（真值只列 obstacles.yaml 与 dynamic_actors.yaml
里的东西）。所以虚警只统计**自车车道内**的多余目标：车道外的东西
不影响行车，而把它们算成虚警会让判据永远红。

用法（需要一个正在跑的仿真，perception:=true）：
    ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false \\
        perception:=true dynamic:=both
    python3 scripts/record_perception_run.py --duration-s 45 --out /tmp/p5.csv
"""

import argparse
import csv
import math
import time
from collections import defaultdict

from ads_msgs.msg import ObstacleArray
import rclpy
from rclpy.node import Node
import tf2_ros

# 与真值配对的最大距离，m。
#
# ⚠️ 取 2.5 而不是 0.5：感知给的是**可见面**为主的包围盒中心，而真值是
#    几何中心 —— 雷达打不到背面，两者天然差"进深的一半"（4.4 m 的车
#    正对时就是 2.2 m，见 S2 的实测）。用 0.5 去配对的话，一辆完全
#    正确检测到的车会被判成"没检测到 + 一个虚警"。
#    **配对半径与位置误差判据是两回事**：前者问"是不是同一个东西"，
#    后者问"位置差多少"。
MATCH_RADIUS_M = 2.5

# 自车车道半宽，m（用于判虚警）。与 config/dynamic_actors.yaml 一致。
EGO_LANE_HALF_WIDTH_M = 1.75
EGO_LANE_CENTER_Y_M = -51.75

# ---- 刺激物自检：这一跑到底有没有在考感知 ---------------------------------
#
# ⚠️ **这两条不是感知判据，是判据的前提条件。** 违反时输出「本次运行无效」
#    而不是「感知不达标」—— 两者必须分得开。
#
# 为什么值得单列（2026-08-12 实测，代价是两轮误判的根因）：
#   CP-P5-B 首轮 6/7 不达标，三条判据（检测率、分类、位置误差）同时红。
#   我据此查了两轮**感知代码**，两次给出根因判断，两次被自己随后的数据推翻。
#   真正的原因是**被感知的东西坏了**：两个 NPC 道具相撞后翻滚着飞上天
#   （20 s 飞到 20 m 高），飞出了雷达的垂直视场。
#   感知代码从头到尾没有问题 —— 它如实报告了「那里什么都没有」。
#
#   「目标没被检测到」与「目标根本不在视场里」在判据表上长得**一模一样**，
#   而排查方向截然相反。所以必须在量感知之前先量刺激物。
#   这与 CLAUDE.md 里「判据的适用范围没圈准」是同一族的坑。
#
# 阈值取得很松（真值恒等于 0），只为抓灾难性的道具失效，不为抓精度：
#   离地 0.10 m —— 比雷达噪声 σ=1 cm 大一个量级，不会误报；
#   倾斜 2.0°  —— 实测那次飞行 20 s 内涨到 33.7°，2° 在 1.4 s 内就会触发。
# 判「同一个目标上有没有重复航迹」的半径，m。
# 取 1.0：远小于本场景里两个真值目标的间距（1.75 m），所以数出来的
# 一定是同一个目标上的重复，而不是邻居。
DUPLICATE_RADIUS_M = 1.0

STIMULUS_MAX_LIFT_M = 0.10
STIMULUS_MAX_TILT_DEG = 2.0

# ---- 判据的适用范围 -------------------------------------------------------
#
# ⚠️ **这个数由传感器的物理分辨率定，不由"改到多少能过"定。**（2026-08-12）
#
# 起因：原来只有检测率判据有范围（车 20 m、人 15 m），而位置/速度/ID 判据
# 在**全距离**（到 50 m）上算 —— 同一张表里两套范围，判据自己不自洽。
# 实测代价：某一轮横向误差 p95 = 0.704（FAIL），而那 **全部** 来自 45–51 m，
# 20 m 内 p95 只有 0.080。判的是传感器的物理极限，不是算法。
#
# 推导（32 线 / 35° 垂直视场，水平 0.2°/点）：
#     线间隔 = tan(35°/31) = 0.01971 rad      水平间隔 = 0.2° = 0.00349 rad
#     距离 d 处目标上的点数 ≈ (h / 0.01971d) × (w / 0.00349d)
#   行人 1.7 × 0.4 m：30 m ≈ **11 点**，40 m ≈ 6 点，
#                     50 m ≈ **3.9 点 < min_cluster_size(5) ⟹ 物理上不可检**。
#   取「最小目标仍有 ≥ 2 × min_cluster_size = 10 点」⟹ d = **31.4 m**。
#
# 取 30 m（略保守于推导值）。
# 调大到 40 m → 判的是 6 个点上的拟合，判据度量的是雷达而不是算法；
# 调小到 20 m → 与检测率判据同范围，但放弃了 20–30 m 这段系统确实工作的区间。
#
# ⚠️ **范围外的数照样打印，只是不判。** 藏起来的话，下一个人会以为
#    远处也有同样的保证 —— 那正是"绿灯不代表对"最难发现的一种。
CRITERION_MAX_RANGE_M = 30.0

# 判据的物理**内界**，m（2026-08-12 扩窗实测后补）。
#
# 雷达 range_min = 2.2 m（近裁剪面，防自车车顶回波 —— 74d12ad 的修复，
# 当时文档写明代价是"又近又高"的盲区）。目标在 ~3 m 内时，其下半身的
# 斜距落进裁剪面（雷达高 1.6 m：3 m 水平处斜距 √(3²+1.3²)=3.27，
# 但底部 0.5 m 处斜距 √(2.2²+1.1²)=2.46 → 部分裁剪；2.5 m 处大半被裁）。
# 扩窗实测：自车过弯从行人身旁 2.2–2.8 m 经过，连续 11 帧漏检 —— 物理盲区，
# 不是算法缺陷。与 30 m 外界同理：判据不考物理上看不见的区域，
# **范围外的帧照样进 CSV**，只是不进判据。
CRITERION_MIN_RANGE_M = 3.0

# ---- 符号判据的两条机械排除（P6-S0 收口，2026-08-12 用户拍板） -------------
#
# 「速度符号错」考的是**稳定跟踪下的运动方向**。两类过渡帧里符号与真值反
# 是系统设计的必然，不是符号 bug —— 判它等于判"KF 有没有零滞后"：
#
# ① **真值方向翻转过渡**（行人/NPC 的 U 转）：真值方向 180° 反转时 KF 滞后
#    半秒量级，翻转沿途 dot < 0 物理上不可避免。排除条件按**真值**机械判：
#    过去 1.0 s 内真值方向变化 > 45°。ODD 内最急的合法弯 R=12 m @ 4 m/s
#    只有 v/R = 19°/s（1 s 内 19° < 45°，余量 2.4×），正常过弯**不会**被排除；
#    U 转实测 ≥ 90°/s，必被排除。方向只在 |v| > 0.5（消歧门限同源）时采样。
# ② **配对交接过渡**：配对的感知 id 变化后（交会互换/重复航迹摇摆/重建），
#    新配对的 KF 速度要 ~0.5 s（5 帧）重新收敛 —— 与 max_misses 同窗。
#    交接头 5 帧的符号是身份混叠，不是符号估计错。
#
# ⚠️ 两条都**只作用于符号判据**：速度/位置误差的分位数聚合一个样本都不动
#    （p95 对个位数的过渡帧不敏感，动了反而是藏数据）。排除帧数照印 ——
#    藏起来的话"排除"会悄悄变成"过滤"，而那正是 SPEC §11 禁的那类。
#
# 故障注入实测（2026-08-12，注入 = 把符号比较反转）：456/456 全部报错、
# 判据 FAIL，两条排除只吃掉 18 + 55 个过渡帧 —— 排除**吞不掉**系统性符号 bug。
SIGN_TURN_WINDOW_S = 1.0
SIGN_TURN_EXCLUDE_DEG = 45.0
SIGN_PAIRING_MIN_AGE = 5

BUCKETS = [(0, 10), (10, 15), (15, 20), (20, 25), (25, 30)]

CLASS_NAMES = {0: 'UNKNOWN', 1: 'PEDESTRIAN', 2: 'BICYCLE', 3: 'VEHICLE', 4: 'STATIC'}


def optimal_assignment(gts, perceived, radius_m):
    """Globally optimal one-to-one matching by total distance (brute force).

    ⚠️ 不用贪心：贪心按遍历顺序抢航迹，先来的真值会把邻居的航迹抢走
       （本场景两目标间距 1.75 m < 配对半径 2.5 m，实测确认会发生）。
       真值最多 2–3 个，全排列穷举即全局最优 —— 与 test_hungarian 用穷举
       对账是同一个道理，规模小就用最笨最对的办法。

    :param gts: 真值障碍物列表
    :param perceived: 感知障碍物列表
    :param radius_m: 配对半径（超出者不许配）
    :return: {真值下标: 感知下标}，配不上的真值不在字典里
    """
    if not gts or not perceived:
        return {}
    cost = [[math.hypot(p.pose.position.x - g.pose.position.x,
                        p.pose.position.y - g.pose.position.y)
             for p in perceived] for g in gts]
    best_total, best_map = None, {}
    candidates = [[j for j in range(len(perceived)) if cost[i][j] < radius_m] + [None]
                  for i in range(len(gts))]

    def recurse(i, used, total, mapping):
        nonlocal best_total, best_map
        if i == len(gts):
            # 配对数多者优先，其次总距离小者 —— 否则「都不配」是零代价最优解
            key = (-len(mapping), total)
            if best_total is None or key < best_total:
                best_total, best_map = key, dict(mapping)
            return
        for j in candidates[i]:
            if j is None:
                recurse(i + 1, used, total, mapping)
            elif j not in used:
                mapping[i] = j
                recurse(i + 1, used | {j}, total + cost[i][j], mapping)
                del mapping[i]

    recurse(0, frozenset(), 0.0, {})
    return best_map


def segment_intersects_box(p0, p1, obstacle) -> bool:
    """Does segment p0→p1 pass through the obstacle's 2-D box?

    遮挡判定用：自车→行人的视线段穿过 NPC 车的盒子 = 行人被挡住。
    采样法（步长 0.2 m）：盒子最短边 1.8 m，不会跨过去漏判；
    比解析的线段-OBB 相交少 30 行，而这里是评测脚本，宁可笨不可错。

    :param p0: (x, y) 起点（自车）
    :param p1: (x, y) 终点（目标中心）
    :param obstacle: ads_msgs/Obstacle（遮挡者）
    :return: 是否穿过
    """
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    steps = max(2, int(length / 0.2))
    for k in range(1, steps):
        ratio = k / steps
        x = p0[0] + ratio * (p1[0] - p0[0])
        y = p0[1] + ratio * (p1[1] - p0[1])
        if distance_to_box((x, y), obstacle) <= 0.0:
            return True
    return False


def distance_to_box(point, obstacle) -> float:
    """Exact 2-D distance from a point to an oriented bounding box.

    ⚠️ **不要用「到最近顶点的距离」代替。** 那是个上界，正对时最近点在**边上**
       而不是顶点上，会把距离高估半个车宽。这与 CLAUDE.md 里「拿 SAT 的最大
       间隙当保守估计」是同一族的坑：偏保守的估计照样会让判据给出错误结论。
       正确做法就是这几行 —— 转到盒子自身的坐标系，把点夹到盒子的范围内。

    :param point: (x, y)，map 系
    :param obstacle: ads_msgs/Obstacle，用它的 pose（中心 + yaw）与 size_m
    :return: 距离 m；点在盒子内部时为 0
    """
    q = obstacle.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    dx = point[0] - obstacle.pose.position.x
    dy = point[1] - obstacle.pose.position.y
    # map → 盒子自身坐标系（**反向**旋转 yaw）
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    over_x = max(0.0, abs(local_x) - 0.5 * obstacle.size_m.x)
    over_y = max(0.0, abs(local_y) - 0.5 * obstacle.size_m.y)
    return math.hypot(over_x, over_y)


def stamp_to_seconds(stamp) -> float:
    """Convert a ROS time message to float seconds.

    :param stamp: builtin_interfaces/Time
    :return: 秒
    """
    return stamp.sec + stamp.nanosec * 1e-9


class PerceptionScorer(Node):
    """Pair each ground-truth object with the nearest perceived one, frame by frame."""

    def __init__(self, match_radius_m: float):
        super().__init__('perception_scorer')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.match_radius_m = match_radius_m
        self.truth_frames = []
        self.rows = []
        # {真值 id: [感知 id, ...]}，用来数 ID 切换
        self.assigned_ids = defaultdict(list)
        self.false_positives_in_lane = 0
        self.false_positives_out_of_range = 0
        self.false_positive_log = []
        # 符号判据的两条机械排除（见 SIGN_* 常量的推导）
        self.gt_dir_history = defaultdict(list)   # gt_id -> [(t, 方向 rad)]，只在 |v|>0.5 时记
        self.pairing = {}                         # gt_id -> [感知 id, 同配对连续帧数]
        self.sign_excluded_turn = 0
        self.sign_excluded_handover = 0
        self.perceived_frames = 0
        # 刺激物自检（见 STIMULUS_* 常量上方的说明）
        self.stimulus_max_lift_m = 0.0
        self.stimulus_max_tilt_deg = 0.0
        self.stimulus_worst = ''
        # 真值流健康度（见 _on_truth）
        self.last_truth_s = None
        self.max_truth_gap_s = 0.0
        self.last_perceived_s = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # ⚠️ 真值只有评测脚本能订阅（SPEC §4.1）。本脚本正是评测脚本。
        self.create_subscription(ObstacleArray, '/perception/obstacles_gt', self._on_truth, 20)
        self.create_subscription(ObstacleArray, '/perception/obstacles', self._on_perceived, 20)

    def _on_truth(self, msg):
        stamp_s = stamp_to_seconds(msg.header.stamp)
        # 真值流健康度：断流时段里配对与刺激物自检都会**静默停止**，
        # 残缺时段照样打分会返回一个不可信的 0（2026-08-12 复检确认）。
        if self.last_truth_s is not None:
            self.max_truth_gap_s = max(self.max_truth_gap_s, stamp_s - self.last_truth_s)
        self.last_truth_s = stamp_s
        self.truth_frames.append((stamp_s, msg))
        if len(self.truth_frames) > 200:
            del self.truth_frames[:100]
        self._check_stimulus(msg)

    def _check_stimulus(self, msg):
        """Watch the *stimulus*, not the perception: are the targets still on the ground?

        ⚠️ 这不是感知判据，是**判据的前提条件**。它回答的是
           「这一跑到底有没有在考感知」，与「感知考得好不好」是两件事。
        """
        for obstacle in msg.obstacles:
            # 所有目标的真值 z 都是**包围盒中心**，贴地物体的包围盒中心恰好在 height/2。
            lift_m = abs(obstacle.pose.position.z - 0.5 * obstacle.size_m.z)
            # 倾斜角 = 物体自身 z 轴与世界 z 轴的夹角。用它而不是分别看
            # roll/pitch：那两个量依赖 yaw 的取法，而这个不依赖，
            # 任意朝向的目标都能用同一个阈值判。
            q = obstacle.pose.orientation
            up_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
            if lift_m > self.stimulus_max_lift_m or tilt_deg > self.stimulus_max_tilt_deg:
                self.stimulus_worst = (
                    f'id={obstacle.id} 离地 {lift_m:.3f} m、倾斜 {tilt_deg:.2f}°')
            self.stimulus_max_lift_m = max(self.stimulus_max_lift_m, lift_m)
            self.stimulus_max_tilt_deg = max(self.stimulus_max_tilt_deg, tilt_deg)

    def stimulus_ok(self) -> bool:
        """Whether the run is even valid — i.e. the targets stayed on the ground.

        :return: 刺激物完好为 True
        """
        return (self.stimulus_max_lift_m <= STIMULUS_MAX_LIFT_M and
                self.stimulus_max_tilt_deg <= STIMULUS_MAX_TILT_DEG)

    def _ego_position(self):
        """Ego position in map, or None when TF is not available yet."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _on_perceived(self, msg):
        if not self.truth_frames:
            return
        ego = self._ego_position()
        if ego is None:
            return
        t = stamp_to_seconds(msg.header.stamp)
        truth = min(self.truth_frames, key=lambda item: abs(item[0] - t))
        if abs(truth[0] - t) > 0.15:
            return
        self.perceived_frames += 1
        self.last_perceived_s = t

        # ---- 全局最优配对（2026-08-12 复检修复）----------------------------
        # ⚠️ 原来是按真值遍历顺序的**贪心**最近邻：车（列表在前）可以抢走
        #    行人的航迹 —— 两目标间距 1.75 m < 配对半径 2.5 m，抢错之后
        #    符号/ID/检测率统计记的都是错误的配对。
        #    真值最多 2–3 个，穷举全排列即全局最优，不需要匈牙利。
        assignment = optimal_assignment(
            truth[1].obstacles, msg.obstacles, self.match_radius_m)
        for gt_index, gt in enumerate(truth[1].obstacles):
            gx, gy = gt.pose.position.x, gt.pose.position.y
            distance_m = math.hypot(gx - ego[0], gy - ego[1])
            best = assignment.get(gt_index)
            best_distance = (math.hypot(
                msg.obstacles[best].pose.position.x - gx,
                msg.obstacles[best].pose.position.y - gy) if best is not None else None)

            # 遮挡标记（判据 ⑥ 用）：自车→本目标的视线穿过**另一个**真值的盒子。
            occluded = any(
                other is not gt and segment_intersects_box(ego, (gx, gy), other)
                for other in truth[1].obstacles)

            # 真值方向历史（符号判据排除 ① 用，推导见 SIGN_* 常量）。
            # ⚠️ **每帧都记，与配没配上无关**：目标在 U 转期间恰好漏检、
            #    转完立刻重现的话，只在配上时记的历史里没有转向样本，
            #    排除条件恒假 —— 恰好在最需要它的工况下失效。
            gvx, gvy = gt.velocity_mps.x, gt.velocity_mps.y
            turning = False
            if math.hypot(gvx, gvy) > 0.5:
                cur_dir = math.atan2(gvy, gvx)
                history = self.gt_dir_history[gt.id]
                turning = any(
                    abs((cur_dir - past_dir + math.pi) % (2.0 * math.pi) - math.pi)
                    > math.radians(SIGN_TURN_EXCLUDE_DEG)
                    for past_t, past_dir in history
                    if t - past_t <= SIGN_TURN_WINDOW_S)
                history.append((t, cur_dir))
                del history[:max(0, len(history) - 40)]

            row = {
                't_s': f'{t:.3f}',
                'gt_id': gt.id,
                'gt_class': CLASS_NAMES.get(gt.classification, '?'),
                'range_m': f'{distance_m:.3f}',
                'detected': 0,
                'position_error_m': '',
                'velocity_error_mps': '',
                'velocity_sign_ok': '',
                'perceived_id': '',
                'perceived_class': '',
                # ---- 诊断列（不参与判据）------------------------------------
                # ⚠️ 把位置误差**沿视线分解**，这是分辨两种误差的唯一办法：
                #    「可见面伪影」是**沿视线、朝向自车**的（雷达打不到背面，
                #    拟合出的盒子中心被拉向传感器）⟹ err_along 系统性为正；
                #    真实的估计误差没有方向偏好 ⟹ 两个分量都在零附近抖。
                #    只看模长的话这两者**完全分不开**，而它们的修法截然相反。
                'err_along_m': '',   # 沿自车→目标视线，正 = 感知比真值**更近**
                'err_cross_m': '',   # 垂直于视线
                'perceived_len_m': '',
                'perceived_wid_m': '',
                'perceived_hgt_m': '',
                'gt_len_m': f'{gt.size_m.x:.3f}',
                # 近边距离误差：自车到**包围盒最近点**的距离，感知 − 真值。
                # ⚠️ 这个量**物理上看得见**（近边正是雷达打到的那一面），
                #    而「中心」在正对时看不见。刹车距离依赖的也是这个。
                'near_edge_err_m': '',
                'occluded': int(occluded),
                # 同一个真值目标附近还有几个感知目标（>0 表示**重复航迹**）。
                # ⚠️ 重复航迹本身就是缺陷：规划会把一个目标当成两个障碍物。
                #    它也是 ID 切换的直接来源 —— 两条航迹都在，
                #    配对逐帧在它们之间摇摆，看起来像"ID 在跳"。
                'extra_within_match_m': '',
                'second_match_dist_m': '',
                # 真值速度与符号排除的取证列（P6-S0 收口加）：出了「符号错」
                # 要能离线复算它属不属于两条排除，而不是重跑一轮。
                'gt_vx_mps': f'{gvx:.3f}',
                'gt_vy_mps': f'{gvy:.3f}',
                'gt_turning': int(turning),
                'pairing_age': '',
            }
            # 落在配对半径内的**全部**感知目标（不只最近那个）
            nearby = sorted(
                math.hypot(o.pose.position.x - gx, o.pose.position.y - gy)
                for o in msg.obstacles)
            # ⚠️ 判"重复"要用**紧半径**，不能用配对半径。配对半径 2.5 m 比
            #    车与行人的真实间距（1.75 m）还大 —— 用它数出来的"重复"里
            #    绝大多数是**另一个目标**，实测第二近者中位 2.10 m 正是那个间距。
            #    判据的适用范围又一次没圈准，只是这次代价只有一次跑。
            nearby = [d for d in nearby if d < DUPLICATE_RADIUS_M]
            row['extra_within_match_m'] = max(0, len(nearby) - 1)
            if len(nearby) >= 2:
                row['second_match_dist_m'] = f'{nearby[1]:.3f}'
            if best is not None:
                obstacle = msg.obstacles[best]
                row['detected'] = 1
                row['position_error_m'] = f'{best_distance:.3f}'
                row['perceived_id'] = obstacle.id
                row['perceived_class'] = CLASS_NAMES.get(obstacle.classification, '?')
                if distance_m > 1e-6:
                    # 视线单位向量（自车 → 真值中心）
                    ux, uy = (gx - ego[0]) / distance_m, (gy - ego[1]) / distance_m
                    dx = obstacle.pose.position.x - gx
                    dy = obstacle.pose.position.y - gy
                    row['err_along_m'] = f'{-(dx * ux + dy * uy):.3f}'
                    row['err_cross_m'] = f'{dx * -uy + dy * ux:.3f}'
                row['perceived_len_m'] = f'{obstacle.size_m.x:.3f}'
                row['perceived_wid_m'] = f'{obstacle.size_m.y:.3f}'
                row['perceived_hgt_m'] = f'{obstacle.size_m.z:.3f}'
                row['near_edge_err_m'] = (
                    f'{distance_to_box(ego, obstacle) - distance_to_box(ego, gt):.3f}')
                pvx, pvy = obstacle.velocity_mps.x, obstacle.velocity_mps.y
                row['velocity_error_mps'] = f'{math.hypot(pvx - gvx, pvy - gvy):.3f}'
                # ⚠️ 符号**单独判**：符号反了大小照样对，而 P7 会因此
                #    认为"对方要开走"而不让行。只对确实在动的目标判。
                # ⚠️ 两侧都要过 0.5 m/s（2026-08-12 扩窗实测后补）：
                #    跟踪器自己的设计就写明 |v|<0.5 时方向由噪声主导、
                #    不做消歧（heading_min_speed_mps 的推导）——刚（重）建的
                #    航迹速度未成熟，那几帧的符号是噪声，判它等于判
                #    "初始化是不是瞬间完成"，而那不是符号判据要考的。
                # ⚠️ 另有两条机械排除（U 转过渡 / 配对交接），推导见 SIGN_* 常量。
                pairing = self.pairing.get(gt.id)
                if pairing is not None and pairing[0] == obstacle.id:
                    pairing[1] += 1
                else:
                    pairing = [obstacle.id, 0]
                    self.pairing[gt.id] = pairing
                row['pairing_age'] = pairing[1]
                if math.hypot(gvx, gvy) > 0.5 and math.hypot(pvx, pvy) > 0.5:
                    if turning:
                        self.sign_excluded_turn += 1
                    elif pairing[1] < SIGN_PAIRING_MIN_AGE:
                        self.sign_excluded_handover += 1
                    else:
                        row['velocity_sign_ok'] = int(pvx * gvx + pvy * gvy > 0.0)
                self.assigned_ids[gt.id].append(obstacle.id)
            self.rows.append(row)

        # ---- 虚警：没配上真值、**且在自车车道内**的感知目标 ----------------
        # ⚠️ 车道外的多余目标不算虚警 —— 路侧杆件是**真实存在**的障碍物，
        #    只是不在真值列表里。把它们算成虚警会让判据永远红。
        for index, obstacle in enumerate(msg.obstacles):
            if index in assignment.values():
                continue
            if abs(obstacle.pose.position.y - EGO_LANE_CENTER_Y_M) > EGO_LANE_HALF_WIDTH_M:
                continue
            distance_m = math.hypot(
                obstacle.pose.position.x - ego[0], obstacle.pose.position.y - ego[1])
            if distance_m <= CRITERION_MAX_RANGE_M:
                self.false_positives_in_lane += 1
                # 虚警必须**可回查**：只有计数的话，2 帧次的偶发虚警连
                # "发生在什么时候、是什么东西"都不知道，没法定位。
                self.false_positive_log.append(
                    (t, obstacle.pose.position.x, obstacle.pose.position.y,
                     obstacle.size_m.x, obstacle.size_m.y, obstacle.id))
            else:
                self.false_positives_out_of_range += 1

    def write(self, path: str) -> None:
        """Dump the per-frame rows so the numbers can be re-checked later."""
        if not self.rows:
            return
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        # 虚警明细**全量**落盘（P6-S0）。score() 只印前 6 条是给人扫一眼的；
        # 幻影缺陷（欠分割污染尺寸记忆）随机 0–5 帧次/轮，要跨多轮汇总
        # 才能看出规律 —— 只靠终端里那 6 行，跑完一关终端数据就没了。
        # 路径取主 CSV 旁边的 <名字>.fp.csv，没虚警时也写（只有表头），
        # 免得「文件不存在」和「没有虚警」两种情况分不开。
        fp_path = (path[:-4] if path.endswith('.csv') else path) + '.fp.csv'
        with open(fp_path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['t_s', 'x_m', 'y_m', 'length_m', 'width_m', 'id'])
            writer.writerows(self.false_positive_log)

    def score(self) -> bool:
        """Print the CP-P5-B table.

        :return: 全部通过为 True
        """
        print(f'\n===== CP-P5-B 感知实测（{self.perceived_frames} 帧配对）=====')
        if not self.rows:
            print('✗ 一条样本都没有 —— perception:=true 了吗？TF 通吗？')
            return False

        # ---- 按类别与距离分桶的检测率 ----
        print('\n检测率（按真值类别与距离；括号内是样本帧数）')
        header = '类别'.ljust(14) + ''.join(f'{lo}-{hi}m'.rjust(13) for lo, hi in BUCKETS)
        print(header)
        print('-' * len(header))
        rates = {}
        for class_name in sorted({row['gt_class'] for row in self.rows}):
            line = class_name.ljust(14)
            for bucket, (lo, hi) in enumerate(BUCKETS):
                subset = [
                    row for row in self.rows
                    if row['gt_class'] == class_name and lo <= float(row['range_m']) < hi]
                if not subset:
                    line += '—'.rjust(13)
                    continue
                rate = sum(row['detected'] for row in subset) / len(subset)
                rates[(class_name, bucket)] = rate
                line += f'{rate * 100:>7.1f}%({len(subset):>3})'
            print(line)

        # ⚠️ **判据只在适用范围内算**，理由与推导见 CRITERION_MAX_RANGE_M。
        #    范围外的值一并打印但**不判** —— 藏起来会让人以为远处也有同样的保证。
        in_range = [r for r in self.rows
                    if CRITERION_MIN_RANGE_M <= float(r['range_m']) <= CRITERION_MAX_RANGE_M]
        detected = [row for row in in_range if row['detected']]
        all_detected = [row for row in self.rows if row['detected']]
        ok = True

        def reference(key):
            """同一个量在**全距离**上的 95 分位，只作参考。"""
            values = sorted(abs(float(r[key])) for r in all_detected if r.get(key))
            return values[int(len(values) * 0.95)] if values else float('nan')

        def check(label, value, limit, unit='', greater_is_better=False, ref=None):
            nonlocal ok
            passed = value > limit if greater_is_better else value < limit
            ok = ok and passed
            arrow = '>' if greater_is_better else '<'
            tail = f'  {unit}' if ref is None else f'  {unit}   (全距离 {ref:.4f})'
            print(f'{label:<34}{value:>10.4f}{arrow:>4} {limit:<8} '
                  f'{"PASS" if passed else "FAIL"}{tail}')

        print(f'\n判据适用范围：**≤ {CRITERION_MAX_RANGE_M:.0f} m**'
              f'（{len(in_range)}/{len(self.rows)} 帧次在内；理由见脚本里的推导）')
        print('\n项                                     实测    判据  结果')
        # ① NPC 车检测率（20 m 内）
        vehicle_close = [
            row for row in self.rows
            if row['gt_class'] == 'VEHICLE' and
            CRITERION_MIN_RANGE_M <= float(row['range_m']) < 20.0]
        if vehicle_close:
            check('NPC 车检测率 (20 m 内)',
                  sum(r['detected'] for r in vehicle_close) / len(vehicle_close),
                  0.95, f'n={len(vehicle_close)}', greater_is_better=True)
        # ② 行人检测率（15 m 内）
        ped_close = [
            row for row in self.rows
            if row['gt_class'] == 'PEDESTRIAN' and
            CRITERION_MIN_RANGE_M <= float(row['range_m']) < 15.0]
        if ped_close:
            check('行人检测率 (15 m 内)',
                  sum(r['detected'] for r in ped_close) / len(ped_close),
                  0.90, f'n={len(ped_close)}', greater_is_better=True)
        # ③ 位置误差 —— **拆成两条可观测的量**，阈值仍是 0.5 m 一字不改。
        #
        # ⚠️ 原来那条量的是「包围盒中心的距离」，而**中心在正对时物理上看不见**：
        #    雷达打不到背面，拟出的盒子中心落在车头上，误差恒等于半个车长
        #    （实测 2.15/2.22，真值半长 2.20）。那条判据要求的是做不到的事，
        #    与 CP-P2-B 那条「a_lat 量的是路不是车」同一族。
        #
        #    换成的两个量都**看得见**，而且正是行车需要的：
        #      · 近边距离误差 —— 近边正是雷达打到的那一面，刹车距离依赖它；
        #      · 横向误差     —— 决定「它在哪条车道」。
        #    实测（三轮）：近边 p95 0.364，横向 p95 0.327，中心 p95 2.235。
        #
        # ⚠️ 这**不是放宽**：阈值没动，而且两条都要过（原来只有一条）。
        #    真正被放弃的只有「远处正对目标的纵深」，那一项没有任何传感器信息。
        if detected:
            near = sorted(abs(float(r['near_edge_err_m'])) for r in detected
                          if r['near_edge_err_m'])
            if near:
                check('近边距离误差 (95 分位, m)', near[int(len(near) * 0.95)], 0.5, 'm',
                      ref=reference('near_edge_err_m'))
            cross = sorted(abs(float(r['err_cross_m'])) for r in detected if r['err_cross_m'])
            if cross:
                check('横向位置误差 (95 分位, m)', cross[int(len(cross) * 0.95)], 0.5, 'm',
                      ref=reference('err_cross_m'))
            # 中心误差仍然打印，但**不判**：留着是为了让下一个人一眼看到
            # 它有多大、以及为什么不拿它当判据。删掉的话这段历史就没了。
            centers = sorted(float(r['position_error_m']) for r in detected)
            print(f'{"  (参考) 中心误差 95 分位":<34}{centers[int(len(centers) * 0.95)]:>10.4f}'
                  f'     不判据    —— 正对时中心不可观测，见上方注释')
        # ④ 速度误差 + 符号
        moving = [r for r in detected if r['velocity_error_mps']]
        if moving:
            v_errors = sorted(float(r['velocity_error_mps']) for r in moving)
            check('速度误差 (95 分位, m/s)', v_errors[int(len(v_errors) * 0.95)], 1.0, 'm/s',
                  ref=reference('velocity_error_mps'))
        signed = [r for r in detected if r['velocity_sign_ok'] != '']
        if signed:
            wrong = sum(1 for r in signed if int(r['velocity_sign_ok']) == 0)
            # 排除数照印：排除一旦不可见就会悄悄变成过滤（SPEC §11 禁的那类）。
            print(f'{"速度符号错的样本":<34}{wrong:>10}   == 0        '
                  f'{"PASS" if wrong == 0 else "FAIL"}  n={len(signed)}'
                  f'   (排除: U转过渡 {self.sign_excluded_turn}、'
                  f'配对交接 {self.sign_excluded_handover})')
            ok = ok and wrong == 0
        # ⑤ ID 切换 —— 在适用范围内重新统计（self.assigned_ids 是全距离的）
        # ⚠️ 中断超过「删除窗口 + 遮挡滑行上限」（0.5 + 3.0 = 3.5 s）后重入，
        #    **允许换 ID**（2026-08-12 扩窗实测后补）：目标离开量程 20 s 再
        #    回来时航迹按设计早已删除，新 ID 是**正确行为**不是缺陷。
        #    判据考的是「连续可跟踪期间的 ID 稳定」——系统设计上就保不住的
        #    中断不算。中断 ≤3.5 s 的换 ID 照旧计为切换。
        switches = 0
        by_gt_rows = defaultdict(list)
        for row in detected:
            by_gt_rows[row['gt_id']].append(row)
        for gt_rows in by_gt_rows.values():
            previous_id, previous_t = None, None
            for row in gt_rows:
                t_now = float(row['t_s'])
                if previous_id is not None and row['perceived_id'] != previous_id \
                        and t_now - previous_t <= 3.5:
                    switches += 1
                previous_id, previous_t = row['perceived_id'], t_now
        all_switches = sum(len(set(ids)) - 1 for ids in self.assigned_ids.values() if ids)
        print(f'{"ID 切换次数":<34}{switches:>10}   <= 2        '
              f'{"PASS" if switches <= 2 else "FAIL"}     (全距离 {all_switches})')
        ok = ok and switches <= 2
        # ⑥ 遮挡后 ID 保持（plan 表第 6 条 —— 2026-08-12 前从未实现，
        #    编号从⑤直接跳到⑦就是那次遗漏留下的痕迹，故意保留提醒）
        #
        # 遮挡窗口 = 某真值被另一真值挡住视线的连续帧段（occluded 列）。
        # 判据：窗口前最后一次配到的感知 ID == 窗口后第一次配到的感知 ID。
        # ⚠️ 一个遮挡事件都没有 ⟹ 判据没被激励 ⟹ **FAIL 而不是空过** ——
        #    「场景先于算法」：dynamic:=both 的设计目的就是互相遮挡，
        #    没发生说明场景变了，绿灯只会骗人。
        occlusion_windows = 0
        occlusion_violations = 0
        by_gt = defaultdict(list)
        for row in in_range:
            by_gt[row['gt_id']].append(row)
        for gt_rows in by_gt.values():
            previous_id = None
            previous_t = None
            in_window = False
            pre_window_id = None
            for row in gt_rows:
                if row.get('occluded'):
                    if not in_window:
                        in_window = True
                        # ⚠️ 窗前 ID 必须**够新**（≤1 s）：目标消失 30 s 后才被
                        #    遮挡再出现，窗前那个 ID 早已按设计删除 ——
                        #    拿它判"遮挡保持"判的是别的东西（扩窗实测踩到）。
                        recent = (previous_id is not None and previous_t is not None and
                                  float(row['t_s']) - previous_t <= 1.0)
                        pre_window_id = previous_id if recent else None
                elif in_window:
                    # 窗口结束后第一次真的配上才结算
                    if row['detected'] and row['perceived_id'] != '':
                        if pre_window_id is not None:
                            occlusion_windows += 1
                            if row['perceived_id'] != pre_window_id:
                                occlusion_violations += 1
                        in_window = False
                        pre_window_id = None
                if row['detected'] and row['perceived_id'] != '':
                    previous_id = row['perceived_id']
                    previous_t = float(row['t_s'])
        occl_ok = occlusion_windows >= 1 and occlusion_violations == 0
        print(f'{"遮挡后 ID 保持":<34}{occlusion_violations:>10}   == 0        '
              f'{"PASS" if occl_ok else "FAIL"}     (窗口 {occlusion_windows} 个'
              + ('' if occlusion_windows else ' —— 场景没激励这条判据！') + ')')
        ok = ok and occl_ok

        # ⑦ 车道内虚警
        print(f'{"车道内虚警帧次":<34}{self.false_positives_in_lane:>10}   == 0        '
              f'{"PASS" if self.false_positives_in_lane == 0 else "FAIL"}'
              f'     (范围外另有 {self.false_positives_out_of_range})')
        for t, x, y, length, width, pid in self.false_positive_log[:6]:
            print(f'    虚警明细: t={t:.1f} map=({x:.1f},{y:.1f}) '
                  f'{length:.2f}×{width:.2f} id={pid}')
        ok = ok and self.false_positives_in_lane == 0

        print('\n全部通过' if ok else '\n有判据未通过')
        return ok


def main() -> int:
    """Entry point.

    :return: 0 = 全部判据通过；1 = 有判据未通过；2 = **本次运行无效**（刺激物坏了）
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=45.0)
    parser.add_argument('--out', default='/tmp/p5_perception.csv')
    parser.add_argument(
        '--match-radius-m', type=float, default=MATCH_RADIUS_M,
        help='与真值配对的最大距离。⚠️ 它与「位置误差」判据是两回事，见文件头')
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionScorer(args.match_radius_m)
    deadline = time.monotonic() + args.duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.write(args.out)
    print(f'CSV: {args.out}')
    print(f'虚警明细: {(args.out[:-4] if args.out.endswith(".csv") else args.out)}.fp.csv')

    # ⚠️ 刺激物自检**在打分之前**，而且不达标时**不打分** ——
    #    刺激物坏了的时候那张判据表上的每一个数都是没有意义的，
    #    印出来只会让人拿着它去查感知（那正是上次踩的坑）。
    stimulus_line = (f'刺激物：最大离地 {node.stimulus_max_lift_m:.3f} m'
                     f'（限 {STIMULUS_MAX_LIFT_M}），'
                     f'最大倾斜 {node.stimulus_max_tilt_deg:.2f}°'
                     f'（限 {STIMULUS_MAX_TILT_DEG}）')
    if not node.stimulus_ok():
        print('\n✗✗ 本次运行**无效**：目标道具在仿真里坏掉了，不是感知不达标 ✗✗')
        print(f'   {stimulus_line}')
        print(f'   最差：{node.stimulus_worst}')
        print('   查 models/npc_car|pedestrian/model.sdf 与 npc_controller，'
              '不要查 ads_perception。')
        node.destroy_node()
        rclpy.shutdown()
        return 2
    print(f'{stimulus_line} ✓')

    # ---- 真值流健康度：断流 ⟹ 本次运行无效（2026-08-12 复检补上）----------
    # 断流时段里配对与刺激物自检都静默停止 —— 残缺时段打出的分不可信，
    # 与「刺激物坏了还打分」同一类，同样拒绝打分而不是打折。
    gap_bad = node.max_truth_gap_s > 1.0
    tail_bad = (node.last_truth_s is not None and node.last_perceived_s is not None and
                node.last_perceived_s - node.last_truth_s > 1.0)
    if gap_bad or tail_bad:
        print('\n✗✗ 本次运行**无效**：真值流断过 ✗✗')
        print(f'   最大帧间隔 {node.max_truth_gap_s:.2f} s（限 1.0）'
              + ('，且结尾缺真值 %.2f s' % (node.last_perceived_s - node.last_truth_s)
                 if tail_bad else ''))
        print('   查 obstacle_truth / parameter_bridge，不要看下面的数字。')
        node.destroy_node()
        rclpy.shutdown()
        return 2
    print(f'真值流：最大帧间隔 {node.max_truth_gap_s:.2f} s ✓')
    dup = [r for r in node.rows if r.get('extra_within_match_m')]
    if dup:
        seconds = sorted(float(r['second_match_dist_m']) for r in dup if r['second_match_dist_m'])
        print(f'重复航迹：{len(dup)} / {len(node.rows)} 帧次的真值目标 '
              f'{DUPLICATE_RADIUS_M} m 内有 2 个以上感知目标'
              + (f'；第二近者距离 中位 {seconds[len(seconds) // 2]:.2f} m、'
                 f'最大 {seconds[-1]:.2f} m' if seconds else ''))

    passed = node.score()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
