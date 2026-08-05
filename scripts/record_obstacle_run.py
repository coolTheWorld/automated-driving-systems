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

"""
Record one CP-P3-B obstacle run and score it against the checkpoint criteria.

    ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false obstacles:=avoid
    python3 scripts/record_obstacle_run.py --scenario avoid --out /tmp/p3_avoid.csv

与 `record_control_run.py` 的分工：那个验 CP-P2-B 的**跟踪质量**（横向误差、
速度、终点），本脚本验 CP-P3-B 的**避障行为**（间距、碰撞、不越线、回中心线、
不可行时停住）。两个都要跑 —— 绕障绕对了但把跟踪弄坏了，同样不算通过。

⚠️ **间距用精确的多边形距离，不用分离轴的最大间隙。**
   后者只是下界，在**顶点对顶点**（车刚越过障碍物）的位形下会低估 0.2 m 量级，
   于是判据假失败，而系统其实是对的。P3-S5 已经被它咬过一次，
   见 CLAUDE.md 陷阱表「拿 SAT 的最大间隙当"保守估计"去做判据」。

⚠️ **障碍物坐标读 config/obstacles.yaml，不在这里再写一份。**
   那是场景的唯一来源（Gazebo 模型由 gen_obstacles.py 从它生成，
   真值发布器也读它）。在这儿抄一份就是第四份数据。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys
import time

from ads_msgs.msg import Trajectory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent

# 自车起点与 CP-P2-B 用的**同一条 560 m 路线**的终点。
# 用同一条路线是有意的：绕障场景与回归场景的数直接可比。
GOAL_X_M = 91.75
GOAL_Y_M = 20.0


def load_config():
    """
    Load the scenario definition and the vehicle/planner numbers it is scored against.

    :return: (obstacles.yaml, vehicle_params.yaml, planning_params.yaml, campus_map.yaml)
    """
    def read(name):
        return yaml.safe_load((REPO_ROOT / 'config' / name).read_text(encoding='utf-8'))
    return (read('obstacles.yaml'), read('vehicle_params.yaml'),
            read('planning_params.yaml'), read('campus_map.yaml'))


def corners(center_x_m, center_y_m, yaw_rad, length_m, width_m):
    """
    Return the four corners in winding order (adjacent pairs are real edges).

    :return: 四个角点
    """
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    half_l, half_w = 0.5 * length_m, 0.5 * width_m
    return [(center_x_m + dx * cos_yaw - dy * sin_yaw,
             center_y_m + dx * sin_yaw + dy * cos_yaw)
            for dx, dy in ((half_l, half_w), (half_l, -half_w),
                           (-half_l, -half_w), (-half_l, half_w))]


def point_to_segment(point, from_point, to_point):
    """
    Return the minimum distance from a point to a segment.

    :return: 距离，米
    """
    dx, dy = to_point[0] - from_point[0], to_point[1] - from_point[1]
    length_squared = dx * dx + dy * dy
    ratio = 0.0
    if length_squared > 0.0:
        ratio = ((point[0] - from_point[0]) * dx + (point[1] - from_point[1]) * dy)
        ratio = max(0.0, min(1.0, ratio / length_squared))
    return math.hypot(point[0] - (from_point[0] + ratio * dx),
                      point[1] - (from_point[1] + ratio * dy))


def polygon_distance(poly_a, poly_b):
    """
    Return the exact minimum distance between two disjoint convex polygons.

    遍历所有（顶点, 边）对 —— 对不相交的凸多边形是精确的（最近点对中至少
    有一个必在顶点上）。**不要退回用分离轴的最大间隙**，理由见模块文档。

    :return: 距离，米
    """
    best = float('inf')
    for first, second in ((poly_a, poly_b), (poly_b, poly_a)):
        for point in first:
            for i in range(len(second)):
                best = min(best, point_to_segment(
                    point, second[i], second[(i + 1) % len(second)]))
    return best


def overlaps(poly_a, poly_b, yaw_a, yaw_b):
    """
    Separating axis theorem: disjoint on any axis means no overlap.

    :return: 是否相交
    """
    for angle in (yaw_a, yaw_a + math.pi / 2, yaw_b, yaw_b + math.pi / 2):
        axis = (math.cos(angle), math.sin(angle))
        proj_a = [p[0] * axis[0] + p[1] * axis[1] for p in poly_a]
        proj_b = [p[0] * axis[0] + p[1] * axis[1] for p in poly_b]
        if min(proj_a) > max(proj_b) or min(proj_b) > max(proj_a):
            return False
    return True


class ObstacleRecorder(Node):
    """Publish one goal and record clearance / offset / planner state every tick."""

    def __init__(self, scenario, goal_delay_s, timeout_s):
        super().__init__('obstacle_run_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

        obstacles_cfg, vehicle, planning, campus = load_config()
        lane = obstacles_cfg['lane']
        spec = obstacles_cfg['scenarios'][scenario]

        geometry = vehicle['geometry']
        self.vehicle_length_m = float(geometry['length_m'])
        self.vehicle_width_m = float(geometry['width_m'])
        # 后轴 → 车体几何中心。**推导量，不让人填**（填错整套间距会偏 1.35 m）。
        self.rear_axle_to_center_m = (
            0.5 * self.vehicle_length_m - float(geometry['rear_overhang_m']))
        self.safety_margin_m = float(planning['safety']['margin_m'])
        self.stop_margin_m = float(planning['safety']['stop_margin_m'])
        self.lane_half_width_m = 0.5 * float(campus['lanes']['width_m'])
        self.lane_center_y_m = float(lane['center_y_m'])
        # 这条直道在世界坐标里的范围，**读 obstacles.yaml 的声明，不自己发明过滤条件**。
        # 「不越车道边线」只在这段里有意义：出了直道进弯角之后，
        # `y − lane_center_y` 根本不是横向偏移，而车在弯角上的 yaw 又大，
        # 硬算出来的"余量"是 −3 m 这种荒谬值。初版就是这么误报 FAIL 的。
        self.straight_from_x_m = float(lane['valid_from_x_m'])
        self.straight_to_x_m = float(lane['valid_to_x_m'])

        self.obstacles = [
            (float(o['along_x_m']),
             self.lane_center_y_m + float(o['lateral_offset_m']),
             float(o['length_m']), float(o['width_m']))
            for o in spec['obstacles']
        ]
        self.expect = spec['expect']

        self.rows = []
        self.planner_status = {}
        self.goal_delay_s = goal_delay_s
        self.timeout_s = timeout_s
        self.goal_sent = False
        self.start_wall = time.monotonic()
        # 车是否真的动过。见 main() 里「停住就收工」那段的说明。
        self.has_moved = False

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # ⚠️ **位姿必须走 TF，不能直接拿 /odom 的 pose 当 map 系用。**
        #    真实栈里 map→odom **不是单位变换** —— Gazebo 的 AckermannSteering
        #    把 odom 原点放在自车 spawn 处（本项目 (30, −51.75)），
        #    所以 /odom 的位姿差整整一个 spawn 偏移。
        #    本脚本初版就是这么错的：记出来的轨迹从 (0,0) 起，
        #    于是"车离障碍物 52 m"——看着像车根本没经过障碍物，
        #    而实际它正正经经绕了过去。**没有任何一层会报错。**
        #    （L3-G 里可以直接用 /odom，因为那边假车就在 map 系里积分、
        #      map→odom 确实是单位变换 —— 那条豁免只对假车成立。）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.create_subscription(
            DiagnosticArray, '/planning/diagnostics', self._on_planning, 100)
        self.create_subscription(
            DiagnosticArray, '/control/diagnostics', self._on_control, 200)
        self.create_subscription(Trajectory, '/planning/trajectory', self._on_trajectory, 10)
        self.latest_control = {}
        self.latest_trajectory_status = None

    def _on_planning(self, msg):
        self.planner_status = {kv.key: kv.value for kv in msg.status[0].values}
        self.planner_status['message'] = msg.status[0].message
        self.planner_status['level'] = msg.status[0].level

    def _on_control(self, msg):
        self.latest_control = {kv.key: kv.value for kv in msg.status[0].values}

    def _on_trajectory(self, msg):
        self.latest_trajectory_status = msg.status

    def _on_odom(self, msg):
        # /odom 只用来取**纵向车速**（它在 base_link 系，与坐标系无关）。
        # 位姿一律走 TF（SPEC §5：禁止手写变换矩阵）。
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return

        class _Pose:
            pass
        pose = _Pose()
        pose.position = transform.transform.translation
        pose.orientation = transform.transform.rotation
        quaternion = pose.orientation
        # 平面运动，roll = pitch = 0，所以这个式子是精确的。
        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
        body_x = pose.position.x + self.rear_axle_to_center_m * math.cos(yaw)
        body_y = pose.position.y + self.rear_axle_to_center_m * math.sin(yaw)
        body = corners(body_x, body_y, yaw, self.vehicle_length_m, self.vehicle_width_m)

        clearance_m = float('inf')
        collided = False
        for obs_x, obs_y, obs_length, obs_width in self.obstacles:
            obstacle = corners(obs_x, obs_y, 0.0, obs_length, obs_width)
            if overlaps(body, obstacle, yaw, 0.0):
                collided = True
                clearance_m = 0.0
            else:
                clearance_m = min(clearance_m, polygon_distance(body, obstacle))

        # 车体外廓的横向占用（世界 y 轴上的投影半径）—— 判「不越车道边线」用。
        #
        # ⚠️ **这一项只在南侧直道上有意义**（范围读 obstacles.yaml 的声明）。
        #    本项目的验收路线绕过环线的弯角，离开直道之后 `y − lane_center_y`
        #    根本不是横向偏移，而弯角上 yaw 又大，硬算出来是 −3 m 这种荒谬值。
        #    初版没限定范围，于是判据 FAIL —— **而系统其实是对的**。
        #    判据算错了和系统错了，现场长得一模一样，这是本轮踩到的第二次。
        half_lateral_m = (0.5 * self.vehicle_width_m * abs(math.cos(yaw)) +
                          0.5 * self.vehicle_length_m * abs(math.sin(yaw)))
        lateral_offset_m = body_y - self.lane_center_y_m
        on_south_straight = self.straight_from_x_m <= body_x <= self.straight_to_x_m

        if abs(msg.twist.twist.linear.x) > 1.0:
            self.has_moved = True

        self.rows.append({
            'time_s': self.get_clock().now().nanoseconds * 1e-9,
            'rear_x_m': pose.position.x,
            'rear_y_m': pose.position.y,
            'yaw_rad': yaw,
            'speed_mps': msg.twist.twist.linear.x,
            'lateral_offset_m': lateral_offset_m,
            'on_south_straight': int(on_south_straight),
            'lane_edge_margin_m': (self.lane_half_width_m - abs(lateral_offset_m) - half_lateral_m
                                   if on_south_straight else ''),
            'clearance_m': clearance_m,
            'collided': int(collided),
            'planner_lateral_offset_m': self.planner_status.get('lateral_offset_m', ''),
            'planner_message': self.planner_status.get('message', ''),
            'blocked_count': self.planner_status.get('blocked_count', ''),
            'candidate_count': self.planner_status.get('candidate_count', ''),
            'control_state': self.latest_control.get('state', ''),
            'control_lateral_error_m': self.latest_control.get('lateral_error_m', ''),
        })

    def tick(self):
        """Publish the goal once the stack has settled; return False when done."""
        elapsed = time.monotonic() - self.start_wall
        if not self.goal_sent and elapsed > self.goal_delay_s:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = GOAL_X_M
            goal.pose.position.y = GOAL_Y_M
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.get_logger().info('已发布目标点 (%.2f, %.2f)' % (GOAL_X_M, GOAL_Y_M))
        return elapsed < self.timeout_s


def score(recorder, scenario):
    """
    Score the run against the CP-P3-B criteria table (tasks/plan.md).

    :return: (是否全部通过, 打印用的行列表)
    """
    # ⚠️ **两套样本集，分开用，混了就出假失败。**
    #
    #    `rows` = 只有控制器在跟踪的拍。**跟踪质量**类的量（横向偏移、车道余量）
    #    只在这里面有意义 —— 车都没在跟轨迹，问"它偏了多少"是没有定义的。
    #
    #    `all_rows` = 整段记录。**安全与末态**类的量（有没有撞、最近到过多少、
    #    最后停没停住）必须在这里面算：车在非跟踪状态下照样在动、照样会撞。
    #
    #    这条是实测踩出来的（CP-P3-B block 场景）：block 场景的正确结局是
    #    「planner 报绕不过去 → 控制器跟停车轨迹 → 到停车点报 GOAL_REACHED →
    #      停车轨迹退化成零长度 → 控制器落到 NO_PATH 降级分支刹停并保持」。
    #    于是**最后 449 拍全是 NO_PATH**，而车正是在这 449 拍里从 0.95 m/s
    #    刹到 0.000。拿 `rows[-1]` 当末速，量到的是**刹车过程中间的一个值**，
    #    判据报 0.950 FAIL，而车其实稳稳停在障碍物前 1.918 m。
    #    **假失败与真失败在现场长得一模一样**，这一条在 CLAUDE.md 里已经写过一次了。
    rows = [r for r in recorder.rows if r['control_state'] in ('TRACKING', 'GOAL_REACHED')]
    all_rows = recorder.rows
    if not rows:
        return False, [('没有 TRACKING 样本', '-', '-', 'FAIL', '控制器起来了吗？')]

    min_clearance_m = min(r['clearance_m'] for r in all_rows)
    collisions = sum(r['collided'] for r in all_rows)
    on_lane = [r for r in rows if r['on_south_straight']]
    min_lane_edge_margin_m = (min(r['lane_edge_margin_m'] for r in on_lane)
                              if on_lane else float('nan'))
    max_planner_offset_m = max(
        abs(float(r['planner_lateral_offset_m'])) for r in rows if r['planner_lateral_offset_m'])
    final_planner_offset_m = abs(float(rows[-1]['planner_lateral_offset_m'] or 0.0))
    final_speed_mps = abs(all_rows[-1]['speed_mps'])
    stopping_ticks = sum(1 for r in all_rows if '绕不过去' in r['planner_message'])

    lines = []

    def check(name, value, ok, limit_text, note=''):
        lines.append((name, value, limit_text, 'PASS' if ok else 'FAIL', note))
        return ok

    passed = True
    passed &= check('碰撞次数', '%d' % collisions, collisions == 0, '0')

    if scenario == 'avoid':
        passed &= check(
            '最小侧向间距 (m)', '%.3f' % min_clearance_m,
            min_clearance_m > recorder.safety_margin_m,
            '> %.2f' % recorder.safety_margin_m, 'SPEC §8 S04')
        passed &= check(
            '规划器最大横向偏移 (m)', '%.3f' % max_planner_offset_m,
            max_planner_offset_m > 0.1, '> 0.10', '证明它确实决定了绕行')
        passed &= check(
            '不越车道边线：最小余量 (m)', '%.3f' % min_lane_edge_margin_m,
            min_lane_edge_margin_m > 0.0, '> 0',
            'ODD 不允许压对向（仅南侧直道 %d 拍）' % len(on_lane))
        passed &= check(
            '绕完回到中心线 (m)', '%.3f' % final_planner_offset_m,
            final_planner_offset_m < 0.10, '< 0.10', 'w_c < w_o 的整链路验证')
    else:
        passed &= check(
            '规划器报告不可行的拍数', '%d' % stopping_ticks, stopping_ticks > 0, '> 0',
            '「车停了」与「车挂了」必须分得开')
        passed &= check(
            '终速 (m/s)', '%.3f' % final_speed_mps, final_speed_mps < 0.1, '< 0.10',
            '停住不是慢慢蹭；取**整段**末拍，车是在 NO_PATH 里刹完的')
        passed &= check(
            '停车点到障碍物间距 (m)', '%.3f' % min_clearance_m,
            min_clearance_m >= recorder.safety_margin_m,
            '>= %.2f' % recorder.safety_margin_m)
    return passed, lines


def main() -> int:
    """
    Entry point.

    :return: 进程退出码
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True, choices=['avoid', 'block'])
    parser.add_argument('--goal-delay-s', type=float, default=8.0)
    parser.add_argument('--timeout-s', type=float, default=120.0)
    parser.add_argument('--out', default='/tmp/obstacle_run.csv')
    args = parser.parse_args()

    rclpy.init()
    recorder = ObstacleRecorder(args.scenario, args.goal_delay_s, args.timeout_s)
    try:
        while rclpy.ok() and recorder.tick():
            rclpy.spin_once(recorder, timeout_sec=0.05)
            rows = [r for r in recorder.rows if r['control_state'] == 'GOAL_REACHED']
            # 「停」场景不会到达终点，靠 timeout 收尾；「绕」场景到了就停。
            if args.scenario == 'avoid' and len(rows) > 40:
                break
            # ⚠️ 「停住就收工」必须先确认车**动过**。
            #    初版只看"最近 100 拍速度都近零"，而目标点是 8 s 后才发的 ——
            #    于是起步前的静止被当成"已经停住"，跑了 4 s 就退出，
            #    一条 TRACKING 样本都没有。**判据在错误的时刻成立，比不成立更难查。**
            if args.scenario == 'block' and recorder.has_moved and len(recorder.rows) > 200:
                if all(abs(r['speed_mps']) < 0.05 for r in recorder.rows[-100:]):
                    break
    finally:
        rclpy.shutdown()

    if not recorder.rows:
        print('一条 /odom 都没收到 —— 仿真起来了吗？', file=sys.stderr)
        return 1

    with open(args.out, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(recorder.rows[0]))
        writer.writeheader()
        writer.writerows(recorder.rows)

    passed, lines = score(recorder, args.scenario)
    print('\n===== CP-P3-B 场景「%s」（%d 拍）=====' % (args.scenario, len(recorder.rows)))
    print('CSV: %s' % args.out)
    print('%-30s %10s %12s  %s' % ('项', '实测', '判据', '结果'))
    for name, value, limit, verdict, note in lines:
        print('%-30s %10s %12s  %s  %s' % (name, value, limit, verdict, note))
    print('\n%s' % ('全部通过' if passed else '**有判据未通过**'))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
