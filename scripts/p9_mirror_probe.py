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

"""点云 y 符号回归探针：非对称地标（NPC 车）上对比正窗/镜像窗点数.

⚠️ **为什么必须有它**：路与护栏墙左右对称 —— 镜像的世界与正确的世界在
一切对称统计上不可区分。y 符号错了，S04/行为判据照样全绿（它们走真值），
直到点云的消费者（P5 感知）上线才以「检测率 0%+车道内虚警」的面目爆炸。
P9-S1 实测：翻错时 npc 车 60 帧正窗 0 点、镜像窗 58054 点。

用法：CARLA 栈（dynamic:=both 或任何有 npc_car 的场景）旁边跑：
    python3 scripts/p9_mirror_probe.py
判据：正窗点数 ≫ 镜像窗（比值 > 10）= PASS；反之 = 点云 y 反了。
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main():
    """入口."""
    rclpy.init()
    node = Node('mirror_probe')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    state = {}
    rows = []

    def on_cloud(msg):
        if 'npc' not in state or 'ego' not in state:
            return
        ego_x, ego_y = state['ego']
        npc_x, npc_y = state['npc']
        body_x, body_y = npc_x - ego_x, npc_y - ego_y  # 自车 yaw≈0 的直道上有效
        if math.hypot(body_x, body_y) > 25.0:
            return
        pts = []
        for i in range(0, len(msg.data) - msg.point_step + 1, msg.point_step):
            x, y, _ = struct.unpack_from('<fff', msg.data, i)
            if math.isfinite(x):
                pts.append((x, y))
        plus = sum(1 for x, y in pts if math.hypot(x - body_x, y - body_y) < 3.0)
        minus = sum(1 for x, y in pts if math.hypot(x - body_x, y + body_y) < 3.0)
        rows.append((plus, minus))

    node.create_subscription(PointCloud2, '/lidar/points', on_cloud, 10)
    node.create_subscription(
        Odometry, '/model/npc_car/pose_gt',
        lambda m: state.__setitem__('npc', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    node.create_subscription(
        Odometry, '/ego_pose_gt',
        lambda m: state.__setitem__('ego', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)

    start = time.monotonic()
    while time.monotonic() - start < 90 and len(rows) < 60:
        rclpy.spin_once(node, timeout_sec=0.2)
    plus = sum(r[0] for r in rows)
    minus = sum(r[1] for r in rows)
    print(f'正窗 {plus} 点，镜像窗 {minus} 点（{len(rows)} 帧）')
    if plus > 10 * max(minus, 1):
        print('PASS：点云 y 符号正确')
    elif minus > 10 * max(plus, 1):
        print('FAIL：点云是镜像的 —— sidecar 中继的 y 符号反了')
    else:
        print('INCONCLUSIVE：npc 没进近场或两窗都空 —— 换有 npc 的场景重跑')


if __name__ == '__main__':
    main()
