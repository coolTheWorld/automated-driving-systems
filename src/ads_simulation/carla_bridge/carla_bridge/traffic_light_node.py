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

"""S06 虚拟红绿灯节点：相位机 → /traffic_light/state（ads_msgs/TrafficLight）.

不依赖 carla —— 灯是虚拟的（campus.xodr 无 signal，SPEC §4.1 把灯态数据源
设计在 sidecar 通道）。相位逻辑在 traffic_light_cycle.py（纯函数，L1 钉着）。

默认**不被任何 launch 拉起**：S06 只挂 L3-C（决策一），云机上的场景编排
按需起它。消费者 planning_node 的语义：从未出现 = 世界无灯（零行为差异）。
"""

import rclpy
from rclpy.node import Node

from ads_msgs.msg import TrafficLight

from carla_bridge.traffic_light_cycle import phase_at


class TrafficLightNode(Node):
    """相位机的 ROS 包装。时间基准 = 仿真钟（use_sim_time）."""

    def __init__(self):
        """声明参数并起定时器."""
        super().__init__('traffic_light')
        # 相位时长。红 25 s：自车从 goal 发布到停止线 ~40 m，巡航+减速 ≈ 12 s，
        # 红窗要盖住「到达 + 停稳 + 判据采样」；绿 20 s 够恢复通过。
        # ⚠️ 上机后按实际场景相位账微调（与 P7 dwell 相位账同一套做法）。
        self.declare_parameter('red_s', 25.0)
        self.declare_parameter('green_s', 20.0)
        self.declare_parameter('red_first', True)
        # 停止线在全局路由参考线上的弧长（米）。**无默认场景值** ——
        # 它与路线绑定，场景编排必须显式给，给错了判据会当场红（可观测）。
        self.declare_parameter('stop_line_s_m', 0.0)
        # 灯从这个仿真时刻起算相位（与 goal 锚点同一套绝对钟约定）。
        self.declare_parameter('phase_anchor_s', 0.0)
        self._pub = self.create_publisher(TrafficLight, '/traffic_light/state', 10)
        self.create_timer(0.5, self._tick)  # 2 Hz：灯态是低频轻量流

    def _tick(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s <= 0.0:
            return  # 仿真钟还没来
        phase = phase_at(
            now_s - self.get_parameter('phase_anchor_s').value,
            self.get_parameter('red_s').value,
            self.get_parameter('green_s').value,
            self.get_parameter('red_first').value)
        msg = TrafficLight()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.state = (
            TrafficLight.STATE_RED if phase == 'RED' else TrafficLight.STATE_GREEN)
        msg.stop_line_s_m = self.get_parameter('stop_line_s_m').value
        self._pub.publish(msg)


def main(args=None):
    """入口."""
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
