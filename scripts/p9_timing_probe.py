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

"""P9-S5 的第一把尺子：四个模块的每拍耗时分布（从各自的 /…/diagnostics 读，两环境同一把）.

在任何一轮场景旁边跑（Gazebo 或 CARLA 都行）：

    python3 scripts/p9_timing_probe.py --duration-s 200 --out /tmp/timing.csv

读的键（各节点已经在发，SPEC §7 的「不允许在回调中做重计算」要靠数字守）：
    /perception/diagnostics  total_ms ground_ms cluster_ms fit_ms track_ms
    /prediction/diagnostics  total_ms
    /planning/diagnostics    cycle_ms
    /control/diagnostics     cycle_time_ms
输出每键的 n / p50 / p95 / max，并把每拍原值落成 csv（topic,key,t,value）。
判据不在这里（S5 拆片时按两环境基线定）—— 这是测量仪。
"""

import argparse
import csv
import statistics
import time

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node

KEYS = {
    '/perception/diagnostics': ('total_ms', 'ground_ms', 'cluster_ms', 'fit_ms', 'track_ms'),
    '/prediction/diagnostics': ('total_ms',),
    '/planning/diagnostics': ('cycle_ms',),
    '/control/diagnostics': ('cycle_time_ms',),
}


def main():
    """入口."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--duration-s', type=float, default=200.0)
    parser.add_argument('--out', default='/tmp/timing.csv')
    args = parser.parse_args()

    rclpy.init()
    node = Node('p9_timing_probe')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    samples = {(topic, key): [] for topic, keys in KEYS.items() for key in keys}
    rows = []

    def make_cb(topic):
        def cb(msg):
            if not msg.status:
                return
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for kv in msg.status[0].values:
                if kv.key in KEYS[topic]:
                    try:
                        value = float(kv.value)
                    except ValueError:
                        continue
                    samples[(topic, kv.key)].append(value)
                    rows.append((topic, kv.key, t, value))
        return cb

    for topic in KEYS:
        node.create_subscription(DiagnosticArray, topic, make_cb(topic), 50)

    start = time.monotonic()
    while time.monotonic() - start < args.duration_s:
        rclpy.spin_once(node, timeout_sec=0.2)

    with open(args.out, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['topic', 'key', 't_s', 'value'])
        writer.writerows(rows)
    print(f'{"topic":26s} {"key":14s} {"n":>6s} {"p50":>8s} {"p95":>8s} {"max":>8s}')
    for (topic, key), values in samples.items():
        if not values:
            print(f'{topic:26s} {key:14s} {0:6d}      —        —        —（没收到）')
            continue
        values.sort()
        p95 = values[int(0.95 * (len(values) - 1))]
        print(f'{topic:26s} {key:14s} {len(values):6d} {statistics.median(values):8.2f} '
              f'{p95:8.2f} {values[-1]:8.2f}')
    print(f'原值 {len(rows)} 行 → {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
