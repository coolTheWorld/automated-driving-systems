#!/usr/bin/env python3
"""校验 TF 树 map → odom → base_link → lidar_link 是否连通（P0a 任务 3.7）。

为什么不用 `ros2 run tf2_ros tf2_echo`
--------------------------------------
它输出到管道时是全缓冲的，被 timeout 打断时缓冲区连同结果一起丢掉，
表现为「查不到变换」，但其实变换一直都在 —— 这个假阴性排查起来很费时间。
（加 `stdbuf -oL` 能绕过，但每查一段就要新建一个节点、重走一遍 DDS 发现，
四段就是四次，在这套环境里要一分多钟。）

一个常驻节点把四段一次查完，只付一次发现代价，结果也确定。

为什么要逐段查而不是只查 map → lidar_link
------------------------------------------
端到端能查通只说明「有一条路径」，断在哪一段看不出来。而这三段的
来源完全不同，坏掉的原因也完全不同：

    map → odom        launch 里的 static_transform_publisher（P4 前是单位变换）
    odom → base_link  Gazebo 的 AckermannSteering 插件，经桥接进 /tf
    base_link → *     robot_state_publisher 读 URDF 得到

逐段报出来，一眼就知道该去查哪个组件。

用法（容器内，仿真已在跑）：
    python3 scripts/check_tf_tree.py
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# 期望连通的父子对。顺序即 TF 树自上而下的顺序。
CHAIN = [
    ("map", "odom"),
    ("odom", "base_link"),
    ("base_link", "lidar_link"),
    ("base_link", "gnss_link"),
    # 端到端：上面每段都通，这段自然也通；单独列出来是为了确认
    # tf2 真的能把它们串成一条链（父子顺序写反时逐段能过、端到端过不了）。
    ("map", "lidar_link"),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="等 TF 树建立起来的最长秒数（含 DDS 发现时间）")
    args = ap.parse_args()

    rclpy.init()
    node = Node("check_tf_tree")
    buffer = Buffer()
    TransformListener(buffer, node)

    # 用零时刻查询 = 「最新可用的那一帧」。
    # 这样就不依赖本节点的时钟与仿真时钟是否对齐 —— 本脚本没设
    # use_sim_time，若按当前时刻去查，会因为系统时间（2026 年）远晚于
    # 仿真时间（几百秒）而永远报 extrapolation。
    zero = rclpy.time.Time()

    results: dict[tuple[str, str], str] = {}
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        for pair in CHAIN:
            if pair in results:
                continue
            parent, child = pair
            try:
                t = buffer.lookup_transform(parent, child, zero).transform.translation
                results[pair] = f"[{t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f}]"
            except Exception:      # noqa: BLE001 —— tf2 的异常类型有好几种，这里只关心成没成
                pass
        if len(results) == len(CHAIN):
            break

    node.destroy_node()
    rclpy.shutdown()

    ok = True
    for parent, child in CHAIN:
        got = results.get((parent, child))
        if got:
            print(f"✓ {parent} → {child}   平移 {got}")
        else:
            print(f"✗ {parent} → {child}   查不到 —— TF 树在这里断了")
            ok = False

    if not ok:
        print("\n  排查方向：")
        print("    map → odom 断      → static_transform_publisher 没起来")
        print("    odom → base_link 断 → 桥接表里 /model/*/tf 那条没生效")
        print("    base_link → * 断    → robot_state_publisher 没收到 URDF，"
              "或 URDF 里没有该 link")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
