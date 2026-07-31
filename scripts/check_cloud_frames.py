#!/usr/bin/env python3
"""校验 /lidar/points 是否满足 SPEC §4.1 的契约：坐标系、频率、以及**真的做了变换**。

为什么这三项要放在一个 Python 脚本里，而不是用 ros2 命令行拼
------------------------------------------------------------
1. `ros2 topic hz` / `tf2_echo` 输出到管道时是全缓冲的，被 timeout 打断时
   缓冲区直接丢弃，表现为「测不到数据」，但其实数据一直在。
2. 每个 ros2 命令都要新建节点，而这套环境里一次 DDS 发现要十几秒。
   拼四五条命令就是四五次发现，又慢又容易间歇性失败。
   一个常驻节点只付一次发现代价。

为什么必须验「真的做了变换」
----------------------------
只看 frame_id 是**验不出**的。把 SDF 里的 gz_frame_id 直接写成 base_link，
frame_id 一样显示 base_link，判据一样变绿，但点的数值一个都没变 ——
每个点都带着雷达安装位置的固定偏移进入下游，且不报任何错。

真正能区分「做了变换」和「只改了标签」的是数值本身：

    雷达装在 base_link 上方 mount_z_m 处（外参来自 config/vehicle_params.yaml）。
    地面在 base_link 系里 z ≈ 0，在 lidar_link 系里 z ≈ -mount_z_m。

所以两个话题的 z 最小值应当相差约 mount_z_m。差值对不上 →
要么外参配置错了，要么变换根本没做。

用法（容器内，仿真已在跑）：
    python3 scripts/check_cloud_frames.py
"""

from __future__ import annotations

import argparse
import sys
import time
from math import isfinite
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

REPO_ROOT = Path(__file__).resolve().parent.parent
PARAMS_FILE = REPO_ROOT / "config" / "vehicle_params.yaml"

RAW_TOPIC = "/lidar/points_raw"
OUT_TOPIC = "/lidar/points"
TARGET_FRAME = "base_link"


def cloud_extent(msg: PointCloud2) -> dict | None:
    """统计一帧点云的坐标范围，跳过无效点。"""
    xs, ys, zs = [], [], []
    n_invalid = 0
    for x, y, z in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True):
        # ⚠️ skip_nans 挡不住无穷大。
        # Gazebo 的 gpu_lidar 对**没有回波的射线**（打向天空、或超出量程）
        # 返回 ±inf 而不是 NaN，所以 skip_nans=True 之后仍会混进 inf。
        # 不过滤的话 min/max 直接变成 ±inf，整个统计失去意义。
        #
        # 这不只是本脚本的问题：P5 做感知时，任何对点云求极值、求质心、
        # 建体素的代码都必须先滤掉非有限值，否则一个 inf 就能把整帧
        # 的计算结果污染成 nan，而且是静默的。
        if not (isfinite(x) and isfinite(y) and isfinite(z)):
            n_invalid += 1
            continue
        xs.append(float(x))
        ys.append(float(y))
        zs.append(float(z))
    if not zs:
        return None
    return {
        "frame_id": msg.header.frame_id,
        "n": len(zs),
        "n_invalid": n_invalid,
        "x_min": min(xs), "x_max": max(xs),
        "z_min": min(zs), "z_max": max(zs),
    }


class CloudContractChecker(Node):
    def __init__(self, want_frames: int) -> None:
        super().__init__("check_cloud_frames")
        self.want_frames = want_frames
        self.extent: dict[str, dict] = {}
        # 频率统计用两条时间轴：
        #   sim  —— 消息头里的时间戳，反映**传感器被配置成多少 Hz**
        #   wall —— 本机墙上时间，反映**实际每秒能拿到几帧**
        # RTF=1 时两者一致；RTF 掉下去时 wall 会低于 sim，
        # 分开报能一眼看出「是传感器配错了」还是「仿真跟不上」。
        #
        # 两个话题都要测，否则定位不了瓶颈：
        #   raw 就慢     → 传感器渲染跟不上，该调线数/采样数（GPU 侧）
        #   raw 快 out 慢 → 是变换节点或 QoS 在丢帧（CPU / 通信侧）
        # 只测输出话题的话，这两种情况长得一模一样。
        self.sim_stamps: dict[str, list[float]] = {RAW_TOPIC: [], OUT_TOPIC: []}
        self.wall_stamps: dict[str, list[float]] = {RAW_TOPIC: [], OUT_TOPIC: []}

        # 必须用 reliable 订阅，与发布端一致。
        # 用 best-effort 的话，本脚本自己就会因为跟不上而丢帧
        # （Python 解析 1.8 MB 的点云不快），于是「测出来的频率低」
        # 反映的是**测量工具**的速度，而不是被测链路的速度 ——
        # 这个坑在定位本项目的丢帧问题时实际踩到过。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=30)
        for topic in (RAW_TOPIC, OUT_TOPIC):
            self.create_subscription(PointCloud2, topic, self._make_cb(topic), qos)

    def _make_cb(self, topic: str):
        def cb(msg: PointCloud2) -> None:
            if topic not in self.extent:
                ext = cloud_extent(msg)
                if ext is not None:
                    self.extent[topic] = ext
            if len(self.sim_stamps[topic]) < self.want_frames:
                self.sim_stamps[topic].append(
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                self.wall_stamps[topic].append(time.monotonic())
        return cb

    def done(self) -> bool:
        return all(
            t in self.extent and len(self.sim_stamps[t]) >= self.want_frames
            for t in (RAW_TOPIC, OUT_TOPIC))


def mean_rate(stamps: list[float]) -> float | None:
    """由时间戳序列算平均频率。用首末之差而不是逐帧求平均，抗抖动。"""
    if len(stamps) < 2:
        return None
    span = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / span if span > 0 else None


def main() -> int:
    params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))
    # 安装高度从**唯一来源**读，不在这里写死。改了 YAML 本脚本自动跟着改。
    mount_z = params["sensors"]["lidar"]["mount_z_m"]

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--timeout", type=float, default=40.0,
                    help="等够帧数的最长秒数（含 DDS 发现时间）")
    ap.add_argument("--frames", type=int, default=20,
                    help="用于测频率的帧数")
    ap.add_argument("--min-rate", type=float, default=9.0,
                    help="频率验收线 Hz（标称 10 Hz，留 10%% 余量）")
    ap.add_argument("--tolerance", type=float, default=0.15,
                    help="z 偏移的允许误差 m。地面不是绝对平的，留一点余量")
    args = ap.parse_args()

    rclpy.init()
    node = CloudContractChecker(args.frames)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not node.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        extent = dict(node.extent)
        rates = {
            t: (mean_rate(node.sim_stamps[t]),
                mean_rate(node.wall_stamps[t]),
                len(node.sim_stamps[t]))
            for t in (RAW_TOPIC, OUT_TOPIC)
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()

    ok = True

    missing = [t for t in (RAW_TOPIC, OUT_TOPIC) if t not in extent]
    if missing:
        print(f"✗ 超时：{args.timeout}s 内没收到 {', '.join(missing)} 的数据",
              file=sys.stderr)
        return 1

    for topic in (RAW_TOPIC, OUT_TOPIC):
        s = extent[topic]
        print(f"  {topic}")
        print(f"    frame_id = {s['frame_id']}   有效点 = {s['n']}   "
              f"无回波(±inf) = {s['n_invalid']}")
        print(f"    z ∈ [{s['z_min']:+.3f}, {s['z_max']:+.3f}]   "
              f"x ∈ [{s['x_min']:+.3f}, {s['x_max']:+.3f}]")

    # ---- 判据 1：输出话题必须声称在 base_link 系 ----
    out = extent[OUT_TOPIC]
    if out["frame_id"] == TARGET_FRAME:
        print(f"\n✓ frame_id = {TARGET_FRAME}（符合 SPEC §4.1）")
    else:
        print(f"\n✗ {OUT_TOPIC} 的 frame_id 是 {out['frame_id']}，"
              f"应为 {TARGET_FRAME}")
        ok = False

    # ---- 判据 2：数值必须真的变了，且变化量等于安装高度 ----
    dz = out["z_min"] - extent[RAW_TOPIC]["z_min"]
    print(f"✓ z 最低点偏移 Δz = {dz:+.3f} m（期望 ≈ +{mount_z:.3f} m）"
          if abs(dz - mount_z) <= args.tolerance
          else f"✗ z 最低点偏移 Δz = {dz:+.3f} m，期望 ≈ +{mount_z:.3f} m")
    if abs(dz - mount_z) > args.tolerance:
        print(f"   偏离超过容差 {args.tolerance} m。")
        print("   Δz ≈ 0 说明只改了 frame_id 标签而没做变换 —— "
              "点云带着雷达安装偏移进了下游，下游一切几何计算都会错。")
        print("   Δz 不为 0 但对不上，则是外参配置与实际安装位置不一致。")
        ok = False
    else:
        print("  （数值确实变了，不是仅改标签）")

    # ---- 判据 3：频率 ----
    print()
    for topic in (RAW_TOPIC, OUT_TOPIC):
        sim_hz, wall_hz, n = rates[topic]
        if sim_hz is None:
            print(f"  {topic}：只收到 {n} 帧，测不出频率")
        else:
            print(f"  {topic}：仿真时间 {sim_hz:.2f} Hz / "
                  f"墙上时间 {wall_hz:.2f} Hz（{n} 帧）")

    raw_hz = rates[RAW_TOPIC][0]
    out_hz = rates[OUT_TOPIC][0]
    if out_hz is None:
        print(f"✗ /lidar/points 收帧不足，测不出频率")
        ok = False
    elif out_hz >= args.min_rate:
        print(f"✓ 点云频率 {out_hz:.2f} Hz ≥ {args.min_rate} Hz")
    else:
        print(f"✗ 点云频率 {out_hz:.2f} Hz < {args.min_rate} Hz")
        # 两个话题的频率差把瓶颈指出来，省得瞎调参数
        if raw_hz is not None and raw_hz - out_hz > 0.5:
            print(f"   瓶颈在**桥接之后**：raw {raw_hz:.2f} Hz → out {out_hz:.2f} Hz，"
                  "中途丢了帧。")
            print("   查 pointcloud_to_base_link 的耗时告警，以及 QoS 队列深度。")
        else:
            print("   瓶颈在**传感器渲染**：raw 本身就只有这么快，GPU 跟不上。")
            print("   降 horizontal_samples 或 channels（config/vehicle_params.yaml）；"
                  "水平方向是更划算的一刀，理由见该文件注释。")
        ok = False

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
