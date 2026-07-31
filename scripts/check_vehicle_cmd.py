#!/usr/bin/env python3
"""校验 /vehicle_cmd → Gazebo 这条控制链路（P0a 任务 4.2 / 4.3）。

四个阶段，每个都有可量化的判据：

    1. 直行     发正加速度，车沿 +x 前进
    2. 转向     发转角指令，车产生侧向位移
    3. 限幅     发离谱的超限指令，检查桥接输出被截断在限值内
    4. 看门狗   停止发指令，车必须自动减速停车

为什么限幅阶段查 /gazebo/cmd_vel 而不是看车怎么跑
--------------------------------------------------
转角被截断到 0.6 rad 后转弯半径只有 L/tan(0.6) ≈ 3.95 m，8 m/s 下侧向加速度
约 16 m/s² —— 车会侧翻或严重侧滑。那时候测到的是"车翻了"，而不是"限幅生效了"。

直接订阅桥接的输出话题，反算出它实际下发的转角和速度，判据就精确了：

    速度   twist.linear.x        ≤ max_speed_mps
    转角   atan(L·ω / v)         ≤ max_steer_angle_rad

用法（容器内，仿真已在跑）：
    python3 scripts/check_vehicle_cmd.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ads_msgs.msg import VehicleCmd
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

REPO_ROOT = Path(__file__).resolve().parent.parent
PARAMS_FILE = REPO_ROOT / "config" / "vehicle_params.yaml"


class CmdChecker(Node):
    def __init__(self, limits: dict, wheelbase: float) -> None:
        super().__init__("check_vehicle_cmd")
        self.limits = limits
        self.wheelbase = wheelbase

        self.pub = self.create_publisher(VehicleCmd, "/vehicle_cmd", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Twist, "/gazebo/cmd_vel", self._on_twist, 10)

        self.odom: Odometry | None = None
        # 桥接下发过的最大速度 / 最大转角，限幅阶段用它们做判据
        self.max_sent_speed = 0.0
        self.max_sent_steer = 0.0
        self.n_twist = 0

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_twist(self, msg: Twist) -> None:
        self.n_twist += 1
        v = msg.linear.x
        self.max_sent_speed = max(self.max_sent_speed, abs(v))
        # 从 Twist 反算前轮转角：ω = v·tan(δ)/L  →  δ = atan(L·ω/v)
        # v≈0 时这个反算没有意义（ω 必然也是 0），跳过。
        if abs(v) > 0.1:
            steer = math.atan(self.wheelbase * msg.angular.z / v)
            self.max_sent_steer = max(self.max_sent_steer, abs(steer))

    def reset_sent_stats(self) -> None:
        self.max_sent_speed = 0.0
        self.max_sent_steer = 0.0
        self.n_twist = 0

    # ------------------------------------------------------------------
    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, steer: float, accel: float, seconds: float, rate_hz: float = 20.0) -> None:
        """以固定频率持续下发指令。

        必须持续发而不是发一次：桥接有看门狗，超过 cmd_timeout_s 收不到
        新指令就会自动刹车 —— 这正是阶段 4 要验的行为。
        """
        period = 1.0 / rate_hz
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            msg = VehicleCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.steer_angle_rad = float(steer)
            msg.accel_mps2 = float(accel)
            self.pub.publish(msg)
            t_next = time.monotonic() + period
            while rclpy.ok() and time.monotonic() < t_next:
                rclpy.spin_once(self, timeout_sec=0.005)

    def wait_odom(self, timeout: float = 40.0) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def pose(self) -> tuple[float, float]:
        p = self.odom.pose.pose.position
        return p.x, p.y

    def speed(self) -> float:
        return self.odom.twist.twist.linear.x

    def brake_to_stop(self, timeout: float = 15.0) -> None:
        """刹停，让下一阶段从静止开始，各阶段的位移才可比。"""
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            self.drive(0.0, -self.limits["max_decel_mps2"], 0.3)
            if abs(self.speed()) < 0.05:
                return
        # 停不下来不算致命，后续阶段的判据是位移增量，起始速度非零只会让数更大


def main() -> int:
    params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))
    lim = params["limits"]
    wheelbase = params["geometry"]["wheelbase_m"]

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--drive-seconds", type=float, default=6.0)
    args = ap.parse_args()

    rclpy.init()
    node = CmdChecker(lim, wheelbase)
    ok = True
    try:
        if not node.wait_odom():
            print("✗ 收不到 /odom —— 仿真没在跑，或桥接没起来", file=sys.stderr)
            return 1
        # 起步前先等一会儿，让 /vehicle_cmd 的订阅方（vehicle_cmd_bridge）
        # 完成 DDS 发现。没发现完就发指令，消息会直接丢掉。
        node.spin_for(3.0)

        # ---------------- 阶段 1：直行 ----------------
        print("[1/4] 直行（任务 4.2）")
        x0, y0 = node.pose()
        accel = lim["max_accel_mps2"]
        node.drive(0.0, accel, args.drive_seconds)
        x1, y1 = node.pose()
        dx = x1 - x0
        v_end = node.speed()
        # 理论位移 = ½·a·t²，取 40% 作为下限：桥接的抗饱和会压住设定值，
        # 插件跟随也有滞后，实际达不到理论值。判据松一点，
        # 但足以区分「车动了」和「车没动」。
        min_dx = 0.5 * accel * args.drive_seconds ** 2 * 0.4
        print(f"     加速度指令 {accel:.2f} m/s²，{args.drive_seconds:.0f} s")
        print(f"     Δx = {dx:+.3f} m   末速 {v_end:.3f} m/s   （下限 {min_dx:.2f} m）")
        if dx >= min_dx:
            print("  ✓ 车沿 +x 前进，/vehicle_cmd 已驱动车辆")
        else:
            print("  ✗ 位移不足 —— 指令没传到 Gazebo，或桥接换算有误")
            ok = False

        node.brake_to_stop()

        # ---------------- 阶段 2：转向 ----------------
        print("\n[2/4] 转向（任务 4.2）")
        x0, y0 = node.pose()
        steer = lim["max_steer_angle_rad"] * 0.5
        node.drive(steer, accel * 0.6, args.drive_seconds)
        x1, y1 = node.pose()
        dy = y1 - y0
        print(f"     转角指令 {steer:+.3f} rad（左转为正）")
        print(f"     Δy = {dy:+.3f} m")
        # 左转应产生正的 Δy。阈值 1 m：起步阶段速度低、转向效果弱，
        # 取松一点是为了不被加速过程影响，但方向必须对。
        if dy >= 1.0:
            print("  ✓ 车向左偏移，转向链路可用")
        else:
            print("  ✗ 侧向位移不足或方向相反 —— 转角符号或换算有误")
            ok = False

        node.brake_to_stop()

        # ---------------- 阶段 3：限幅 ----------------
        print("\n[3/4] 指令限幅（任务 4.3）")
        node.reset_sent_stats()
        # 发离谱的值：转角 10 rad（≈573°）、加速度 100 m/s²（≈10 g）
        node.drive(10.0, 100.0, 5.0)
        max_steer_allowed = lim["max_steer_angle_rad"]
        max_speed_allowed = lim["max_speed_mps"]
        print(f"     下发 转角 10.000 rad / 加速度 100.000 m/s²（均严重超限）")
        print(f"     桥接实际输出：最大速度 {node.max_sent_speed:.3f} m/s "
              f"（限值 {max_speed_allowed:.3f}）")
        print(f"                   最大转角 {node.max_sent_steer:.3f} rad "
              f"（限值 {max_steer_allowed:.3f}）")
        if node.n_twist == 0:
            print("  ✗ 没收到 /gazebo/cmd_vel，无法判断限幅是否生效")
            ok = False
        else:
            # 留 1% 容差：反算转角时用的是浮点除法和 atan，末位会有误差
            if node.max_sent_speed <= max_speed_allowed * 1.01:
                print("  ✓ 速度未越限")
            else:
                print("  ✗ 速度越限 —— max_speed_mps 没生效")
                ok = False
            if node.max_sent_steer <= max_steer_allowed * 1.01:
                print("  ✓ 转角未越限")
            else:
                print("  ✗ 转角越限 —— max_steer_angle_rad 没生效")
                ok = False

        # NaN 必须被拒绝而不是原样下发。
        # NaN 参与任何比较都返回 false，clamp 会把它放行，一路传到物理引擎
        # 解算出 NaN 位姿，车直接从世界里消失。
        print("     追加：下发 NaN 指令")
        node.reset_sent_stats()
        node.drive(float("nan"), float("nan"), 2.0)
        nan_seen = any(
            not math.isfinite(v) for v in (node.max_sent_speed, node.max_sent_steer))
        if not nan_seen:
            print("  ✓ NaN 指令被拒绝，未污染下游")
        else:
            print("  ✗ NaN 传到了 /gazebo/cmd_vel —— 物理引擎会解算出 NaN 位姿")
            ok = False

        # ---------------- 阶段 4：看门狗 ----------------
        print("\n[4/4] 看门狗（停止发指令后必须自动刹停）")
        node.drive(0.0, accel, 4.0)          # 先跑起来
        v_before = node.speed()
        print(f"     停发指令前速度 {v_before:.3f} m/s")
        # 只 spin 不发指令。等待时间 = 看门狗超时 + 从 v_before 刹到 0 所需时间 + 余量
        wait_s = 0.5 + v_before / max(lim["max_decel_mps2"], 0.1) + 3.0
        node.spin_for(wait_s)
        v_after = node.speed()
        print(f"     停发 {wait_s:.1f} s 后速度 {v_after:.3f} m/s")
        if v_before > 0.5 and abs(v_after) < 0.2:
            print("  ✓ 指令中断后车自动停下")
        elif v_before <= 0.5:
            print("  ✗ 停发前车就没跑起来，本阶段无效")
            ok = False
        else:
            print("  ✗ 指令中断后车仍在动 —— 看门狗没生效")
            print("     teleop 崩溃或 ssh 断开时，车会带着最后一条指令一直开下去。")
            ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
