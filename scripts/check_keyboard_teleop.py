#!/usr/bin/env python3
"""校验键盘 teleop 节点（P0a 任务 4.1）：按键是否变成正确的 /vehicle_cmd。

为什么要用 pty
--------------
teleop 节点要把终端切到 raw 模式（`tcgetattr`）才能逐字符读键盘。
把管道接到它的 stdin 是不行的 —— 管道不是终端，`tcgetattr` 会失败，
节点会走"不是终端"的分支，按键永远读不到。

所以这里用 `pty.openpty()` 造一个**伪终端**：节点那头看到的是货真价实的
终端，我们这头可以往里写字符，等于程序化地"按键"。
这样 4.1 就有了可重复的自动化判据，不必每次靠人按一遍。

用法（容器内）：
    python3 scripts/check_keyboard_teleop.py
"""

from __future__ import annotations

import os
import pty
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ads_msgs.msg import VehicleCmd
from rclpy.node import Node

REPO_ROOT = Path(__file__).resolve().parent.parent
PARAMS_FILE = REPO_ROOT / "config" / "vehicle_params.yaml"


class CmdListener(Node):
    def __init__(self) -> None:
        super().__init__("check_keyboard_teleop")
        self.last: VehicleCmd | None = None
        self.count = 0
        self.create_subscription(VehicleCmd, "/vehicle_cmd", self._cb, 10)

    def _cb(self, msg: VehicleCmd) -> None:
        self.last = msg
        self.count += 1

    def settle(self, seconds: float) -> None:
        """转够时间，让按键的效果反映到最新一条消息上。"""
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def main() -> int:
    params = yaml.safe_load(PARAMS_FILE.read_text(encoding="utf-8"))
    lim = params["limits"]

    master, slave = pty.openpty()
    proc = subprocess.Popen(
        [
            "ros2", "run", "ads_teleop", "keyboard_teleop", "--ros-args",
            "-p", f"limits.max_steer_angle_rad:={lim['max_steer_angle_rad']}",
            "-p", f"limits.max_accel_mps2:={lim['max_accel_mps2']}",
            "-p", f"limits.max_decel_mps2:={lim['max_decel_mps2']}",
            "-p", f"limits.emergency_decel_mps2:={lim['emergency_decel_mps2']}",
        ],
        stdin=slave, stdout=slave, stderr=slave,
        # 自成进程组，收尾时能整组带走，不留孤儿
        start_new_session=True,
    )
    os.close(slave)

    # ⚠️ 必须持续把节点写到伪终端的输出读掉。
    #
    # 伪终端的缓冲区只有几 KB。不读的话缓冲区很快填满，节点那边的 printf
    # 就会**阻塞**，定时器回调卡死，从此不再响应按键 ——
    # 现象是"前几个按键有效，之后全部无效"，非常容易误判成节点有 bug。
    # 这个坑在本脚本第一版里真的踩到了。
    drain_stop = threading.Event()

    def drain() -> None:
        while not drain_stop.is_set():
            try:
                if not os.read(master, 4096):
                    return
            except OSError:
                return

    drain_thread = threading.Thread(target=drain, daemon=True)
    drain_thread.start()

    rclpy.init()
    node = CmdListener()
    ok = True

    def press(keys: str, settle: float = 1.0) -> None:
        for k in keys:
            os.write(master, k.encode())
            time.sleep(0.15)      # 让节点的定时器有机会读到这一下
        node.settle(settle)

    try:
        # 等节点起来并完成 DDS 发现
        node.settle(12.0)
        if node.count == 0:
            print("✗ 12 s 内没收到 /vehicle_cmd —— teleop 节点没起来或没在发布",
                  file=sys.stderr)
            return 1
        print(f"  节点已启动，正在以约 20 Hz 发布（已收到 {node.count} 条）")

        # ---- 初始状态必须是全零：启动即空挡，不能一上来就带着指令 ----
        if abs(node.last.accel_mps2) < 1e-9 and abs(node.last.steer_angle_rad) < 1e-9:
            print("  ✓ 启动初值为零（转角 0，加速度 0）")
        else:
            print(f"  ✗ 启动初值非零：转角 {node.last.steer_angle_rad}，"
                  f"加速度 {node.last.accel_mps2}")
            ok = False

        # ---- w 加速：连按 5 次应到 max_accel（每次 1/5）----
        press("wwwww")
        accel = node.last.accel_mps2
        expect = lim["max_accel_mps2"]
        print(f"  按 w×5 → 加速度 {accel:+.3f} m/s²（期望 {expect:+.3f}）")
        if abs(accel - expect) < 1e-6:
            print("  ✓ 加速键累加到上限且不越限")
        else:
            print("  ✗ 加速键行为不符")
            ok = False

        # ---- 再按 w 不应越过上限 ----
        press("ww")
        if abs(node.last.accel_mps2 - expect) < 1e-6:
            print("  ✓ 到达上限后继续按不会越限")
        else:
            print(f"  ✗ 越过了上限：{node.last.accel_mps2}")
            ok = False

        # ---- a 左转：左转为正（ROS 右手系绕 z）----
        press("aa")
        steer = node.last.steer_angle_rad
        print(f"  按 a×2 → 转角 {steer:+.3f} rad")
        if steer > 0:
            print("  ✓ 左转为正，符号约定正确")
        else:
            print("  ✗ 左转应为正 —— 符号约定反了，接上控制器后车会往反方向拐")
            ok = False

        # ---- d 右转应把转角拉回负方向 ----
        press("dddd")
        if node.last.steer_angle_rad < steer:
            print(f"  ✓ 右转使转角减小（{node.last.steer_angle_rad:+.3f} rad）")
        else:
            print("  ✗ 右转没有使转角减小")
            ok = False

        # ---- 空格归零 ----
        press(" ")
        if (abs(node.last.accel_mps2) < 1e-9
                and abs(node.last.steer_angle_rad) < 1e-9):
            print("  ✓ 空格键把转角和加速度都归零")
        else:
            print(f"  ✗ 空格没归零：转角 {node.last.steer_angle_rad}，"
                  f"加速度 {node.last.accel_mps2}")
            ok = False

        # ---- b 紧急制动：这是唯一允许突破舒适减速度的路径 ----
        press("b")
        if abs(node.last.accel_mps2 + lim["emergency_decel_mps2"]) < 1e-6:
            print(f"  ✓ b 键下发紧急制动 {node.last.accel_mps2:+.3f} m/s²")
        else:
            print(f"  ✗ b 键未下发紧急制动：{node.last.accel_mps2}")
            ok = False
    finally:
        try:
            os.write(master, b"q")
            time.sleep(1.0)
        except OSError:
            pass
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        drain_stop.set()
        os.close(master)
        node.destroy_node()
        rclpy.shutdown()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
