# =============================================================================
#  keyboard_teleop.launch.py —— 参数装配用，**不能用来交互驾驶**
#
#  ⚠️ 想用键盘开车请用 scripts/drive.sh，不要用本文件。
#
#  原因（实测确认）：`ros2 launch` 会接管子进程的 stdio，子进程的 stdin 是
#  一根管道而不是你的终端 —— **键盘输入根本到不了节点**。节点会打一条
#  "stdin 不是终端" 的告警，然后一直发零指令，车不会动。
#
#  那本文件还留着干什么：它把 vehicle_params.yaml 里的限值装配成节点参数。
#  将来 S5 的 stack.launch.py 若要以非交互方式带起 teleop（比如接手柄、
#  或做录制回放的指令注入），include 本文件即可，不必重复一遍参数装配。
#
#  交互驾驶的正确姿势（两个终端）：
#      终端 A: ros2 launch gazebo_bridge gazebo_sim.launch.py
#      终端 B: /workspace/scripts/drive.sh
# =============================================================================

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ads_teleop")

    # 车辆限值来自唯一来源。teleop 用它算每次按键的增量和上下限 ——
    # 节点里不写任何车辆物理参数（SPEC §4.1）。
    vehicle_params = yaml.safe_load(
        (Path(share) / "config" / "vehicle_params.yaml").read_text(encoding="utf-8"))
    lim = vehicle_params["limits"]

    return LaunchDescription([
        Node(
            package="ads_teleop",
            executable="keyboard_teleop",
            name="keyboard_teleop",
            parameters=[{
                "limits.max_steer_angle_rad": lim["max_steer_angle_rad"],
                "limits.max_accel_mps2": lim["max_accel_mps2"],
                "limits.max_decel_mps2": lim["max_decel_mps2"],
                "limits.emergency_decel_mps2": lim["emergency_decel_mps2"],
                # 所有节点都必须 use_sim_time=true（SPEC §3.3）。
                # 这里影响的是消息头上的时间戳 —— 下游看门狗按仿真时间判超时，
                # 用真实时间盖戳会让两边对不上。
                "use_sim_time": True,
            }],
            # emulate_tty 让子进程拿到一个伪终端，输出不被行缓冲卡住；
            # 状态行靠 \r 原地刷新，没有这个会看不到实时数值。
            emulate_tty=True,
            output="screen",
        ),
    ])
