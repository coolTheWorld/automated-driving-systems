"""S06 判据器：锚定虚拟灯相位、发 goal、判红灯停止与绿灯恢复."""
import math
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

rclpy.init()
n = Node("s06_judge")
n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
state = {"x": None, "v": 0.0}


def on_gt(m):
    state["x"] = m.pose.pose.position.x
    tw = m.twist.twist.linear
    state["v"] = math.hypot(tw.x, tw.y)


n.create_subscription(Odometry, "/ego_pose_gt", on_gt, 10)
from ads_msgs.msg import TrafficLight  # noqa: E402  运行环境已 source install
from ads_msgs.msg import Trajectory  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
light = {"s": "?"}
n.create_subscription(
    TrafficLight, "/traffic_light/state",
    lambda m: light.update(s=("RED" if m.state == TrafficLight.STATE_RED else "GRN")), 10)
diag = {"msg": "?", "stop_at": "?"}
traj = {"n": -1}


def on_diag(m):
    if not m.status:
        return
    diag["msg"] = m.status[0].message[:30]
    diag["stop_at"] = "?"
    for kv in m.status[0].values:
        if kv.key == "behavior_stop_at_s_m":
            diag["stop_at"] = kv.value[:8]


n.create_subscription(DiagnosticArray, "/planning/diagnostics", on_diag, 10)
n.create_subscription(Trajectory, "/planning/trajectory",
                      lambda m: traj.update(n=len(m.points)), 10)
timeline = open("/workspace/.l3c_s06.timeline", "w")
while n.get_clock().now().nanoseconds == 0:
    rclpy.spin_once(n, timeout_sec=0.2)
RED_S, GREEN_S, STOP_S = 45.0, 40.0, 30.0
STOP_X = 60.0  # route s=30 → x=60（南直道 s = x − 30）
FRONT = 3.55
pub = n.create_publisher(
    PoseStamped, "/goal_pose",
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL))
# 等 map_node 的订阅真正 match 再发（CARLA bringup 期 DDS 发现慢达十几秒，
# 盲发 5 次全部落空 = 上一轮车没动的根因）。照抄 record_control_run 的做法。
waited = 0.0
while pub.get_subscription_count() == 0 and waited < 120.0:
    rclpy.spin_once(n, timeout_sec=0.5)
    waited += 0.5
print("goal 订阅者 %d 个（等了 %.0f s）" % (pub.get_subscription_count(), waited), flush=True)
anchor = n.get_clock().now().nanoseconds * 1e-9 + 1.0
subprocess.Popen([
    "ros2", "run", "carla_bridge", "traffic_light_node", "--ros-args",
    "-p", "use_sim_time:=true", "-p", "red_s:=%f" % RED_S,
    "-p", "green_s:=%f" % GREEN_S, "-p", "stop_line_s_m:=%f" % STOP_S,
    "-p", "phase_anchor_s:=%f" % anchor])
g = PoseStamped()
g.header.frame_id = "map"
g.pose.position.x = 91.75
g.pose.position.y = 20.0
g.pose.orientation.w = 1.0
for _ in range(5):
    pub.publish(g)
    rclpy.spin_once(n, timeout_sec=0.4)
print("锚 %.1f，红窗 [%.0f, %.0f]，停止线 x=%.1f" % (
    anchor, anchor, anchor + RED_S, STOP_X), flush=True)

red_tail = []
crossed_at = None
last_line_t = 0.0
while True:
    rclpy.spin_once(n, timeout_sec=0.2)
    t = n.get_clock().now().nanoseconds * 1e-9
    if state["x"] is None:
        continue
    front_x = state["x"] + FRONT
    if t - last_line_t >= 0.5:
        last_line_t = t
        line = "%.1f  front_x=%.2f  v=%.3f  灯=%s  traj_n=%d  stop_at=%s  diag=%s" % (
            t, front_x, state["v"], light["s"], traj["n"], diag["stop_at"], diag["msg"])
        timeline.write(line + chr(10))
        timeline.flush()
    if anchor + RED_S - 8.0 <= t < anchor + RED_S:
        red_tail.append((front_x, state["v"]))
    if t >= anchor + RED_S and front_x > STOP_X + 1.0 and crossed_at is None:
        crossed_at = t - (anchor + RED_S)
    if t > anchor + RED_S + 60.0:
        break

if red_tail:
    gaps = [STOP_X - fx for fx, _ in red_tail]
    vmax = max(v for _, v in red_tail)
    gmin, gmax = min(gaps), max(gaps)
    ok1 = (0.0 <= gmin <= 2.0) and vmax < 0.1
    print("红窗末 8 s：车头-停止线间距 [%.2f, %.2f] m，vmax=%.3f" % (gmin, gmax, vmax))
    print("判据①（红灯停在停止线前 0-2 m 且停稳）: " + ("PASS" if ok1 else "FAIL"))
else:
    print("判据①: FAIL（红窗末无样本）")
if crossed_at is not None:
    print("判据②（绿灯后 60 s 内驶过停止线）: PASS（%.1f s）" % crossed_at)
else:
    print("判据②（绿灯后 60 s 内驶过停止线）: FAIL")
