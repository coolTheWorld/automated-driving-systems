#!/bin/bash
# L3-C 单场景一体化轮次（云端自足）：起栈 → goal → 记录 → 判定。
# 铁律：所有等待带超时；服务器死亡即中止；trap 清理。
# 用法：l3c_round.sh <标签> <gx> <gy> [额外 launch 参数...]
set -u
TAG="$1"; GX="$2"; GY="$3"; shift 3
EXTRA="$*"
LOG=/workspace/.l3c_${TAG}
# ⚠️ 陷阱表第一条：set -u 下 source ROS 会炸 AMENT_TRACE_SETUP_FILES ——
#    source 期间临时 set +u（本脚本首版就栽在这，死得连 VERDICT 都没印）。
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
set -u

cleanup() {
  pkill -f "install/carla_bridge/li[b]" 2>/dev/null
  pkill -f "gazebo_bridge/lidar_preproces[s]or" 2>/dev/null
  pkill -f "robot_state_publishe[r] " 2>/dev/null
  pkill -f "ros2 launc[h]" 2>/dev/null
  pkill -f "map_nod[e]" 2>/dev/null; pkill -f "planning_nod[e]" 2>/dev/null
  pkill -f "control_nod[e]" 2>/dev/null
  sleep 2
}
trap cleanup EXIT
cleanup

server_ok() { timeout 5 bash -c "echo > /dev/tcp/127.0.0.1/2000" 2>/dev/null; }
server_ok || { echo "VERDICT=SERVER_DEAD（起跑前）"; exit 3; }

ros2 launch ads_bringup stack.launch.py sim:=carla gui:=false rviz:=false $EXTRA \
  > ${LOG}.launch.log 2>&1 &
for i in $(seq 1 40); do
  grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log 2>/dev/null && break
  server_ok || { echo "VERDICT=SERVER_DEAD（bringup 中）"; exit 3; }
  sleep 5
done
grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log || { echo "VERDICT=BRINGUP_TIMEOUT"; exit 2; }
sleep 12

# 位置遥测：崩溃时要知道车死在哪（S6 第 3 次 Segfault 后加）
python3 - > ${LOG}.pos 2>&1 <<'TEOF' &
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ads_msgs.msg import VehicleCmd
rclpy.init()
n = Node('pos_tap')
state = {'steer': float('nan'), 'accel': float('nan')}
def on_cmd(m):
    state['steer'] = m.steer_angle_rad
    state['accel'] = m.accel_mps2
n.create_subscription(VehicleCmd, '/vehicle_cmd', on_cmd, 10)
def on_gt(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    tw = m.twist.twist.linear
    print(f"{p.x:.1f},{p.y:.1f},{math.degrees(yaw):.0f},{math.hypot(tw.x,tw.y):.2f},"
          f"{state['steer']:.3f},{state['accel']:.2f}", flush=True)
n.create_subscription(Odometry, '/ego_pose_gt', on_gt, 10)
rclpy.spin(n)
TEOF
TAP=$!

timeout 400 python3 /workspace/scripts/record_control_run.py \
  --goal "$GX" "$GY" --out ${LOG}.csv > ${LOG}.txt 2>&1 &
REC=$!
while kill -0 $REC 2>/dev/null; do
  server_ok || { echo "VERDICT=SERVER_CRASHED（记录中）"; echo "末段遥测 x,y,yaw°,v,steer,accel："; grep -v "^0.0,-0.0" ${LOG}.pos | tail -12; kill $TAP 2>/dev/null; exit 3; }
  sleep 10
done
kill $TAP 2>/dev/null
if grep -q 全部通过 ${LOG}.txt; then echo "VERDICT=PASS"
elif grep -q 有判据未通过 ${LOG}.txt; then echo "VERDICT=FAIL"
else echo "VERDICT=INCOMPLETE（记录器没跑完 —— 超时或异常）"; fi
grep -E "PASS|FAIL|m/s|误差" ${LOG}.txt | head -12
