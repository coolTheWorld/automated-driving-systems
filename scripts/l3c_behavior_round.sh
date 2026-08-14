#!/bin/bash
# L3-C 行为场景单轮（云端自足）：起栈(sim:=carla) → goal_warm(37.0) → 行为记录器 → 判定。
# 用法：behavior_round.sh <场景> <层> <时长> <gx> <gy>
set -u
SCEN="$1"; LAYER="$2"; DUR="$3"; GX="$4"; GY="$5"
TAG="${SCEN}_${LAYER}"
LOG=/workspace/.l3c_${TAG}
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
set -u

cleanup() {
  PATTERNS=(
    "install/carla_bridge/li[b]" "install/gazebo_bridge/li[b]"
    "install/ads_[a-z]*/li[b]" "robot_state_publishe[r] "
    "ros2 launc[h]" "traffic_light_nod[e]")
  for PAT in "${PATTERNS[@]}"; do pkill -f "$PAT" 2>/dev/null; done
  for i in 1 2 3 4 5; do
    LIVE=0
    for PAT in "${PATTERNS[@]}"; do pgrep -f "$PAT" >/dev/null 2>&1 && LIVE=1; done
    [ "$LIVE" = "0" ] && break
    sleep 2
  done
  if [ "${LIVE:-0}" = "1" ]; then
    for PAT in "${PATTERNS[@]}"; do pkill -9 -f "$PAT" 2>/dev/null; done
    sleep 2
  fi
}
trap cleanup EXIT
cleanup
LEFT=$(ps -eo stat=,args= | awk '$1 !~ /^Z/' | grep -cE "install/(ads_|gazebo_bridge|carla_bridge)" || true)
if [ "${LEFT}" != "0" ]; then
  echo "VERDICT=RESIDUE(${LEFT} 个残留进程，拒绝起跑)"
  ps -eo stat=,args= | awk '$1 !~ /^Z/' | grep -E "install/(ads_|gazebo_bridge|carla_bridge)"
  exit 4
fi
server_ok() { timeout 5 bash -c "echo > /dev/tcp/127.0.0.1/2000" 2>/dev/null; }
server_ok || { echo "VERDICT=SERVER_DEAD"; exit 3; }

if [ "$LAYER" = "truth" ]; then PFLAG=false; else PFLAG=true; fi
ros2 launch ads_bringup stack.launch.py sim:=carla gui:=false rviz:=false \
  perception:=${PFLAG} prediction:=true dynamic:=${SCEN} \
  > ${LOG}.launch.log 2>&1 &
for i in $(seq 1 40); do
  grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log 2>/dev/null && break
  server_ok || { echo "VERDICT=SERVER_DEAD"; exit 3; }
  sleep 5
done
grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log || { echo "VERDICT=BRINGUP_TIMEOUT"; exit 2; }
sleep 8

# goal_warm：绝对仿真钟 37.0 起每秒重发（扩窗协议，照抄 run_all_scenarios.sh）
python3 - "$GX" "$GY" <<'PYEOF' > ${LOG}.goal.log 2>&1 &
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
gx, gy = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); n = Node('goal_warm')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
pub = n.create_publisher(PoseStamped, '/goal_pose',
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL))
state = {'t': 0.0, 'sent': 0, 'last': 0.0}
n.create_subscription(Clock, '/clock',
    lambda m: state.__setitem__('t', m.clock.sec + m.clock.nanosec*1e-9), 10)
w0 = time.monotonic()
while state['t'] < 47.0 and time.monotonic() - w0 < 400:
    rclpy.spin_once(n, timeout_sec=0.1)
    if state['t'] >= 37.0 and state['t'] - state['last'] >= 1.0:
        g = PoseStamped(); g.header.frame_id = 'map'
        g.pose.position.x = gx; g.pose.position.y = gy; g.pose.orientation.w = 1.0
        pub.publish(g); state['sent'] += 1; state['last'] = state['t']
print("sent %d" % state['sent'], flush=True)
PYEOF
GOAL_PID=$!

timeout 500 python3 /workspace/scripts/record_behavior_run.py \
  --scenario "$SCEN" --layer "$LAYER" --duration-s "$DUR" \
  --out ${LOG}.csv > ${LOG}.txt 2>&1 &
REC=$!
while kill -0 $REC 2>/dev/null; do
  server_ok || { echo "VERDICT=SERVER_CRASHED（记录中）"; kill $GOAL_PID 2>/dev/null; exit 3; }
  sleep 10
done
kill $GOAL_PID 2>/dev/null

# 行为记录器不打「全部通过」总结行，按判据行的 ✅/❌ 计（实测措辞对齐）
if grep -q "❌" ${LOG}.txt; then echo "VERDICT=FAIL"
elif grep -q "✅" ${LOG}.txt; then echo "VERDICT=PASS"
else echo "VERDICT=INCOMPLETE（记录器没跑完 —— 超时或异常）"
fi
grep -vE "^\[INFO\]|^$" ${LOG}.txt | tail -30
