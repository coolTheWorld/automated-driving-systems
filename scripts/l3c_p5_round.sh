#!/bin/bash
# L3-C P5 感知复测轮（云端自足）：CP-P5-B 协议 —— 记录器早起 + goal_warm 37。
# 用法：l3c_p5_round.sh [场景，默认 both]
#   P9_INSTRUMENTS=1  随轮起两件 P9 仪器（栈就绪后起、记录器结束时收）：
#     ${LOG}.actors.log     p9_actor_watch —— 独立客户端 actor 普查（消失时刻 + z）
#     ${LOG}.timeline.csv   p9_capture_frame --timeline —— 检测 vs 确认逐帧时间线
#     ${LOG}.cap.txt        同上的采样报告（挂高/剃刀门/邻域点数）
set -u
SCEN="${1:-both}"
LOG=/workspace/.l3c_p5_${SCEN}
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
[ "${LEFT}" != "0" ] && { echo "VERDICT=RESIDUE(${LEFT})"; exit 4; }
server_ok() { timeout 5 bash -c "echo > /dev/tcp/127.0.0.1/2000" 2>/dev/null; }
server_ok || { echo "VERDICT=SERVER_DEAD"; exit 3; }

ros2 launch ads_bringup stack.launch.py sim:=carla gui:=false rviz:=false \
  perception:=true dynamic:=${SCEN} > ${LOG}.launch.log 2>&1 &
for i in $(seq 1 40); do
  grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log 2>/dev/null && break
  server_ok || { echo "VERDICT=SERVER_DEAD"; exit 3; }
  sleep 5
done
grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log || { echo "VERDICT=BRINGUP_TIMEOUT"; exit 2; }

# 记录器**立即早起**（CP-P5-B 协议：域内遮挡在 goal 前发生，晚起整个错过）
timeout 500 python3 /workspace/scripts/record_perception_run.py \
  --duration-s 72 --out ${LOG}.csv > ${LOG}.txt 2>&1 &
REC=$!

# P9 仪器（可选）：必须在 sidecar 载完世界之后起 —— 普查客户端连的是当前
# episode，早起会连到默认 Town、随后每次查询都抛 expired episode。
INSTR_PIDS=()
if [ "${P9_INSTRUMENTS:-0}" = "1" ]; then
  : > ${LOG}.actors.log
  python3 /workspace/scripts/p9_actor_watch.py --period-s 5 --duration-s 600 \
    --out ${LOG}.actors.log > /dev/null 2>&1 &
  INSTR_PIDS+=($!)
  python3 /workspace/scripts/p9_capture_frame.py --frames 20 --out-prefix ${LOG}.cap \
    --timeline ${LOG}.timeline.csv --timeout-s 600 > ${LOG}.cap.txt 2>&1 &
  INSTR_PIDS+=($!)
fi

python3 - <<'PYEOF' > ${LOG}.goal.log 2>&1 &
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node('goal_warm')
pub = n.create_publisher(PoseStamped, '/goal_pose',
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL))
state = {'t': 0.0, 'last': 0.0}
n.create_subscription(Clock, '/clock',
    lambda m: state.__setitem__('t', m.clock.sec + m.clock.nanosec*1e-9), 10)
w0 = time.monotonic()
while state['t'] < 47.0 and time.monotonic() - w0 < 400:
    rclpy.spin_once(n, timeout_sec=0.1)
    if state['t'] >= 37.0 and state['t'] - state['last'] >= 1.0:
        g = PoseStamped(); g.header.frame_id = 'map'
        g.pose.position.x = 91.75; g.pose.position.y = 20.0; g.pose.orientation.w = 1.0
        pub.publish(g); state['last'] = state['t']
PYEOF
GOAL_PID=$!
while kill -0 $REC 2>/dev/null; do
  server_ok || { echo "VERDICT=SERVER_CRASHED"; kill $GOAL_PID 2>/dev/null; exit 3; }
  sleep 10
done
kill $GOAL_PID 2>/dev/null
# 仪器收尾：TERM 让 p9_capture_frame 出报告再退（它装了信号钩子），等它写完
for PID in "${INSTR_PIDS[@]:-}"; do [ -n "$PID" ] && kill -TERM $PID 2>/dev/null; done
for PID in "${INSTR_PIDS[@]:-}"; do [ -n "$PID" ] && wait $PID 2>/dev/null; done
if grep -q "❌\|FAIL" ${LOG}.txt; then echo "VERDICT=FAIL"
elif grep -qE "✅|PASS" ${LOG}.txt; then echo "VERDICT=PASS"
else echo "VERDICT=INCOMPLETE"
fi
grep -vE "^\[INFO\]|^$" ${LOG}.txt | tail -30
