#!/bin/bash
# CP-P7-B ⑩ 回归：CP-P2-B、CP-P3-B avoid/block、CP-P6-B 抽一轮（真值层）
set -u
LOGDIR=/workspace/.p7s4
cleanup() {
  [ -n "${PGID:-}" ] && kill -INT -- -"$PGID" 2>/dev/null
  [ -n "${GOAL_PID:-}" ] && kill "$GOAL_PID" 2>/dev/null
  sleep 3
  [ -n "${PGID:-}" ] && kill -KILL -- -"$PGID" 2>/dev/null
  rm -f /workspace/core.*
}
trap cleanup EXIT
cd /workspace
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; set -u

launch_stack() {  # launch_stack <额外参数...>
  n=$(ps -eo comm,stat | awk '$2 !~ /^Z/' | grep -cx -E 'ruby|control_node|planning_node|perception_node|prediction_node' || true)
  [ "${n:-0}" -gt 0 ] && { echo "❌ 残留 $n"; exit 1; }
  setsid ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false "$@" \
    > "$LOGDIR/regress_last.launch.log" 2>&1 &
  LPID=$!
  sleep 2
  PGID=$(ps -o pgid= -p "$LPID" | tr -d ' ')
  [ -z "$PGID" ] && { echo "❌ launch 没起来"; exit 1; }
  timeout 150 python3 -c "
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node('wc'); done=[False]
n.create_subscription(Clock, '/clock', lambda m: done.__setitem__(0, m.clock.sec>=8), 10)
while not done[0]: rclpy.spin_once(n, timeout_sec=0.2)
" || { echo '❌ bringup 失败'; tail -20 "$LOGDIR/regress_last.launch.log"; exit 1; }
}

teardown() {
  kill -INT -- -"$PGID" 2>/dev/null
  sleep 4
  kill -KILL -- -"$PGID" 2>/dev/null
  PGID=""
  sleep 2
  rm -f /workspace/core.*
}

echo "===== ⑩a CP-P2-B 跟踪回归 ====="
launch_stack
python3 scripts/record_control_run.py --goal 91.75 20.0 --out "$LOGDIR/regress_p2b.csv"
echo "p2b_rc=$?"
teardown

echo "===== ⑩b CP-P3-B avoid ====="
launch_stack obstacles:=avoid
python3 scripts/record_obstacle_run.py --scenario avoid --out "$LOGDIR/regress_avoid.csv"
echo "avoid_rc=$?"
teardown

echo "===== ⑩c CP-P3-B block ====="
launch_stack obstacles:=block
python3 scripts/record_obstacle_run.py --scenario block --out "$LOGDIR/regress_block.csv"
echo "block_rc=$?"
teardown

echo "===== ⑩d CP-P6-B 抽一轮（真值层 curve）====="
launch_stack prediction:=true dynamic:=curve
python3 - <<'PYEOF' > "$LOGDIR/regress_goal.log" 2>&1 &
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node('goal_warm')
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
        g.pose.position.x = 91.75; g.pose.position.y = 20.0; g.pose.orientation.w = 1.0
        pub.publish(g); state['sent'] += 1; state['last'] = state['t']
print(f"sent {state['sent']}", flush=True)
PYEOF
GOAL_PID=$!
python3 scripts/record_prediction_run.py --duration-s 72 --layer truth --out "$LOGDIR/regress_p6b.csv"
echo "p6b_rc=$?"
kill "$GOAL_PID" 2>/dev/null; GOAL_PID=""
teardown
echo "===== 回归全部结束 ====="
