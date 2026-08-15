#!/bin/bash
# =============================================================================
#  l3g_p5_round.sh —— CP-P5-B 感知验收的 Gazebo 一轮（本机，零租金）
#
#      ./scripts/l3g_p5_round.sh            # 一轮 both 场景，产物在 .scenario_runs/p5_<时间戳>/
#      ./scripts/l3g_p5_round.sh --rounds 3 # 连跑三轮（各自独立起收栈）
#
#  与 l3c_p5_round.sh（CARLA 云端）同一协议：记录器**早起**（仿真钟 ≥ 8 就起，
#  域内遮挡在 goal 前发生）、goal 由预热发布器在绝对仿真钟 37.0 起每秒重发、
#  时长 72 s。判据全在 record_perception_run.py 里（CP-P5-B 原表），本脚本只做编排。
#
#  ## 为什么单独一个脚本而不并进 run_all_scenarios.sh
#  run_all 是 SPEC §8 场景表（S01–S07）的一键；CP-P5-B 是**模块**验收，
#  P8-S2 的注册表当年就没把它算进去，它的记录器也不写 metrics/history.csv。
#  P9-S3 要在改共享感知/传感器几何时反复拿它当基线（2026-08-15 之前只能手敲
#  一串命令），值得一条可复现的命令。起收栈的机制照抄 run_all（setsid + PGID
#  收整组 + 残留守卫 + wait_clock），改一处必改另一处。
# =============================================================================
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUNDS=1
for arg in "$@"; do
  case "$arg" in
    --rounds) ;;
    [0-9]*) ROUNDS="$arg" ;;
    *) echo "用法：l3g_p5_round.sh [--rounds N]"; exit 2 ;;
  esac
done

set +u; source /opt/ros/jazzy/setup.bash; source "${REPO}/install/setup.bash"; set -u
cd "$REPO"

PGID=""; GOAL_PID=""
cleanup() {
  [ -n "${GOAL_PID:-}" ] && kill "$GOAL_PID" 2>/dev/null
  [ -n "${PGID:-}" ] && kill -INT -- -"$PGID" 2>/dev/null
  sleep 3
  [ -n "${PGID:-}" ] && kill -KILL -- -"$PGID" 2>/dev/null
  rm -f "${REPO}"/core.*
}
trap cleanup EXIT

guard_residual() {
  local n
  n=$(ps -eo comm,stat | awk '$2 !~ /^Z/' \
      | grep -cx -E 'ruby|control_node|planning_node|perception_node|prediction_node|localization_node' || true)
  if [ "${n:-0}" -gt 0 ]; then
    echo "❌ 有 ${n} 个残留仿真进程，先收干净（按 PGID，别用 pkill -f）"
    ps -eo pgid,comm,stat | awk '$3 !~ /^Z/' | grep -E 'ruby|_node' | head
    exit 1
  fi
}

wait_clock() {  # wait_clock <目标仿真秒> <墙钟超时秒>
  timeout "$2" python3 - "$1" <<'PYEOF'
import sys, rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
target = float(sys.argv[1])
rclpy.init(); n = Node('wait_clock'); done = [False]
n.create_subscription(Clock, '/clock',
                      lambda m: done.__setitem__(0, m.clock.sec + m.clock.nanosec*1e-9 >= target), 10)
while not done[0]:
    rclpy.spin_once(n, timeout_sec=0.2)
PYEOF
}

goal_warm() {  # 绝对仿真钟 37.0 起每秒重发 (91.75, 20)（CP-P5-B 扩窗协议）
  python3 - <<'PYEOF' > "$RUN_DIR/goal.log" 2>&1 &
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
}

teardown() {
  [ -n "${GOAL_PID:-}" ] && kill "$GOAL_PID" 2>/dev/null; GOAL_PID=""
  [ -n "${PGID:-}" ] && kill -INT -- -"$PGID" 2>/dev/null
  sleep 4
  [ -n "${PGID:-}" ] && kill -KILL -- -"$PGID" 2>/dev/null
  PGID=""
  sleep 2
  rm -f "${REPO}"/core.*
}

FAILED=0
for round in $(seq 1 "$ROUNDS"); do
  RUN_DIR="${REPO}/.scenario_runs/p5_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$RUN_DIR"
  echo "════════ CP-P5-B Gazebo 第 ${round}/${ROUNDS} 轮 → ${RUN_DIR} ════════"
  guard_residual
  setsid ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false \
    perception:=true dynamic:=both > "$RUN_DIR/launch.log" 2>&1 &
  lpid=$!
  sleep 2
  PGID=$(ps -o pgid= -p "$lpid" 2>/dev/null | tr -d ' ')
  if [ -z "$PGID" ]; then echo "❌ launch 没起来"; tail -5 "$RUN_DIR/launch.log"; FAILED=1; continue; fi
  if ! wait_clock 8 150; then
    echo "❌ bringup 失败（150 s 墙钟内仿真钟没到 8）"; tail -20 "$RUN_DIR/launch.log"
    FAILED=1; teardown; continue
  fi
  goal_warm
  # 记录器立即早起（约 72 s 仿真 + 起停 ≈ 2.5 min 一轮）
  python3 scripts/record_perception_run.py --duration-s 72 --out "$RUN_DIR/p5.csv" \
    > "$RUN_DIR/p5.txt" 2>&1
  rc=$?
  teardown
  grep -vE "^\[INFO\]|^$" "$RUN_DIR/p5.txt" | tail -22
  case "$rc" in
    0) echo "VERDICT=PASS（第 ${round} 轮）" ;;
    2) echo "VERDICT=INVALID（刺激物坏了，第 ${round} 轮不算数）"; FAILED=1 ;;
    *) echo "VERDICT=FAIL（第 ${round} 轮）"; FAILED=1 ;;
  esac
done
exit "$FAILED"
