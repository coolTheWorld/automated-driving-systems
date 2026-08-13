#!/bin/bash
# =============================================================================
#  P7-S4 单场编排：起栈 → 预热 goal（绝对钟锚 37.0）→ 早起记录器 → 打分 → 收
#  用法：p7s4_run.sh <follow|crossing|junction> <truth|perception> <时长s> <输出csv>
#  铁律：所有等待带超时；失败中止并清理；trap EXIT 收尾。
# =============================================================================
set -u
SCEN=$1; LAYER=$2; DUR=$3; OUT=$4
LOGDIR=/workspace/.p7s4
mkdir -p "$LOGDIR"

cleanup() {
  [ -n "${PGID:-}" ] && kill -INT -- -"$PGID" 2>/dev/null
  [ -n "${GOAL_PID:-}" ] && kill "$GOAL_PID" 2>/dev/null
  sleep 3
  [ -n "${PGID:-}" ] && kill -KILL -- -"$PGID" 2>/dev/null
  rm -f /workspace/core.* 2>/dev/null
}
trap cleanup EXIT

cd /workspace
set +u; source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; set -u

# 残留守卫（滤僵尸）
n=$(ps -eo comm,stat | awk '$2 !~ /^Z/' | grep -cx -E 'ruby|control_node|planning_node|perception_node|prediction_node' || true)
if [ "${n:-0}" -gt 0 ]; then echo "❌ 残留 $n 个仿真进程"; ps -eo pgid,comm,stat | awk '$3 !~ /^Z/' | grep -E 'ruby|_node' | head; exit 1; fi

if [ "$LAYER" = truth ]; then PFLAG=false; else PFLAG=true; fi
# junction goal 取 (-1.75, -30)：初版 -20 踩进 P3 期的规划边界 —— ego 带着
# 出弯/跟车残余的 0.5 m 横向偏差进入路线末端时，1 m 跨度上的收敛五次多项式
# 曲率爆表 → 全候选淘汰 → kStopping 单点 → 车在离 goal 1.16 m 处 NO_PATH
# 停保持。南移 10 m 给收敛留跑道（P8 台账记规划近端+偏移边界）。
if [ "$SCEN" = junction ]; then GX=-1.75; GY=-30.0; else GX=91.75; GY=20.0; fi

setsid ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false \
  perception:=$PFLAG prediction:=true dynamic:=$SCEN \
  > "$LOGDIR/${SCEN}_${LAYER}.launch.log" 2>&1 &
LPID=$!
sleep 2
PGID=$(ps -o pgid= -p "$LPID" 2>/dev/null | tr -d ' ')
[ -z "$PGID" ] && { echo "❌ launch 没起来"; tail -5 "$LOGDIR/${SCEN}_${LAYER}.launch.log"; exit 1; }

# bringup：仿真钟到 8（墙钟超时 150）
timeout 150 python3 -c "
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init(); n = Node('wc'); done=[False]
n.create_subscription(Clock, '/clock', lambda m: done.__setitem__(0, m.clock.sec>=8), 10)
while not done[0]: rclpy.spin_once(n, timeout_sec=0.2)
" || { echo '❌ bringup 失败'; tail -20 "$LOGDIR/${SCEN}_${LAYER}.launch.log"; exit 1; }

# 预热 goal 发布器（绝对钟锚 37.0，RELIABLE+TRANSIENT_LOCAL，每秒重发到 47）
python3 - "$GX" "$GY" <<'PYEOF' > "$LOGDIR/${SCEN}_${LAYER}.goal.log" 2>&1 &
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
gx, gy = float(sys.argv[1]), float(sys.argv[2])
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
        g.pose.position.x = gx; g.pose.position.y = gy; g.pose.orientation.w = 1.0
        pub.publish(g); state['sent'] += 1; state['last'] = state['t']
print(f"[goal_warm] 发送 {state['sent']} 次", flush=True)
PYEOF
GOAL_PID=$!

echo "===== [$SCEN/$LAYER] 记录 ${DUR}s ====="
python3 /workspace/scripts/record_behavior_run.py \
  --scenario "$SCEN" --layer "$LAYER" --duration-s "$DUR" --out "$OUT"
RC=$?
echo "score_rc=$RC"
exit $RC
