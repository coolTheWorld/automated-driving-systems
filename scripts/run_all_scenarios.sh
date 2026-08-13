#!/bin/bash
# =============================================================================
#  run_all_scenarios.sh —— L3 全场景一键（SPEC §6 承诺的那条命令，P8-S2 兑现）
#
#      ./scripts/run_all_scenarios.sh gazebo                # 全场景（合并前跑）
#      ./scripts/run_all_scenarios.sh gazebo S03            # 只跑一个
#      ./scripts/run_all_scenarios.sh gazebo --with-regression   # 附加 P6 抽轮
#
#  ## 它是「编排」，不是「判据」
#
#  每个场景的判据都活在对应的 record_*_run.py 里（判据来自 tasks/plan.md，
#  脚本不重新发明）——本脚本只做：起栈 → 预热 goal → 起记录器 → 收栈 →
#  汇总判定 → 追加 metrics/history.csv（SPEC §8 L4 的指标曲线数据落点）。
#
#  ## 场景 → 判据的代表关系（SPEC §8 场景表 ↔ record_* 家族）
#
#  | SPEC | 由谁代表 | 说明 |
#  |---|---|---|
#  | S01+S02+S07 | record_control_run（CP-P2-B 8 条） | 95 m 路线含直道+R13.75 弯+到达停止 |
#  | S03 | record_behavior_run follow（双层） | TTC/跟停/驶离恢复 |
#  | S04 | record_obstacle_run avoid+block | 绕行 0.5 m / 停住 |
#  | S05 | record_behavior_run crossing（双层） | 完全停止/最近距离/恢复 |
#  | junction | record_behavior_run junction（双层） | P7 的无信号路口（SPEC 表外，让行原语的路口实例） |
#  | S06 | —— | 仅 L3-C（SPEC §8 注解，P7 拍板）；run_all carla 才跑 |
#
#  ## 为什么它不在 GHA CI 里跑（P8 决策二）
#
#  真 Gazebo 需要 GPU（RTF 判据），GHA 免费 runner 没有。所以分工是：
#  GHA 自动跑无-GPU 的九个 launch 测试；本脚本在**本机**合并前一键跑。
#
#  铁律（与 P7-S4 编排同一套）：所有等待带超时；失败中止并清理；trap 收尾。
# =============================================================================
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_LABEL="${1:-}"
shift || true
ONLY_SCENARIO=""
WITH_REGRESSION=0
for arg in "$@"; do
  case "$arg" in
    --with-regression) WITH_REGRESSION=1 ;;
    S0*|junction) ONLY_SCENARIO="$arg" ;;
    *) echo "未知参数：$arg"; exit 2 ;;
  esac
done

if [ "$ENV_LABEL" != gazebo ]; then
  if [ "$ENV_LABEL" = carla ]; then
    echo "carla：L3-C 在云机上跑（P8-S5/S6），本机没有 Vulkan —— 见 CLAUDE.md 环境陷阱表"
    exit 2
  fi
  echo "用法：run_all_scenarios.sh gazebo [S01|S03|S04|S05|junction] [--with-regression]"
  exit 2
fi

RUN_DIR="${REPO}/.scenario_runs/$(date +%Y%m%d_%H%M%S)"
METRICS="${REPO}/metrics/history.csv"
mkdir -p "$RUN_DIR"

cleanup() {
  [ -n "${PGID:-}" ] && kill -INT -- -"$PGID" 2>/dev/null
  [ -n "${GOAL_PID:-}" ] && kill "$GOAL_PID" 2>/dev/null
  sleep 3
  [ -n "${PGID:-}" ] && kill -KILL -- -"$PGID" 2>/dev/null
  rm -f "${REPO}"/core.*
}
trap cleanup EXIT

set +u; source /opt/ros/jazzy/setup.bash; source "${REPO}/install/setup.bash"; set -u
cd "$REPO"

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

goal_warm() {  # goal_warm <x> <y>：绝对仿真钟 37.0 起每秒重发（扩窗协议）
  python3 - "$1" "$2" <<'PYEOF' > "$RUN_DIR/goal.log" 2>&1 &
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
print(f"sent {state['sent']}", flush=True)
PYEOF
  GOAL_PID=$!
}

launch_stack() {
  guard_residual
  setsid ros2 launch ads_bringup stack.launch.py gui:=false rviz:=false "$@" \
    > "$RUN_DIR/${TAG}.launch.log" 2>&1 &
  local lpid=$!
  sleep 2
  PGID=$(ps -o pgid= -p "$lpid" 2>/dev/null | tr -d ' ')
  if [ -z "$PGID" ]; then echo "❌ [$TAG] launch 没起来"; tail -5 "$RUN_DIR/${TAG}.launch.log"; return 1; fi
  if ! wait_clock 8 150; then
    echo "❌ [$TAG] bringup 失败（150 s 墙钟内仿真钟没到 8）"
    tail -20 "$RUN_DIR/${TAG}.launch.log"; return 1
  fi
}

teardown() {
  [ -n "${GOAL_PID:-}" ] && kill "$GOAL_PID" 2>/dev/null; GOAL_PID=""
  kill -INT -- -"$PGID" 2>/dev/null
  sleep 4
  kill -KILL -- -"$PGID" 2>/dev/null
  PGID=""
  sleep 2
  rm -f "${REPO}"/core.*
}

declare -A RESULT
FAILED=0

run_one() {  # run_one <标签> <记录器命令数组通过全局 REC_CMD> ；launch 参数在 LAUNCH_ARGS
  TAG="$1"
  echo ""
  echo "════════ [$TAG] ════════"
  if ! launch_stack "${LAUNCH_ARGS[@]}"; then
    RESULT[$TAG]="LAUNCH_FAIL"; FAILED=1; teardown 2>/dev/null || true; return
  fi
  if [ -n "${GOAL_XY:-}" ]; then
    goal_warm ${GOAL_XY}
  fi
  if "${REC_CMD[@]}"; then
    RESULT[$TAG]="PASS"
  else
    RESULT[$TAG]="FAIL"; FAILED=1
  fi
  teardown
}

want() {  # 场景过滤
  [ -z "$ONLY_SCENARIO" ] || [ "$ONLY_SCENARIO" = "$1" ]
}

# ---------------------------------------------------------------------------
#  场景注册表（一处定义：launch 参数 / goal / 记录器）
#  ⚠️ 与 ads_bringup/launch/scenario.launch.py 的映射表是同一套对应关系
#     （那边只起栈不带判定）—— 改一处必改另一处。
# ---------------------------------------------------------------------------
if want S01; then
  # S01+S02+S07：CP-P2-B 的 95 m 路线（直道巡航 + 弯道 + 到达停止）。
  # record_control_run 自带 goal 发布（8 s 延迟协议），不用 goal_warm。
  LAUNCH_ARGS=(); GOAL_XY=""
  REC_CMD=(python3 scripts/record_control_run.py --goal 91.75 20.0
           --out "$RUN_DIR/s01.csv" --metrics-out "$METRICS")
  run_one "S01_S02_S07"
fi

if want S04; then
  LAUNCH_ARGS=(obstacles:=avoid); GOAL_XY=""
  REC_CMD=(python3 scripts/record_obstacle_run.py --scenario avoid
           --out "$RUN_DIR/s04a.csv" --metrics-out "$METRICS")
  run_one "S04_avoid"
  LAUNCH_ARGS=(obstacles:=block); GOAL_XY=""
  REC_CMD=(python3 scripts/record_obstacle_run.py --scenario block
           --out "$RUN_DIR/s04b.csv" --metrics-out "$METRICS")
  run_one "S04_block"
fi

# 行为场景：双层协议（真值层 + 感知层）。
behavior_pair() {  # behavior_pair <场景> <时长> <goal_x> <goal_y>
  local scen="$1" dur="$2" gx="$3" gy="$4"
  for layer in truth perception; do
    if [ "$layer" = truth ]; then PFLAG=false; else PFLAG=true; fi
    LAUNCH_ARGS=(perception:=$PFLAG prediction:=true dynamic:=$scen)
    GOAL_XY="$gx $gy"
    REC_CMD=(python3 scripts/record_behavior_run.py --scenario "$scen" --layer "$layer"
             --duration-s "$dur" --out "$RUN_DIR/${scen}_${layer}.csv" --metrics-out "$METRICS")
    run_one "${scen}_${layer}"
  done
}

if want S03; then behavior_pair follow 90 91.75 20.0; fi
if want S05; then behavior_pair crossing 75 91.75 20.0; fi
# junction goal (-1.75,-30)：路由唯一解经 j_north 左转（让行几何由地图保证）；
# -30 而不是 -20 是避开规划近端+横向偏移边界（P8 台账 S2d，见 plan.md）。
if want junction; then behavior_pair junction 135 -1.75 -30.0; fi

if [ "$WITH_REGRESSION" = 1 ] && [ -z "$ONLY_SCENARIO" ]; then
  # CP-P6-B 抽轮（真值层 curve）—— 阶段回归判据 ⑩ 的常备形式。
  LAUNCH_ARGS=(prediction:=true dynamic:=curve)
  GOAL_XY="91.75 20.0"
  REC_CMD=(python3 scripts/record_prediction_run.py --duration-s 72 --layer truth
           --out "$RUN_DIR/regression_p6.csv" --metrics-out "$METRICS")
  run_one "regression_p6"
fi

# ---------------------------------------------------------------------------
#  汇总
# ---------------------------------------------------------------------------
echo ""
echo "════════════ L3-G 全场景汇总 ════════════"
for key in "${!RESULT[@]}"; do
  printf '  %-22s %s\n' "$key" "${RESULT[$key]}"
done | sort
echo "metrics 追加至：$METRICS"
echo "过程产物：$RUN_DIR"
if [ "$FAILED" -ne 0 ]; then
  echo "❌ 有场景未通过"
  exit 1
fi
echo "✅ 全部通过"
