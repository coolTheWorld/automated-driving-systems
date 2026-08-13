#!/usr/bin/env bash
# =============================================================================
#  P1 任务 4.6 —— 地图与路由的可复跑量化验收
#
#  验三件事：
#     1  SPEC §3.3 的机械校验：libads_map.so **不许**链接任何 ROS 库
#     2  map_node 起得来、车道图发得出、路由算得对（check_map_node.py 的 7 组判据）
#     3  失败路径也讲得清楚：拿不到 TF 时不静默、不崩
#
#  ★ 本脚本**不需要 Gazebo**。map_node 只要 TF map→base_link 就能工作，
#    用一个 static_transform_publisher 把自车钉在已知位姿上就够了。
#    好处不是省事，是这样它**能进 CI**：不要 GPU、不到 10 秒、结果完全确定。
#    仿真那一侧该验的东西由 verify_ros_bridge.sh 覆盖，两者不重叠。
#
#  ★ 4.4「RViz 里看到车道图、点一下出路径」是肉眼验收，不在本脚本内 ——
#    见 tasks/todo.md。本脚本能证明数据是对的，证明不了它画出来好不好看。
#
#  在**容器内**执行：
#      docker compose exec dev /workspace/scripts/verify_map.sh
# =============================================================================
set -uo pipefail

# 自车的起始位姿。取 campus_loop.sdf 里自车的 spawn 位姿，
# 这样验收走的是**真实系统会走的那条路**，而不是一个为测试挑出来的方便位置。
# 它落在 (road 1, lane −1) 的 s=19 处，朝向 +x。
EGO_X="${EGO_X:-30.0}"
EGO_Y="${EGO_Y:--51.75}"
EGO_YAW="${EGO_YAW:-0.0}"

STARTUP_WAIT=5

PASS=0
fail() { echo "  ✗ $1"; PASS=1; }
ok()   { echo "  ✓ $1"; }

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /workspace/install/setup.bash
set -u

# 进程收尾：用 setsid 让节点自成进程组，收尾时对整组发信号。
#
# ⚠️ PGID **不能**用 `ps -o pgid= -p $!` 取 —— setsid 会 fork 出新进程组后
#    自己立刻退出，$! 指的是那个已经死掉的 setsid，查出来是空字符串，
#    于是 `kill -INT -- -` 变成空操作**还不报错**。按进程名查才对。
#    （这一条是 P0a 实测踩出来的，见 CLAUDE.md 陷阱表。）
NODE_PGIDS=""
cleanup() {
  for pgid in ${NODE_PGIDS}; do
    kill -INT -- "-${pgid}" 2>/dev/null
  done
  sleep 2
  for pgid in ${NODE_PGIDS}; do
    kill -9 -- "-${pgid}" 2>/dev/null
  done
  sleep 1
  # 收尾没收干净要**说出来**。留下的 map_node 会让下一次运行直接被开头的
  # 残留检查拦下，届时只会看到「已经有 2 个在跑」，追不到是哪一次漏的。
  # ⚠️ 要滤掉僵尸（P6-S2 实测）：pgrep 连 <defunct> 一起数，容器 PID 1 不 reap，
  #    编排跑多了僵尸成百上千 —— 「残留 66 个」全是尸体。CLAUDE.md 陷阱表
  #    「pgrep -x gz 查残留」那条的同族，按 stat 首字母滤 Z。
  local left
  left="$(ps -eo stat=,comm= | awk '$1 !~ /^Z/ && ($2 == "map_node" || $2 == "static_transform_publisher")' | wc -l)"
  [[ "${left}" -gt 0 ]] && echo "  ! 收尾后仍有 ${left} 个进程残留，请手动清理" >&2
  return 0
}
trap cleanup EXIT

# start_node <日志文件> <查进程组用的关键字> <命令...>
#
# ⚠️ 关键字必须显式传，不能从命令里猜。第一版写的是取 shift 后的 $2，
#    而 `ros2 run ads_map map_node` 的 $2 是 "run" —— grep "run" 命中一堆
#    无关进程，于是收尾时**杀了别人的进程组，自己的留了下来**。
start_node() {
  local log="$1"
  local key="$2"
  shift 2
  setsid "$@" > "${log}" 2>&1 &
  sleep 1
  local pgid
  pgid="$(ps -eo pgid,args | grep -F -- "${key}" | grep -v grep | head -1 | awk '{print $1}')"
  if [[ -n "${pgid}" ]]; then
    NODE_PGIDS="${NODE_PGIDS} ${pgid}"
  else
    echo "  ! 没查到「${key}」的进程组，收尾时会留下残留进程" >&2
  fi
}

echo "=============================================="
echo " P1 4.1-4.3 / 4.6  地图与路由量化验收"
echo "=============================================="

# -----------------------------------------------------------------------------
echo
echo "[1/3] SPEC §3.3：算法库不许链接 ROS"
# -----------------------------------------------------------------------------
# 这条以前由「CMakeLists 里没有 find_package(rclcpp)」保证。S4 加了 map_node
# 之后 rclcpp 必然出现在那个文件里，于是约束的落点变成了「本 target 的
# target_link_libraries」—— 那是靠纪律维持的，纪律迟早会破。所以在这里补一道机械校验。
LIB="/workspace/build/ads_map/libads_map.so"
if [[ ! -f "${LIB}" ]]; then
  fail "找不到 ${LIB} —— 先 colcon build"
else
  ROS_LIBS="$(ldd "${LIB}" | grep -Ei 'librclcpp|librcl\.|librmw|librosidl|libtf2' || true)"
  if [[ -z "${ROS_LIBS}" ]]; then
    ok "libads_map.so 零 ROS 依赖（L1 测试因此能保持毫秒级）"
  else
    fail "libads_map.so 链接了 ROS 库："
    echo "${ROS_LIBS}" | sed 's/^/       /'
  fi
fi

# -----------------------------------------------------------------------------
echo
echo "[2/3] 还没有 TF 时必须说清楚，而不是静默"
# -----------------------------------------------------------------------------
# 顺序是有讲究的：**先只起 map_node、不起 TF**，点一个目标，期望日志里出现
# 「拿不到 TF」。这正是真实场景里最常见的一种失败 —— 节点起来了、仿真还没起。
#
# ⚠️ 不要反过来做成「先起 TF 再杀掉它」：`static_transform_publisher` 发的是
#    /tf_static，而 **tf2 的 buffer 对静态变换永不过期**。发布者死了之后
#    lookupTransform 照样成功，测不出任何东西。这条是本脚本第一版实测踩出来的。
#
# 这条检查的是**故障可诊断性**：这条链路上四种失败（TF 没有、车不在路上、
# 目标点太远、两点不可达）的表象都是「RViz 里没有路径」。分不开的话只能靠猜。
# ⚠️ 用 `pgrep -x`（按**进程名**精确匹配）而不是 `pgrep -f`（按完整命令行）。
# -f 会匹配到**执行本脚本的那条命令行自己**，只要它里面出现过 map_node 这几个字
# （比如 `clang-format -i src/ads_map/node/map_node.cpp && verify_map.sh`）——
# 于是脚本报「已经有 1 个在跑」然后拒绝启动，而实际上一个都没有。
# 这和 CLAUDE.md 陷阱表里 `pkill -f "gz sim"` 杀掉自己是同一个形状的坑。
# ⚠️ 滤僵尸（P6-S2 实测）：pgrep 连 <defunct> 一起数，见 cleanup 里的同款注释。
STALE="$(ps -eo stat=,comm= | awk '$1 !~ /^Z/ && $2 == "map_node"' | wc -l)"
if [[ "${STALE}" -gt 0 ]]; then
  echo "  ✗ 已经有 ${STALE} 个 map_node 在跑，两个节点抢同一个话题，测出来的数不可信" >&2
  ps -eo pid,pgid,stat,args | grep map_node | grep -v grep >&2
  exit 1
fi

start_node /tmp/verify_map_node.log "ads_map map_node" ros2 run ads_map map_node
ros2 daemon start >/dev/null 2>&1 || true
sleep "${STARTUP_WAIT}"

if ! pgrep -x map_node >/dev/null 2>&1; then
  fail "map_node 没起来或已退出"
  tail -30 /tmp/verify_map_node.log | sed 's/^/       /'
  exit 1
fi
ok "map_node 在跑"

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.75, y: 1.0}, orientation: {w: 1.0}}}' \
  >/dev/null 2>&1
sleep 2

if grep -q "拿不到" /tmp/verify_map_node.log; then
  ok "TF 缺失时有明确日志，指出的是 TF 这一环而不是别的"
else
  fail "TF 缺失时没有对应日志 —— 这类失败会退化成「点了没反应」"
  tail -15 /tmp/verify_map_node.log | sed 's/^/       /'
fi

# -----------------------------------------------------------------------------
echo
echo "[3/3] 给上 TF 之后，map_node 的对外行为"
# -----------------------------------------------------------------------------
# 自车位姿：直接发 map → base_link 的静态 TF。
# 真实系统里这一段由 odom 链路提供，这里把它换成一个确定的常量，
# 被测的东西（map_node 怎么用这个位姿）完全一样。
start_node /tmp/verify_map_tf.log "static_transform_publisher" \
  ros2 run tf2_ros static_transform_publisher \
  --x "${EGO_X}" --y "${EGO_Y}" --z 0 --roll 0 --pitch 0 --yaw "${EGO_YAW}" \
  --frame-id map --child-frame-id base_link
sleep 3

echo
if python3 /workspace/scripts/check_map_node.py; then
  ok "7 组判据全部通过"
else
  fail "量化判据未通过（详见上面）"
  echo "  map_node 日志："
  tail -20 /tmp/verify_map_node.log | sed 's/^/       /'
fi

echo
echo "=============================================="
if [[ ${PASS} -eq 0 ]]; then
  echo " 结果：通过 ✓   4.1-4.3 / 4.6 达成，剩 4.4 肉眼验收"
else
  echo " 结果：失败 ✗   日志：/tmp/verify_map_node.log"
fi
echo "=============================================="
exit ${PASS}
