#!/usr/bin/env bash
# =============================================================================
#  P0a 任务 3.4 - 3.7 —— 检查点 CP2：仿真数据是否以规范格式进了 ROS
#
#  验六件事：
#     1  SPEC §4.1 的规范话题是否齐全
#     2  /clock 是否在推进（所有节点靠它对时）
#     3  各节点 use_sim_time 是否都为 true
#     4  点云契约：frame_id / 频率 / 是否真的做了坐标变换
#     5  TF 树 map → odom → base_link → 各传感器 是否连通
#     6  加了传感器之后 RTF 是否仍达标
#
#  ★ 这个脚本将来要能原样验 carla_bridge。它检查的全是 SPEC §4.1 规定的
#    对外契约，不涉及任何 Gazebo 特有的东西 —— 换成 CARLA 时只需换掉
#    下面 LAUNCH_* 两个变量。这正是「仿真数据源可插拔」的验收方式。
#
#  在**容器内**执行：
#      docker compose exec dev /workspace/scripts/verify_ros_bridge.sh
# =============================================================================
set -uo pipefail

LAUNCH_PKG="${LAUNCH_PKG:-gazebo_bridge}"
LAUNCH_FILE="${LAUNCH_FILE:-gazebo_sim.launch.py}"
WORLD_NAME="${WORLD_NAME:-campus_minimal}"

HZ_MIN=9.0        # 计划 3.5 的验收线（10 Hz 标称，留 10% 余量）
RTF_MIN=0.8       # 与 S2 同一条线
STARTUP_WAIT=25   # Gazebo 加载 + 传感器出第一帧所需时间

PASS=0
fail() { echo "  ✗ $1"; PASS=1; }
ok()   { echo "  ✓ $1"; }

# ROS 与 Gazebo 都装在 /opt/ros/jazzy 下，脚本不走 .bashrc 所以要显式 source。
# setup.bash 会读未定义变量，set -u 下会直接退出，所以临时关掉。
if ! command -v gz >/dev/null 2>&1; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
set +u
# shellcheck disable=SC1091
source /workspace/install/setup.bash
set -u

# -----------------------------------------------------------------------------
# 进程收尾
#
# 这一段被返工过两次，两个坑都值得记住：
#
# ⚠️ 坑一：不要用 pkill -f "gz sim" 这类模式清理。
#    pkill -f 匹配的是**完整命令行**，而本脚本的命令行里就含这些字符串 ——
#    结果是脚本把自己先杀了，后面的清理一行都没执行。
#
# ⚠️ 坑二：不要只对 ros2 launch 的 PID 发信号。
#    launch 被 SIGINT 后需要若干秒去逐个关子进程，中途补一刀 SIGKILL 的话，
#    gz 服务端和 parameter_bridge 会变成**孤儿**（PPID=1）继续运行。
#    下次再跑就有两套仿真同时发 /clock 和 /tf，症状是 TF 疯狂报
#    "Detected jump back in time" / "TF_OLD_DATA"，而所有测量值都是垃圾。
#
# 所以：用 setsid 让 launch 自成一个进程组，收尾时对**整个进程组**发信号
#（kill 的 PID 参数取负数即为进程组）。这样漏网的子进程也一并带走。
# -----------------------------------------------------------------------------
LAUNCH_PGID=""
cleanup() {
  [[ -z "${LAUNCH_PGID}" ]] && return
  kill -INT -- "-${LAUNCH_PGID}" 2>/dev/null
  for _ in $(seq 1 30); do
    pgrep -g "${LAUNCH_PGID}" >/dev/null 2>&1 || return
    sleep 0.5
  done
  # 15 秒还没退干净才动 SIGKILL，此时整组一起收，不会留孤儿
  kill -9 -- "-${LAUNCH_PGID}" 2>/dev/null
}
trap cleanup EXIT

# -----------------------------------------------------------------------------
# 开跑前拦截：已经有仿真在跑就直接退出
#
# 两套仿真并存时，所有测量值都不可信，但脚本照样会跑完并给出一个数 ——
# 这比直接失败更危险。宁可拒绝启动。
# -----------------------------------------------------------------------------
STALE="$(pgrep -f 'gz sim' 2>/dev/null | grep -v "^$$\$" | wc -l)"
if [[ "${STALE}" -gt 0 ]]; then
  echo "✗ 检测到已有 ${STALE} 个 gz sim 进程在运行。" >&2
  echo "  两套仿真会同时发 /clock 与 /tf，测出来的数全部不可信。" >&2
  echo "  先清理（按 PID，不要用 pkill -f）：" >&2
  ps -eo pid,etime,args | grep -E 'gz sim|parameter_bridge' | grep -v grep >&2
  exit 1
fi

echo "=============================================="
echo " P0a 3.4-3.7  CP2：ROS 2 桥接与规范话题验证"
echo "=============================================="
echo " launch：${LAUNCH_PKG} / ${LAUNCH_FILE}"

echo
echo "[1/6] 启动仿真 + 桥接（headless，无 RViz）"
setsid ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" gui:=false rviz:=false \
  > /tmp/verify_bridge.log 2>&1 &
LAUNCH_PID=$!
sleep 2
LAUNCH_PGID="$(ps -o pgid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d ' ')"
if [[ -z "${LAUNCH_PGID}" ]]; then
  fail "launch 没能启动"
  tail -30 /tmp/verify_bridge.log | sed 's/^/       /'
  exit 1
fi

# ROS 2 的命令行工具靠一个长驻 daemon 缓存节点图。daemon 刚起来时图是空的，
# 立刻查 topic list 只会拿到 /rosout 和 /parameter_events —— 不是话题没有，
# 是还没发现完。这里先把 daemon 叫醒，再等仿真加载，两件事并行省时间。
ros2 daemon start >/dev/null 2>&1 || true
sleep "${STARTUP_WAIT}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  fail "launch 进程已退出"
  tail -30 /tmp/verify_bridge.log | sed 's/^/       /'
  exit 1
fi

# SPEC §4.1 的规范话题名。改这张表 = 改跨模块契约，先改 SPEC。
# 注：/vehicle_cmd 是 S4 才接的反向通道，/ego_pose_gt 留到 P1 评测时再定，
#     这里不检查它们。
EXPECTED_TOPICS=(
  /clock
  /lidar/points
  /imu
  /gnss
  /odom
  /tf
  /joint_states
)
TOPIC_LIST="$(ros2 topic list 2>/dev/null)"
MISSING=0
for t in "${EXPECTED_TOPICS[@]}"; do
  if ! grep -qx -- "${t}" <<< "${TOPIC_LIST}"; then
    fail "缺少规范话题 ${t}"
    MISSING=1
  fi
done
if [[ ${MISSING} -eq 0 ]]; then
  ok "SPEC §4.1 规范话题齐全（${#EXPECTED_TOPICS[@]} 个）"
else
  echo "     实际话题列表："
  sed 's/^/       /' <<< "${TOPIC_LIST}"
fi

echo
echo "[2/6] 仿真时钟 /clock 是否在推进"
# 超时给到 30 s：ros2 topic echo 每次都新建一个节点，而这套环境里
# 新节点完成 DDS 发现要十几秒（host 网络下网卡多，发现报文要遍历一遍）。
# 给 10 s 的话会间歇性失败，看起来像「话题没数据」，其实只是还没发现完。
C0="$(timeout 30 ros2 topic echo /clock --once 2>/dev/null \
      | awk '/sec:/ && !/nanosec/ {print $2; exit}')"
sleep 2
C1="$(timeout 30 ros2 topic echo /clock --once 2>/dev/null \
      | awk '/sec:/ && !/nanosec/ {print $2; exit}')"
if [[ -z "${C0}" || -z "${C1}" ]]; then
  fail "读不到 /clock —— 桥接表里漏了 clock，或 Gazebo 没在跑"
  echo "     所有节点都在等这个时钟，缺了它整条链路会静默卡死。"
elif (( C1 > C0 )); then
  ok "/clock 从 ${C0}s 推进到 ${C1}s"
else
  fail "/clock 停在 ${C0}s 没动 —— 世界可能是暂停状态（gz sim 少了 -r）"
fi

echo
echo "[3/6] 各节点 use_sim_time"
BAD_NODES=""
for n in /lidar_preprocessor /vehicle_cmd_bridge /robot_state_publisher /gazebo_bridge /map_to_odom_identity; do
  V="$(timeout 8 ros2 param get "${n}" use_sim_time 2>/dev/null | grep -oiE 'true|false' | head -1)"
  if [[ "${V,,}" != "true" ]]; then
    BAD_NODES="${BAD_NODES} ${n}(${V:-读不到})"
  fi
done
if [[ -z "${BAD_NODES}" ]]; then
  ok "五个节点的 use_sim_time 均为 true"
else
  fail "以下节点 use_sim_time 不为 true：${BAD_NODES}"
  echo "     混用真实时间会让时间戳对不上，TF 报 extrapolation，且 RTF≈1 时看不出来。"
fi

echo
echo "[4/6] 点云契约：坐标系 / 频率 / 是否真的做了变换"
# 这三项交给 Python 脚本一次查完，不用 ros2 topic echo|hz 拼。
# 原因见该脚本头部：CLI 输出到管道是全缓冲的，被 timeout 打断会丢结果，
# 表现为「测不到数据」的假阴性；而且每条命令都要重走一遍 DDS 发现。
if python3 /workspace/scripts/check_cloud_frames.py \
     --timeout 40 --min-rate "${HZ_MIN}"; then
  ok "点云契约全部满足"
else
  fail "点云契约未满足（详见上面的数值）"
fi

echo
echo "[5/6] TF 树 map → odom → base_link → 各传感器"
if python3 /workspace/scripts/check_tf_tree.py --timeout 30; then
  ok "TF 树连通"
else
  fail "TF 树有断裂"
fi

echo
echo "[6/6] 实时率 RTF（带激光雷达，验收线 ${RTF_MIN}）"
RTF="$(gz topic -e -t "/world/${WORLD_NAME}/stats" -n 12 2>/dev/null \
        | awk '/real_time_factor:/ {s+=$2; n++} END{if(n>0) printf "%.3f", s/n}')"
if [[ -z "${RTF}" ]]; then
  fail "读不到 real_time_factor"
else
  echo "     12 帧均值：RTF = ${RTF}"
  if awk -v r="${RTF}" -v m="${RTF_MIN}" 'BEGIN{exit !(r >= m)}'; then
    ok "RTF ${RTF} ≥ ${RTF_MIN}"
  else
    fail "RTF ${RTF} < ${RTF_MIN} —— 激光雷达把帧率拖下去了"
  fi
fi

echo
echo "=============================================="
if [[ ${PASS} -eq 0 ]]; then
  echo " 结果：通过 ✓   CP2 达成，数据流已打通"
else
  echo " 结果：失败 ✗   完整日志：/tmp/verify_bridge.log"
fi
echo "=============================================="
exit ${PASS}
