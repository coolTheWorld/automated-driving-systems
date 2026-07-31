#!/usr/bin/env bash
# =============================================================================
#  P0a 任务 4.1 - 4.3 —— 键盘 teleop 与控制指令链路验证
#
#  验两件事：
#     1  键盘节点：按键 → /vehicle_cmd 是否正确（含符号约定与限值）
#     2  控制链路：/vehicle_cmd → Gazebo 是否驱动车辆、限幅与看门狗是否生效
#
#  4.4「开车绕障碍物一圈」是肉眼验收，不在本脚本内 —— 见 tasks/todo.md。
#
#  ★ 与 verify_ros_bridge.sh 一样，这里检查的是 SPEC §4.1 的对外契约
#    （/vehicle_cmd 的语义和限值），不含任何 Gazebo 特有的东西。
#    P0b 换成 carla_bridge 时只需换掉 LAUNCH_* 两个变量。
#
#  在**容器内**执行：
#      docker compose exec dev /workspace/scripts/verify_teleop.sh
# =============================================================================
set -uo pipefail

LAUNCH_PKG="${LAUNCH_PKG:-gazebo_bridge}"
LAUNCH_FILE="${LAUNCH_FILE:-gazebo_sim.launch.py}"
STARTUP_WAIT=25

PASS=0
fail() { echo "  ✗ $1"; PASS=1; }
ok()   { echo "  ✓ $1"; }

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

# 进程收尾：用 setsid 让 launch 自成进程组，收尾时对整组发信号。
# 两个坑的详细说明见 scripts/verify_ros_bridge.sh 里同样的一段
# （别用 pkill -f，别只 kill launch 的 PID）。
LAUNCH_PGID=""
cleanup() {
  [[ -z "${LAUNCH_PGID}" ]] && return
  kill -INT -- "-${LAUNCH_PGID}" 2>/dev/null
  for _ in $(seq 1 30); do
    pgrep -g "${LAUNCH_PGID}" >/dev/null 2>&1 || return
    sleep 0.5
  done
  kill -9 -- "-${LAUNCH_PGID}" 2>/dev/null
}
trap cleanup EXIT

STALE="$(pgrep -f 'gz sim' 2>/dev/null | wc -l)"
if [[ "${STALE}" -gt 0 ]]; then
  echo "✗ 检测到已有 ${STALE} 个 gz sim 进程在运行。" >&2
  echo "  两套仿真会同时发 /clock 与 /odom，测出来的数全部不可信。" >&2
  ps -eo pid,pgid,etime,args | grep -E 'gz sim|ros2 launch' | grep -v grep >&2
  exit 1
fi

echo "=============================================="
echo " P0a 4.1-4.3  键盘 teleop 与控制指令链路"
echo "=============================================="

echo
echo "[1/3] 启动仿真 + 桥接（headless，无 RViz）"
setsid ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" gui:=false rviz:=false \
  > /tmp/verify_teleop.log 2>&1 &
LAUNCH_PID=$!
sleep 2
LAUNCH_PGID="$(ps -o pgid= -p "${LAUNCH_PID}" 2>/dev/null | tr -d ' ')"
if [[ -z "${LAUNCH_PGID}" ]]; then
  fail "launch 没能启动"
  tail -30 /tmp/verify_teleop.log | sed 's/^/       /'
  exit 1
fi
ros2 daemon start >/dev/null 2>&1 || true
sleep "${STARTUP_WAIT}"

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  fail "launch 进程已退出"
  tail -30 /tmp/verify_teleop.log | sed 's/^/       /'
  exit 1
fi
if ros2 topic list 2>/dev/null | grep -qx -- /vehicle_cmd; then
  ok "/vehicle_cmd 已被 vehicle_cmd_bridge 订阅"
else
  fail "/vehicle_cmd 不在话题列表里 —— vehicle_cmd_bridge 没起来"
  grep -iE "error|failed" /tmp/verify_teleop.log | head -10 | sed 's/^/       /'
fi

echo
echo "[2/3] 键盘节点：按键 → /vehicle_cmd（任务 4.1）"
if python3 /workspace/scripts/check_keyboard_teleop.py; then
  ok "键盘节点行为正确"
else
  fail "键盘节点行为不符（详见上面）"
fi

echo
echo "[3/3] 控制链路：/vehicle_cmd → Gazebo（任务 4.2 / 4.3）"
if python3 /workspace/scripts/check_vehicle_cmd.py; then
  ok "控制链路、限幅与看门狗均正常"
else
  fail "控制链路验证未通过（详见上面）"
fi

echo
echo "=============================================="
if [[ ${PASS} -eq 0 ]]; then
  echo " 结果：通过 ✓   4.1-4.3 达成，剩 4.4 肉眼验收"
else
  echo " 结果：失败 ✗   完整日志：/tmp/verify_teleop.log"
fi
echo "=============================================="
exit ${PASS}
