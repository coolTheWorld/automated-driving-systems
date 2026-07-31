#!/usr/bin/env bash
# =============================================================================
#  drive.sh —— 键盘开车（P0a 任务 4.4）
#
#  用法（容器内，需要一个真正的终端）：
#      docker compose exec dev /workspace/scripts/drive.sh
#
#  ⚠️ 为什么不用 `ros2 launch ads_teleop keyboard_teleop.launch.py`
#  ---------------------------------------------------------------
#  `ros2 launch` 会接管子进程的 stdio：子进程的 stdin 是一根管道，不是你的终端，
#  **键盘输入根本到不了节点**。节点会打一条告警然后一直发零指令，车不会动。
#
#  （节点本身在 launch 下不会假死 —— stdin 上加了 O_NONBLOCK。
#    早期版本没加，症状是进程活着、publisher 也在，但一条消息都发不出来，
#    因为 read() 在管道上永久阻塞、把定时器回调卡死了。）
#
#  所以交互驾驶一律用 `ros2 run`，本脚本只是帮你把参数从 YAML 里取出来拼好。
#
#  典型流程（两个终端）：
#      终端 A: docker compose exec dev bash -c 'source /opt/ros/jazzy/setup.bash && \
#                source /workspace/install/setup.bash && \
#                ros2 launch gazebo_bridge gazebo_sim.launch.py'
#      终端 B: docker compose exec dev /workspace/scripts/drive.sh
# =============================================================================
set -uo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
set +u
# shellcheck disable=SC1091
source /workspace/install/setup.bash
set -u

if [[ ! -t 0 ]]; then
  echo "✗ stdin 不是终端，键盘输入进不来。" >&2
  echo "  请用 'docker compose exec dev /workspace/scripts/drive.sh'（带 -it，默认就有）。" >&2
  exit 1
fi

PARAMS="/workspace/config/vehicle_params.yaml"

# 限值从**唯一来源**取（SPEC §4.1），脚本里不写任何车辆参数。
read -r MAX_STEER MAX_ACCEL MAX_DECEL EMER_DECEL <<< "$(
  python3 - "${PARAMS}" <<'PYEOF'
import sys, yaml
lim = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["limits"]
print(lim["max_steer_angle_rad"], lim["max_accel_mps2"],
      lim["max_decel_mps2"], lim["emergency_decel_mps2"])
PYEOF
)"

if [[ -z "${MAX_STEER:-}" ]]; then
  echo "✗ 读不到 ${PARAMS} 里的 limits 段" >&2
  exit 1
fi

echo "车辆限值（来自 config/vehicle_params.yaml）："
echo "  最大转角 ${MAX_STEER} rad   最大加速度 ${MAX_ACCEL} m/s²"
echo "  最大减速度 ${MAX_DECEL} m/s²   紧急制动 ${EMER_DECEL} m/s²"

# use_sim_time=true：消息头的时间戳必须走仿真钟，否则下游看门狗
# 按仿真时间判超时，两边对不上（SPEC §3.3）。
exec ros2 run ads_teleop keyboard_teleop --ros-args \
  -p "limits.max_steer_angle_rad:=${MAX_STEER}" \
  -p "limits.max_accel_mps2:=${MAX_ACCEL}" \
  -p "limits.max_decel_mps2:=${MAX_DECEL}" \
  -p "limits.emergency_decel_mps2:=${EMER_DECEL}" \
  -p use_sim_time:=true
