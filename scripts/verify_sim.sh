#!/usr/bin/env bash
# =============================================================================
#  P0a 任务 2.5 / 2.6 —— Gazebo 仿真基线验证
#
#  验两件事：
#     2.5  车能不能被指令驱动（发 Twist 后位姿真的变了）
#     2.6  实时率 RTF 是否 ≥ 0.8
#
#  为什么不靠肉眼读 GUI 状态栏：RTF 会随场景复杂度、后台负载波动，
#  肉眼扫一眼记住的那个数不能用来做前后对比。S3 加了激光雷达之后要再测一次，
#  那时需要的是「和这次同样方法测出来的数」，否则根本判断不出传感器拖了多少帧率。
#
#  本脚本用 headless 服务端（gz sim -s）测，测的是**纯物理**的基线。
#  带 GUI 的实际体验会更低一些，那部分靠肉眼确认。
#
#  在**容器内**执行：
#      docker compose exec dev /workspace/scripts/verify_sim.sh
# =============================================================================
set -uo pipefail

# Gazebo 是通过 ros-jazzy-gz-sim-vendor 装的，gz 命令在 ROS 前缀下
# （/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz），不 source 就不在 PATH 上。
# 交互式 shell 靠 .bashrc 自动 source，但脚本和 docker exec 不走那条路径。
if ! command -v gz >/dev/null 2>&1; then
  # ROS 的 setup.bash 会读 AMENT_TRACE_SETUP_FILES 等未定义变量，
  # 在 set -u 下会直接报错退出，所以 source 期间临时关掉。
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if ! command -v gz >/dev/null 2>&1; then
  echo "✗ 找不到 gz 命令 —— 检查镜像是否装了 ros-jazzy-ros-gz" >&2
  exit 1
fi

WORLD_FILE="${WORLD_FILE:-/workspace/worlds/campus_minimal.sdf}"
WORLD_NAME="${WORLD_NAME:-campus_minimal}"
MODEL_NAME="${MODEL_NAME:-ego_vehicle}"

RTF_MIN=0.8          # 计划 2.6 的验收线
DRIVE_SECONDS=6      # 施加指令后观察的时长
DRIVE_SPEED=3.0      # m/s，约 11 km/h，在 ODD 巡航区间内
DRIVE_YAWRATE=0.0    # 先直行，转向单独测

PASS=0
fail() { echo "  ✗ $1"; PASS=1; }
ok()   { echo "  ✓ $1"; }

SERVER_PID=""
cleanup() {
  if [[ -n "${SERVER_PID}" ]] && kill -0 "${SERVER_PID}" 2>/dev/null; then
    kill "${SERVER_PID}" 2>/dev/null
    wait "${SERVER_PID}" 2>/dev/null
  fi
}
trap cleanup EXIT

# 取 Odometry 消息里的位置分量。gz topic -e 输出的是文本格式 protobuf，
# pose.position 是嵌套结构，直接 grep 'x:' 会撞上 orientation 里的 x。
odom_field() {
  local field="$1"
  gz topic -e -t "/model/${MODEL_NAME}/odometry" -n 1 2>/dev/null \
    | awk -v f="${field}" '
        /^  position \{/ { inpos=1; next }
        inpos && $1 == f":" { print $2; exit }
        inpos && /^  \}/    { inpos=0 }
      '
}

echo "=============================================="
echo " P0a 2.5 / 2.6  Gazebo 仿真基线验证"
echo "=============================================="
echo " 世界：${WORLD_FILE}"
echo " 车辆：${MODEL_NAME}"

echo
echo "[1/6] 启动 Gazebo 服务端（headless）"
gz sim -s -r "${WORLD_FILE}" > /tmp/gz_server.log 2>&1 &
SERVER_PID=$!

for _ in $(seq 1 40); do
  if gz topic -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/stats"; then break; fi
  sleep 0.5
done

if ! gz topic -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/stats"; then
  fail "20 秒内没等到 /world/${WORLD_NAME}/stats —— 世界没能加载"
  echo "     服务端日志："
  sed 's/^/       /' /tmp/gz_server.log | tail -20
  exit 1
fi
ok "世界已加载，服务端 PID ${SERVER_PID}"

echo
echo "[2/6] 车辆模型是否存在"
if gz topic -l 2>/dev/null | grep -q "/model/${MODEL_NAME}/"; then
  ok "/model/${MODEL_NAME}/* 话题存在"
else
  fail "找不到 /model/${MODEL_NAME}/* —— 世界里的 <include> 没解析到模型"
  echo "     多半是 GZ_SIM_RESOURCE_PATH 没指向 /workspace/models"
  echo "     现值：GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-未设置}"
  exit 1
fi

echo
echo "[3/6] Ackermann 控制接口"
if gz topic -l 2>/dev/null | grep -q "/model/${MODEL_NAME}/cmd_vel"; then
  ok "cmd_vel 指令话题已就绪"
else
  fail "没有 /model/${MODEL_NAME}/cmd_vel —— AckermannSteering 插件没加载"
  echo "     看服务端日志里有没有 'Unable to load' 之类的字样："
  grep -iE "unable|failed|error" /tmp/gz_server.log | sed 's/^/       /' | head -10
  exit 1
fi

echo
echo "[4/6] 施加指令，看车动不动（任务 2.5）"
X0="$(odom_field x)"; Y0="$(odom_field y)"
if [[ -z "${X0}" ]]; then
  fail "读不到里程计 —— /model/${MODEL_NAME}/odometry 没有数据"
  exit 1
fi
printf "     起点  x=%.3f  y=%.3f\n" "${X0}" "${Y0}"

gz topic -t "/model/${MODEL_NAME}/cmd_vel" -m gz.msgs.Twist \
         -p "linear: {x: ${DRIVE_SPEED}}, angular: {z: ${DRIVE_YAWRATE}}" >/dev/null 2>&1
echo "     已下发 linear.x=${DRIVE_SPEED} m/s，观察 ${DRIVE_SECONDS} s ..."
sleep "${DRIVE_SECONDS}"

X1="$(odom_field x)"; Y1="$(odom_field y)"
printf "     终点  x=%.3f  y=%.3f\n" "${X1}" "${Y1}"

# 松开油门，免得后面测 RTF 时车一直撞墙
gz topic -t "/model/${MODEL_NAME}/cmd_vel" -m gz.msgs.Twist \
         -p "linear: {x: 0.0}, angular: {z: 0.0}" >/dev/null 2>&1

DX="$(awk -v a="${X1}" -v b="${X0}" 'BEGIN{printf "%.3f", a-b}')"
# 理论位移 = 速度 × 时长，取 50% 作为下限：起步有加速过程，不该按满速算
MIN_DX="$(awk -v v="${DRIVE_SPEED}" -v t="${DRIVE_SECONDS}" 'BEGIN{printf "%.3f", v*t*0.5}')"
if awk -v d="${DX}" -v m="${MIN_DX}" 'BEGIN{exit !(d >= m)}'; then
  ok "沿 +x 前进 ${DX} m（下限 ${MIN_DX} m）—— 车辆可被指令驱动"
else
  fail "只前进了 ${DX} m，低于下限 ${MIN_DX} m"
  echo "     常见原因：车轮摩擦系数过低在打滑，或驱动关节名字与插件配置对不上"
fi

echo
echo "[5/6] 施加转向指令，看车拐不拐（任务 2.5）"
# 直行测的是驱动关节，转向测的是转向关节 —— 两条链路互相独立，
# 只测直行的话，转向关节名字写错了也照样「通过」。
XT0="$(odom_field x)"; YT0="$(odom_field y)"
gz topic -t "/model/${MODEL_NAME}/cmd_vel" -m gz.msgs.Twist \
         -p "linear: {x: ${DRIVE_SPEED}}, angular: {z: 0.3}" >/dev/null 2>&1
echo "     已下发 linear.x=${DRIVE_SPEED} m/s, angular.z=0.3 rad/s，观察 ${DRIVE_SECONDS} s ..."
sleep "${DRIVE_SECONDS}"
XT1="$(odom_field x)"; YT1="$(odom_field y)"
gz topic -t "/model/${MODEL_NAME}/cmd_vel" -m gz.msgs.Twist \
         -p "linear: {x: 0.0}, angular: {z: 0.0}" >/dev/null 2>&1

DY="$(awk -v a="${YT1}" -v b="${YT0}" 'BEGIN{printf "%.3f", a-b}')"
printf "     侧向位移 Δy = %s m\n" "${DY}"
# 左转（angular.z 为正）应产生正的 Δy。阈值 1 m：6 s 内以 0.3 rad/s 转过约 1 rad，
# 半径约 10 m，侧向位移远大于 1 m，取这么松是为了不被起步阶段的加速过程影响。
if awk -v d="${DY}" 'BEGIN{exit !(d >= 1.0)}'; then
  ok "车辆向左偏移 ${DY} m —— 转向链路可用"
else
  fail "侧向位移仅 ${DY} m，转向未生效"
  echo "     检查 front_*_steer_joint 的名字是否与 AckermannSteering 插件里写的一致"
fi

echo
echo "[6/6] 实时率 RTF（任务 2.6，验收线 ${RTF_MIN}）"
RTF="$(gz topic -e -t "/world/${WORLD_NAME}/stats" -n 12 2>/dev/null \
        | awk '/real_time_factor:/ {v=$2; n++; s+=v} END{if(n>0) printf "%.3f", s/n}')"

if [[ -z "${RTF}" ]]; then
  fail "读不到 real_time_factor"
else
  echo "     12 帧统计均值：RTF = ${RTF}"
  if awk -v r="${RTF}" -v m="${RTF_MIN}" 'BEGIN{exit !(r >= m)}'; then
    ok "RTF ${RTF} ≥ ${RTF_MIN}"
  else
    fail "RTF ${RTF} < ${RTF_MIN}"
    echo "     不要带着这个数往下做 S3 —— 加了激光雷达只会更低。"
    echo "     先简化世界几何（减面数、确认 <shadows>false</shadows>），或放大物理步长。"
  fi
fi

echo
echo "=============================================="
if [[ ${PASS} -eq 0 ]]; then
  echo " 结果：通过 ✓   S2 基线成立，可以进入 S3（接 ROS 2 桥接）"
else
  echo " 结果：失败 ✗   按上面的提示排查，不要跳过。"
fi
echo "=============================================="
exit ${PASS}
