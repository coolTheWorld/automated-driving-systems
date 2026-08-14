#!/bin/bash
# L3-C 单场景一体化轮次（云端自足）：起栈 → goal → 记录 → 判定。
# 铁律：所有等待带超时；服务器死亡即中止；trap 清理。
# 用法：l3c_round.sh <标签> <gx> <gy> [额外 launch 参数...]
set -u
TAG="$1"; GX="$2"; GY="$3"; shift 3
EXTRA="$*"
LOG=/workspace/.l3c_${TAG}
# ⚠️ 陷阱表第一条：set -u 下 source ROS 会炸 AMENT_TRACE_SETUP_FILES ——
#    source 期间临时 set +u（本脚本首版就栽在这，死得连 VERDICT 都没印）。
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
set -u

cleanup() {
  # 按**安装路径前缀**杀，不按节点名枚举 —— S06 排查五轮的教训：名单漏了
  # obstacle_truth，三个孤儿带着 block 锥桶世界观（publish_as_perception）
  # 污染后续所有轮：幻影 stop_at 与真规划**交替发布**，控制被两股轨迹拉扯成
  # 微冲-停；S01 退化轮的车正是停在幻影锥桶前。与陷阱表「两套仿真并存」同族：
  # 这是 ROS 层的「两套世界观并存」，症状同样是所有测量作废。
  PATTERNS=(
    "install/carla_bridge/li[b]" "install/gazebo_bridge/li[b]"
    "install/ads_[a-z]*/li[b]" "robot_state_publishe[r] "
    "ros2 launc[h]" "traffic_light_nod[e]")
  for PAT in "${PATTERNS[@]}"; do pkill -f "$PAT" 2>/dev/null; done
  # TERM 后必须给「等待 + 升级 -9」：sidecar 的 SIGTERM 钩子走 finally 还原
  # 异步模式，服务器已被 -9 时那个 RPC 永远不回来 —— 实测留下孤儿 sidecar，
  # 被下一轮的残留守卫拦下（守卫立功，但轮次白等一场）。
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
# 残留守卫：清完还有活的算法/桥节点就拒跑（僵尸除外 —— stat 首字母 Z，
# 容器 PID 1 不 reap，defunct 无 DDS 存在、无害）。守卫必须有 —— 正是它
# 这一类检查在 P5 拦下过两次会被污染的测量（CLAUDE.md 陷阱表）。
LEFT=$(ps -eo stat=,args= | awk '$1 !~ /^Z/' | grep -cE "install/(ads_|gazebo_bridge|carla_bridge)" || true)
if [ "${LEFT}" != "0" ]; then
  echo "VERDICT=RESIDUE(${LEFT} 个残留进程，拒绝起跑)"
  ps -eo stat=,args= | awk '$1 !~ /^Z/' | grep -E "install/(ads_|gazebo_bridge|carla_bridge)"
  exit 4
fi

server_ok() { timeout 5 bash -c "echo > /dev/tcp/127.0.0.1/2000" 2>/dev/null; }
server_ok || { echo "VERDICT=SERVER_DEAD（起跑前）"; exit 3; }

ros2 launch ads_bringup stack.launch.py sim:=carla gui:=false rviz:=false $EXTRA \
  > ${LOG}.launch.log 2>&1 &
for i in $(seq 1 40); do
  grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log 2>/dev/null && break
  server_ok || { echo "VERDICT=SERVER_DEAD（bringup 中）"; exit 3; }
  sleep 5
done
grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log || { echo "VERDICT=BRINGUP_TIMEOUT"; exit 2; }
sleep 12

# 位置遥测：崩溃时要知道车死在哪（S6 第 3 次 Segfault 后加）
python3 - > ${LOG}.pos 2>&1 <<'TEOF' &
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ads_msgs.msg import VehicleCmd
rclpy.init()
n = Node('pos_tap')
state = {'steer': float('nan'), 'accel': float('nan')}
def on_cmd(m):
    state['steer'] = m.steer_angle_rad
    state['accel'] = m.accel_mps2
n.create_subscription(VehicleCmd, '/vehicle_cmd', on_cmd, 10)
def on_gt(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    tw = m.twist.twist.linear
    print(f"{p.x:.1f},{p.y:.1f},{math.degrees(yaw):.0f},{math.hypot(tw.x,tw.y):.2f},"
          f"{state['steer']:.3f},{state['accel']:.2f}", flush=True)
n.create_subscription(Odometry, '/ego_pose_gt', on_gt, 10)
rclpy.spin(n)
TEOF
TAP=$!

# S04 场景（TAG 含 avoid/block）换判据仪器（家规：判据活在 record_* 里）
if [[ "$TAG" == *avoid* || "$TAG" == *block* ]]; then
  SCEN=${TAG##*_}
  timeout 550 python3 /workspace/scripts/record_obstacle_run.py \
    --scenario "$SCEN" --out ${LOG}.csv > ${LOG}.txt 2>&1 &
else
  timeout 550 python3 /workspace/scripts/record_control_run.py \
    --goal "$GX" "$GY" --out ${LOG}.csv > ${LOG}.txt 2>&1 &
fi
REC=$!
while kill -0 $REC 2>/dev/null; do
  server_ok || { echo "VERDICT=SERVER_CRASHED（记录中）"; echo "末段遥测 x,y,yaw°,v,steer,accel："; grep -v "^0.0,-0.0" ${LOG}.pos | tail -12; kill $TAP 2>/dev/null; exit 3; }
  sleep 10
done
kill $TAP 2>/dev/null
if grep -qE "全部通过|^0 项未通过" ${LOG}.txt; then echo "VERDICT=PASS"
elif grep -qE "有判据未通过|[1-9][0-9]* 项未通过" ${LOG}.txt; then echo "VERDICT=FAIL"
else echo "VERDICT=INCOMPLETE（记录器没跑完 —— 超时或异常）"; fi
grep -E "PASS|FAIL|m/s|误差" ${LOG}.txt | head -12
