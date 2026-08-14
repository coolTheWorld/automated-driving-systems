#!/bin/bash
# S06 红绿灯最小闭环（决策四）：栈 → 虚拟灯 → goal → 判据。
set -u
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
set -u
LOG=/workspace/.l3c_s06

cleanup() {
  # 按**安装路径前缀**杀，不按节点名枚举 —— S06 五轮排查的教训：
  # 名单漏了 obstacle_truth，三个孤儿带着 block 锥桶世界观污染后续所有轮
  # （幻影 stop_at=22.2 与真规划交替，车被拉扯成微冲-停）。
  pkill -f "install/carla_bridge/li[b]" 2>/dev/null
  pkill -f "install/gazebo_bridge/li[b]" 2>/dev/null
  pkill -f "install/ads_[a-z]*/li[b]" 2>/dev/null
  pkill -f "robot_state_publishe[r] " 2>/dev/null
  pkill -f "ros2 launc[h]" 2>/dev/null
  pkill -f "traffic_light_nod[e]" 2>/dev/null
  sleep 2
}
trap cleanup EXIT
cleanup
# 残留守卫：清完还有活的算法节点就拒跑（僵尸除外 —— stat 首字母 Z）。
LEFT=$(ps -eo stat=,args= | awk '$1 !~ /^Z/' | grep -cE "install/(ads_|gazebo_bridge|carla_bridge)" || true)
if [ "${LEFT}" != "0" ]; then
  echo "VERDICT=RESIDUE(${LEFT} 个残留进程，拒绝起跑)"
  ps -eo stat=,args= | awk '$1 !~ /^Z/' | grep -E "install/(ads_|gazebo_bridge|carla_bridge)"
  exit 4
fi
server_ok() { timeout 5 bash -c "echo > /dev/tcp/127.0.0.1/2000" 2>/dev/null; }
server_ok || { echo "VERDICT=SERVER_DEAD"; exit 3; }

ros2 launch ads_bringup stack.launch.py sim:=carla gui:=false rviz:=false \
  > ${LOG}.launch.log 2>&1 &
for i in $(seq 1 40); do
  grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log 2>/dev/null && break
  server_ok || { echo "VERDICT=SERVER_DEAD"; exit 3; }
  sleep 5
done
grep -q "传感器已 spawn+中继：gnss" ${LOG}.launch.log || { echo "VERDICT=BRINGUP_TIMEOUT"; exit 2; }
sleep 10

timeout 300 python3 /workspace/.s06_judge.py > ${LOG}.txt 2>&1
while pgrep -f "s06_judge.p[y]" >/dev/null; do
  server_ok || { echo "VERDICT=SERVER_CRASHED"; exit 3; }
  sleep 10
done
if grep -q FAIL ${LOG}.txt; then echo "VERDICT=FAIL"; else
  grep -q PASS ${LOG}.txt && echo "VERDICT=PASS" || echo "VERDICT=INCOMPLETE"
fi
grep -E "判据|红窗|锚|Error|Trace" ${LOG}.txt | head -6
