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
