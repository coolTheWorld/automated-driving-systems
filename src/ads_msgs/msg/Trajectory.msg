# =============================================================================
#  一条带速度的轨迹 —— ads_planning → ads_control（话题 /planning/trajectory）
#
#  ⚠️ **空的 points 与 status = STOPPING 是两回事，下游必须分开处理：**
#       points 为空  = 规划器没能给出轨迹（没收到路径 / 没有 TF / 挂了）→ 降级
#       STOPPING     = 规划器**明确说**前方绕不过去，请按这条轨迹停住 → 正常执行
#     混为一谈的后果是「车停了」与「车挂了」在日志里长得一模一样。
# =============================================================================

std_msgs/Header header

# 轨迹状态。
uint8 STATUS_OK = 0               # 正常跟踪。lateral_offset_m != 0 表示正在绕行。
uint8 STATUS_STOPPING = 1         # 前方几何上绕不过去，本轨迹在障碍物前停住。
uint8 STATUS_ROUTE_EXHAUSTED = 2  # 参考线剩余长度不足，通常是到终点了。
uint8 status

# 轨迹点，按 s_m 递增。第一个点在自车当前投影处附近。
TrajectoryPoint[] points

# 本周期选中的终点横向偏移，m，**左为正**。0 = 沿车道中心线。
# 给可视化和诊断用；规划器自己在内部做周期间的一致性。
float64 lateral_offset_m
