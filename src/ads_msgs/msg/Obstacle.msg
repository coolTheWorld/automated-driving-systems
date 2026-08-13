# =============================================================================
#  单个障碍物 —— 感知模块 → 预测 / 规划
#
#  P0a 阶段不产生这个消息（感知在 P5），先把接口定下来的原因见 CLAUDE.md：
#  改 ads_msgs 会影响所有下游模块，越晚改代价越大。
#
#  用轴对齐/朝向包围盒（OBB）而不是点云簇来描述障碍物：规划做碰撞检查时
#  需要的是几何体，不是原始点。多边形轮廓更精确但计算量大得多，
#  园区低速场景下矩形足够 —— 这个取舍在 P5 如果发现不够再改。
# =============================================================================

# ---- 分类常量 ----
# 分开定义而不是用字符串：字符串比较慢，且拼写错误不会在编译期暴露。
uint8 CLASSIFICATION_UNKNOWN=0
uint8 CLASSIFICATION_PEDESTRIAN=1
uint8 CLASSIFICATION_BICYCLE=2
uint8 CLASSIFICATION_VEHICLE=3
uint8 CLASSIFICATION_STATIC=4

std_msgs/Header header

# 跟踪 ID。同一个物理目标在连续帧中应保持同一 ID ——
# 预测模块靠它把历史轨迹串起来，ID 跳变会让预测结果直接失效。
uint32 id

# 见上面的 CLASSIFICATION_* 常量
uint8 classification

# 障碍物包围盒中心的位姿，坐标系由 header.frame_id 指定。
# 姿态的 yaw 表示包围盒朝向；pitch/roll 在园区平坦场景下应接近 0。
geometry_msgs/Pose pose

# yaw 的语义标志（P6-S3 加，定死前置台账第 3 条的约定）：
#   true  = yaw 是消歧过的**车头朝向**（跟踪器用速度方向挑过 180° 候选）
#   false = yaw 只是 L-Shape 的**轴向**（[0, π)，车头车尾不分，180° 二义）
#
# ⚠️ 加这个字段之前，感知在消歧失败时把轴向**静默**填进 yaw，下游无法分辨
#    拿到的是哪种 —— 预测拿轴向当运动方向有 50% 概率给出**逆行**轨迹。
# 下游约定：
#   · 碰撞检查照用 yaw，**不看这个标志**（OBB 对 180° 翻转对称，轴向足够）；
#   · 预测的运动方向**永远取 velocity_mps 的方向**，yaw 只用来摆盒子 ——
#     所以这个标志对预测是防御性信息，对可视化是"画不画车头箭头"的开关。
# 填充方：perception_node 透传跟踪器的标志；obstacle_truth_node 恒填 true
#（真值朝向可信）。两个发布者都必须填 —— 少填的默认 false 是**保守方向**
#（把可信朝向当轴向用不会引入错误，反之会）。
bool heading_resolved

# 包围盒尺寸，单位 m：x = 长（沿朝向）、y = 宽、z = 高。
geometry_msgs/Vector3 size_m

# 速度，单位 m/s，表达在 header.frame_id 系下。
# 静止障碍物应为全零而非留空 —— 规划器不区分「速度为零」和「速度未知」，
# 未填的字段默认就是零，所以感知必须保证这个零是真的。
geometry_msgs/Vector3 velocity_mps

# 存在概率 [0, 1]。低于阈值的目标规划器可以选择忽略，
# 但**安全相关的碰撞检查不得使用这个字段做过滤**（SPEC §11）。
float32 existence_probability
