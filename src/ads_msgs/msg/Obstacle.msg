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

# 包围盒尺寸，单位 m：x = 长（沿朝向）、y = 宽、z = 高。
geometry_msgs/Vector3 size_m

# 速度，单位 m/s，表达在 header.frame_id 系下。
# 静止障碍物应为全零而非留空 —— 规划器不区分「速度为零」和「速度未知」，
# 未填的字段默认就是零，所以感知必须保证这个零是真的。
geometry_msgs/Vector3 velocity_mps

# 存在概率 [0, 1]。低于阈值的目标规划器可以选择忽略，
# 但**安全相关的碰撞检查不得使用这个字段做过滤**（SPEC §11）。
float32 existence_probability
