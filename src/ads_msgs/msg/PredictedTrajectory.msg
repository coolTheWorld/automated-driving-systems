# =============================================================================
#  一个目标的一条预测轨迹假设 —— ads_prediction → 下游
#
#  同一个 obstacle_id 允许出现在多条轨迹里（路口分叉的多假设），
#  各假设的 probability 之和为 1。v1 概率均分（没有意图信息，
#  意图分类器是 SPEC 的升级路径）。
# =============================================================================

# 对应 /perception/obstacles 里的 Obstacle.id —— 消费方靠它把预测
# 与感知目标对上。ID 跳变时预测历史清空（与感知同一条约定）。
uint32 obstacle_id

# 用的哪个运动模型。常量值与 ads_prediction::ModelKind 一致。
# STATIC 的语义是"这里有东西、动向不明"，**不是**"可以忽略"——
# 它可能是真静止，也可能是速度状态不可信被物理闸拦下（见 prediction.md §5）。
uint8 MODEL_STATIC = 0
uint8 MODEL_CONSTANT_VELOCITY = 1
uint8 MODEL_LANE_FOLLOW = 2
uint8 model

# 本假设的概率，(0, 1]。同一 obstacle_id 的各假设之和为 1。
float64 probability

# 按 t_s 递增。车道在视界内走到头且无后继时**如实截断**（点数变少）——
# 消费方不要假设每条都有完整视界的点数。
PredictedTrajectoryPoint[] points
