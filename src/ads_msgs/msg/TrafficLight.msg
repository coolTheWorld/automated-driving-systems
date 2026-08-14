# 红绿灯灯态 + 停止线（S06，P8 决策四「最小闭环」，用户拍板 2026-08-14）。
#
# 数据源：CARLA sidecar 的 traffic_light 通道（SPEC §4.1 —— 环境 A 没有
# 灯态激励源，S06 仅 L3-C）。消费者：planning_node 的行为层 ——
# RED ⟹ 停止线处 stop_at（复用 P7 的停车注入），GREEN ⟹ 释放。
#
# ⚠️ 最小闭环的刻意省略（决策四拍板范围，不是没想到）：
#    不做黄灯决策（YELLOW 保留值但行为层按 RED 处理 —— 保守方向）、
#    不做多灯路口（单灯，含 id 的版本等有第二个消费者再说）、
#    不做灯态预测/相位剩余时间。

std_msgs/Header header

# 灯态。YELLOW 行为层按 RED 处理（保守）；UNKNOWN 同样按 RED ——
# 「不知道灯色」在语义上不允许通过。
uint8 STATE_RED = 0
uint8 STATE_YELLOW = 1
uint8 STATE_GREEN = 2
uint8 STATE_UNKNOWN = 3
uint8 state

# 停止线在**全局路由参考线**上的弧长坐标，米（与行为层 stop_at 同一坐标）。
# 由 sidecar 按地图配置发布；行为层直接拿它做纵向约束，不再自己找停止线。
float64 stop_line_s_m
