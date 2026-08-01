#!/usr/bin/env python3
"""校验 map_node 的对外行为（P1 任务 4.1-4.3、4.6）.

七个判据，全部可量化：

    1. /map/lane_graph 收得到，且 marker 数 = 2 × 车道数
    2. 车道图与路径的 frame_id 都是 map
    3. 点一个目标点 → /route/path 有点，且首点贴自车、末点贴目标
    4. 路径**连续**：相邻点距不超过采样步长的 1.5 倍
    5. 路径**朝向自洽**：每点的四元数与「指向下一点」的方向夹角 < 15°
    6. 路径长度与独立算出的期望值一致（±2%）
    7. 目标点点在地图外 → 路径被清空，且节点仍然活着

为什么第 5 条值得单列
--------------------
它把**发布出来的朝向**和**发布出来的位置**对起来比，不碰任何库函数。
正编号车道逆 s 行驶，采样时要把朝向翻 180°、且要按 entry→exit 倒着走 ——
这两件事漏掉任何一件，路径的点还是那串点，四元数却指着反方向。
下游（P2 的 Stanley）拿这个朝向算横向误差，符号会整个反过来，车会越跑越偏。
而在 RViz 里，一条 Path 默认只画线不画箭头，**肉眼完全看不出来**。

为什么这个脚本不需要 Gazebo
--------------------------
map_node 只要 TF map→base_link 就能工作。用一个 static_transform_publisher
把自车钉在已知位姿上，比起一整套仿真：快 25 秒、不要 GPU、结果完全确定，
而且**能进 CI**。仿真那一侧该验的东西已经由 verify_ros_bridge.sh 覆盖了。

用法（容器内，map_node 与 static TF 已在跑）：
    python3 scripts/check_map_node.py
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

# ---------------------------------------------------------------------------
#  期望值。**由独立的穷举脚本算出**，不是把 map_node 跑一遍抄下来的。
#
#  起点 = campus_loop.sdf 里自车的 spawn 位姿，落在 (road 1, lane -1) 的 s=19 处。
#  终点 = (road 3, lane -1) 在 s=40 处的车道中心。
#  最短路 (1,-1) → (16,-1) → (2,-1) → (13,-1) → (3,-1)，长 571.460 m。
# ---------------------------------------------------------------------------
EGO_X_M, EGO_Y_M = 30.0, -51.75
GOAL_X_M, GOAL_Y_M = 1.75, 1.0
EXPECTED_ROUTE_LENGTH_M = 571.460169

# 期望的车道数（S3 已手数并测过：3 条常规路 × 双向 + 12 条连接路）。
EXPECTED_LANE_COUNT = 18

# map_node 的 path_sample_step_m 默认值。相邻点距的判据基于它。
PATH_STEP_M = 0.5

# ---- 判据 ----
# 路径首点到自车的距离。一个采样步长足矣：起点就在自车脚下。
START_TOLERANCE_M = 1.0
# 路径末点到目标点的距离。同上。
GOAL_TOLERANCE_M = 1.0
# 相邻点距的上限 = 步长 × 1.5。
# 留 1.5 倍余量是因为采样按**参考线**弧长走，弯道外侧的实际点距会大最多 14.6%。
# 定成 1.0 会在弯道上误报；定到 3 倍就抓不住「路径在路口处断开」了 ——
# 而断开正是拼接逻辑最容易出的错。
GAP_TOLERANCE_RATIO = 1.5
# 相邻点距的**下限**。判据只针对「两个点重合」这一件事，所以取 1 µm 而不是
# 一个「看起来合理」的值：真实的短段（比如目标点恰好落在路口后 1 mm）是合法的，
# 不该被判失败；而重合点会让下游按弧长参数化时除以零，且在 RViz 里完全看不出来。
MIN_GAP_M = 1e-6
# 每点朝向与「指向下一点」的夹角上限。
# 15° 足够松地容纳采样离散化（0.5 m 步长、R=8 m 弯 → 相邻点转角 3.6°），
# 又远远抓得住 180° 的方向反转。
HEADING_TOLERANCE_RAD = math.radians(15.0)
LENGTH_TOLERANCE_RATIO = 0.02

WAIT_TIMEOUT_S = 15.0


def yaw_from_quaternion(q) -> float:
    """Extract yaw from a geometry_msgs quaternion.

    只取绕 z 的分量：路径是平面的，roll/pitch 恒为 0。
    """
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(angle_rad: float) -> float:
    """Fold an angle into [-pi, pi]."""
    return math.remainder(angle_rad, 2.0 * math.pi)


class MapChecker(Node):
    """Subscribe to map_node's outputs and publish goals at it."""

    def __init__(self) -> None:
        super().__init__("check_map_node")
        # 车道图是 transient_local 发的。本节点**晚于** map_node 启动，
        # 所以「能收到」这件事本身就验证了 QoS 配对 —— 用 volatile 订阅
        # 也能收到，但那样就只是验证了「碰巧还没发完」。
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.lane_graph: MarkerArray | None = None
        self.paths: list[Path] = []

        self.create_subscription(MarkerArray, "/map/lane_graph", self._on_graph, latched)
        self.create_subscription(Path, "/route/path", self._on_path, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def _on_graph(self, msg: MarkerArray) -> None:
        self.lane_graph = msg

    def _on_path(self, msg: Path) -> None:
        self.paths.append(msg)

    def spin_for(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def send_goal(self, x_m: float, y_m: float) -> None:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x_m
        goal.pose.position.y = y_m
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


def main() -> int:  # noqa: C901  判据是平铺的，拆成函数反而更难对照
    rclpy.init()
    node = MapChecker()
    failures: list[str] = []

    def check(condition: bool, message: str) -> None:
        if condition:
            print(f"  ✓ {message}")
        else:
            print(f"  ✗ {message}")
            failures.append(message)

    try:
        # ---- 1. 车道图 ----
        print("[1/7] /map/lane_graph")
        got = node.wait_until(lambda: node.lane_graph is not None, WAIT_TIMEOUT_S)
        check(got, f"{WAIT_TIMEOUT_S:.0f} s 内收到车道图（transient_local 生效）")
        if not got:
            return 1

        markers = node.lane_graph.markers
        lines = [m for m in markers if m.ns == "lane_centerline"]
        arrows = [m for m in markers if m.ns == "lane_direction"]
        check(
            len(lines) == EXPECTED_LANE_COUNT,
            f"车道中心线 {len(lines)} 条（期望 {EXPECTED_LANE_COUNT}）",
        )
        check(
            len(arrows) == EXPECTED_LANE_COUNT,
            f"方向箭头 {len(arrows)} 个（期望 {EXPECTED_LANE_COUNT}）",
        )
        check(
            all(m.header.frame_id == "map" for m in markers),
            "所有 marker 的 frame_id 都是 map",
        )
        check(all(len(m.points) >= 2 for m in lines), "每条车道中心线至少 2 个点")

        # ---- 2. 路由 ----
        print()
        print("[2/7] 点一个目标点 → /route/path")
        node.paths.clear()
        node.send_goal(GOAL_X_M, GOAL_Y_M)
        got = node.wait_until(lambda: len(node.paths) > 0, WAIT_TIMEOUT_S)
        check(got, f"{WAIT_TIMEOUT_S:.0f} s 内收到路径")
        if not got:
            return 1
        path = node.paths[-1]
        check(len(path.poses) > 0, f"路径有 {len(path.poses)} 个点")
        check(path.header.frame_id == "map", f"路径 frame_id = {path.header.frame_id}")
        if not path.poses:
            return 1

        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]

        # ---- 3. 端点 ----
        print()
        print("[3/7] 端点位置")
        start_gap = math.dist(points[0], (EGO_X_M, EGO_Y_M))
        goal_gap = math.dist(points[-1], (GOAL_X_M, GOAL_Y_M))
        check(
            start_gap <= START_TOLERANCE_M,
            f"首点离自车 {start_gap:.3f} m（≤ {START_TOLERANCE_M} m）",
        )
        check(
            goal_gap <= GOAL_TOLERANCE_M,
            f"末点离目标 {goal_gap:.3f} m（≤ {GOAL_TOLERANCE_M} m）",
        )

        # ---- 4. 连续性 ----
        print()
        print("[4/7] 路径连续性")
        gaps = [math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
        check(len(gaps) > 0, f"路径有 {len(gaps)} 段")
        max_gap = max(gaps)
        limit = PATH_STEP_M * GAP_TOLERANCE_RATIO
        check(max_gap <= limit, f"最大相邻点距 {max_gap:.3f} m（≤ {limit:.3f} m）")
        # 最小点距要打印**足够多位**并带下标。只断言 > 0 的话，一个 1e-9 m 的
        # 重复点也算通过，而下游拿它做弧长参数化就会除以零。
        # 这是 S2 学到的：只看绿灯的判据，会一直绿到某天突然红。
        min_gap = min(gaps)
        min_index = gaps.index(min_gap)
        check(
            min_gap >= MIN_GAP_M,
            f"最小相邻点距 {min_gap:.6f} m（第 {min_index} 段，≥ {MIN_GAP_M} m，无重复点）",
        )

        # ---- 5. 朝向自洽 ----
        print()
        print("[5/7] 朝向与行进方向一致")
        worst_rad = 0.0
        worst_index = -1
        for i in range(len(path.poses) - 1):
            yaw = yaw_from_quaternion(path.poses[i].pose.orientation)
            travel = math.atan2(points[i + 1][1] - points[i][1], points[i + 1][0] - points[i][0])
            diff = abs(wrap(yaw - travel))
            if diff > worst_rad:
                worst_rad, worst_index = diff, i
        check(
            worst_rad <= HEADING_TOLERANCE_RAD,
            f"最大朝向偏差 {math.degrees(worst_rad):.2f}°（第 {worst_index} 点，"
            f"≤ {math.degrees(HEADING_TOLERANCE_RAD):.0f}°）",
        )

        # ---- 6. 长度 ----
        print()
        print("[6/7] 路径长度 vs 独立算出的期望值")
        polyline_m = sum(gaps)
        error = abs(polyline_m - EXPECTED_ROUTE_LENGTH_M) / EXPECTED_ROUTE_LENGTH_M
        check(
            error <= LENGTH_TOLERANCE_RATIO,
            f"折线长 {polyline_m:.3f} m vs 期望 {EXPECTED_ROUTE_LENGTH_M:.3f} m"
            f"（相对误差 {error * 100:.3f}%，≤ {LENGTH_TOLERANCE_RATIO * 100:.0f}%）",
        )

        # ---- 7. 目标点在地图外 ----
        print()
        print("[7/7] 目标点点到地图外")
        node.paths.clear()
        node.send_goal(500.0, 500.0)
        got = node.wait_until(lambda: len(node.paths) > 0, WAIT_TIMEOUT_S)
        check(got, "节点有响应（没有一声不吭）")
        if got:
            check(
                len(node.paths[-1].poses) == 0,
                f"路径被清空（{len(node.paths[-1].poses)} 个点）—— 旧路径不会留在屏幕上误导人",
            )
        # 节点必须还活着：一个越界目标点不该让它崩。
        node.paths.clear()
        node.send_goal(GOAL_X_M, GOAL_Y_M)
        recovered = node.wait_until(
            lambda: len(node.paths) > 0 and len(node.paths[-1].poses) > 0, WAIT_TIMEOUT_S
        )
        check(recovered, "越界目标之后仍能正常规划（节点没死）")

    finally:
        node.destroy_node()
        rclpy.shutdown()

    print()
    if failures:
        print(f"失败 {len(failures)} 项：")
        for item in failures:
            print(f"  - {item}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
