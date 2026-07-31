# =============================================================================
#  gazebo_sim.launch.py —— 一条命令拉起「Gazebo + 桥接 + TF + RViz」
#
#      ros2 launch gazebo_bridge gazebo_sim.launch.py
#      ros2 launch gazebo_bridge gazebo_sim.launch.py gui:=false rviz:=false
#
#  这个 launch 的边界：它只负责**仿真数据源**这一侧。
#  感知/规划/控制等算法节点不在这里起 —— S5 的 ads_bringup/stack.launch.py
#  才是全栈入口，它会 include 本文件（或换成 carla_bridge 的同名 launch）。
#  这个分层就是 SPEC §4.1「切换仿真源 = 换一个 launch 参数」的落地方式。
# =============================================================================

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory("gazebo_bridge")
    viz_share = get_package_share_directory("ads_visualization")

    bridge_config = str(Path(bridge_share) / "config" / "bridge_topics.yaml")
    urdf_file = Path(viz_share) / "urdf" / "ego_vehicle.urdf"
    rviz_config = str(Path(viz_share) / "rviz" / "default.rviz")

    # robot_state_publisher 要的是 URDF 的**内容字符串**，不是路径。
    # 在这里一次读进来，比让节点自己去读省事，也能在文件缺失时立刻报错。
    robot_description = urdf_file.read_text(encoding="utf-8")

    # -------------------------------------------------------------------------
    # 所有节点都必须 use_sim_time=true（SPEC §3.3）。
    #
    # 为什么这条不能有例外：仿真时间和真实时间是两条独立的时间轴，RTF 不等于 1
    # 时两者流速不同。只要有一个节点用了真实时间，它盖的时间戳就和别人对不上，
    # TF 会报 extrapolation 错误，多传感器同步会静默错配。
    # 而且这类问题在 RTF≈1.0 时几乎看不出来 —— 等场景变复杂 RTF 掉下去才爆发。
    # -------------------------------------------------------------------------
    use_sim_time = {"use_sim_time": True}

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="campus_minimal.sdf",
            description="世界文件名。靠 GZ_SIM_RESOURCE_PATH 解析，不用写绝对路径"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="是否开 Gazebo 图形界面。CI 里必须 false"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="是否开 RViz2"),

        # ---------------------------------------------------------------------
        # 1. Gazebo
        #
        # -r 表示加载后立刻开始仿真。不加的话世界是暂停的，表现为
        #    所有话题都存在但永远没数据 —— 很容易误判成桥接坏了。
        # -s 表示只起服务端（headless）。CI 和批量跑场景测试时用。
        #
        # 写成两个互斥的 ExecuteProcess 而不是用替换拼参数：
        # 拼出来的空字符串会作为一个空参数传给 gz，gz 会把它当成文件名去找。
        # ---------------------------------------------------------------------
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            condition=IfCondition(gui),
            output="screen",
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", world],
            condition=UnlessCondition(gui),
            output="screen",
        ),

        # ---------------------------------------------------------------------
        # 2. 话题桥接（3.4 的主体）
        #    翻译表在 config/bridge_topics.yaml，那个文件才是契约所在。
        # ---------------------------------------------------------------------
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gazebo_bridge",
            parameters=[{"config_file": bridge_config}, use_sim_time],
            output="screen",
        ),

        # ---------------------------------------------------------------------
        # 3. 点云 lidar_link → base_link
        #    见 src/pointcloud_to_base_link_node.cpp 顶部的说明。
        # ---------------------------------------------------------------------
        Node(
            package="gazebo_bridge",
            executable="pointcloud_to_base_link",
            name="pointcloud_to_base_link",
            parameters=[use_sim_time],
            output="screen",
        ),

        # ---------------------------------------------------------------------
        # 4. robot_state_publisher
        #    读 URDF + 订阅 /joint_states → 发布 base_link 以下的整棵 TF 子树
        #    （含 base_link→lidar_link 这个外参，上面那个节点要用）。
        # ---------------------------------------------------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}, use_sim_time],
            output="screen",
        ),

        # ---------------------------------------------------------------------
        # 5. map → odom（单位变换）
        #
        # 真实系统里这一段由**定位模块**发布，它表示的是「轮式里程计累积了
        # 多少漂移」。P4 之前没有定位，所以这里发单位变换 ——
        # 含义是「我们假装里程计不漂移」。这不是敷衍：TF 树必须连通，
        # 否则 RViz 以 map 为固定坐标系时什么都画不出来。
        #
        # ⚠️ P4 接上定位后必须删掉这个节点，否则会和定位模块抢着发同一段 TF，
        #    症状是车在 RViz 里疯狂跳动。
        # ---------------------------------------------------------------------
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_identity",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            parameters=[use_sim_time],
            output="screen",
        ),

        # ---------------------------------------------------------------------
        # 6. RViz2
        # ---------------------------------------------------------------------
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
