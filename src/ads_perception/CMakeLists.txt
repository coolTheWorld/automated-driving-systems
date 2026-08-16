cmake_minimum_required(VERSION 3.16)
project(ads_perception)

# SPEC §7：C++17，全量警告。理由见 ads_common/CMakeLists.txt。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ads_common REQUIRED)

# -----------------------------------------------------------------------------
# 库：纯 C++17，**不链接任何 ROS 库**
# -----------------------------------------------------------------------------
# SPEC §3.3「算法与 ROS 强制解耦」。机械校验在下面的 BUILD_TESTING 段
# （no_ros_dependencies），跟着 colcon test 走 —— **从本包第一天就在**。
add_library(${PROJECT_NAME} SHARED
  src/ground_segmentation.cpp
  src/euclidean_cluster.cpp
  src/lshape_fit.cpp
  src/detection_gates.cpp
  src/size_classifier.cpp
  src/hungarian.cpp
  src/tracker.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# PUBLIC Eigen：公开头文件的签名里出现 Eigen::Vector3d，类型进了对外契约。
# ads_common 只在 .cpp 里用（RequireFinite），所以是 PRIVATE。
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
  PRIVATE ads_common::ads_common)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3 ads_common)

# -----------------------------------------------------------------------------
# 节点：ROS 包装层（P5-S5）
# -----------------------------------------------------------------------------
# 这一层**只做翻译**：ROS 消息 ↔ lib 的调用。四阶段算法一行都不在这里，
# 所以它没有自己的单元测试也不可怕 —— 该测的都在 lib 的 L1 用例里。
find_package(rclcpp REQUIRED)
find_package(ads_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(perception_node node/perception_node.cpp)
target_link_libraries(perception_node ${PROJECT_NAME})
ament_target_dependencies(perception_node
  rclcpp ads_msgs sensor_msgs geometry_msgs diagnostic_msgs tf2 tf2_ros)

install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME})

# 感知的调参装进本包 share（理由同 ads_control：ROS 包安装后必须自包含）。
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/perception_params.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# -----------------------------------------------------------------------------
# L1 单元测试
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效
  # —— 那个函数在调用时就把 linter 列表定死了，设晚了不生效**且不报错**。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # L1：地面分割。
  #
  # ⚠️ **判据必须跑在带坡度的合成地面上。** campus 的地面是理想平面，
  #    在那上面 RANSAC 与「z < 阈值」结果完全相同 —— 判据没有区分力。
  #    见 ground_segmentation.hpp 的文件头与 plan.md P5-1 决策二。
  ament_add_gtest(test_ground_segmentation test/test_ground_segmentation.cpp)
  target_link_libraries(test_ground_segmentation ${PROJECT_NAME} ads_common::ads_common)

  # L1：欧式聚类。
  #
  # ⚠️ 点云按**真实雷达角分辨率**采样，不是均匀网格 —— 均匀采样下聚类
  #    怎么写都对，而真实点云里同一目标的相邻两线相隔 0.4–0.6 m
  #    （比水平点间距大 5.6 倍），**那才是 tolerance 被顶住的地方**。
  ament_add_gtest(test_euclidean_cluster test/test_euclidean_cluster.cpp)
  target_link_libraries(test_euclidean_cluster ${PROJECT_NAME} ads_common::ads_common)

  # L1：检测准入门（剃刀条 / 浮空碎片，两道都是双条件门）。每个阈值两侧各一个实测形态。
  ament_add_gtest(test_detection_gates test/test_detection_gates.cpp)
  target_link_libraries(test_detection_gates ${PROJECT_NAME} ads_common::ads_common)

  # L1：L-Shape 朝向拟合 + 尺寸分类。
  #
  # ⚠️ 核心判据是**180° 二义性**：一帧点云里没有信息能区分车头朝哪，
  #    所以拟合只给「轴向」（值域 [0, π)）。用例必须证明它**没有**假装
  #    解决这件事 —— 猜错的概率是 50%，而错了之后 P6 会预测出一条逆行轨迹。
  ament_add_gtest(test_lshape_fit test/test_lshape_fit.cpp)
  target_link_libraries(test_lshape_fit ${PROJECT_NAME} ads_common::ads_common)

  # L1：匈牙利算法。
  #
  # ⚠️ 这个算法容易写错，而写错的表现是「大部分时候对、偶尔配错一对」。
  #    所以判据必须与**穷举**对账（小规模全排列）——
  #    与 ads_map 的 Dijkstra 用穷举脚本对账是同一条规矩。
  ament_add_gtest(test_hungarian test/test_hungarian.cpp)
  target_link_libraries(test_hungarian ${PROJECT_NAME} ads_common::ads_common)

  # L1：多目标跟踪。
  #
  # ⚠️ 生命周期参数直接来自 S1 的实测（目标闪烁，命中率 33–74%），
  #    所以判据必须在**带丢帧的**序列上跑 —— 每帧都命中的理想序列上，
  #    「累计命中」与「连续命中」给出一样的结果，判据就没区分力了。
  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker ${PROJECT_NAME} ads_common::ads_common)

  # L3-G：**不需要 GPU** 的感知闭环（合成点云 + perception_node），能进 CI。
  #
  # 验的是**接线**（话题/QoS/参数/base_link→map 变换/四阶段串没串起来），
  # **不是感知质量** —— 合成点云没有遮挡、没有噪声起伏、只有一个目标，
  # CP-P5-B 那些真正难的东西（可见面伪影、轴向翻转、重复航迹）都不会发生。
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_closed_loop_perception.py
    TIMEOUT 120
    ENV
      # ⚠️ 独占一个 ROS_DOMAIN_ID。colcon 按**包**并行跑测试，默认 domain 0 下
      #    本测试会与 ads_control / ads_localization 的闭环共享话题与 TF。
      #    ads_localization 加进来那次就是这么同时变红的（见 CLAUDE.md）。
      #    分配表：ads_control 41/42/46/48/49，ads_localization 43，
      #    ads_perception 44，gazebo_bridge 45，ads_prediction 47。
      ROS_DOMAIN_ID=44)

  # 机械校验：libads_perception.so 的 DT_NEEDED 里不许有 ROS 库。
  add_test(NAME no_ros_dependencies
    COMMAND ${CMAKE_COMMAND}
      -DTARGET_FILE=$<TARGET_FILE:${PROJECT_NAME}>
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_no_ros_deps.cmake)
endif()

ament_package()
