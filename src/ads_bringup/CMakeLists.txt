cmake_minimum_required(VERSION 3.16)
project(ads_bringup)

find_package(ament_cmake REQUIRED)

# 本包不编译任何代码，只安装 launch 文件。
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效，
  # 详细理由见 gazebo_bridge/CMakeLists.txt 与仓库根 .clang-format。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()

  # L1 单元测试：仿真源注册表的解析逻辑。
  #
  # 用 ament_add_pytest_test 而不是裸调 pytest，理由和 ads_common 用
  # ament_add_gtest 一样：它生成 xunit XML，colcon test-result 才汇总得到。
  # 裸写的话测试跑了但汇总里是 0 tests —— CI 的「防假绿」断言反而会拦下它。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_source test/test_sim_source.py)
  # 生产 yaml 的键 ⊆ perception_node 声明的参数（静态对账；ROS 对未声明参数是静默忽略）
  ament_add_pytest_test(test_params_declared test/test_params_declared.py)
endif()

ament_package()
