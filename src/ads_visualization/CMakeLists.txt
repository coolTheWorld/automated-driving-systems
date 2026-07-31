cmake_minimum_required(VERSION 3.16)
project(ads_visualization)

find_package(ament_cmake REQUIRED)

# 纯资源包：不编译任何代码，只把 URDF 和 RViz 配置装到 share 下，
# 让别的包能用 get_package_share_directory() 找到它们。
install(DIRECTORY urdf rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 用 clang-format 取代 uncrustify。必须设在 find_test_dependencies 之前才生效，
  # 详细理由见 gazebo_bridge/CMakeLists.txt 与仓库根 .clang-format。
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  # 指向仓库根那份 .clang-format。只能用这个变量，不要手动调 ament_clang_format()
  # ——lint hook 已经自动注册了同名测试，再调一次会撞名。
  # 详细理由见 gazebo_bridge/CMakeLists.txt。
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
