# Copyright 2026 孙帅
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# =============================================================================
#  机械校验：libads_planning.so 不许链接任何 ROS 库（SPEC §3.3）
#
#  用法（由 CMakeLists 的 add_test 调用）：
#      cmake -DTARGET_FILE=<.so 路径> -P check_no_ros_deps.cmake
#
#  为什么要有这个东西
#  ------------------
#  SPEC §3.3 要求 lib/ 与 ROS 解耦，理由是 L1 单元测试要保持毫秒级 ——
#  一旦链上 rclcpp，每个测试都要初始化 ROS 上下文，从 0.001 s 变成 0.5 s 起步。
#  测试一慢，人就不跑了，测试就白写了。
#
#  但这条约束的**落点很脆**：P3-S5 之后本包的 CMakeLists 里会同时有
#  libads_planning.so 和 planning_node 两个 target，后者理所当然地链接一堆 ROS。
#  （现在还只有前者 —— 正因如此，这个检查要**在第二个 target 出现之前**就位。）
#  哪天有人为了图方便把某个 ROS 依赖从 planning_node 挪到公共位置（比如写成
#  link_libraries() 而不是 target_link_libraries()），约束就破了，
#  **而且不会有任何报错** —— 编译照过，测试照绿，只是慢了下来。
#
#  ⚠️ 这个检查从本包**第一天**就在（P3-S1），不是事后补的。
#     `ads_control` 是反面教材：那里的 CMakeLists 从 P2-S1 起就写着「verify_control.sh
#     会 ldd 这个 .so」，而那个脚本**从来没被写出来过**，直到 P2-S5 逐个 grep 才发现 ——
#     整个 P2 期间这条约束只靠纪律维持。教训不是"要仔细"，而是
#     **凡是声称"有脚本守着"的地方，都得能被 grep 证伪**；做成 CTest 就自动满足
#     （`colcon test` 会跑它，CI 也跟着跑）。**一个要人记得去执行的检查，等于没有检查。**
#
#  ⚠️ 这个检查覆盖不到什么（故障注入实测出来的边界）
#  ------------------------------------------------
#  往 target_link_libraries 里加一个 `rclcpp::rclcpp` 而**不使用任何 ROS 符号**，
#  本检查**不会红**。原因是 Ubuntu 的链接器默认带 `--as-needed`：
#  用不到符号的库根本不会写进 DT_NEEDED，产物里确实一条 ROS 依赖都没有。
#
#  这不是漏洞，是**检查的对象选对了**：它验的是**构建产物**，不是 CMakeLists 的文本。
#  而约束本身要买的东西（L1 测试不必初始化 ROS 上下文、lib 能脱离 ROS 使用）
#  在那种情形下并没有被破坏 —— 一条没生效的链接项不改变任何运行期事实。
#
#  真正会被抓住的是「lib 里真的用了 ROS 符号」——
#  在 ads_control 上做过故障注入实测：往 lib 的 .cpp 里加一句 RCLCPP_INFO，
#  DT_NEEDED 立刻出现 librclcpp，检查随即失败。本包同一份实现，同样成立。
#
#  **别指望它守住 CMakeLists 的写法** —— 它守的是产物。
#
#  为什么用 readelf 而不是 ldd
#  --------------------------
#  ldd 会**递归解析**并尝试定位每一个库，于是它既受 LD_LIBRARY_PATH 影响
#  （install 空间没 source 时 libads_common.so 显示 "not found"），
#  又会把间接依赖一并列出来 —— 而我们要问的是「本 .so 自己的 DT_NEEDED 里
#  有没有 ROS」，那是个静态事实，不该依赖运行环境。
#  readelf -d 直接读 ELF 的动态段，不解析、不递归，问什么答什么。
# =============================================================================

if(NOT DEFINED TARGET_FILE)
  message(FATAL_ERROR "必须传 -DTARGET_FILE=<.so 路径>")
endif()

if(NOT EXISTS "${TARGET_FILE}")
  message(FATAL_ERROR "找不到 ${TARGET_FILE} —— 先构建再跑这个检查")
endif()

find_program(READELF_EXECUTABLE NAMES readelf)
if(NOT READELF_EXECUTABLE)
  # 找不到工具就**失败**，不是跳过。
  # "静默跳过"的检查是最坏的一种：它让人以为约束被守着，而实际没有。
  message(FATAL_ERROR "找不到 readelf（binutils）——无法验证 ROS 依赖，拒绝放行")
endif()

execute_process(
  COMMAND "${READELF_EXECUTABLE}" -d "${TARGET_FILE}"
  OUTPUT_VARIABLE dynamic_section
  RESULT_VARIABLE readelf_result
  ERROR_VARIABLE readelf_error)

if(NOT readelf_result EQUAL 0)
  message(FATAL_ERROR "readelf 失败：${readelf_error}")
endif()

# 逐行取出 DT_NEEDED。格式形如：
#   0x0000000000000001 (NEEDED)  Shared library: [libstdc++.so.6]
string(REGEX MATCHALL "Shared library: \\[[^]]+\\]" needed_entries "${dynamic_section}")

set(offenders "")
foreach(entry IN LISTS needed_entries)
  string(REGEX REPLACE "Shared library: \\[([^]]+)\\]" "\\1" soname "${entry}")
  # ROS 库的命名前缀。注意 ads_common 也以 lib 开头但它是本仓库的纯 C++ 库，
  # 不在此列 —— 它自己同样禁止链接 ROS（见 ads_common/package.xml）。
  if(soname MATCHES "^lib(rclcpp|rcl|rcutils|rmw|rosidl|ros|tf2|class_loader|ament)")
    list(APPEND offenders "${soname}")
  endif()
endforeach()

if(offenders)
  string(REPLACE ";" ", " offenders_text "${offenders}")
  message(FATAL_ERROR
    "${TARGET_FILE} 链接了 ROS 库：${offenders_text}\n"
    "SPEC §3.3 要求 lib/ 与 ROS 解耦 —— 算法要能脱离 ROS 做毫秒级单元测试。\n"
    "ROS 依赖只允许出现在 node/ 的 target 上（P3-S5 的 planning_node），"
    "且必须用 target_link_libraries 而不是 link_libraries。")
endif()

list(LENGTH needed_entries needed_count)
message(STATUS "✓ ${TARGET_FILE}：${needed_count} 条动态依赖，零 ROS")
