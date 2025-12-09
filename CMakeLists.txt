# Copyright 2021 eve autonomy inc. All Rights Reserved.
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

cmake_minimum_required(VERSION 3.5)
project(eve_cmd_gate)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(eve_cmd_gate SHARED
  src/eve_cmd_gate.cpp
)

# ヘッダファイル
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# launchファイル
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

rclcpp_components_register_node(eve_cmd_gate
  PLUGIN "eve_cmd_gate::EveCmdGate"
  EXECUTABLE eve_cmd_gate_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_eve_cmd_gate test/test_eve_cmd_gate.cpp)
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
  src
)
endif()

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   ament_add_ros_isolated_gmock(test_${PROJECT_NAME}
#     test/test_eve_cmd_gate.cpp
#   )
#   target_link_libraries(test_${PROJECT_NAME}
#     ${PROJECT_NAME}
#   )
#   target_include_directories(test_${PROJECT_NAME} PRIVATE
#   src
# )
# endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
# ament_add_launch_test(test/test_eve_cmd_gate_launch.py)
