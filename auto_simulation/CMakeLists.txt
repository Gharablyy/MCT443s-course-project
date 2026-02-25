cmake_minimum_required(VERSION 3.8)
project(auto_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)

# Add your executables here
# Example:
# add_executable(node_name src/node_name.cpp)
# ament_target_dependencies(node_name rclcpp)

# Install directories
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  control
  launch
  config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
# 2. Install the python script to the lib folder
install(PROGRAMS
  control/combined_odom.py
  control/mission_01.py
  control/lane_switching.py
  control/visual_lane_switching.py
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
