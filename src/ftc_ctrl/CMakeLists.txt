cmake_minimum_required(VERSION 3.14)
project(ftc_ctrl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# --- Controller node ---
add_executable(ftc_ctrl_node
  src/ftc_ctrl_node.cpp
  src/ftc_ctrl.cpp
  src/filter.cpp
)
ament_target_dependencies(ftc_ctrl_node
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs quad_msgs Eigen3
)
target_include_directories(ftc_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

# --- State interface node (simulation glue) ---
add_executable(state_interface_node
  src/state_interface_node.cpp
)
ament_target_dependencies(state_interface_node
  rclcpp nav_msgs geometry_msgs quad_msgs Eigen3
)
target_include_directories(state_interface_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS ftc_ctrl_node state_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/takeoff.sh
  scripts/goto.sh
  scripts/fail_motor.sh
  scripts/set_yaw.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
