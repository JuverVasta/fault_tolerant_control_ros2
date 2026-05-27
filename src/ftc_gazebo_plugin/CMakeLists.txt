cmake_minimum_required(VERSION 3.14)
project(ftc_gazebo_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quad_msgs REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(ftc_motor_model SHARED
  src/ftc_motor_model.cpp
)
ament_target_dependencies(ftc_motor_model
  gazebo_dev gazebo_ros rclcpp nav_msgs quad_msgs
)
target_link_libraries(ftc_motor_model ${GAZEBO_LIBRARIES})

install(TARGETS ftc_motor_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_export_libraries(ftc_motor_model)
ament_package()
