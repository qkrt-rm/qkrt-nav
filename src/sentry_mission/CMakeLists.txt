cmake_minimum_required(VERSION 3.8)
project(sentry_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_executable(mission_executor
  src/mission_executor.cpp
  src/health_above.cpp
  src/health_below.cpp
  src/navigate_center.cpp
  src/navigate_home.cpp
)

ament_target_dependencies(mission_executor
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
)

install(TARGETS mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()