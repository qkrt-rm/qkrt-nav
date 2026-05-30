cmake_minimum_required(VERSION 3.8)
project(sentry_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/battery_mission_controller.py
  scripts/battery_health_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()