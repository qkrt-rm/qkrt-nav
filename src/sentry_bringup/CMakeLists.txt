cmake_minimum_required(VERSION 3.8)
project(sentry_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/laser_merger.py
  scripts/cmd_vel_gimbal_rotator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
