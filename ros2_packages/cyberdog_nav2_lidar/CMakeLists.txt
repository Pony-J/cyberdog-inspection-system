cmake_minimum_required(VERSION 3.5)
project(cyberdog_nav2_lidar)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_bridge.py
  scripts/fix_cyberdog_timestamps.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
