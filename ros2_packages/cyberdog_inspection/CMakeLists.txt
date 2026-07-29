cmake_minimum_required(VERSION 3.10)
project(cyberdog_inspection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_search_module(GRPC REQUIRED grpc)
pkg_search_module(GRPCPP REQUIRED grpc++)

add_executable(cyberdog_inspection_node
  src/main.cpp
  src/local_inspection.cpp
  src/nav2_client.cpp
  src/communication/ros2/pose_provider.cpp
  src/algorithm/carrot_chaser.cpp
  src/mstc_wrapper/mstc.cpp
  grpc/cpp/src/client/client.cpp
  grpc/cpp/src/generated/mstc_service.pb.cc
  grpc/cpp/src/generated/mstc_service.grpc.pb.cc
)

add_executable(origin_tf_publisher
  src/origin_tf_publisher.cpp
)

add_executable(mock_nav2_server
  src/mock_nav2_server.cpp
)

target_include_directories(cyberdog_inspection_node PRIVATE
  include
  grpc/cpp/include
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(origin_tf_publisher PRIVATE
  include
)

target_include_directories(mock_nav2_server PRIVATE
  include
)

target_link_libraries(cyberdog_inspection_node
  Threads::Threads
  yaml-cpp
  ${OpenCV_LIBS}
  OpenSSL::SSL
  OpenSSL::Crypto
  protobuf::libprotobuf
  grpc++
  grpc
)

ament_target_dependencies(cyberdog_inspection_node
  ament_index_cpp
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(origin_tf_publisher
  geometry_msgs
  rclcpp
  tf2_ros
)

ament_target_dependencies(mock_nav2_server
  nav2_msgs
  rclcpp
  rclcpp_action
)

install(TARGETS
  cyberdog_inspection_node
  origin_tf_publisher
  mock_nav2_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_mstc_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config grpc
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
