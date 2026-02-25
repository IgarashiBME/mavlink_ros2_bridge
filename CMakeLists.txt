cmake_minimum_required(VERSION 3.8)
project(mavlink_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bme_common_msgs REQUIRED)

# Source files
add_executable(mavlink_bridge_node
  src/mavlink_bridge_node.cpp
  src/udp_socket.cpp
)

# Include paths:
#   include/            — project headers (mavlink_ros2_bridge/*.hpp)
#   include/c_library_v2 — MAVLink v2 C library (common/mavlink.h etc.)
target_include_directories(mavlink_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(mavlink_bridge_node SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/c_library_v2>
)

target_compile_features(mavlink_bridge_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(mavlink_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  bme_common_msgs
)

install(TARGETS mavlink_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
