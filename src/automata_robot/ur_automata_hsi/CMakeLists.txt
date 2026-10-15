cmake_minimum_required(VERSION 3.8)
project(ur_automata_hsi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# planning_scene_client.hpp (header only) from ur_automata_scan, which exports
# no CMake targets: found in its install prefix.
find_package(ur_automata_scan REQUIRED)
set(UR_SCAN_PREFIX "${ur_automata_scan_DIR}/../../..")
find_path(UR_SCAN_INCLUDE_DIR ur_automata_scan/planning_scene_client.hpp
  HINTS "${UR_SCAN_PREFIX}/include" REQUIRED)

add_executable(hsi_node src/hsi_node.cpp)
target_include_directories(hsi_node PRIVATE ${UR_SCAN_INCLUDE_DIR})
target_link_libraries(hsi_node nlohmann_json::nlohmann_json)
ament_target_dependencies(hsi_node
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  Eigen3
  std_msgs
)

install(TARGETS hsi_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
