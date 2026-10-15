cmake_minimum_required(VERSION 3.8)
project(ur_automata_scan_tsp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# OR-Tools from /opt/ortools (docker/Dockerfile); without it the package builds
# nothing. Searched after the ROS packages: its prefix holds a Boost config that
# MoveIt would otherwise pick instead of the system Boost.
list(APPEND CMAKE_PREFIX_PATH /opt/ortools)
find_package(ortools CONFIG QUIET)
if(NOT ortools_FOUND)
  message(WARNING "OR-Tools not found in /opt/ortools: ur_automata_scan_tsp is not built.")
  ament_package()
  return()
endif()

# ur_automata_scan exports no CMake targets: find its files in its install prefix.
find_package(ur_automata_scan REQUIRED)
set(UR_SCAN_PREFIX "${ur_automata_scan_DIR}/../../..")
find_path(UR_SCAN_INCLUDE_DIR ur_automata_scan/scan_recording.hpp
  HINTS "${UR_SCAN_PREFIX}/include" REQUIRED)
find_library(UR_SCAN_WAYPOINTS_LIB sphere_waypoint_generator HINTS "${UR_SCAN_PREFIX}/lib" REQUIRED)
find_library(UR_SCAN_PLANNER_LIB   scan_sequence_planner     HINTS "${UR_SCAN_PREFIX}/lib" REQUIRED)
find_library(UR_SCAN_RECORDING_LIB scan_recording            HINTS "${UR_SCAN_PREFIX}/lib" REQUIRED)

add_library(tsp_planning src/tsp_planning.cpp)
target_include_directories(tsp_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${UR_SCAN_INCLUDE_DIR})
target_link_libraries(tsp_planning ${UR_SCAN_PLANNER_LIB})

# OR-Tools headers (abseil, protobuf) stay inside route_solver.cpp.
add_library(route_solver src/route_solver.cpp)
target_include_directories(route_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(route_solver PRIVATE ortools::ortools)

add_library(plan_file src/plan_file.cpp)
target_include_directories(plan_file PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${UR_SCAN_INCLUDE_DIR})
target_link_libraries(plan_file ${UR_SCAN_RECORDING_LIB} yaml-cpp::yaml-cpp)

add_executable(scan_tsp_node
  src/scan_tsp_node.cpp
  src/scan_common.cpp
)
target_include_directories(scan_tsp_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${UR_SCAN_INCLUDE_DIR})
target_link_libraries(scan_tsp_node
  tsp_planning
  route_solver
  plan_file
  ${UR_SCAN_WAYPOINTS_LIB}
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(scan_tsp_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  Eigen3
  std_msgs
  std_srvs
  geometric_shapes
)
# Keep /opt/ortools/lib in the rpath of the installed binary.
set_target_properties(scan_tsp_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS scan_tsp_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tsp_planning test/test_tsp_planning.cpp)
  target_link_libraries(test_tsp_planning tsp_planning route_solver)
  ament_add_gtest(test_plan_file test/test_plan_file.cpp)
  target_link_libraries(test_plan_file plan_file)
endif()

ament_package()
