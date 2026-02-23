cmake_minimum_required(VERSION 3.8)
project(robot_arm_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# Eigen3 for Kinematics
find_package(Eigen3 REQUIRED)

# include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(kinematics_solver src/kinematics_solver.cpp)
target_include_directories(kinematics_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinematics_solver PUBLIC c_std_99 cxx_std_17)  # Requiring C++17
ament_target_dependencies(
  kinematics_solver
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(kinematics_solver Eigen3::Eigen)

# Obstacle publisher node
add_executable(add_obstacles src/add_obstacles.cpp)
ament_target_dependencies(add_obstacles
  rclcpp
  moveit_msgs
  shape_msgs
  geometry_msgs
)

install(TARGETS kinematics_solver add_obstacles
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
