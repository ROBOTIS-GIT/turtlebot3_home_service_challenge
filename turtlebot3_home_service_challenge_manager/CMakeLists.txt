################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_home_service_challenge_manager)

################################################################################
# Compile as C++11, supported in ROS Kinetic and newer
################################################################################
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS  
  roslib
  roscpp
  rospy
  tf
  turtlebot3_msgs
  geometry_msgs
  std_msgs
  cmake_modules
  eigen_conversions
  move_base_msgs
  actionlib
  actionlib_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

################################################################################
# System dependencies are found with CMake's conventions
################################################################################
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")

find_package(Eigen REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roslib
    roscpp
    rospy
    turtlebot3_msgs
    geometry_msgs
    std_msgs
    move_base_msgs
    actionlib
    actionlib_msgs
    sensor_msgs
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_LIBRARIES}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

################################################################################
# Build
################################################################################
add_executable(tb3_hsc_manager_node 
  src/tb3_hsc_manager_node.cpp
  src/tb3_hsc_manager.cpp
  src/tb3_hsc_manager_core.cpp
  src/tb3_hsc_manager_mobile.cpp
  src/tb3_hsc_manager_manipulation.cpp
  src/service.cpp
)
add_dependencies(tb3_hsc_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tb3_hsc_manager_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Boost_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
