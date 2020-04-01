cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_home_service_challenge_simulation)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  urdf
  xacro
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
