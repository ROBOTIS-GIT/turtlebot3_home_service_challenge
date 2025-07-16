cmake_minimum_required(VERSION 3.8)
project(turtlebot3_home_service_challenge_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch map rviz param urdf config gazebo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
