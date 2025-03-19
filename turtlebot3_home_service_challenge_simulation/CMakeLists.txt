cmake_minimum_required(VERSION 3.8)
project(turtlebot3_home_service_challenge_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

include_directories(
  ${controller_manager_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${xacro_INCLUDE_DIRS}
)

install(DIRECTORY launch config models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
