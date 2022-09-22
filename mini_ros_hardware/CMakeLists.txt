################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mini_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    message_generation
    dynamixel_sdk
    gazebo_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    dynamixel_sdk
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(inplace_node src/inplace_walking.cpp)
add_dependencies(inplace_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inplace_node ${catkin_LIBRARIES})

add_executable(ball_chasing_node src/ball_chasing.cpp)
add_dependencies(ball_chasing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_chasing_node ${catkin_LIBRARIES})

add_executable(foot_position_input src/foot_position_input.cpp)
add_dependencies(foot_position_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(foot_position_input ${catkin_LIBRARIES})