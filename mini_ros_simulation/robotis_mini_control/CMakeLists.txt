cmake_minimum_required(VERSION 2.8.3)
project(robotis_mini_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  dynamic_reconfigure
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  config/RobotisMini.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES darwin_control
#  CATKIN_DEPENDS controller_manager effort_controllers
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})