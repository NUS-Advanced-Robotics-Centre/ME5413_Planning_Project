cmake_minimum_required(VERSION 3.0.2)
project(me5413_world)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  rviz
  gazebo_ros
  jsk_rviz_plugins
  jackal_gazebo
  jackal_navigation
  teleop_twist_keyboard
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/path_publisher.cfg
  cfg/path_tracker.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES me5413_world
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs dynamic_reconfigure
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

# Add Nodes
add_executable(path_publisher_node src/path_publisher_node.cpp)
target_link_libraries(path_publisher_node ${catkin_LIBRARIES})
add_dependencies(path_publisher_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(path_tracker_node src/path_tracker_node.cpp)
target_link_libraries(path_tracker_node ${catkin_LIBRARIES})
add_dependencies(path_tracker_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)