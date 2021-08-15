cmake_minimum_required(VERSION 2.8.3)
project(segment_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_core
  costmap_2d
  base_local_planner
  pluginlib
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  dynamic_reconfigure
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/SegmentGlobalPlannerParams.cfg
#   cfg/DynReconf2.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES segment_global_planner
  CATKIN_DEPENDS
    nav_core
    costmap_2d
    base_local_planner
    pluginlib
    roscpp
    geometry_msgs
    nav_msgs
    tf2
    dynamic_reconfigur
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(segment_global_planner
  src/segment_global_planner.cpp
  src/line_segment.cpp
)
add_dependencies(segment_global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(segment_global_planner
  ${catkin_LIBRARIES}
)
