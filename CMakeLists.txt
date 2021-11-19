cmake_minimum_required(VERSION 2.8.3)
project(rrt_star_global_planner)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  nav_core
  base_local_planner
  geometry_msgs
  angles
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    costmap_2d
    pluginlib
    nav_core
    base_local_planner
    geometry_msgs
    angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/rrt_star_global_planner.cpp
)

add_executable(rrt_star_global src/rrt_star_global_planner_node.cpp)
add_dependencies(rrt_star_global ${rrt_star_global_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_star_global
  ${catkin_LIBRARIES}
)

 install(
   FILES rrt_star_planner_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

# if (CATKIN_ENABLE_TESTING)
# endif()