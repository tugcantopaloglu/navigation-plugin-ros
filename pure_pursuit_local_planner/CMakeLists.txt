cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_local_planner)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
  angles
)


catkin_package(

  CATKIN_DEPENDS base_local_planner costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp tf2_geometry_msgs tf2_ros angles

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/pure_pursuit_local_planner.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION} 
)

install(FILES pure_pursuit_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} 
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} 
  FILES_MATCHING PATTERN "*.h"
)