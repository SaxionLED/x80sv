cmake_minimum_required(VERSION 2.8.3)
project(smooth_local_planner)

find_package(catkin REQUIRED
  COMPONENTS
    costmap_2d
    base_local_planner
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smooth_local_planner_lib
  CATKIN_DEPENDS
    costmap_2d
    base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(smooth_local_planner_lib
  src/smooth_local_planner.cpp
)
target_link_libraries(smooth_local_planner_lib ${catkin_LIBRARIES})

add_dependencies(smooth_local_planner_lib ${PROJECT_NAME}_gencfg)

install(TARGETS smooth_local_planner_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES smooth_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
