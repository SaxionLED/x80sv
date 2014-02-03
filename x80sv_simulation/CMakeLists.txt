cmake_minimum_required(VERSION 2.8.3)
project(x80sv_simulation)

find_package(catkin REQUIRED roscpp sensor_msgs)


catkin_package(
CATKIN_DEPENDS roscpp sensor_msgs
)

add_executable(laser_filter src/laser_filter.cpp)
target_link_libraries(laser_filter ${catkin_LIBRARIES})
