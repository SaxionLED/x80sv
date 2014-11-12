cmake_minimum_required(VERSION 2.8.3)
project(x80sv_driver)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp tf rostest nav_msgs dynamic_reconfigure)
find_package(Boost REQUIRED)

add_message_files(
  FILES
  CustomSensor.msg
  WheelVelocities.msg
  MotorInfo.msg
  MotorInfoArray.msg
  PowerInfo.msg
  StandardSensor.msg
  RangeArray.msg
  RangeDefined.msg
  RangeDefinedArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
    cfg/x80sv_driver.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp tf
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# x80 control and sensors
add_library(x80_sensor_driver src/x80/DrRobotMotionSensorDriver.cpp src/x80/serial_comm.cpp src/x80/network_comm.cpp src/x80/comm_base.cpp)
add_executable(x80_robot src/x80/drrobot_player.cpp src/x80/player_node.cpp)
add_dependencies(x80_robot x80sv_driver_gencfg)
target_link_libraries(x80_robot x80_sensor_driver)
target_link_libraries(x80_robot ${catkin_LIBRARIES})

# neato revo lds 
add_library(xv11_laser_driver src/revo_lds/xv11_laser.cpp)
add_executable(neato_laser_publisher src/revo_lds/neato_laser_publisher.cpp)
target_link_libraries(neato_laser_publisher xv11_laser_driver)
target_link_libraries(neato_laser_publisher ${Boost_LIBRARIES})
target_link_libraries(neato_laser_publisher ${catkin_LIBRARIES})

# robot wrapper
add_executable(robot_wrapper src/robot_wrapper.cpp)
target_link_libraries(robot_wrapper ${catkin_LIBRARIES})
add_dependencies(robot_wrapper x80sv_driver_gencpp)

# sensor safety
add_executable(sensor_safety src/sensor_safety.cpp)
target_link_libraries(sensor_safety ${catkin_LIBRARIES})
add_dependencies(sensor_safety x80sv_driver_gencpp)


catkin_add_gtest(laser_driver_test test/test_laser.cpp)
target_link_libraries(laser_driver_test xv11_laser_driver)

catkin_add_gtest(x80sv_driver_test test/test_x80svdriver.cpp)
target_link_libraries(x80sv_driver_test x80_sensor_driver)





