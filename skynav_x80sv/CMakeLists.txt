cmake_minimum_required(VERSION 2.8.3)
project(skynav_x80sv)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp skynav_msgs tf joy)
find_package(Boost REQUIRED)

add_message_files(
  FILES
  CustomSensor.msg
  MotorInfo.msg
  MotorInfoArray.msg
  PowerInfo.msg
  StandardSensor.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp skynav_msgs tf joy
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# x80 control and sensors
add_library(x80_sensor_driver src/x80/sensor_driver/DrRobotMotionSensorDriver.cpp)
add_executable(x80_robot src/x80/drrobot_player.cpp)
target_link_libraries(x80_robot x80_sensor_driver)
target_link_libraries(x80_robot ${catkin_LIBRARIES})

# input
add_executable(input_keyboard src/x80/drrobot_keyboard_teleop.cpp)
target_link_libraries(input_keyboard ${catkin_LIBRARIES})

add_executable(input_joystick src/x80/drrobot_joystick_teleop.cpp)
target_link_libraries(input_joystick ${catkin_LIBRARIES})

# neato revo lds 
add_library(xv11_laser_driver src/revo_lds/xv11_laser.cpp)
add_executable(neato_laser_publisher src/revo_lds/neato_laser_publisher.cpp)
target_link_libraries(neato_laser_publisher xv11_laser_driver)
target_link_libraries(neato_laser_publisher ${Boost_LIBRARIES})
target_link_libraries(neato_laser_publisher ${catkin_LIBRARIES})

# robot wrapper
add_executable(robot_wrapper src/robot_wrapper.cpp)
target_link_libraries(robot_wrapper ${catkin_LIBRARIES})
add_dependencies(robot_wrapper skynav_msgs_gencpp)

# sensor safety
add_executable(sensor_safety src/sensor_safety.cpp)
target_link_libraries(sensor_safety ${catkin_LIBRARIES})
add_dependencies(sensor_safety skynav_msgs_gencpp)







