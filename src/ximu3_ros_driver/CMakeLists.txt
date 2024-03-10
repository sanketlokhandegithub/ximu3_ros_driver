cmake_minimum_required(VERSION 3.0.2)
project(ximu3_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation
  genmsg
)


include_directories(
  include
  msg
  lib
  ${catkin_INCLUDE_DIRS}
)

add_message_files (
	DIRECTORY msg
	FILES Accel.msg Euler.msg Gyro.msg LinearAccel.msg Mag.msg Quaternion.msg RotMatrix.msg
)

generate_messages (
	DEPENDENCIES
	std_msgs
)

catkin_package( CATKIN_DEPENDS message_runtime )

add_library(ximu_static_library STATIC IMPORTED)
set_target_properties(ximu_static_library PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libximu3.a)


add_executable(ximu3_node src/ximu3.cpp ) #
target_link_libraries(ximu3_node ${catkin_LIBRARIES} ximu_static_library pthread dl m udev)
add_dependencies(ximu3_node ximu3_ros_driver_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ximu3_ros_driver_generate_messages_cpp)
