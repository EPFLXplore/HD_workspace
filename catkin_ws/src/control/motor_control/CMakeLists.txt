cmake_minimum_required(VERSION 3.0.2)
project(motor_control)


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
message_filters
message_generation
)

add_message_files(
 FILES
 simJointState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
CATKIN_DEPENDS rospy roscpp message_runtime
)


include_directories(
# include
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
)

message(STATUS "DIR : --------->  ${PROJECT_SOURCE_DIR}")


