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
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)


catkin_package(
CATKIN_DEPENDS rospy roscpp message_runtime
)


include_directories(
# include
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/include
)

message(STATUS "DIR : --------->  ${PROJECT_SOURCE_DIR}")


add_executable(motor_control src/motor_control.cpp)

target_link_libraries(motor_control ${catkin_LIBRARIES} 
                    ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-core_ethercatcpp-core.so 
                    ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-epos_ethercatcpp-epos.so 
                    ${PROJECT_SOURCE_DIR}/lib/libpid-os-utilities_pid-signal-manager.so 
                    ${PROJECT_SOURCE_DIR}/lib/libsoem.a 
            ${PROJECT_SOURCE_DIR}/lib/libxcontrol_xcontrol.so)
#target_link_libraries(motor_control ${catkin_LIBRARIES}
#		      ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-core_ethercatcpp-core.so 
#                      ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-epos_ethercatcpp-epos.so)

add_dependencies(motor_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(testt src/testt.cpp)

target_link_libraries(testt ${catkin_LIBRARIES}
            ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-core_ethercatcpp-core.so 
                    ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-epos_ethercatcpp-epos.so)

add_dependencies(testt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})