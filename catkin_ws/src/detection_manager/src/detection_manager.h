//ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Image.h>

//c++ includes
#include <iostream>
#include <sstream>

enum CtrlTasks {MANUAL = 1, NAVIGATION, MAINTENANCE, SCIENCE};
enum CtrlCommands {LAUNCH = 1, ABORT, WAIT, RESUME, RETRY};
enum States {INACTIVE = 1, INITIALISATION, WAITING, MEASUREMENT, ERROR};
enum Element_IDs {ALL, ELEMENT1, ELEMENT2, ELEMENT3, Number_of_Elements}; //ALL = init_pos

int determine_state(void);
void state_action(void);
void inactive_state_change_check(void);
void initialisation_state_change_check(void);
void waiting_state_change_check(void);
void measurement_state_change_check(void);
void error_state_change_check(void);


void task_Callback(const std_msgs::Int8MultiArray::ConstPtr& task_array);
void HD_SemiAuto_Id_Callback(const std_msgs::Int8::ConstPtr& received_semi_auto_id);
void HD_mode_Callback(const std_msgs::Int8::ConstPtr& received_mode);

void end_of_movement_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_movement);
void end_of_task_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_task);

void TaskProgress_Callback(const std_msgs::Int8::ConstPtr& received_taskprogress);

//void state_Callback(const std_msgs::UInt8::ConstPtr& state);
void detected_elements_Callback(const std_msgs::Int16MultiArray::ConstPtr& received_detected_elements);
//void current_element_Callback(const std_msgs::UInt8::ConstPtr& current_element);
void bounding_boxes_Callback(const sensor_msgs::Image::ConstPtr& received_bounding_boxes);
void RGB_intel_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_intel);
void RGB_webcam_1_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_webcam_1);
void RGB_webcam_2_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_webcam_2);