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

enum CtrlCommands {ABORT, RETRY, CONFIRM, MANUAL, AUTO};
enum States {IDLE, AUTO_INIT, AUTO_TASK, AUTO_WAIT, MANUAL_INIT, MANUAL_TASK};

//leave this for now but it isnt necessary for what we are doing in the new fsm design
enum Element_IDs {ELEMENT1, ELEMENT2, ELEMENT3, Number_of_Elements, ALL, MANUAL_CTRL}; //ALL = init_pos, MANUAL_CTRL = all but without associated movement

//logic selector
void determine_state();
void state_action();

//state change logic
void idle_state_change_check();
void auto_init_state_change_check();
void auto_task_state_change_check();
void auto_wait_state_change_check();
void manual_init_state_change_check();
void manual_task_state_change_check();

//action logic (why is this a seperate thing??)
void idle_action();
void auto_init_action();
void auto_task_action();
void auto_wait_action();
void manual_init_action();
void manual_task_action();


int detected_elements_check(void);          //checks detected elements table for multiple situations used to change state (all tasks complete, reliability)

//publishing helpers
void status_publish(const char* status);
void abort();

//Callbacks -- NEEDS CLEANUP -- PK 2022
void task_Callback(const std_msgs::Int8::ConstPtr& task_array);
void HD_SemiAuto_Id_Callback(const std_msgs::Int8::ConstPtr& received_semi_auto_id);
void HD_mode_Callback(const std_msgs::Int8::ConstPtr& received_mode);
void end_of_movement_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_movement);
void end_of_task_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_task);
void TaskProgress_Callback(const std_msgs::Int8::ConstPtr& received_taskprogress);
//void state_Callback(const std_msgs::UInt8::ConstPtr& state);
void detected_elements_Callback(const std_msgs::Int16MultiArray::ConstPtr& received_detected_elements);
//void current_element_Callback(const std_msgs::UInt8::ConstPtr& current_element);
void bounding_boxes_Callback(const sensor_msgs::Image::ConstPtr& received_bounding_boxes);
