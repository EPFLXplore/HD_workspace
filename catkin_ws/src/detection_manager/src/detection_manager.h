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

enum CtrlCommands {ABORT, RETRY, CONFIRM, MANUAL, AUTO};                            //Commands from CS
enum States {IDLE, MANUAL_INIT, MANUAL_TASK, AUTO_INIT, AUTO_TASK};      //STATE LIST
enum ArmModes {HOME, MAN_MODE, AUTO_MODE};                                                    //Possible arm modes

//leave this for now but it isnt necessary for what we are doing in the new fsm design
enum Element_IDs {ELEMENT1, ELEMENT2, ELEMENT3, Number_of_Elements, ALL, MANUAL_CTRL}; //ALL = init_pos, MANUAL_CTRL = all but without associated movement

//logic selector
void determine_state();
void state_action();

//state change logic
void idle_state_change_check();
void auto_init_state_change_check();
void auto_task_state_change_check();
void manual_init_state_change_check();
void manual_task_state_change_check();

//action logic (why is this a seperate thing??)
void idle_action();
void auto_init_action();
void auto_task_action();
void manual_init_action();
void manual_task_action();


//publishing helpers
void change_mode(const ArmModes new_mode);
void status_publish(const char* status);
void abort_func();

//Callbacks 
void task_Callback(const std_msgs::Int8::ConstPtr& task_array);
void HD_mode_Callback(const std_msgs::Int8::ConstPtr& received_mode);
