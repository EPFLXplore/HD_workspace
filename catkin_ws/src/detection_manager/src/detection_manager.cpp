/*
detection_manager.cpp houses the node listerner_node. It was originally written by Ben but
has since been taken over by PK in MAY 2022. It undertook a big redesign.

States:
    - IDLE
    - AUTO_INIT
    - AUTO_TASK
    - AUTO_WAIT
    - MAN_INIT

Publishers:
    -TBD

Subscribers:
    -TBD

-PK May 21 2022

P.S. I hate the styling convention here but I am way too lazy to change this.
*/



//ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Image.h>

//c++ includes
#include <iostream>
#include <sstream>
#include "detection_manager.h"

#define LOW_BYTE_TRONCATION 0x00FF  //multiplication by this value allows for troncation to the lower byte of a 16-bit int
#define RELIABILITY_THRESHOLD 10    //minimum number of frames that detect the desired AR tag of (x frames) before an error is sent
#define MASK 127                    //This is the mask so the fsm knows it is waiting for CS input in AUTO_INIT

using namespace std;

//Control Station Topic Values
//Task
static int8_t ctrl_command;
static int8_t semi_auto_id;
static int8_t mode;
static int8_t task_progress;

//Arm Control Topic Values
static uint8_t end_of_movement;
static uint8_t end_of_task;

//Detection Software Topic Values
static uint8_t state = 0;   //start in IDLE
static uint8_t current_element = MASK; //setting the button index to mask - stores which tube we are on
static bool arm_confirm = false;    //confirms if the arm has autocompleted
static int16_t detected_elements[Number_of_Elements][7];        //note that an element is actually a sub-task.

//Publishers setup
ros::Publisher state_pub;
std_msgs::UInt8 state_msg;

ros::Publisher status_pub;
std_msgs::String status_msg;

ros::Publisher current_element_pub;
std_msgs::UInt8 current_element_msg;

ros::Publisher camera_enable_pub;
std_msgs::Bool camera_enable_msg;



int main(int argc, char **argv)
{
    ros::init(argc,argv, "listener_node");
    ros::NodeHandle manager;
    ros::Rate loop_rate(10);

    //Publishers setup
    state_pub = manager.advertise<std_msgs::UInt8>("detection/state",1000);
    status_pub = manager.advertise<std_msgs::String>("detection/status",1000);
    current_element_pub = manager.advertise<std_msgs::UInt8>("detection/current_element",1000);
    camera_enable_pub = manager.advertise<std_msgs::Bool>("detection/camera_enable",1000);

    
    //Subscribers setup
    ros::Subscriber task_sub = manager.subscribe("Task",                    1000, task_Callback);
    ros::Subscriber HD_SemiAuto_Id_sub = manager.subscribe("HD_SemiAuto_Id",1000, HD_SemiAuto_Id_Callback);
    ros::Subscriber HD_mode_sub = manager.subscribe("HD_mode",              1000, HD_mode_Callback);
    
    ros::Subscriber end_of_movement_sub = manager.subscribe("/arm_control/end_of_movement", 1000, end_of_movement_Callback);
    ros::Subscriber end_of_task_sub = manager.subscribe("/arm_control/end_of_task",         1000, end_of_task_Callback);
    
    ros::Subscriber TaskProgress_sub = manager.subscribe("TaskProgress", 1000, TaskProgress_Callback);
    
    //ros::Subscriber state_sub = manager.subscribe("detection/state",                            1000, state_Callback);
    ros::Subscriber detected_elements_sub = manager.subscribe("detection/detected_elements",    1000, detected_elements_Callback);
    //ros::Subscriber current_element_sub = manager.subscribe("detection/current_element",        1000, current_element_Callback);
    ros::Subscriber bounding_boxes_sub = manager.subscribe("detection/bounding_boxes",          1000, bounding_boxes_Callback);
    
    while(ros::ok()){
        determine_state();
        state_action();
        ros::spinOnce();
        loop_rate.sleep();
        // std::cout<<__LINE__<<" STATE:"<<(int)state << " " <<ros::ok()<<std::endl;
    }
    return 0;
}


/*----------------- LOGIC SELECTOR -----------------*/
//checks current state and topic values, determines what state the detection software should be in. changes state.
void determine_state(void){
    switch(state){
        case IDLE:
            idle_state_change_check();
            break;

        case AUTO_INIT:
            auto_init_state_change_check();
            break;
        
        case AUTO_TASK:
            auto_task_state_change_check();
            break;
        
        case AUTO_WAIT:
            auto_wait_state_change_check();
            break;
        
        case MANUAL_INIT:
            manual_init_state_change_check();
            break;
        
        case MANUAL_TASK:
            manual_task_state_change_check();
            break;

        default:
            ROS_INFO("huh, a default state");
            ros::shutdown();
    }

    ROS_INFO("the state is %i", state);
}

//carries out actions that should occur in current state
void state_action(void){
    switch(state){
        case IDLE:
            idle_action();
            break;

        case AUTO_INIT:
            auto_init_action();
            break;
        
        case AUTO_TASK:
            auto_task_action();
            break;
        
        case AUTO_WAIT:
            auto_wait_action();
            break;
        
        case MANUAL_INIT:
            manual_init_action();
            break;
        
        case MANUAL_TASK:
            manual_task_action();
            break;

        default:
            ROS_INFO("huh, a default action");
    }
}

/*----------------- STATE CHANGE -----------------*/

void idle_state_change_check(void)
{
    if(ctrl_command == ABORT)
        abort();
    else if(ctrl_command == AUTO)                              //waiting for arm to move to home position  
        {status_publish("Moving to AUTO_INIT");
        state = AUTO_INIT;}
    else if(ctrl_command == MANUAL)
        {status_publish("Moving to MANUAL_INIT");
        state = MANUAL_INIT;}
}

void auto_init_state_change_check(void)
{
    if(ctrl_command == ABORT)                              //abort from control station
        abort();
    
    //check the button index to see if its a mask
    //If it isn't then we have the data we need to proceed 
    else if(current_element == 0)
     {   status_publish("Moving to AUTO_TASK");
        state = AUTO_TASK;}
}

void auto_task_state_change_check(void)
{
    arm_confirm = true; //temp
    int END_SEQUENCE = 4;
    if(ctrl_command == ABORT)                              //abort from control station
        abort();
    //this only preps for the next task.
    //once prep is done it can move to the
    //next state
    else if(current_element == END_SEQUENCE) //END_SEQUENCE IS A PLACEHOLDER FOR LEN OF VECTOR
    {
        status_publish("End of Elements.");
        status_publish("Moving to IDLE");
        state = IDLE;
    }
    //auto confirmation that arm has done the click
    else if (arm_confirm)
        {status_publish("Moving to AUTO_WAIT");
        state = AUTO_WAIT;}
}

void auto_wait_state_change_check(void)
{
    if(ctrl_command == ABORT)                              //abort from control station
        abort();
    //this only preps for the next task.
    //once prep is done it can move to the
    //next state
    else if(ctrl_command == 1)
    {
        status_publish("Task Retry");
        status_publish("Moving to AUTO_TASK");
        state = AUTO_TASK;
        ctrl_command = MASK; //resetting the control
    }
    else if( ctrl_command == 2)
    {
        status_publish("Task Confirmed");
        status_publish("Moving to AUTO_TASK");
        state = AUTO_TASK;
        current_element++;
        ctrl_command = MASK;
    }
}

void manual_init_state_change_check(void)
{
    if(ctrl_command == ABORT)                              //abort from control station
        abort();
    //after the init commands are sent we move
    //to the task state
    else
        {status_publish("Moving to MANUAL_TASK");
        state = MANUAL_TASK;}
}

void manual_task_state_change_check(void)
{
    if(ctrl_command == ABORT)
        abort();
    //maybe implement a transition to auto_init? 
}


/*----------------- ACTION -----------------*/

void idle_action()
{
    //PUBLISH TURN THE FUCKING CAMERA OFF
    camera_enable_msg.data = false;
    camera_enable_pub.publish(camera_enable_msg);

    //clear button index
    current_element = MASK;
}

void auto_init_action()
{
    //send the camera enable command
    camera_enable_msg.data = true;
    camera_enable_pub.publish(camera_enable_msg);

    //wait for the button list?

    //wait for positions from camera? Useful for confirming lock on.

    //after the list is recieved, set the index off mask
    if(true)
        current_element = 0;
}

void auto_task_action()
{
    //send the index to the arm so that the iterative approach can start
    current_element_msg.data = current_element;
    current_element_pub.publish(current_element_msg);

    //the autconfirm of the arm happens in the msg callback
}

void auto_wait_action()
{
    //just wait lol nothing to do
    //maybe put a bit of a pause here
    // exit(1);
}

void manual_init_action()
{
    //send the camera enable command
    camera_enable_msg.data = true;
    camera_enable_pub.publish(camera_enable_msg);

    //manual control command is mask
    current_element_msg.data = MASK;
    current_element_pub.publish(current_element_msg);
}

void manual_task_action()
{
    //more waiting
    // exit(1);
}

/*----------------- HELPERS -----------------*/

// int detected_elements_check(void)
// {
//     //check for all done
//     if(state == WAITING)
//     {
//         int8_t all_done = 0;
//         for(auto element : detected_elements)
//         {
//             if(element[0] < 0)
//                 all_done++;
//         }
//         if(all_done == Number_of_Elements)
//             return 1;
//     }
//     //check for reliability error on current element
//     else if(state == MEASUREMENT)
//     {
//         uint8_t current_elem_reliability = detected_elements[current_element][0] & LOW_BYTE_TRONCATION;
//         if(current_elem_reliability < RELIABILITY_THRESHOLD)
//         {
//             return -1;
//         }
//     }
//     return 0;
// }

void status_publish(const char* status)
{   
    status_msg.data = status;
    status_pub.publish(status_msg);
}

void abort()
{
    //this function aborts to IDLE
    status_publish("Aborting to IDLE");
    ctrl_command = MASK;        //masking to prevent spam
    state = IDLE;
}

/*----------------- CALLBACKS -----------------*/
//callbacks need to return void, value changes should affect global static variables
//void state_Callback(const std_msgs::UInt8::ConstPtr& state){}
void detected_elements_Callback(const std_msgs::Int16MultiArray::ConstPtr& received_detected_elements) {}     //need to figure out how Aly is sending
// {
//     if(current_element == MANUAL_CTRL || current_element == ALL)
//     {
//         for(uint8_t element_counter = ELEMENT1; element_counter < Number_of_Elements; element_counter++)
//         {
//             for(uint8_t data_counter = 0; data_counter < sizeof(detected_elements)/sizeof(detected_elements[0]); data_counter++)
//             {
//                 detected_elements[element_counter][data_counter] = received_detected_elements->data[data_counter];
//             }
//         }
//     }
// }
//void current_element_Callback(const std_msgs::UInt8::ConstPtr& current_element){}
void bounding_boxes_Callback(const sensor_msgs::Image::ConstPtr& received_bounding_boxes){}                 // image stuff

void task_Callback(const std_msgs::Int8::ConstPtr& task_array)//const std_msgs::Int8MultiArray::ConstPtr& task_array)
{
    ctrl_command = task_array->data;

    ROS_INFO("command = %i", ctrl_command);
}

void HD_SemiAuto_Id_Callback(const std_msgs::Int8::ConstPtr& received_semi_auto_id){semi_auto_id = received_semi_auto_id->data;}
void HD_mode_Callback(const std_msgs::Int8::ConstPtr& received_mode){mode = received_mode->data;}
 
void end_of_movement_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_movement){end_of_movement = received_end_of_movement->data;}
void end_of_task_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_task){end_of_task = received_end_of_task->data;}

void TaskProgress_Callback(const std_msgs::Int8::ConstPtr& received_taskprogress){task_progress = received_taskprogress->data;} // not sure what we are doing with task progress exactly

