/*
detection_manager.cpp houses the node listerner_node. It was originally written by Ben but
has since been taken over by PK in MAY 2022. It undertook a big redesign.

States:
    - IDLE
    - AUTO_INIT
    - AUTO_TASK
    - MAN_INIT
    - MAN_TASK

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
#define IDLE_MASK 127               //This is the mask so the fsm knows it is waiting for CS input in AUTO_INIT and AUTO_MAN
#define MAN_MASK 126                //This mask tells the FSM it is clear to go to manual control.

using namespace std;

//Control Station Topic Values
static int8_t ctrl_command;     //CS control command
static int8_t mode;             //The mode of the arm

//Arm Control Topic Values
static uint8_t mode_confirm;    //confirms which mode the arm is in

//Detection Software Topic Values
static uint8_t state = 0;                                       //start in IDLE
static uint8_t current_element = IDLE_MASK;                     //setting the button index to mask - stores which tube we are on
static int16_t detected_elements[Number_of_Elements][7];        //List of elements recieved from the vision system

//Publishers setup
ros::Publisher state_pub;
std_msgs::UInt8 state_msg;

ros::Publisher mode_pub;
std_msgs::UInt8 mode_msg;

ros::Publisher status_pub;
std_msgs::String status_msg;

ros::Publisher camera_enable_pub;
std_msgs::Bool camera_enable_msg;




int main(int argc, char **argv)
{
    ros::init(argc,argv, "listener_node");
    ros::NodeHandle manager;
    ros::Rate loop_rate(10);

    //Publishers setup
    state_pub = manager.advertise<std_msgs::UInt8>("detection/state",1000);
    mode_pub = manager.advertise<std_msgs::UInt8>("detection/mode",1000);
    status_pub = manager.advertise<std_msgs::String>("detection/status",1000);
    camera_enable_pub = manager.advertise<std_msgs::Bool>("detection/camera_enable",1000);
    
    //Subscribers setup
    //FROM CS
    ros::Subscriber task_sub = manager.subscribe("av_task", 1000, task_Callback);

    //FROM ARM
    ros::Subscriber mode_confirm_sub = manager.subscribe("arm_control/mode_confirm", 1000, HD_mode_Callback);
    
    //FROM VISION SYSTEM
    // ros::Subscriber detected_elements_sub = manager.subscribe("detection/detected_elements",    1000, detected_elements_Callback);
    
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
        abort_func();
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
        abort_func();
    //check the button index to see if its a mask
    //If it isn't then we have the data we need to proceed
    //We also need to make sure that the arm has confirmed it is ready.
    else if(current_element == 0)
     {   status_publish("Moving to AUTO_TASK");
        state = AUTO_TASK;}
}

void auto_task_state_change_check(void)
{
    if(ctrl_command == ABORT)                              //abort from control station
        abort_func();
}


void manual_init_state_change_check(void)
{
    if(ctrl_command == ABORT)                              //abort from control station
        abort_func();

    //after the init commands are sent we move
    //to the task state
    else if(current_element == MAN_MASK)
        {status_publish("Moving to MANUAL_TASK");
        state = MANUAL_TASK;}
}

void manual_task_state_change_check(void)
{
    if(ctrl_command == ABORT)
        abort_func();
    
}


/*----------------- ACTION -----------------*/

void idle_action()
{
    //PUBLISH TURN THE CAMERA OFF
    camera_enable_msg.data = false;
    camera_enable_pub.publish(camera_enable_msg);

    //PUBLISH RETURN ARM TO HOME POSITION
    change_mode(HOME);

    //clear button index
    current_element = IDLE_MASK;
}

void auto_init_action()
{
    //send the camera enable command
    camera_enable_msg.data = true;
    camera_enable_pub.publish(camera_enable_msg);

    change_mode(AUTO_MODE);

    //wait for the button list?

    //wait for positions from camera? Useful for confirming lock on.

    //after the list is recieved, set the index off mask
    if(mode_confirm == mode) current_element = 0;
}

void auto_task_action()
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

    //changing the arm mode
    change_mode(MAN_MODE);

    if(mode == mode_confirm) current_element = MAN_MASK;
}

void manual_task_action()
{
    //more waiting
    // exit(1);
}



void change_mode(const ArmModes new_mode)
{
    mode = new_mode;
    mode_msg.data = mode;
    mode_pub.publish(mode_msg);
}

void status_publish(const char* status)
{   
    status_msg.data = status;
    status_pub.publish(status_msg);
}

void abort_func()
{
    //this function aborts to IDLE
    status_publish("Aborting to IDLE");
    ctrl_command = IDLE_MASK;        //masking to prevent spam
    state = IDLE;
}

/*----------------- CALLBACKS -----------------*/
//callbacks need to return void, value changes should affect global static variables
void task_Callback(const std_msgs::Int8::ConstPtr& task_array)
{
    ctrl_command = task_array->data;

    ROS_INFO("command = %i", ctrl_command);
}

void HD_mode_Callback(const std_msgs::Int8::ConstPtr& received_mode){mode = received_mode->data;}
 
