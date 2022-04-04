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
#include "detection_manager.h"

using namespace std;

//Control Station Topic Values
//Task
static int8_t ctrl_task;
static int8_t ctrl_command;

static int8_t semi_auto_id;
static int8_t mode;
static int8_t task_progress;

//Arm Control Topic Values
static uint8_t end_of_movement;
static uint8_t end_of_task;

//Detection Software Topic Values
static uint8_t state = 1;   //start in Inactive
static uint16_t detected_elements[Number_of_Elements-1][7];
static uint8_t current_element;
//Images???

int main(int argc, char **argv)
{
    ros::init(argc,argv, "listener_node");
    ros::NodeHandle manager;
    ros::Rate loop_rate(10);

    //Publishers setup
    ros::Publisher state_pub = manager.advertise<std_msgs::UInt8>("detection/state",1000);
    ros::Publisher current_element_pub = manager.advertise<std_msgs::UInt8>("detection/current_element",1000);
    
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
    ros::Subscriber RGB_intel_sub = manager.subscribe("detection/RGB_intel",                    1000, RGB_intel_Callback);
    ros::Subscriber RGB_webcam_1_sub = manager.subscribe("detection/RGB_webcam_1",              1000, RGB_webcam_1_Callback);
    ros::Subscriber RGB_webcam_2_sub = manager.subscribe("detection/RGB_webcam_2",              1000, RGB_webcam_2_Callback);
    
    while(ros::ok()){
        state = determine_state();
        state_action();
        state_pub.publish(state);       //not sure about order of operations here, I think it depends on how long each thing takes
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

//checks current state and topic values, determines what state the detection software should be in. changes state.
int determine_state(void){
    switch(state){
        case INACTIVE:
            inactive_state_change_check();
            break;

        case INITIALISATION:
            initialisation_state_change_check();
            break;

        case WAIT:
            waiting_state_change_check();
            break;
        
        case MEASUREMENT:
            measurement_state_change_check();
            break;
        
        case ERROR:
            error_state_change_check();
            break;

        default:
            ROS_INFO("huh, a default state");
    }

    ROS_INFO("the state is %i", state);
}

//carries out actions that should occur in current state
void state_action(void){

}

//callbacks need to return void, value changes should affect global static variables
void task_Callback(const std_msgs::Int8MultiArray::ConstPtr& task_array)//const std_msgs::Int8MultiArray::ConstPtr& task_array)
{
    ctrl_task = task_array->data[0];
    ctrl_command = task_array->data[1];

    ROS_INFO("task = %i", ctrl_task);
    ROS_INFO("command = %i", ctrl_command);
}

void HD_SemiAuto_Id_Callback(const std_msgs::Int8::ConstPtr& received_semi_auto_id){semi_auto_id = received_semi_auto_id->data;}
void HD_mode_Callback(const std_msgs::Int8::ConstPtr& received_mode){mode = received_mode->data;}

void end_of_movement_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_movement){end_of_movement = received_end_of_movement->data;}
void end_of_task_Callback(const std_msgs::UInt8::ConstPtr& received_end_of_task){end_of_task = received_end_of_task->data;}

void TaskProgress_Callback(const std_msgs::Int8::ConstPtr& received_taskprogress){task_progress = received_taskprogress->data;} // not sure what we are doing with task progress exactly

//void state_Callback(const std_msgs::UInt8::ConstPtr& state){}
void detected_elements_Callback(const std_msgs::Int16MultiArray::ConstPtr& received_detected_elements){}   //complicated, need to figure out how Aly is sending
//void current_element_Callback(const std_msgs::UInt8::ConstPtr& current_element){}
void bounding_boxes_Callback(const sensor_msgs::Image::ConstPtr& received_bounding_boxes){}                 // image stuff
void RGB_intel_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_intel){}                           // image stuff
void RGB_webcam_1_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_webcam_1){}                     // image stuff
void RGB_webcam_2_Callback(const sensor_msgs::Image::ConstPtr& received_RGB_webcam_2){}                     // image stuff

//State Transtions - check if a change from the current state should be done and then changes to the correct state
void inactive_state_change_check(void)
{
    if(ctrl_task == MAINTENANCE && ctrl_command == LAUNCH)  //activation from control station
        state = INITIALISATION;
    else
        state = INACTIVE;
}
void initialisation_state_change_check(void)
{
    if( ctrl_task == MAINTENANCE &&
        ctrl_command == ABORT)                              //abort from control station
        state = INACTIVE;
    else if( ctrl_task == MAINTENANCE && 
        ctrl_command == LAUNCH && 
        current_element == ALL &&
        task_progress == true)                              //waiting for arm to move to home position  
        state = WAITING;
    else
        state = INITIALISATION;
}
void waiting_state_change_check(void)
{
    if( ctrl_task == MAINTENANCE && 
        ctrl_command == ABORT)                              //abort from control station
        state = INACTIVE;
    else if(ctrl_task == MAINTENANCE && 
        ctrl_command == LAUNCH &&
        end_of_movement == true &&
        end_of_task == true &&
        task_progress == true
        //need to write a function to check state of detection_elements[]
        )                                                   //all tasks complete
        state = INACTIVE;
}
void measurement_state_change_check(void);
void error_state_change_check(void);