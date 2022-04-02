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
enum Element_IDs {ALL, ELEMENT1, ELEMENT2, ELEMENT3, Number_of_Elements};

using namespace std;

//Control Station Topics
//Task
static int8_t ctrl_task;
static int8_t ctrl_command;

static int8_t semi_auto_id;
static int8_t mode;

//Arm Control Topics
static uint8_t end_of_movement;
static uint8_t end_of_task;

//Detection Software Topics
static uint8_t state = 1;   //start in Inactive
static uint16_t detected_elements[Number_of_Elements-1][7];
static uint8_t current_element;
//Images???


//callbacks need to return void, value changes should affect global static variables
void task_Callback(const std_msgs::Int8MultiArray::ConstPtr& task_array)//const std_msgs::Int8MultiArray::ConstPtr& task_array)
{
    ctrl_task = task_array->data[0];
    ctrl_command = task_array->data[1];

    ROS_INFO("task = %i", ctrl_task);
    ROS_INFO("command = %i", ctrl_command);
}

void HD_SemiAuto_Id_Callback(const std_msgs::Int8::ConstPtr& semi_auto_id){}
void HD_mode_Callback(const std_msgs::Int8::ConstPtr& mode){}

void end_of_movement_Callback(const std_msgs::UInt8::ConstPtr& end_of_movement){}
void end_of_task_Callback(const std_msgs::UInt8::ConstPtr& end_of_task){}

void TaskProgress_Callback(const std_msgs::Int8::ConstPtr& taskprogress){}

//void state_Callback(const std_msgs::UInt8::ConstPtr& state){}
void detected_elements_Callback(const std_msgs::Int16MultiArray::ConstPtr& detected_elements){}
//void current_element_Callback(const std_msgs::UInt8::ConstPtr& current_element){}
void bounding_boxes_Callback(const sensor_msgs::Image::ConstPtr& bounding_boxes){}
void RGB_intel_Callback(const sensor_msgs::Image::ConstPtr& RGB_intel){}
void RGB_webcam_1_Callback(const sensor_msgs::Image::ConstPtr& RGB_webcam_1){}
void RGB_webcam_2_Callback(const sensor_msgs::Image::ConstPtr& RGB_webcam_2){}


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
    
    ros::spin();
    return 0;
}


