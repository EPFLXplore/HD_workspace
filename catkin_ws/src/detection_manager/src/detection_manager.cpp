//ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Image.h>



//c++ includes
#include <iostream>
#include <sstream>

enum CtrlTasks {MANUAL = 1, NAVIGATION, MAINTENANCE, SCIENCE};
enum CtrlCommands {LAUNCH = 1, ABORT, WAIT, RESUME, RETRY};
enum States {INACTIVE = 1, INITIALISATION, WAITING, MEASUREMENT, ERROR};
enum Element_IDs {ELEMENT1, ELEMENT2, ELEMENT3};

using namespace std;

static uint8_t ctrl_task;
static uint8_t ctrl_command;

//callbacks need to return void, value changes should affect global static variables
void taskCallback(const std_msgs::Int8MultiArray::ConstPtr& task_array)
{
    ctrl_task = task_array->data[0];
    ctrl_command = task_array->data[1];

    ROS_INFO("task = %i", ctrl_task);
    ROS_INFO("command = %i", ctrl_command);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "detection_manager_node");
    ros::NodeHandle manager;

    //Publishers setup
    ros::Publisher state_pub = manager.advertise<std_msgs::UInt8>("detection/state",1000);
    ros::Publisher current_element_pub = manager.advertise<std_msgs::UInt8>("detection/current_element",1000);

    //Subscribers setup
    ros::Subscriber task_sub = manager.subscribe("Task",1000,taskCallback);
    ros::Rate loop_rate(10);
}


