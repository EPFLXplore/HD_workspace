//ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.msg>
#include <std_msgs/UInt8.msg>
#include <std_msgs/Int8MultiArray.msg>
#include <std_msgs/Int16MultiArray.msg>
#include <std_msgs/Image.msg>


//c++ includes
#include <iostream>
#include <sstream>

enum CtrlTasks {void, MANUAL, NAVIGATION, MAINTENANCE, SCIENCE}
enum CtrlCommands {void, LAUNCH, ABORT, WAIT, RESUME, RETRY}
enum States {void, INACTIVE, INITIALISATION, WAITING, MEASUREMENT, ERROR}
enum Element_IDs {ELEMENT1, ELEMENT2, ELEMENT3}

using namespace std;

static uint8_t ctrl_task;
static uint8_t ctrl_command;

uint8_t taskCallback(const std_msgs::Int8MultiArray::ConstPtr& task_array)
{
    task = task_array->data[0];
    ctrl_command = task_array->data[1];

    ROS_INFO("task = %i", task);
    ROS_INFO("command = %i", ctrl_command);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "HD_detection_manager");
    ros::NodeHandle manager;

    //Publishers setup
    ros::Publisher state_pub = manager.advertise<std_msgs/UInt8.msg>("detection/state",100);
    ros::Publisher current_element_pub = manager.advertise<std_msgs/UInt8.msg>("detection/current_element",100);

    //Subscribers setup
    ros::Subscriber task_sub = manager.subscribe("Task",100,taskCallback);
    //ros::Rate loop_rate(10);
}


