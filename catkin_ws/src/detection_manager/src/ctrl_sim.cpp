//ros includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

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

int main(int argc, char **argv)
{
    ros::init(argc,argv, "ctrl_sim");
    ros::NodeHandle ctrl;

    //Publishers setup
    ros::Publisher task_pub = ctrl.advertise<std_msgs::Int8MultiArray>("Task",100);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::Int8MultiArray task;

        task->data[0] = ctrl_task;
        task->data[1] = ctrl_command;

        task_pub.publish(task);
        ROS_INFO("published the task!");
        ros::spinOnce();

    }
}