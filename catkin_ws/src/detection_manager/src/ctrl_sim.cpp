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
enum Element_IDs {ALL, ELEMENT1, ELEMENT2, ELEMENT3};

using namespace std;

static uint8_t ctrl_task = MAINTENANCE;
static uint8_t ctrl_command = WAIT;

int main(int argc, char **argv)
{
    ros::init(argc,argv, "ctrl_sim");
    ros::NodeHandle ctrl;
    ros::Rate loop_rate(10);

    //Publishers setup
    ros::Publisher task_pub = ctrl.advertise<std_msgs::Int8MultiArray>("Task",100);
    ros::Publisher Semi_auto_pub = ctrl.advertise<std_msgs::Int8>("HD_SemiAuto_Id",100);
    ros::Publisher mode_pub = ctrl.advertise<std_msgs::Int8>("HD_mode",100);

    std_msgs::Int8MultiArray task;
    std_msgs::Int8 semi_auto_id;
    std_msgs::Int8 mode;



    int count = 0;
    while (ros::ok())
    {
        task.data.push_back(ctrl_command);
        task.data.push_back(ctrl_task);
        semi_auto_id.data = ELEMENT1;
        mode.data = 1;

        task_pub.publish(task);
        ROS_INFO("published the task!");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}