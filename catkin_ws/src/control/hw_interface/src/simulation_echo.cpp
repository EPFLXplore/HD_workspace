#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

ros::Publisher feedback_pub;


void cmdCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
   sensor_msgs::JointState feedback;
    for (int i=0; i < msg->position.size(); i++)
    {
        feedback.position.push_back(msg->position[i]);
        feedback.velocity.push_back(msg->velocity[i]);
        //feedback.effort.push_back(msg->effort[i]);
    }

    feedback_pub.publish(feedback);
}



//// main ////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_echo");

    ros::NodeHandle n;

    ros::Subscriber cmd_sub = n.subscribe("/arm_control/joint_cmd", 10, cmdCallback);

    feedback_pub = n.advertise<sensor_msgs::JointState>("/arm_control/joint_telemetry", 10);

    ros::spin();
}