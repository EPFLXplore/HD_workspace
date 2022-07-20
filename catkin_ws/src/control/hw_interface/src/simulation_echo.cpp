#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <motor_control/simJointState.h>

ros::Publisher feedback_pub;
ros::Publisher sim_feedback_pub;


void cmdCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
   sensor_msgs::JointState feedback;
   motor_control::simJointState sim_feedback;   // for simulation only
    for (int i=0; i < msg->position.size(); i++)
    {
        feedback.position.push_back(msg->position[i]);
        feedback.velocity.push_back(msg->velocity[i]);
        //feedback.effort.push_back(msg->effort[i]);

        sim_feedback.position[i] = msg->position[i];
        sim_feedback.velocity[i] = msg->velocity[i];
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
    sim_feedback_pub = n.advertise<motor_control::simJointState>("/arm_control/sim_joint_telemetry", 10);

    ros::spin();
}