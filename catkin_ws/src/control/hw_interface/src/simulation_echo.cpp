#include <ros/ros.h>
#include <robot_control2/jointCmd.h>
#include <robot_control2/jointFdbk.h>

ros::Publisher feedback_pub;


void cmdCallback(const robot_control2::jointCmd::ConstPtr &msg)
{
    /*
    float32[5] current  # amps
    float32[5] accel    # rad/s²
    float32[5] vel      # rad/s
    float32[5] angle    # rad
    uint32 msg_ctr      # count sent msgs to detect missing ones
    */
   static robot_control2::jointFdbk feedback;
    for (int i=0; i < msg->angle.size(); i++)
    {
        feedback.angle[i] = msg->angle[i];
        feedback.vel[i] = msg->vel[i];
        feedback.current[i] = msg->current[i];

    }

    feedback_pub.publish(feedback);
}



//// main ////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_echo");

    ros::NodeHandle n;

    ros::Subscriber cmd_sub = n.subscribe("jointCommand", 10, cmdCallback);

    feedback_pub = n.advertise<robot_control2::jointFdbk>("jointFeedback", 10);

    ros::spin();
}