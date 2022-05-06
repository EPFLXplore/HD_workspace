#include <hw_interface/my_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace my_robot_ns
{
MyHWInterface::MyHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  fdbk_sub = nh.subscribe("jointFeedback", 1, &MyHWInterface::fdbkCallback, this);
  cmd_pub = nh.advertise<sensor_msgs::JointState>("jointCommand", 1);
  ROS_INFO("MyHWInterface constructed");

  try
  {
    for (int i=0; i<num_joints_; i++)
    {
      previous_angle_command_[i] = joint_position_[i];
    }
    ROS_WARN("error not in constructor");
  }
  catch (const std::exception& e)
  {
    ROS_WARN("wtf constructor");
  }
  ROS_WARN("ye error def not in constructor");
}

void MyHWInterface::fdbkCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

  for (int i=0; i<num_joints_; i++)
  {
    joint_position_[i] = msg->position[i];
    joint_velocity_[i] = msg->velocity[i];
  }
}

void MyHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("MyHWInterface Ready.");
}

void MyHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
  ros::spinOnce();
}

void MyHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);


  static sensor_msgs::JointState joint_cmd;

  bool new_cmd = false;
  //ROS_WARN("entering HI write");
  try
  {
    //ROS_WARN("checkpoint1");
    for (int i=0; i<num_joints_; i++)
  {
    //ROS_WARN("checkpoint2");
    joint_cmd.position[i] = joint_position_command_[i];
    if (previous_angle_command_[i] != joint_cmd.position[i])
    {
      //ROS_WARN("checkpoint3");
      new_cmd = true;
      joint_cmd.velocity[i] = (joint_cmd.position[i]-previous_angle_command_[i])/elapsed_time.toSec();
      //ROS_WARN("checkpoint4");
      //ROS_WARN("checkpoint5");
      previous_angle_command_[i] = joint_cmd.position[i];
      //ROS_WARN("checkpoint6");
    }
  }
  //ROS_WARN("checkpoint5");
  if (new_cmd)
  {
    cmd_pub.publish(joint_cmd);
  }
  }
  catch (const std::exception& e)
  {
    ROS_WARN("wtfffffff");
  }


}

void MyHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
}



}  // namespace my_robot_ns
