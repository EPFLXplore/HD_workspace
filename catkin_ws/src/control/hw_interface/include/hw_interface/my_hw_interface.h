#ifndef MY_ROS_CONTROL__SIM_HW_INTERFACE_H
#define MY_ROS_CONTROL__SIM_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <sensor_msgs/JointState.h>


namespace my_robot_ns
{
/** \brief Hardware interface for a robot */
class MyHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  MyHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \brief Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:

  ros::Subscriber fdbk_sub;
  void fdbkCallback(const sensor_msgs::JointState::ConstPtr &msg);

  ros::Publisher cmd_pub;

  double previous_angle_command_[6];

};  // class

}  // namespace ros_control_boilerplate

#endif
