#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <hw_interface/my_hw_interface.h>

int main(int argc, char** argv)
{
  ROS_WARN("entering hw main");
  try
  {
    ros::init(argc, argv, "my_hw_interface");
  ros::NodeHandle nh;
  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();
  // Create the hardware interface specific to your robot
  boost::shared_ptr<my_robot_ns::MyHWInterface> my_hw_interface(
      new my_robot_ns::MyHWInterface(nh));
  my_hw_interface->init();
  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, my_hw_interface);
  control_loop.run();  // Blocks until shutdown signal recieved
  ROS_WARN("error not in hw main");
  }
  catch (const std::exception& e)
  {
    ROS_WARN("wtf hw main");
  }
  ROS_WARN("exiting hw main");
  return 0;
}
