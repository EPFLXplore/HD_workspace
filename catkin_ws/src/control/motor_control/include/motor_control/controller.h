#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/cache.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <sys/mman.h>
#include <pid/signal_manager.h>
#include <math.h>

#include <ethercatcpp/epos4.h>
#include <ethercatcpp/master.h>
#include <xcontrol_v2/network_master.h>
#include <xcontrol_v2/one_axis_slot.h>
#include <xcontrol_v2/three_axis_slot.h>

class Controller {
public:
    Controller();
    void accountForJoint56Dependency();
    void manualCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg):
    void stateCommandCallback(const sensor_msgs::JointState::ConstPtr& msg);
    bool command_too_old();
    double security_angle(double vel, size_t it);
    void enforce_limits();
    void update_targets();
    void stop();
    void set_goals();
    void ros_loop();
private:
    
};



#endif