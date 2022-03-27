#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include <string>
#include <iostream>

#include <ethercatcpp/master.h>
#include <ethercatcpp/epos4.h>


using namespace std;
using namespace ethercatcpp;


bool verbose = true;
Epos4::control_mode_t control_mode = Epos4::position_CSP;
string network_interface_name = "eth0";  // can be adapted by starting the node with the argument _interface:=ethn
double target_pos = 0;
double target_vel = -60;
bool set_new_target = true;



// ---------- subscriber callback function ----------
void posCallback(const std_msgs::Int32::ConstPtr& msg){
    target_pos = msg->data;
    set_new_target = true;
    //cout << "new target: " << msg->data << endl;
}


void velCallback(const std_msgs::Int32::ConstPtr& msg){
    target_vel = msg->data;
    //cout << "new target: " << msg->data << endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle n;
    cout << "checkpoint1" << endl;
    n.getParam("interface", network_interface_name);
    cout << "checkpoint2" << endl;
    ros::Subscriber sub_pos = n.subscribe<std_msgs::Int32>("pos_cmd", 10, posCallback);
    ros::Subscriber sub_vel = n.subscribe<std_msgs::Int32>("vel_cmd", 10, velCallback);
    ros::Rate loop_rate(25);
    Master ethercat_master;
    EthercatBus robot;
    Epos4 scb_motor;
    cout << "checkpoint3" << endl;

    scb_motor.set_Id("EPOS4", 0x000000fb, 0x60500000); //these numbers are Manufaturer ID (maxon) and and device model ID. They can be found using EPOS studios   0x000000fb
    cout << "checkpoint4" << endl;
    robot.add_Device(scb_motor);
    cout << "checkpoint5" << endl;
    ethercat_master.add_Interface_Primary(network_interface_name);
    cout << "checkpoint6" << endl;
    ethercat_master.add_Bus(robot);
    cout << "checkpoint7" << endl;

    // the main loop of the node
    while (ros::ok()) {

        // check device state, restart if necessary
        if (scb_motor.get_Device_State_In_String() == "Switch on disable") {
            scb_motor.set_Device_State_Control_Word(Epos4::shutdown);
            if (verbose)
                cout << "Device restart necessary" << endl;
        }
        else if (scb_motor.get_Device_State_In_String() == "Ready to switch ON") {
            scb_motor.set_Device_State_Control_Word(Epos4::switch_on_and_enable_op);
            scb_motor.set_Control_Mode(control_mode);
            if (verbose)
                cout << "device setup done" << endl;
        }
        else {
            // if we're okay, we can set the device position
            if (set_new_target){
                scb_motor.set_Target_Position_In_Qc(target_pos);
                //scb_motor.set_Target_Velocity_In_Rpm(target_vel);
                set_new_target = true;
            }
            if (verbose) {
                //cout << "Desired position value = " << std::dec <<target_value << " qc" << endl;
                cout << "Actual Position : " << scb_motor.get_Actual_Position_In_Qc() << " qc" << endl; 
            }
        }


        bool wkc = ethercat_master.next_Cycle(); // this  necessary for magic reasons (sorry, I don't know myself)
        if (wkc) {
            if (verbose) {
                cout << "Selected control mode = " << scb_motor.get_Control_Mode_In_String() << endl;
                cout << "Motor State : " << scb_motor.get_Device_State_In_String() << endl;
            }
            if (scb_motor.get_Device_State_In_String() == "Fault" ) 
                cout << "At least one motor is in fault state, restart the network \n";
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}