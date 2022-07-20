#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <motor_control/simJointState.h>    // for simulation only

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

using namespace std;
using namespace ethercatcpp;
using namespace pid;
//using namespace xcontrol;

#define MOTOR_COUNT 1
#define MAX_MOTOR_COUNT 8
#define PRINT_STATE true
#define QC_SPEED_CONVERSION 4000
#define RAD_TO_QC_CONVERSION 10000
#define JOINT56_DEPENDENCY 1    // TODO
#define JOINT56_DEPENDENT true // TODO
#define PERIOD 25  // [ms]
#define INF 1000000000


/*double current_pos[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
double target_pos[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
double target_vel[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
bool active[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
double max_current[MOTOR_COUNT] = {0.155, 4.255, 2.195, 0.135, 0.652, 0.652, 2.120};  // TODO
float step_size[MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
Epos4::control_mode_t control_mode(Epos4::position_CSP);

static const int period = 25; // [ms]
static const int security_qc = 1000;    // change this
static const float reference_step_size[MOTOR_COUNT] = {60.0*period, 100.0*period, 40.0*period, 60.0*period, 200.0*period, 200.0*period, 80.0*period};   // TODO
static const float max_qc[MOTOR_COUNT] = {2*M_PI, 2*M_PI, 1000000000, 2*M_PI, 2*M_PI, 2*M_PI, 2*M_PI};  // TODO
static const float min_qc[MOTOR_COUNT] = {-474270, 0, -1000000000, 0, 0, 0, 0}; // TODO
static const double max_velocity[MOTOR_COUNT] = {60, 60, 60, 60, 60, 60, 60};    // [rpm]    TODO
static const double reduction[MOTOR_COUNT] = {70, 70, 70, 70, 70, 70, 70};    // TODO*/



//====================================================================================================
double current_pos[MAX_MOTOR_COUNT] = {0,0};
double target_pos[MAX_MOTOR_COUNT] = {0,0};
double target_vel[MAX_MOTOR_COUNT] = {0,0};
bool active[MAX_MOTOR_COUNT] = {0,0};
double max_current[MAX_MOTOR_COUNT] = {0.155};
float step_size[MAX_MOTOR_COUNT] = {0};
Epos4::control_mode_t control_mode(Epos4::velocity_CSV);

double offset[MAX_MOTOR_COUNT] = {0};   // TODO: makes the max/min angles unusable : correct that


bool taking_commands = true;
bool resetting = false;
auto last_command_time = chrono::steady_clock::now();

static const Epos4::control_mode_t direct_control_mode = Epos4::velocity_CSV;
static const Epos4::control_mode_t autonomous_control_mode = Epos4::position_CSP;
static const double command_expiration = 200;
static const int period = 25;
static const float reference_step_size[MAX_MOTOR_COUNT] = {60.0*period};
static const float max_qc[MAX_MOTOR_COUNT] = {2*M_PI};
static const float min_qc[MAX_MOTOR_COUNT] = {-474270};
static const float max_angle[MAX_MOTOR_COUNT] = {9.77, 2.3, 411, 9.63, 7.26, INF, 0.395, INF};
static const float min_angle[MAX_MOTOR_COUNT] = {-9.6, -1.393, -259, -9.54, -0.79, -INF, -0.14, -INF};
static const double max_velocity[MAX_MOTOR_COUNT] = {5, 1, 700, 5, 6, 12, 1, 0};
//static const double reduction[MAX_MOTOR_COUNT] = {2*231, 480*16, 676.0/49.0, 2*439, 2*439, 2*231, 1*16*700, 0};
static const double reduction[MAX_MOTOR_COUNT] = {1};
static const double security_angle_coef[MAX_MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
static const vector<int> order = {1, 2, 3, 8, 4, 5, 6, 7};
//====================================================================================================

static const double CCC(131.5);


/*
adapt target velocity and position for joint 6 given the ones for joint 5 
*/
void accountForJoint56Dependency() {
    if (MOTOR_COUNT >= 6) {
        double vel = target_vel[4]/reduction[4]*reduction[5];
        target_vel[5] -= vel*JOINT56_DEPENDENCY;
        // TODO: for position_CSP and profile_position_PPM modes
    }
}


void manualCommandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (!taking_commands) return;
    last_command_time = chrono::steady_clock::now();
    float vel;
    bool empty_command = true;
    cout << "received velocity   :";
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        vel = msg->data[it];    // between -1 and 1
        empty_command = empty_command && (vel == 0);
        active[it] = (vel != 0);
        target_vel[it] = double(vel*max_velocity[it]*reduction[it]);
        step_size[it] = double(vel*reference_step_size[it]);
        cout << setw(9) << target_vel[it];
    }
    cout << endl;
    
    if (JOINT56_DEPENDENT) {
        accountForJoint56Dependency();
    }

    if (!empty_command) {
        control_mode = direct_control_mode;
    }
}


void resetCallback(const std_msgs::Bool::ConstPtr& msg) {
    taking_commands = !msg->data;
    resetting = msg->data;
}


/*
sets zero position of the arm at the current position
*/
void set_zero_position() {
    for (size_t it = 0; it < MOTOR_COUNT; it++) {
        offset[it] += current_pos[it];
    }
}


void setZeroCallback(const std_msgs::Bool::ConstPtr& msg) {
    set_zero_position();
}


void stateCommandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    cout << "================= COMMAND FROM HW INTERFACE ======================" << endl;
    last_command_time = chrono::steady_clock::now();
    control_mode = autonomous_control_mode;
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        cout << "position :     " << msg->position[it] << endl;
        target_pos[it] = double(msg->position[it]*CCC);
        target_vel[it] = double(msg->velocity[it]*CCC); //RAD_TO_QC_CONVERSION TODO: maybe I need a different conversion constant here
    }
}


/*
indicates if last received command is too old and should be ignored
*/
bool command_too_old() {
    auto now = chrono::steady_clock::now();
    return (chrono::duration_cast<chrono::milliseconds>(now-last_command_time).count() > command_expiration);
}


/*
for velocity mode, calculates an angle at which the velocity must be set to zero in order for the motor not to overshoot its limit
*/
double security_angle(double vel, size_t it) {
    // TODO
    if (vel < 0) vel = -vel;
    return vel*security_angle_coef[it];
}


/*
makes sure the motor stays in its predefined limits (in velocity mode a slight exceeding of the limit may occur)
*/
void enforce_limits(vector<xcontrol::Epos4Extended*> chain){
    for (size_t it=0; it<MOTOR_COUNT; ++it) {
        if (chain[it]->get_has_motor()) {
            switch (control_mode) {
                case Epos4::position_CSP:
                    // TODO
                    break;

                case Epos4::velocity_CSV:
                    if ((current_pos[it] > max_angle[it]-security_angle(target_vel[it], it) && target_vel[it] > 0) || (current_pos[it] < min_angle[it]+security_angle(target_vel[it], it) && target_vel[it] < 0)) {
                        target_vel[it] = 0;
                    }
                    break;

                case Epos4::profile_position_PPM:
                    if (target_pos[it] > max_angle[it]) {
                        target_pos[it] = max_angle[it];
                    }
                    if (target_pos[it] < min_angle[it]) {
                        target_pos[it] = min_angle[it];
                    }
                    break;
            }
        }
    }
}


/*
updates the target positions and velocities
TODO: decide wether to suppress this function or not
*/
void update_targets(vector<xcontrol::Epos4Extended*> chain) {
    for (size_t it=0; it<chain.size(); ++it) {
        if (chain[it]->get_has_motor()) {
            chain[it]->set_Control_Mode(control_mode);

            if (chain[it]->get_Device_State_In_String() == "Operation enable") {
                switch (control_mode) {
                    case Epos4::position_CSP:
                        // target_pos[it] += active[it] * step_size[it];
                        break;

                    case Epos4::velocity_CSV:
                        break;

                    case Epos4::profile_position_PPM:
                        // TODO:
                        break;
                }
            }
        }
    }
}


/*
Updates the target positions and velocities in order to stop the motion.
immediate stop is not guaranteed, particularly in velocity mode
*/
void stop(vector<xcontrol::Epos4Extended*> chain) {
    control_mode = Epos4::velocity_CSV;
    for (size_t it=0; it<chain.size(); ++it) {
        if (chain[it]->get_has_motor()) {
            cout << "current_pos :     " << current_pos[it] << endl;
            switch (control_mode) {
                case Epos4::position_CSP:
                    target_pos[it] = current_pos[it];
                    break;

                case Epos4::velocity_CSV:
                    target_vel[it] = 0;
                    break;

                case Epos4::profile_position_PPM:
                    target_pos[it] = current_pos[it];
                    break;
            }
        }
    }
}


double petit(size_t it, double distance) {
    static const double critical_angle[MAX_MOTOR_COUNT] = {0.087, 0.087, 10.0, 0.087, 0.087, 0.087, 0.087};
    distance = abs(distance);
    double speed = 0.5;
    if (distance > critical_angle[it]) return speed;
    double k = speed/critical_angle[it];
    return distance*k;
}


/*
updates target positions and velocities in order to lead the motor to its home (zero) position
*/
void reset_position(vector<xcontrol::Epos4Extended*> chain) {
    last_command_time = chrono::steady_clock::now();
    for (size_t it=0; it<chain.size(); ++it) {
        if (chain[it]->get_has_motor()) {
            switch (control_mode) {
                case Epos4::position_CSP:
                    // chain[it]->set_Target_Position_In_Qc(target_pos[it]);
                    // TODO:
                    break;

                case Epos4::velocity_CSV:
                    if ((current_pos[it] > -security_angle(target_vel[it], it) && target_vel[it] > 0) || (current_pos[it] < security_angle(target_vel[it], it) && target_vel[it] < 0)) {
                        target_vel[it] = 0;
                        cout << "QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ" << endl;
                    }
                    else {
                        cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
                        double dir = 1.0;
                        double distance = current_pos[it];
                        if (current_pos[it] > 0) dir = -1.0;
                        target_vel[it] = dir*petit(it, distance)*max_velocity[it]*reduction[it];
                        cout << "IIIIIIIIIIII" << target_vel[it] << "IIIIIIIIIIIIIIIII" << endl;
                    }
                    break;

                case Epos4::profile_position_PPM:
                    // TODO:
                    break; 
            }
        }
    }
    cout << "IIIIIIIIIIII" << target_vel[0] << "IIIIIIIIIIIIIIIII" << endl;
}


/*
gives the target positions and velocities to the motors
*/
void set_goals(vector<xcontrol::Epos4Extended*> chain) {
    if (command_too_old()) {
        cout << "COMMAND TOO OLD" << endl;
        stop(chain);
    }
    else if (resetting) {
        cout << "RESETTTTTTTTTTTTTTTTTTTTTTTTTTTTT" << endl;
        reset_position(chain);
    }
    else {
        update_targets(chain);
    }

    enforce_limits(chain);

    for (size_t it=0; it<chain.size(); ++it) {
        chain[it]->set_Control_Mode(control_mode);
        if (chain[it]->get_has_motor()) {
            chain[it]->set_Control_Mode(control_mode);

            if (chain[it]->get_Device_State_In_String() == "Operation enable") {
                switch (control_mode) {
                    case Epos4::position_CSP:
                        chain[it]->set_Target_Position_In_Rad(target_pos[it]);
                        break;

                    case Epos4::velocity_CSV:
                        chain[it]->set_Target_Velocity_In_Rpm(target_vel[it]);
                        cout << "set velocity       " << target_vel[it] << endl;
                        break;

                    case Epos4::profile_position_PPM:
                        // unlock axle
                        chain[it]->halt_Axle(false);
                        // Starting new positionning at receive new order (or wait finish before start new with "false state")
                        chain[it]->change_Starting_New_Pos_Config(true);
                        // normal mode (not in endless)
                        chain[it]->active_Endless_Movement(false);
                        //epos_1.active_Absolute_Positionning();
                        chain[it]->active_Relative_Positionning();
                        chain[it]->activate_Profile_Control(true);
                        //chain[it]->set_Target_Velocity_In_Rpm(target_vel[it]);
                        chain[it]->set_Target_Position_In_Rad(target_pos[it]);
                        break;
                }
            }
        }
    }
}


void definitive_stop(vector<xcontrol::Epos4Extended*> chain) {
    taking_commands = false;
    stop(chain);
    set_goals(chain);
}


int main(int argc, char **argv) {

    std::string network_interface_name("eth0");
    ros::init(argc, argv, "hd_controller_motors");
    ros::NodeHandle n;
    ros::Subscriber man_cmd_sub = n.subscribe<std_msgs::Float32MultiArray>("/arm_control/manual_cmd", 10, manualCommandCallback);
    ros::Subscriber reset_sub = n.subscribe<std_msgs::Bool>("/arm_control/reset_arm_pos", 10, resetCallback);
    ros::Subscriber set_zero_sub = n.subscribe<std_msgs::Bool>("/arm_control/set_zero_arm_pos", 10, setZeroCallback);
    ros::Subscriber state_cmd_sub = n.subscribe<sensor_msgs::JointState>("/arm_control/joint_cmd", 10, stateCommandCallback);
    ros::Publisher telem_pub = n.advertise<sensor_msgs::JointState>("/arm_control/joint_telemetry", 1000);
    ros::Publisher sim_telem_pub = n.advertise<motor_control::simJointState>("/arm_control/sim_joint_telemetry", 1000);  // for simulation only
    ros::Rate loop_rate(PERIOD);
    cout << "ROS node initialized" << endl;

    // Device definition
    // 3-axis: 1st slot next to ETHERNET-IN
    xcontrol::OneAxisSlot epos_1(true, 0x000000fb, 0x60500000);
    //xcontrol::OneAxisSlot epos_2(true, 0x000000fb, 0x65510000);
    //xcontrol::ThreeAxisSlot empty(true, 0x000000fb, 0x69500000), epos_3(false, 0x000000fb, 0x69500000), epos_4(true, 0x000000fb, 0x69500000);
    //xcontrol::ThreeAxisSlot epos_6(true, 0x000000fb, 0x69500000), epos_7(true, 0x000000fb, 0x69500000), epos_5(true, 0x000000fb, 0x69500000);

    //Epos4Extended epos_1(true);
    //epos_1.set_Id("EPOS4", 0x000000fb, 0x60500000);
    //xcontrol::OneAxisSlot epos_2(true);
    //xcontrol::ThreeAxisSlot epos_2(true), epos_3(true), epos_4(true);
    //xcontrol::ThreeAxisSlot epos_5(true), epos_6(true), epos_7(true);
    vector<xcontrol::Epos4Extended*> chain = {&epos_1};//, &epos_2, &empty, &epos_3, &epos_4, &epos_6, &epos_7, &epos_5};

    bool is_scanning = true;

    xcontrol::NetworkMaster ethercat_master(chain, network_interface_name);

	std::vector<xcontrol::Epos4Extended*> temp;
	/*for (size_t i = 0; i < chain.size(); i++) {
        temp.push_back(chain[i]);
    }
	for (size_t i = 0; i < order.size(); i++) {
        chain[order[i]-1] = temp[i];
    }*/

    cout << "BBBBBBBBBBBBBBBBBBb" << endl;
    ethercat_master.init_network();
    //Master ethercat_master;
    //EthercatBus robot;
    //ethercat_master.add_Interface_Primary(network_interface_name);
    //robot.add_Device(epos_1);
    cout << "AAAAAAAAAAAAAAAAA" << endl;
    //ethercat_master.add_Bus(robot);

    cout << "Ethercat network online" << endl;

    while (ros::ok()){
        // check device status
        ethercat_master.switch_motors_to_enable_op();
        //epos_1.switch_to_enable_op();

        if (!is_scanning) { // && taking_commands) {    TODO
            set_goals(chain);
        }

        bool wkc = ethercat_master.next_Cycle(); // Function used to launch next cycle of the EtherCat net
        if (wkc) {
            for (size_t it=0; it<chain.size(); ++it) {
                if (chain[it]->get_has_motor()) {
                    if (PRINT_STATE) {
                        cout << "Motor " << it << "\n";
                        cout << "State device : " << chain[it]->get_Device_State_In_String() << "\n";
                        cout << "Control mode = " << chain[it]->get_Control_Mode_In_String() << "\n";
                    }
                    current_pos[it] = chain[it]->get_Actual_Position_In_Rad()/reduction[it]/CCC - offset[it];
                    if (is_scanning) {
                        target_pos[it] = current_pos[it];
                    }
                    if (PRINT_STATE) { 
                        cout << "Actual position : " << std::dec << current_pos[it] << " rad" << "\n";
                        cout << "Actual velocity : " << std::dec << chain[it]->get_Actual_Velocity_In_Rpm()/reduction[it] << " rpm" << "\n";
                        cout << "Actual current value = " << chain[it]->get_Actual_Current_In_A() << "A" << "\n";
                        cout << "\n";
                    }
                }
            }
            is_scanning = false;
        } //end of valid workcounter

        sensor_msgs::JointState msg;
        motor_control::simJointState sim_msg;   // for simulation only
        for (size_t it=0; it<chain.size(); ++it) {
            msg.position.push_back(chain[it]->get_Actual_Position_In_Rad()/reduction[it]/CCC);
            msg.velocity.push_back(chain[it]->get_Actual_Velocity_In_Rpm()/reduction[it]/CCC);
            sim_msg.position[it] = chain[it]->get_Actual_Position_In_Rad()/reduction[it]/CCC;
            sim_msg.velocity[it] = chain[it]->get_Actual_Velocity_In_Rpm()/reduction[it]/CCC;
        }
        if (chain.size() < 6) {
            // populate the message with zeros if less than 6 actual motors
            for (size_t it=chain.size(); it < 6; ++it) {
                msg.position.push_back(0);
                msg.velocity.push_back(0);
                sim_msg.position[it] = 0;
                sim_msg.velocity[it] = 0;
            }
        }
        telem_pub.publish(msg);
        sim_telem_pub.publish(sim_msg);
        ros::spinOnce();
        loop_rate.sleep();
        if (PRINT_STATE) {
            cout << "\n" << endl;
        }
    }
    definitive_stop(chain);
    cout << "End program" << endl;
    return 0;
}
