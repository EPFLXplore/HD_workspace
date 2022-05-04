#include <xcontrol_v2/network_master.h>

#include <iostream>

using namespace std;
using namespace ethercatcpp;

namespace xcontrol {

NetworkMaster::NetworkMaster(vector<Epos4Extended*> chain, std::string network_interface_name) : 
	Master(),
	epos_chain_(chain),
	network_interface_name_(network_interface_name) { }

void NetworkMaster::init_network() {
  	// Bus creation
  	EthercatBus robot;

  	// Adding network interface
  	add_Interface_Primary(network_interface_name_);

    for (size_t i = 0; i < epos_chain_.size(); i++) {
        robot.add_Device(*(epos_chain_[i]));
    }
	
  	add_Bus(robot);
}

void NetworkMaster::switch_motors_to_enable_op() {
    for (size_t i = 0; i < epos_chain_.size(); i++) {
        epos_chain_[i]->switch_to_enable_op();
    }
}


}