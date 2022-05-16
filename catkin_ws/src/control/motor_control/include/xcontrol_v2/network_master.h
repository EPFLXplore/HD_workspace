#pragma once
#include <ethercatcpp/master.h>
#include <ethercatcpp/soem_master_pimpl.h>
#include "epos4_extended.h"

#include <string>
#include <vector>

namespace xcontrol {

class NetworkMaster: public ethercatcpp::Master {
public:

    NetworkMaster(std::vector<Epos4Extended*> chain, std::string network_interface_name);

    void init_network();
    void switch_motors_to_enable_op();

private:

    std::vector<Epos4Extended*> epos_chain_;
    std::string network_interface_name_;
};

}
