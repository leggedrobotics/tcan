#pragma once


// tcan
#include <tcan/EtherCatDevice.hpp>


namespace tcan_example {


class ElmoTwitter : public tcan::EtherCatDevice {
 public:
    ElmoTwitter(const uint32_t address, const std::string& name);

    void dumpSlaveStatusInfo();
    void configureSlave();
};


} // tcan_example
