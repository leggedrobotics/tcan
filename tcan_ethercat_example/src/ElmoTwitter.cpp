// tcan ethercat
#include <tcan_ethercat/EtherCatBus.hpp>

// tcan ethercat example
#include "tcan_ethercat_example/ElmoTwitter.hpp"


namespace tcan_ethercat_example {


ElmoTwitter::ElmoTwitter(const uint32_t address, const std::string& name)
: tcan_ethercat::EtherCatSlave(address, name) {}

bool ElmoTwitter::initDevice() {
    return true;
}

bool ElmoTwitter::initializeInterface() {

    // Set state
    bus_->setStatePreOp();

    // RxPDO assignments in SM2
    sendSdoWrite(0x1c12, 0, false, uint8_t(1));
    sendSdoWrite(0x1c12, 1, true, uint16_t(0x1602));

    // RxPDO assignments in SM3
    sendSdoWrite(0x1c13, 0, false, uint8_t(3));
    sendSdoWrite(0x1c13, 1, true, uint64_t(0x1a1f1a181a03));

    // DC Sync0
    syncDistributedClocks(true);

    // Set state
    bus_->setStateSafeOp();

    sendSdoWrite(0x6060, 0, false, uint8_t(4));


    // Set state to EtherCat Operatoinal
    bus_->setStateOperational();

    // wait for all slaves to reach OP state
    bus_->waitForStateOperational();

    return true;
}


} // tcan_example
