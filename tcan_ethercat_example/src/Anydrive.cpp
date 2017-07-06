// tcan ethercat
#include <tcan_ethercat/EtherCatBus.hpp>

// tcan ethercat example
#include "tcan_ethercat_example/Anydrive.hpp"


namespace tcan_ethercat_example {


Anydrive::Anydrive(const uint32_t address, const std::string& name)
: tcan_ethercat::EtherCatSlave(address, name) {}

bool Anydrive::initDevice() {
    return true;
}

bool Anydrive::initializeInterface() {

    // Go to state Pre Op.
    bus_->setStatePreOp();
    bus_->waitForStatePreOp();

    // RxPDO assignments in SM2
    sendSdoWrite(0x1c12, 0, false, uint8_t(1));
    sendSdoWrite(0x1c12, 1, true, uint16_t(0x1600));

    // RxPDO assignments in SM3
    sendSdoWrite(0x1c13, 0, false, uint8_t(1));
    sendSdoWrite(0x1c13, 1, true, uint64_t(0x1a00));

    // DC Sync0
    syncDistributedClocks(true);

    // Go to state Safe Op.
    bus_->setStateSafeOp();
    bus_->waitForStateSafeOp();

    sendSdoWrite(0x6060, 0, false, uint8_t(4));

    // Go to state Operational.
    bus_->setStateOperational();
    bus_->waitForStateOperational();

    return true;
}


} // tcan_example
