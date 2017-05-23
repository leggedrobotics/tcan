// tcan ethercat
#include <tcan_ethercat/EtherCatBus.hpp>

// tcan ethercat example
#include "tcan_ethercat_example/Anydrive.hpp"


namespace tcan_ethercat_example {


Anydrive::Anydrive(const uint32_t address, const std::string& name)
: tcan_ethercat::EtherCatSlave(address, name) {}

void Anydrive::printStatusInfo() {
    MELO_INFO_STREAM("Status info:");
    sendSdoReadAndPrint(0x1001, 0, false);
}

bool Anydrive::initDevice() {
    return true;
}

bool Anydrive::initializeInterface() {

    // Go to state Pre Op.
    bus_->setStatePreOp();
    bus_->waitForStatePreOp();

    // RxPDO assignments in SM2
    sendSdoWrite(0x1c12, 0, false, uint8_t(1)); // TODO read only?
    sendSdoReadAndPrint(0x1c12, 0, false);

    sendSdoWrite(0x1c12, 1, true, uint16_t(0x1600)); // TODO read only?
    sendSdoReadAndPrint(0x1c12, 1, true);

    // RxPDO assignments in SM3
    sendSdoWrite(0x1c13, 0, false, uint8_t(1)); // TODO read only?
    sendSdoReadAndPrint(0x1c13, 0, false);

    sendSdoWrite(0x1c13, 1, true, uint64_t(0x1a00)); // TODO read only?
    sendSdoReadAndPrint(0x1c13, 1, true);

    // DC Sync0
    syncDistributedClocks(true);

    // Go to state Safe Op.
    bus_->setStateSafeOp();
    bus_->waitForStateSafeOp();

    sendSdoWrite(0x6060, 0, false, uint8_t(4));
    sendSdoReadAndPrint(0x6060, 0, false);
    sendSdoReadAndPrint(0x6061, 0, false);

    // Go to state Operational.
    bus_->setStateOperational();
    bus_->waitForStateOperational();

    printStatusInfo();
    return true;
}


} // tcan_example
