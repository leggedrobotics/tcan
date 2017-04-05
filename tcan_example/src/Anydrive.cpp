// tcan
#include <tcan/EtherCatBus.hpp>

// tcan example
#include "tcan_example/Anydrive.hpp"


namespace tcan_example {


Anydrive::Anydrive(const uint32_t address, const std::string& name)
: tcan::EtherCatDevice(address, name) {}

void Anydrive::printStatusInfo() {
    printf("\nDevice Errors: \n");
    sendSdoReadAndPrint(0x1001, 0, false);

    sendSdoReadAndPrint(0x1002, 0, false);

    sendSdoReadAndPrint(0x1003, 0, true);

    sendSdoReadAndPrint(0x2081, 0, false);
    sendSdoReadAndPrint(0x2081, 1, false);
    sendSdoReadAndPrint(0x2081, 2, false);
    sendSdoReadAndPrint(0x2081, 3, false);
    sendSdoReadAndPrint(0x2081, 4, false);
    sendSdoReadAndPrint(0x2081, 5, false);
    sendSdoReadAndPrint(0x2081, 6, false);

    sendSdoReadAndPrint(0x2085, 0, false);
}

bool Anydrive::initializeInterface() {

    // Set state
    bus_->setStatePreOp();

    printf("\nSetting device configuratons...\n");

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
    syncDc(true);

    // Set state
    bus_->setStateSafeOp();

    sendSdoWrite(0x6060, 0, false, uint8_t(4));
    sendSdoReadAndPrint(0x6060, 0, false);
    sendSdoReadAndPrint(0x6061, 0, false);

    // Set state to Operational
    bus_->setStateOperational();

    // send one valid process data to make outputs in slaves happy
    bus_->sendProcessData();
    bus_->receiveProcessData();

    // wait for all slaves to reach OP state
    bus_->waitForStateOperational();

    printStatusInfo();
    return true;
}


} // tcan_example
