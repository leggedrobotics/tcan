// tcan ethercat
#include <tcan_ethercat/EtherCatBus.hpp>

// tcan ethercat example
#include "tcan_ethercat_example/ElmoTwitter.hpp"


namespace tcan_ethercat_example {


ElmoTwitter::ElmoTwitter(const uint32_t address, const std::string& name)
: tcan_ethercat::EtherCatSlave(address, name) {}

void ElmoTwitter::printStatusInfo() {
    MELO_INFO_STREAM("Status info:");
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

bool ElmoTwitter::initDevice() {
    return true;
}

bool ElmoTwitter::initializeInterface() {

    // Set state
    bus_->setStatePreOp();

    // RxPDO assignments in SM2
    sendSdoWrite(0x1c12, 0, false, uint8_t(1));
    sendSdoReadAndPrint(0x1c12, 0, false);

    sendSdoWrite(0x1c12, 1, true, uint16_t(0x1602));
    sendSdoReadAndPrint(0x1c12, 1, true);

    // RxPDO assignments in SM3
    sendSdoWrite(0x1c13, 0, false, uint8_t(3));
    sendSdoReadAndPrint(0x1c13, 0, false);

    sendSdoWrite(0x1c13, 1, true, uint64_t(0x1a1f1a181a03));
    sendSdoReadAndPrint(0x1c13, 1, false);
    sendSdoReadAndPrint(0x1c13, 2, false);
    sendSdoReadAndPrint(0x1c13, 3, false);

    // bufferu16 = 0x1a03; // position + velocity feedback
    // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 1, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x1c13, 1, TRUE);
    //
    // bufferu16 = 0x1a18; // DC bus voltage
    // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 2, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x1c13, 2, TRUE);
    //
    // bufferu16 = 0x1a1f; // motor current
    // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x1c13, 3, TRUE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x1c13, 3, TRUE);

    // DC Sync0
    syncDistributedClocks(true);

    // // enable voltage + quick-stop + fault reset
    // controlword.bits.quick_stop = 1;
    // controlword.bits.enable_voltage = 1;
    // controlword.bits.fault_reset = 1;
    // bufferu16 = controlword.all;
    // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x6040, 0, FALSE, 2, &bufferu16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x6040, 0, FALSE);
    // ecatcomm_slave_check_sdo(0x6041, 0, FALSE);

    // Set state
    bus_->setStateSafeOp();

    // // Unkown config
    // buffers16 = 0x5f;
    // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x3034, 7, FALSE, 2, &buffers16, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x3034, 7, FALSE);
    //
    // // Halp Option Code
    // bufferu8 = 3;
    // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x605D, 0, TRUE, 1, &bufferu8, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x605D, 0, TRUE);

    // buffers8 = 3;
    // wkc_ = ecx_SDOwrite(&ecatContext_, 1, 0x605D, 0, FALSE, 1, &buffers8, EC_TIMEOUTRXM);
    // ecatcomm_slave_check_sdo(0x605D, 0, FALSE);

    sendSdoWrite(0x6060, 0, false, uint8_t(4));
    sendSdoReadAndPrint(0x6060, 0, false);
    sendSdoReadAndPrint(0x6061, 0, false);

    // Set initial process data - SWITCH ON
    // outdata.controlword.all = 0;
    // outdata.controlword.bits.enable_voltage = 1;
    // outdata.controlword.bits.quick_stop = 1;
    // outdata.controlword.bits.fault_reset = 1;
    // for (i=0; i<1000; i++)
    // {
    //     ecatcomm_slave_set_rxpdo(&outdata, SWITCH_ON, 0.0);
    //     ecx_send_processdata(&ecatContext_);
    //     ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
    //     ecatcomm_slave_get_txpdo(&indata);
    //     ecatcomm_slave_print_controlword(controlword);
    //     ecatcomm_slave_print_statusword(indata.statusword);
    //     osal_usleep(1000);
    // }

    // Set state to EtherCat Operatoinal
    bus_->setStateOperational();

    // wait for all slaves to reach OP state
    bus_->waitForStateOperational();

    // Get error data
    printStatusInfo();

    return true;
}


} // tcan_example
