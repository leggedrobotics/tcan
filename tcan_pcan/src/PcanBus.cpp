/*
 * PcanBus.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: Christian Gehring
 */


#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

//#include <string.h>
//#include <sys/ioctl.h>
//#include <unistd.h>
#include <fcntl.h>

#include "tcan_pcan/PcanBus.hpp"

#include "message_logger/message_logger.hpp"
#include <sstream>

namespace tcan {

PcanBus::PcanBus(const std::string& interface):
    PcanBus(std::unique_ptr<PcanBusOptions>(new PcanBusOptions(interface)))
{
}

PcanBus::PcanBus(std::unique_ptr<PcanBusOptions>&& options):
    CanBus(std::move(options)),
    handle_(NULL)
{
}

PcanBus::~PcanBus()
{
    stopThreads();
    CAN_Close(handle_);
}

bool PcanBus::initializeInterface()
{
    const PcanBusOptions* options = static_cast<const PcanBusOptions*>(options_.get());
    const char* interface = options->name_.c_str();
    MELO_INFO("Reading device: %s", interface);

//    int port = atoi(options->name_.substr(3,1).c_str());
//    MELO_INFO("Opening CAN channel %s (port: %d) ", interface, port);
//    std::string szDevNode;
//    __u16 wIrq = 0;
//    __u32 dwPort = 0;
//    switch (port) {
//      case 0:
//        dwPort = 0;
//        wIrq = 0;
//        szDevNode = "/dev/pcanpci0";
//        break;
//      case 1:
//        dwPort = 1;
//        wIrq = 1;
//        szDevNode = "/dev/pcanpci1";
//        break;
//      case 2:
//        dwPort = 0;
//        wIrq = 0;
//        szDevNode = "/dev/pcanpci2";
//        break;
//      case 3:
//        dwPort = 1;
//        wIrq = 1;
//        szDevNode = "/dev/pcanpci3";
//        break;
//      default:
//        MELO_ERROR("Port: %d is unknown", port);
//    }
//    handle_ = LINUX_CAN_Open(szDevNode.c_str(), O_RDWR);

    handle_ = LINUX_CAN_Open(options->name_.c_str(), O_RDWR);


    if  (handle_ == NULL) {
//      MELO_FATAL("Opening CAN channel %s (port: %d) failed!", interface, port);
      MELO_FATAL("Opening CAN %s failed!", interface);
      return false;
    }
    DWORD status = CAN_Init(handle_, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
    if (status != CAN_ERR_OK) {
      MELO_FATAL("Could not initialize CAN %s.", interface);
      return false;
    }

    MELO_INFO("Opened CAN %s.", interface);

    return true;
}


bool PcanBus::readData() {

  DWORD status;
  TPCANRdMsg msg;

//  MELO_INFO("readData now read the shit from mother  %s", options_->name_.c_str());

  for (int i=0; i<12; i++) {
    status = LINUX_CAN_Read_Timeout(handle_, &msg, 20);

//  MELO_INFO("DONE readData now read the shit from mother  %s", options_->name_.c_str());
//  status = LINUX_CAN_Read(handle_, &msg);
//  MELO_INFO("Received CAN message on bus %s COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
//            options_->name_.c_str(), msg.Msg.ID, msg.Msg.DATA[1], msg.Msg.DATA[0], msg.Msg.DATA[0], msg.Msg.DATA[1], msg.Msg.DATA[2], msg.Msg.DATA[3], msg.Msg.DATA[4], msg.Msg.DATA[5], msg.Msg.DATA[6], msg.Msg.DATA[7]);

//    MELO_INFO("Received CAN message on bus %s COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
//              options_->name_.c_str(), msg.ID, msg.DATA[1], msg.DATA[0], msg.DATA[0], msg.DATA[1], msg.DATA[2], msg.DATA[3], msg.DATA[4], msg.DATA[5], msg.DATA[6], msg.DATA[7]);


  if(status != CAN_ERR_OK) {
//    MELO_ERROR("PcanBus::readData: read nothing");
//    return false;
  }
  else {
//		printf("CanManager:bus_routine: Data received from iBus %i, n. Bytes: %i \n", iBus, bytes_read);

    can_frame frame;
    frame.can_id = msg.Msg.ID;
    frame.can_dlc = msg.Msg.LEN;
    std::copy(msg.Msg.DATA, &(msg.Msg.DATA[ msg.Msg.LEN]), frame.data);


    if(frame.can_id > CAN_ERR_FLAG && frame.can_id < CAN_RTR_FLAG) {
        handleBusError( frame );
    }
    else{
        handleMessage( CanMsg(frame.can_id, frame.can_dlc, frame.data) );
    }
  }
  }
    return false;
}


bool PcanBus::writeData(const CanMsg& cmsg) {
    DWORD status;
    TPCANMsg msg;
    msg.ID = cmsg.getCobId();
    msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.LEN = cmsg.getLength();
    std::copy(cmsg.getData(), &(cmsg.getData()[msg.LEN]), msg.DATA);

    status = LINUX_CAN_Write_Timeout(handle_, &msg, 20);

//    if(status != CAN_ERR_OK) {
//      return false;
//    }
    return true;
}

void PcanBus::handleBusError(const can_frame& msg) {

    // todo: really ignore arbitration lost?
    //if(msg.can_id & CAN_ERR_LOSTARB) {
    //    return;
    //}

    busErrorFlag_ = true;

    if(static_cast<const CanBusOptions*>(options_.get())->passivateOnBusError_) {
        passivate();
        MELO_WARN("Bus error on bus %s. This bus is now PASSIVE!", options_->name_.c_str());
    }

    std::stringstream errorMsg;
    errorMsg << "received bus error frame on bus " << options_->name_ << " (";
    for (int i = 0; i < 8; i++)
      errorMsg << " " << int(msg.data[i]);
    errorMsg << ")";
    // cob id
    if(msg.can_id & CAN_ERR_TX_TIMEOUT) {
      errorMsg << "TX timeout (by netdevice driver). ";
    }
    if(msg.can_id & CAN_ERR_LOSTARB) {
      errorMsg << "lost arbitration. ";
    }
    if(msg.can_id & CAN_ERR_CRTL) {
      errorMsg << "controller problems. ";
    }
    if(msg.can_id & CAN_ERR_PROT) {
      errorMsg << "protocol violations. ";
    }
    if(msg.can_id & CAN_ERR_TRX) {
      errorMsg << "transceiver status. ";
    }
    if(msg.can_id & CAN_ERR_ACK) {
      errorMsg << "received no ACK on transmission. ";
    }
    if(msg.can_id & CAN_ERR_BUSOFF) {
      errorMsg << "bus off. ";
    }
    if(msg.can_id & CAN_ERR_BUSOFF) {
      errorMsg << "bus error (may flood!). ";
    }
    if(msg.can_id & CAN_ERR_RESTARTED) {
      errorMsg << "controller restarted. ";
    }

    // bit 0
    errorMsg << "bit number in bitstream: 0x" <<  std::hex << static_cast<int>(msg.data[0]);

    // bit 1
    switch(msg.data[1]) {
        default: // fall-through!
        case CAN_ERR_CRTL_UNSPEC:
          errorMsg << " / error status of CAN-controller: unspecified";
            break;

        case CAN_ERR_CRTL_RX_OVERFLOW:
          errorMsg << " / error status of CAN-controller: rx buffer overflow";
            break;

        case CAN_ERR_CRTL_TX_OVERFLOW:
          errorMsg << " / error status of CAN-controller: tx buffer overflow";
            break;

        case CAN_ERR_CRTL_RX_WARNING:
          errorMsg << " / error status of CAN-controller: reached warning level for RX errors";
            break;

        case CAN_ERR_CRTL_TX_WARNING:
          errorMsg << " / error status of CAN-controller: reached warning level for TX errors";
            break;

        case CAN_ERR_CRTL_RX_PASSIVE:
          errorMsg << " / error status of CAN-controller: reached error passive status RX";
            break;

        case CAN_ERR_CRTL_TX_PASSIVE:
          errorMsg << " / error status of CAN-controller: reached error passive status TX";
            break;
    }

    // bit 2
    switch(msg.data[2]) {
        default: // fall-through!
        case CAN_ERR_PROT_UNSPEC:
            errorMsg << " / error in CAN protocol (type): unspecified";
            break;

        case CAN_ERR_PROT_BIT:
            errorMsg << " / error in CAN protocol (type): single bit error";
            break;

        case CAN_ERR_PROT_FORM:
            errorMsg << " / error in CAN protocol (type): frame format error";
            break;

        case CAN_ERR_PROT_STUFF:
            errorMsg << " / error in CAN protocol (type): bit stuffing error";
            break;

        case CAN_ERR_PROT_BIT0:
            errorMsg << " / error in CAN protocol (type): unable to send dominant bit";
            break;

        case CAN_ERR_PROT_BIT1:
            errorMsg << " / error in CAN protocol (type): unable to send recessive bit";
            break;

        case CAN_ERR_PROT_OVERLOAD:
            errorMsg << " / error in CAN protocol (type): bus overload";
            break;

        case CAN_ERR_PROT_ACTIVE:
            errorMsg << " / error in CAN protocol (type): active error announcement";
            break;

        case CAN_ERR_PROT_TX:
            errorMsg << " / error in CAN protocol (type): error occurred on transmission";
            break;
    }

    // bit 3
    switch(msg.data[3]) {
        default:
        case CAN_ERR_PROT_LOC_UNSPEC:
            errorMsg << " / error in CAN protocol (location): unspecified";
            break;

        case CAN_ERR_PROT_LOC_SOF:
            errorMsg << " / error in CAN protocol (location): start of frame";
            break;

        case CAN_ERR_PROT_LOC_ID28_21:
            errorMsg << " / error in CAN protocol (location): ID bits 28 - 21 (SFF: 10 - 3)";
            break;

        case CAN_ERR_PROT_LOC_ID20_18:
            errorMsg << " / error in CAN protocol (location): ID bits 20 - 18 (SFF: 2 - 0 )";
            break;

        case CAN_ERR_PROT_LOC_SRTR:
            errorMsg << " / error in CAN protocol (location): substitute RTR (SFF: RTR)";
            break;

        case CAN_ERR_PROT_LOC_IDE:
            errorMsg << " / error in CAN protocol (location): identifier extension";
            break;

        case CAN_ERR_PROT_LOC_ID17_13:
            errorMsg << " / error in CAN protocol (location): ID bits 17-13";
            break;

        case CAN_ERR_PROT_LOC_ID12_05:
            errorMsg << " / error in CAN protocol (location): ID bits 12-5";
            break;

        case CAN_ERR_PROT_LOC_ID04_00:
            errorMsg << " / error in CAN protocol (location): ID bits 4-0";
            break;

        case CAN_ERR_PROT_LOC_RTR:
            errorMsg << " / error in CAN protocol (location): RTR";
            break;

        case CAN_ERR_PROT_LOC_RES1:
            errorMsg << " / error in CAN protocol (location): reserved bit 1";
            break;

        case CAN_ERR_PROT_LOC_RES0:
            errorMsg << " / error in CAN protocol (location): reserved bit 0";
            break;

        case CAN_ERR_PROT_LOC_DLC:
            errorMsg << " / error in CAN protocol (location): data length code";
            break;

        case CAN_ERR_PROT_LOC_DATA:
            errorMsg << " / error in CAN protocol (location): data section";
            break;

        case CAN_ERR_PROT_LOC_CRC_SEQ:
            errorMsg << " / error in CAN protocol (location): CRC sequence";
            break;

        case CAN_ERR_PROT_LOC_CRC_DEL:
            errorMsg << " / error in CAN protocol (location): CRC delimiter";
            break;

        case CAN_ERR_PROT_LOC_ACK:
            errorMsg << " / error in CAN protocol (location): ACK slot";
            break;

        case CAN_ERR_PROT_LOC_ACK_DEL:
            errorMsg << " / error in CAN protocol (location): ACK delimiter";
            break;

        case CAN_ERR_PROT_LOC_EOF:
            errorMsg << " / error in CAN protocol (location): end of frame";
            break;

        case CAN_ERR_PROT_LOC_INTERM:
            errorMsg << " / error in CAN protocol (location): intermission";
            break;
    }

    // bit 4
    switch(msg.data[4]) {
        default:
        case CAN_ERR_TRX_UNSPEC:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_UNSPEC";
            break;

        case CAN_ERR_TRX_CANH_NO_WIRE:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_NO_WIRE";
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_BAT:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_BAT";
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_VCC:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_VCC";
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_GND:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_GND";
            break;

        case CAN_ERR_TRX_CANL_NO_WIRE:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_NO_WIRE";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_BAT:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_BAT";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_VCC:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_VCC";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_GND:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_GND";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_CANH:
            errorMsg << " / error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_CANH";
            break;
    }

    // bit 5-7
    errorMsg << " / controller specific additional information: 0x" << std::hex << static_cast<int>(msg.data[5]) << " 0x" <<  std::hex << static_cast<int>(msg.data[6]) << " 0x" <<  std::hex << static_cast<int>(msg.data[7]);

    MELO_ERROR_THROTTLE_STREAM(static_cast<const PcanBusOptions*>(options_.get())->canErrorThrottleTime_, errorMsg.str());
}

} /* namespace tcan */
