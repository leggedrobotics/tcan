/*
 * PcanBus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

//#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
//#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include <libpcan.h>

#include "tcan/PcanBus.hpp"

#include "message_logger/message_logger.hpp"
#include <sstream>

namespace tcan {

PcanBus::PcanBus(const std::string& interface):
    PcanBus(new PcanBusOptions(interface))
{
}

PcanBus::PcanBus(PcanBusOptions* options):
    CanBus(options),
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
    const PcanBusOptions* options = static_cast<const PcanBusOptions*>(options_);
    const char* interface = options->name_.c_str();

    /* open CAN */
    char buffer[256];
    int port = atoi( interface[3]);
    handle_ = CAN_Open(HW_PCI, port);
    if  (handle_ == NULL) {
      MELO_FATAL("Opening CAN channel %s (port: %d) failed!", interface, port);
      return false;
    }
    DWORD status = CAN_Init(handle_, CAN_BAUD_1M, CAN_INIT_TYPE_ST);

    MELO_INFO("Opened CAN %s.", interface);

    return true;
}


bool PcanBus::readData() {

  DWORD status;
  TPCANMsg msg;

  /* wait for a CAN 2.0 msg received from the CAN channel
  * (the function may block)
  */
  status = CAN_Read(handle_, &msg);

  if(true) {
    //		printf("read nothing\n");
    return false;
  }
  else {
//		printf("CanManager:bus_routine: Data received from iBus %i, n. Bytes: %i \n", iBus, bytes_read);


    can_frame frame;
    frame.can_id = msg.ID;
    frame.can_dlc = msg.LEN;
    std::copy(msg.DATA, &(msg.DATA[msg.can_dlc]), frame.data);

    if(frame.can_id > CAN_ERR_FLAG && frame.can_id < CAN_RTR_FLAG) {
        handleBusError( frame );
    }
    else{
        handleMessage( CanMsg(frame.can_id, frame.can_dlc, frame.data) );
    }
  }

    return true;
}


bool PcanBus::writeData(const CanMsg& cmsg) {

    /* CAN 2.0 channel handle */
    DWORD status;
    TPCANMsg msg;
    msg.ID = = cmsg.getCobId();
    msg.MSGTYPE = MSGTYPE_STANDARD;
    msg.LEN = cmsg.getLength();
    std::copy(cmsg.getData(), &(cmsg.getData()[msg.LEN]), msg.DATA);

    /* write standard msg
    * (the function may block)
    */
    status = CAN_Write(handle_, &msg);

    return true;
}

void PcanBus::handleBusError(const can_frame& msg) {

    // todo: really ignore arbitration lost?
    //if(msg.can_id & CAN_ERR_LOSTARB) {
    //    return;
    //}

    busErrorFlag_ = true;
    std::stringstream errorMsg;
    errorMsg << "received bus error frame on bus " << options_->name_ << " (";
    for (int i = 0; i < 8; i++)
      errorMsg << " " << int(msg.data[i]);
    errorMsg << ")";
    // cob id
    if(msg.can_id & CAN_ERR_TX_TIMEOUT) {
      errorMsg << "  TX timeout (by netdevice driver)";
    }
    if(msg.can_id & CAN_ERR_LOSTARB) {
      errorMsg << "   lost arbitration";
    }
    if(msg.can_id & CAN_ERR_CRTL) {
      errorMsg << "   controller problems";
    }
    if(msg.can_id & CAN_ERR_PROT) {
      errorMsg << "   protocol violations";
    }
    if(msg.can_id & CAN_ERR_TRX) {
      errorMsg << "   transceiver status";
    }
    if(msg.can_id & CAN_ERR_ACK) {
      errorMsg << "   received no ACK on transmission";
    }
    if(msg.can_id & CAN_ERR_BUSOFF) {
      errorMsg << "   bus off";
    }
    if(msg.can_id & CAN_ERR_BUSOFF) {
      errorMsg << "   bus error (may flood!)";
    }
    if(msg.can_id & CAN_ERR_RESTARTED) {
      errorMsg << "   controller restarted";
    }


    // bit 0
    errorMsg << "    bit number in bitstream: " <<  ((int)msg.data[0]);

    // bit 1
    switch(msg.data[1]) {
        default: // fall-through!
        case CAN_ERR_CRTL_UNSPEC:
          errorMsg << "    error status of CAN-controller: unspecified";
            break;

        case CAN_ERR_CRTL_RX_OVERFLOW:
          errorMsg << "    error status of CAN-controller: rx buffer overflow";
            break;

        case CAN_ERR_CRTL_TX_OVERFLOW:
          errorMsg << "    error status of CAN-controller: tx buffer overflow";
            break;

        case CAN_ERR_CRTL_RX_WARNING:
          errorMsg << "    error status of CAN-controller: reached warning level for RX errors";
            break;

        case CAN_ERR_CRTL_TX_WARNING:
          errorMsg << "    error status of CAN-controller: reached warning level for TX errors";
            break;

        case CAN_ERR_CRTL_RX_PASSIVE:
          errorMsg << "    error status of CAN-controller: reached error passive status RX";
            break;

        case CAN_ERR_CRTL_TX_PASSIVE:
          errorMsg << "    error status of CAN-controller: reached error passive status TX";
            break;
    }

    // bit 2
    switch(msg.data[2]) {
        default: // fall-through!
        case CAN_ERR_PROT_UNSPEC:
            errorMsg << "    error in CAN protocol (type): unspecified";
            break;

        case CAN_ERR_PROT_BIT:
            errorMsg << "    error in CAN protocol (type): single bit error";
            break;

        case CAN_ERR_PROT_FORM:
            errorMsg << "    error in CAN protocol (type): frame format error";
            break;

        case CAN_ERR_PROT_STUFF:
            errorMsg << "    error in CAN protocol (type): bit stuffing error";
            break;

        case CAN_ERR_PROT_BIT0:
            errorMsg << "    error in CAN protocol (type): unable to send dominant bit";
            break;

        case CAN_ERR_PROT_BIT1:
            errorMsg << "    error in CAN protocol (type): unable to send recessive bit";
            break;

        case CAN_ERR_PROT_OVERLOAD:
            errorMsg << "    error in CAN protocol (type): bus overload";
            break;

        case CAN_ERR_PROT_ACTIVE:
            errorMsg << "    error in CAN protocol (type): active error announcement";
            break;

        case CAN_ERR_PROT_TX:
            errorMsg << "    error in CAN protocol (type): error occurred on transmission";
            break;
    }

    // bit 3
    switch(msg.data[3]) {
        default:
        case CAN_ERR_PROT_LOC_UNSPEC:
            errorMsg << "    error in CAN protocol (location): unspecified";
            break;

        case CAN_ERR_PROT_LOC_SOF:
            errorMsg << "    error in CAN protocol (location): start of frame";
            break;

        case CAN_ERR_PROT_LOC_ID28_21:
            errorMsg << "    error in CAN protocol (location): ID bits 28 - 21 (SFF: 10 - 3)";
            break;

        case CAN_ERR_PROT_LOC_ID20_18:
            errorMsg << "    error in CAN protocol (location): ID bits 20 - 18 (SFF: 2 - 0 )";
            break;

        case CAN_ERR_PROT_LOC_SRTR:
            errorMsg << "    error in CAN protocol (location): substitute RTR (SFF: RTR)";
            break;

        case CAN_ERR_PROT_LOC_IDE:
            errorMsg << "    error in CAN protocol (location): identifier extension";
            break;

        case CAN_ERR_PROT_LOC_ID17_13:
            errorMsg << "    error in CAN protocol (location): ID bits 17-13";
            break;

        case CAN_ERR_PROT_LOC_ID12_05:
            errorMsg << "    error in CAN protocol (location): ID bits 12-5";
            break;

        case CAN_ERR_PROT_LOC_ID04_00:
            errorMsg << "    error in CAN protocol (location): ID bits 4-0";
            break;

        case CAN_ERR_PROT_LOC_RTR:
            errorMsg << "    error in CAN protocol (location): RTR";
            break;

        case CAN_ERR_PROT_LOC_RES1:
            errorMsg << "    error in CAN protocol (location): reserved bit 1";
            break;

        case CAN_ERR_PROT_LOC_RES0:
            errorMsg << "    error in CAN protocol (location): reserved bit 0";
            break;

        case CAN_ERR_PROT_LOC_DLC:
            errorMsg << "    error in CAN protocol (location): data length code";
            break;

        case CAN_ERR_PROT_LOC_DATA:
            errorMsg << "    error in CAN protocol (location): data section";
            break;

        case CAN_ERR_PROT_LOC_CRC_SEQ:
            errorMsg << "    error in CAN protocol (location): CRC sequence";
            break;

        case CAN_ERR_PROT_LOC_CRC_DEL:
            errorMsg << "    error in CAN protocol (location): CRC delimiter";
            break;

        case CAN_ERR_PROT_LOC_ACK:
            errorMsg << "    error in CAN protocol (location): ACK slot";
            break;

        case CAN_ERR_PROT_LOC_ACK_DEL:
            errorMsg << "    error in CAN protocol (location): ACK delimiter";
            break;

        case CAN_ERR_PROT_LOC_EOF:
            errorMsg << "    error in CAN protocol (location): end of frame";
            break;

        case CAN_ERR_PROT_LOC_INTERM:
            errorMsg << "    error in CAN protocol (location): intermission";
            break;
    }

    // bit 4
    switch(msg.data[4]) {
        default:
        case CAN_ERR_TRX_UNSPEC:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_UNSPEC";
            break;

        case CAN_ERR_TRX_CANH_NO_WIRE:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANH_NO_WIRE";
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_BAT:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_BAT";
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_VCC:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_VCC";
            break;

        case CAN_ERR_TRX_CANH_SHORT_TO_GND:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANH_SHORT_TO_GND";
            break;

        case CAN_ERR_TRX_CANL_NO_WIRE:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANL_NO_WIRE";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_BAT:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_BAT";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_VCC:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_VCC";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_GND:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_GND";
            break;

        case CAN_ERR_TRX_CANL_SHORT_TO_CANH:
            errorMsg << "    error status of CAN-transceiver: CAN_ERR_TRX_CANL_SHORT_TO_CANH";
            break;
    }

    // bit 5-7
    errorMsg << "    controller specific additional information: " << ((int)msg.data[5]) << " " <<  ((int)msg.data[6]) << " " <<  ((int)msg.data[7]);

    MELO_ERROR_THROTTLE_STREAM(0.5, errorMsg.str());
}

} /* namespace tcan */
