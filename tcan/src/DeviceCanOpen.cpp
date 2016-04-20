/*
 * DeviceCanOpen.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include <stdio.h>

#include "tcan/DeviceCanOpen.hpp"
#include "tcan/Bus.hpp"

namespace tcan {

DeviceCanOpen::DeviceCanOpen(const uint32_t nodeId, const std::string& name):
    DeviceCanOpen(new DeviceCanOpenOptions(nodeId, name))
{

}

DeviceCanOpen::DeviceCanOpen(DeviceCanOpenOptions* options):
    Device(options),
    nmtState_(NMTStates::initializing),
    sdoTimeoutCounter_(0),
    sdoSentCounter_(0),
    sdoMsgsMutex_(),
    sdoMsgs_()
{

}

DeviceCanOpen::~DeviceCanOpen()
{

}

bool DeviceCanOpen::sanityCheck() {
    if(!isMissing() && !checkDeviceTimeout()) {
        nmtState_ = NMTStates::missing;
        printf("Device %s timed out!\n", getName().c_str());
        return false;
    }

    if(!checkSdoTimeout()) {

    }
    return true;

}


bool DeviceCanOpen::parseHeartBeat(const CanMsg& cmsg) {

    deviceTimeoutCounter_ = 0;

    if(cmsg.getLength() != 1) {
        printf("Invalid Heartbeat message length from nodeId %x: %d\n", getNodeId(), cmsg.getLength());
        return false;
    }

    if(isInitializing()) {
        configureDevice();
    }

    switch(cmsg.readuint8(0)) {
        case 0x0: // boot up
            nmtState_ = NMTStates::preOperational; // devices should switch automatically to pre-operational after bootup
            break;

        case 0x7F: // pre-operational
            nmtState_ = NMTStates::preOperational;
            break;

        case 0x04: // stopped
            nmtState_ = NMTStates::stopped;
            break;

        case 0x05: // operational
            nmtState_ = NMTStates::operational;
            break;

        default:
            printf("Invalid Heartbeat message data from nodeId %x: %x\n", getNodeId(), cmsg.readuint8(0));
            return false;
            break;
    }

    return true;
}

bool DeviceCanOpen::parseSDOAnswer(const CanMsg& cmsg) {
    const uint8_t responseMode = cmsg.readuint8(0);
    const uint16_t index = cmsg.readuint16(1);
    const uint8_t subindex = cmsg.readuint8(3);

    if(sdoMsgs_.size() != 0) {
        std::lock_guard<std::mutex> guard(sdoMsgsMutex_); // lock sdoMsgsMutex_ to prevent checkSdoTimeout() from making changes on sdoMsgs_
        const SdoMsg& sdo = sdoMsgs_.front();

        if(sdo.getIndex() == index && sdo.getSubIndex() == subindex) {

            if(responseMode == 0x43 || responseMode == 0x4B || responseMode == 0x4F) { // read responses (4, 2 or 1 byte)
                handleReadSdoAnswer( dynamic_cast<const SdoMsg&>(cmsg) );
            }else if(responseMode == 0x80) { // error response
                const int32_t error = cmsg.readint32(4);
                printf("Received SDO error: %s. COB=%x / index=%x / subindex=%x / error=%x\n", SdoMsg::getErrorName(error).c_str(), cmsg.getCobId(), index, subindex, error);
                // todo: further error handling
            }

            sendNextSdo();

            return true;
        }
    }

    printf("Received unexpected SDO answer. COB=%x / index=%x / subindex=%x / data=%x\n", cmsg.getCobId(), index, subindex, cmsg.readuint32(4));
    return false;
}


void DeviceCanOpen::sendSdo(const SdoMsg& sdoMsg) {

    std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
    sdoMsgs_.push(sdoMsg);

    if(sdoMsgs_.size() == 1) {
        // sdo queue was empty before, so put the new message in the bus output queue
        bus_->sendMessage(sdoMsgs_.front());

        if(!sdoMsg.getRequiresAnswer()) {
            sdoMsgs_.pop();
        }
    }
}

bool DeviceCanOpen::checkSdoTimeout() {
    const DeviceCanOpenOptions* options = static_cast<const DeviceCanOpenOptions*>(options_);

    if(options->maxSdoTimeoutCounter != 0 && sdoMsgs_.size() != 0 && (sdoTimeoutCounter_++ > options->maxSdoTimeoutCounter) ) {
        // sdoTimeoutCounter_ is only increased if options_->maxSdoTimeoutCounter != 0 and sdoMsgs_.size() != 0

        std::lock_guard<std::mutex> guard(sdoMsgsMutex_); // lock sdoMsgsMutex_ to prevent parseSDOAnswer from making changes on sdoMsgs_
        const SdoMsg& msg = sdoMsgs_.front();
        if(sdoSentCounter_ > options->sdoSendTries) {
            printf("Device %s: SDO timeout (COB=%x / index=%x / sub-index=%x / data=%x)\n", getName().c_str(), msg.getCobId(), msg.getIndex(), msg.getSubIndex(), msg.readuint32(4));

            sdoMsgs_.pop();

            sendNextSdo();

            return false;
        }else{
            sdoSentCounter_++;

            bus_->sendMessage( msg );
        }
    }

    return true;
}

void DeviceCanOpen::sendNextSdo() {
    sdoTimeoutCounter_ = 0;
    sdoSentCounter_ = 0;

    // put next SDO message(s) into the bus output queue
    while(sdoMsgs_.size() > 0) {
        bus_->sendMessage( sdoMsgs_.front() );

        if(!sdoMsgs_.front().getRequiresAnswer()) {
            sdoMsgs_.pop(); // if sdo requires no answer (e.g. NMT state requests), pop it from the SDO queue and proceed to the next SDO
        }else{
            break; // if SDO requires answer, wait for it
        }
    }

}

void DeviceCanOpen::setNmtEnterPreOperational() {
    {
        std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
        // swap with an empty queue to clear it
        std::queue<SdoMsg>().swap(sdoMsgs_);
    }
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x80) );
    // todo: wait some time?

    // the remote device will not tell us in which state it is if heartbeat message is disabled
    //   => assume that the state switch will be successful
    if(static_cast<const DeviceCanOpenOptions*>(options_)->producerHeartBeatTime == 0) {
        nmtState_ = NMTStates::preOperational;
    }
}

void DeviceCanOpen::setNmtStartRemoteDevice() {
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x1) );

    // the remote device will not tell us in which state it is if heartbeat message is disabled
    //   => assume that the state switch will be successful
    if(static_cast<const DeviceCanOpenOptions*>(options_)->producerHeartBeatTime == 0) {
        nmtState_ = NMTStates::operational;
    }
}

void DeviceCanOpen::setNmtStopRemoteDevice() {
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x2) );

    // the remote device will not tell us in which state it is if heartbeat message is disabled
    //   => assume that the state switch will be successful
    if(static_cast<const DeviceCanOpenOptions*>(options_)->producerHeartBeatTime == 0) {
        nmtState_ = NMTStates::stopped;
    }
}

void DeviceCanOpen::setNmtResetRemoteCommunication() {
    {
        std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
        // swap with an empty queue to clear it
        std::queue<SdoMsg>().swap(sdoMsgs_);
    }
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x82) );

    nmtState_ = NMTStates::initializing;
}

void DeviceCanOpen::setNmtRestartRemoteDevice() {
    {
        std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
        // swap with an empty queue to clear it
        std::queue<SdoMsg>().swap(sdoMsgs_);
    }
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x81) );

    nmtState_ = NMTStates::initializing;
}

} /* namespace tcan */
