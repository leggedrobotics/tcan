/*
 * DeviceCanOpen.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan_can/DeviceCanOpen.hpp"
#include "tcan/Bus.hpp"

#include "message_logger/message_logger.hpp"

namespace tcan {

DeviceCanOpen::DeviceCanOpen(const uint32_t nodeId, const std::string& name):
    DeviceCanOpen(std::unique_ptr<DeviceCanOpenOptions>(new DeviceCanOpenOptions(nodeId, name)))
{

}

DeviceCanOpen::DeviceCanOpen(std::unique_ptr<DeviceCanOpenOptions>&& options):
    CanDevice(std::move(options)),
    nmtState_(NMTStates::preOperational),
    sdoTimeoutCounter_(0),
    sdoSentCounter_(0),
    sdoMsgsMutex_(),
    sdoMsgs_()
{

}

DeviceCanOpen::~DeviceCanOpen()
{

}

void DeviceCanOpen::sanityCheck() {
    if(!isMissing()) {
        if(isTimedOut()) {
            state_ = Missing;
            MELO_WARN("Device %s timed out!", getName().c_str());

            clearSdoQueue();
        }else{
            checkSdoTimeout();
        }
    }
}

void DeviceCanOpen::sendPdo(const CanMsg& pdoMsg) {
    bus_->sendMessage(pdoMsg);
}

void DeviceCanOpen::sendSdo(const SdoMsg& sdoMsg) {

    std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
    sdoMsgs_.push(sdoMsg);

    if(sdoMsgs_.size() == 1) {
        // sdo queue was empty before, so put the new message in the bus output queue
        sdoTimeoutCounter_ = 0;
        sdoSentCounter_ = 0;

        bus_->sendMessage(sdoMsgs_.front());

        if(sdoMsg.getRequiresAnswer()) {
            // if an answer to a previously sent similar sdo has been received but not fetched, erase it to prevent storing outdated data
            std::lock_guard<std::mutex> guard(sdoAnswerMapMutex_);
            sdoAnswerMap_.erase(getSdoAnswerId(sdoMsg.getIndex(), sdoMsg.getSubIndex()));
        }else{
            sdoMsgs_.pop();
        }
    }
}

void DeviceCanOpen::handleTimedoutSdo(const SdoMsg& msg) {
    MELO_WARN("Device %s: SDO timeout (COB=%x / index=%x / subindex=%x / data=%x)", getName().c_str(), msg.getCobId(), msg.getIndex(), msg.getSubIndex(), msg.readuint32(4));
    if(state_ == Active) {
        // only set state to 'error' if state is 'active', to prevent overriding a 'Missing' state
        setNmtStopRemoteDevice();
        state_ = Error;
    }
}

void DeviceCanOpen::handleSdoError(const SdoMsg& request, const SdoMsg& answer) {
    const int32_t error = answer.readint32(4);
    MELO_WARN("Received SDO error from device %s: %s. COB=%x / index=%x / subindex=%x / error=%x / sent data=%x", options_->name_.c_str(), SdoMsg::getErrorName(error).c_str(), answer.getCobId(), answer.getIndex(), answer.getSubIndex(), error, request.readint32(4));
    setNmtStopRemoteDevice();
    state_ = Error;
}

bool DeviceCanOpen::getSdoAnswer(SdoMsg& sdoAnswer) {
    std::lock_guard<std::mutex> guard(sdoAnswerMapMutex_);
    auto it = sdoAnswerMap_.find(getSdoAnswerId(sdoAnswer.getIndex(), sdoAnswer.getSubIndex()));
    if (it == sdoAnswerMap_.end()) {
        return false;
    }
    sdoAnswer = it->second;
    sdoAnswerMap_.erase(it);
    return true;
}

void DeviceCanOpen::setNmtEnterPreOperational() {
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x80) );

    // the remote device will not tell us in which state it is if heartbeat message is disabled
    //   => assume that the state switch will be successful
    if(static_cast<const DeviceCanOpenOptions*>(options_.get())->producerHeartBeatTime_ == 0) {
        nmtState_ = NMTStates::preOperational;
    }
}

void DeviceCanOpen::setNmtStartRemoteDevice() {
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x1) );

    // the remote device will not tell us in which state it is if heartbeat message is disabled
    //   => assume that the state switch will be successful
    if(static_cast<const DeviceCanOpenOptions*>(options_.get())->producerHeartBeatTime_ == 0) {
        nmtState_ = NMTStates::operational;
    }
}

void DeviceCanOpen::setNmtStopRemoteDevice() {
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x2) );

    // the remote device will not tell us in which state it is if heartbeat message is disabled
    //   => assume that the state switch will be successful
    if(static_cast<const DeviceCanOpenOptions*>(options_.get())->producerHeartBeatTime_ == 0) {
        nmtState_ = NMTStates::stopped;
    }
}

void DeviceCanOpen::setNmtResetRemoteCommunication() {
    clearSdoQueue();
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x82) );

    state_ = Initializing;
}

void DeviceCanOpen::setNmtRestartRemoteDevice() {
    clearSdoQueue();
    deviceTimeoutCounter_ = 0;
    sendSdo( SdoMsg(static_cast<uint8_t>(getNodeId()), 0x81) );

    state_ = Initializing;
}

void DeviceCanOpen::resetDevice() {
    setNmtRestartRemoteDevice();
}

bool DeviceCanOpen::parseHeartBeat(const CanMsg& cmsg) {
    if(cmsg.getLength() != 1) {
        MELO_WARN("Invalid Heartbeat message length from %s (nodeId %x): %d", getName().c_str(), getNodeId(), cmsg.getLength());
        return false;
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
            MELO_WARN("Invalid Heartbeat message data from device %s (nodeId %x): %x", options_->name_.c_str(), getNodeId(), cmsg.readuint8(0));
            return false;
            break;
    }

    return true;
}

bool DeviceCanOpen::parseSDOAnswer(const CanMsg& cmsg) {
    const uint8_t responseMode = cmsg.readuint8(0);
    const uint16_t index = cmsg.readuint16(1);
    const uint8_t subindex = cmsg.readuint8(3);

    std::unique_lock<std::mutex> guard(sdoMsgsMutex_); // lock sdoMsgsMutex_ to prevent checkSdoTimeout() from making changes on sdoMsgs_
    if(sdoMsgs_.size() != 0) {
        const SdoMsg& sdo = sdoMsgs_.front();

        if(sdo.getIndex() == index && sdo.getSubIndex() == subindex) {

            if(responseMode == 0x42 || responseMode == 0x43 || responseMode == 0x4B || responseMode == 0x4F) { // read responses (unspecified length, 4, 2 or 1 byte)
                {
                  std::lock_guard<std::mutex> mapGuard(sdoAnswerMapMutex_);
                  sdoAnswerMap_[getSdoAnswerId(index, subindex)] = static_cast<const SdoMsg&>(cmsg);
                }
                guard.unlock(); // unlock guard here, otherwise the user will not be able to put any sdo in the sdo ouput queue
                handleReadSdoAnswer( static_cast<const SdoMsg&>(cmsg) );
                guard.lock();
            }else if(responseMode == 0x80) { // error response
                guard.unlock(); // unlock guard here, otherwise the user will not be able to put any sdo in the sdo ouput queue
                handleSdoError(sdo, static_cast<const SdoMsg&>(cmsg));
                guard.lock();
            }

            sendNextSdo();

            return true;
        }
    }

    MELO_WARN("Received unexpected SDO answer from device %s. COB=%x / index=%x / subindex=%x / data=%x", options_->name_.c_str(), cmsg.getCobId(), index, subindex, cmsg.readuint32(4));
    return false;
}


bool DeviceCanOpen::checkSdoTimeout() {
    const DeviceCanOpenOptions* options = static_cast<const DeviceCanOpenOptions*>(options_.get());

    if(options->maxSdoTimeoutCounter_ != 0) {
        std::unique_lock<std::mutex> guard(sdoMsgsMutex_); // lock sdoMsgsMutex_ to prevent parseSDOAnswer from making changes on sdoMsgs_
        if( sdoMsgs_.size() != 0 && (sdoTimeoutCounter_++ > options->maxSdoTimeoutCounter_) ) {
            // sdoTimeoutCounter_ is only increased if options_->maxSdoTimeoutCounter != 0 and sdoMsgs_.size() != 0

            const SdoMsg &msg = sdoMsgs_.front();
            if (sdoSentCounter_ > options->maxSdoSentCounter_) {
                guard.unlock(); // unlock guard here, otherwise the user will not be able to put any sdo in the sdo ouput queue
                handleTimedoutSdo(msg);
                guard.lock();
                sendNextSdo();

                return false;
            } else {
                sdoSentCounter_++;

                bus_->sendMessage(msg);
            }
        }
    }

    return true;
}

void DeviceCanOpen::sendNextSdo() {
    sdoTimeoutCounter_ = 0;
    sdoSentCounter_ = 0;

    sdoMsgs_.pop();

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

void DeviceCanOpen::clearSdoQueue() {
    std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
    // swap with an empty queue to clear it
    std::queue<SdoMsg>().swap(sdoMsgs_);
}

uint32_t DeviceCanOpen::getSdoAnswerId(const uint16_t index, const uint8_t subIndex) {
    return (static_cast<uint32_t>(index) << 8) + static_cast<uint32_t>(subIndex);
}

} /* namespace tcan */
