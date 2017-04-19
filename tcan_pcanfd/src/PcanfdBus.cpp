/*
 * PcanBus.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: Philipp Leemann
 */


#include "tcan_pcanfd/PcanfdBus.hpp"

#include "message_logger/message_logger.hpp"

#include "pcan_driver/libpcanfd.h"


namespace tcan {

PcanfdBus::PcanfdBus(const std::string& interface):
    PcanfdBus(new PcanfdBusOptions(interface))
{
}

PcanfdBus::PcanfdBus(PcanfdBusOptions* options):
    CanBus(options),
    fd_(0)
{
}

PcanfdBus::~PcanfdBus()
{
    stopThreads();
    pcanfd_close(fd_);
}

bool PcanfdBus::initializeInterface()
{
    const PcanfdBusOptions* options = static_cast<const PcanfdBusOptions*>(options_);
    const char* interface = options->name_.c_str();

    unsigned int flags = OFD_BITRATE;
    
    // make read/write non-blocking in synchronous mode
    if(!options->asynchronous_) {
        flags |= OFD_NONBLOCKING;
    }
    fd_ = pcanfd_open(interface, flags, options->bitrate_);
    if(fd_ <= 0) {
        MELO_FATAL("Opening CAN %s failed!", interface);
        return false;
    }
    
    // todo: filter

    MELO_INFO("Opened CAN %s.", interface);
    return true;
}


bool PcanfdBus::readData() {
    // todo: does return when bus is down?

    const unsigned int maxMsgs = static_cast<const PcanfdBusOptions*>(options_)->maxMessagesPassed_;
    pcanfd_msg inMsgs[maxMsgs];
    const int numReceived = pcanfd_recv_msgs_list(fd_, maxMsgs, inMsgs);

    if(numReceived <= 0) {
        if(numReceived != EWOULDBLOCK) {
            MELO_ERROR("Error while receiving messages: %d", numReceived);
        }
        return false;
    }
    
    for(unsigned int i=0; i<maxMsgs; ++i) {
        handleMessage( CanMsg(inMsgs[i].id, inMsgs[i].data_len, inMsgs[i].data) );
    }
 
    return true;
}


bool PcanfdBus::writeData(std::unique_lock<std::mutex>* lock) {
    // todo: does return when bus is down?
    
    const unsigned int numMsgs = std::min(static_cast<const PcanfdBusOptions*>(options_)->maxMessagesPassed_, static_cast<unsigned int>(outgoingMsgs_.size()));
    pcanfd_msg outMsgs[numMsgs];
    
    for(unsigned int i=0; i<numMsgs; ++i) {
        const CanMsg& cmsg = outgoingMsgs_[i];
        outMsgs[i].id = cmsg.getCobId();
        outMsgs[i].data_len = cmsg.getLength();
        outMsgs[i].type = PCANFD_MSG_STD;
        std::copy(cmsg.getData(), &(cmsg.getData()[outMsgs[i].data_len]), outMsgs[i].data);
    }
    if(lock != nullptr) {
        lock->unlock();
    }
    const int numSent = pcanfd_send_msgs_list(fd_, numMsgs, outMsgs);
    if(numSent <= 0) {
        if(numSent != EWOULDBLOCK) {
            MELO_ERROR("Error while sending messages: %d", numSent);
        }
        return false;
    }

    if(lock != nullptr) {
        lock->lock();
    }
    outgoingMsgs_.erase(outgoingMsgs_.begin(), outgoingMsgs_.begin()+numSent);
    return true;
}


} /* namespace tcan */
