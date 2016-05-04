/*
 * Bus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include <utility>
#include <chrono>
#include <pthread.h>

#include "tcan/Bus.hpp"
#include "message_logger/log/log_messages.hpp"

namespace tcan {

Bus::Bus(BusOptions* options):
    isOperational_(false),
    options_(options),
    cobIdToFunctionMap_(),
    outgointMsgsMutex_(),
    outgoingMsgs_(),
    receiveThread_(),
    transmitThread_(),
    sanityCheckThread_(),
    running_(false),
    condTransmitThread_(),
    condOutputQueueEmpty_()
{

}

Bus::~Bus()
{
    stopThreads();

    for(Device* device : devices_) {
        delete device;
    }

    delete options_;
}

void Bus::stopThreads() {
    running_ = false;
    condTransmitThread_.notify_all();
    condOutputQueueEmpty_.notify_all();

    if(receiveThread_.joinable()) {
        receiveThread_.join();
    }

    if(transmitThread_.joinable()) {
        transmitThread_.join();
    }

    if(sanityCheckThread_.joinable()) {
        sanityCheckThread_.join();
    }
}

bool Bus::initBus() {

    if(!initializeCanBus()) {
        return false;
    }

    running_ = true;

    if(options_->asynchronous_) {
        receiveThread_ = std::thread(&Bus::receiveWorker, this);
        transmitThread_ = std::thread(&Bus::transmitWorker, this);

        sched_param sched;
        sched.sched_priority = options_->priorityReceiveThread_;
        if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
            MELO_WARN("Failed to set receive thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
        }

        sched.sched_priority = options_->priorityTransmitThread_;
        if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
        	MELO_WARN("Failed to set transmit thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
        }

        if(options_->sanityCheckInterval_ > 0) {
            sanityCheckThread_ = std::thread(&Bus::sanityCheckWorker, this);

            sched.sched_priority = options_->prioritySanityCheckThread_;
            if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
            	MELO_WARN("Failed to set receive thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
            }
        }
    }

    return true;
}

void Bus::handleMessage(const CanMsg& cmsg) {

    // Check if CAN message is handled.
    CobIdToFunctionMap::iterator it = cobIdToFunctionMap_.find(cmsg.getCobId());
    if (it != cobIdToFunctionMap_.end()) {

        it->second(cmsg); // call function pointer
    } else {
        auto value = cmsg.getData();
        MELO_WARN("Received CAN message that is not handled: COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
               cmsg.getCobId(),
               value[1],
               value[0],
               value[0],
               value[1],
               value[2],
               value[3],
               value[4],
               value[5],
               value[6],
               value[7]
        );
    }
}


bool Bus::processOutputQueue() {
    bool writeSuccess = false;
    std::unique_lock<std::mutex> lock(outgointMsgsMutex_);

    while(outgoingMsgs_.size() == 0 && running_) {
        condOutputQueueEmpty_.notify_all();
        condTransmitThread_.wait(lock);
    }
    // after the wait function we own the lock. copy data and unlock.

    if(!running_) {
        return true;
    }

    CanMsg cmsg = outgoingMsgs_.front();
    lock.unlock();

    writeSuccess = writeCanMessage( cmsg );

    lock.lock();

    if(writeSuccess) {
        // only pop the message from the queue if sending was successful
        outgoingMsgs_.pop();
    }

    return writeSuccess;
}

bool Bus::sanityCheck() {
    bool allFine = true;
    for(auto device : devices_) {
        if(!device->sanityCheck()) {
            allFine = false;
        }
    }

    isOperational_ = allFine;
    return allFine;
}

void Bus::sendMessageWithoutLock(const CanMsg& cmsg) {
    if(outgoingMsgs_.size() >= options_->maxQueueSize_) {
    	MELO_WARN("Exceeding max queue size on bus %s! Dropping message!", options_->name_.c_str());
    }
    outgoingMsgs_.push( cmsg );	// do not use emplace here. We need a copy of the message in some situations
    // (SDO queue processing). Declaring another sendMessage(..) which uses move
    // semantics does not increase performance as CANMsg has only POD members

    condTransmitThread_.notify_all();
}

void Bus::receiveWorker() {
    while(running_) {
        readMessage();
    }

    MELO_INFO("receive thread for bus %s terminated", options_->name_.c_str());
}

void Bus::transmitWorker() {
    while(running_) {
        processOutputQueue();
    }

    MELO_INFO("transmit thread for bus %s terminated", options_->name_.c_str());
}

void Bus::sanityCheckWorker() {
    auto nextLoop = std::chrono::steady_clock::now();

    while(running_) {
        sanityCheck();

        nextLoop += std::chrono::milliseconds(options_->sanityCheckInterval_);
        std::this_thread::sleep_until(nextLoop);
    }

    MELO_INFO("sanityCheck thread for bus %s terminated", options_->name_.c_str());
}

} /* namespace tcan */

