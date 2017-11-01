/*
 * BusManager.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan_can/CanBusManager.hpp"

namespace tcan {

CanBusManager::CanBusManager():
    BusManager<CanMsg>()
{
}

CanBusManager::~CanBusManager()
{
}

void CanBusManager::sendSyncOnAllBuses(const bool waitForEmptyQueues) {
    const unsigned int bussize = buses_.size();
    std::unique_lock<std::mutex> locks[bussize];

    if(waitForEmptyQueues) {
        for(unsigned int i=0; i<bussize; i++) {
            auto bus = getCanBus(i);
            if(bus->isAsynchronous()) {
                bus->waitForEmptyQueue(locks[i]);
            }
        }

        // we now own a lock on all output message queues
    }

    for(unsigned int i=0; i<bussize; i++) {
        static_cast<CanBus*>(buses_[i])->sendSyncWithoutLock();
    }
}

void CanBusManager::sendSync(const unsigned int busIndex) {
    if(busIndex < buses_.size()) {
        static_cast<CanBus*>(buses_[busIndex])->sendSync();
    }
}

bool CanBusManager::getErrorMsgFlag() const {
    for(auto bus : buses_) {
        if(static_cast<CanBus*>(bus)->getErrorMsgFlag()) {
            return true;
        }
    }
    return false;
}

bool CanBusManager::resetErrorMsgFlag() {
    bool hadBusError = false;
    for(auto bus : buses_) {
        if(static_cast<CanBus*>(bus)->resetErrorMsgFlag()) {
            hadBusError = true;
        }
    }
    return hadBusError;
}

void CanBusManager::resetAllDevices() {
    for(auto bus : buses_) {
        static_cast<CanBus*>(bus)->resetAllDevices();
    }
}

} /* namespace tcan */
