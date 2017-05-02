/*
 * BusManager.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan/CanBusManager.hpp"

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
            static_cast<CanBus*>(buses_[i])->waitForEmptyQueue(locks[i]);
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

bool CanBusManager::getErrorFlag() const {
    for(auto bus : buses_) {
        if(static_cast<CanBus*>(bus)->getErrorFlag()) {
            return true;
        }
    }
    return false;
}

bool CanBusManager::resetErrorFlag() {
    bool hadBusError = false;
    for(auto bus : buses_) {
        if(static_cast<CanBus*>(bus)->resetErrorFlag()) {
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
