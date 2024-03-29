#include "tcan_can/CanBusManager.hpp"

#include <vector>

namespace tcan_can {

void CanBusManager::sendSyncOnAllBuses(const bool waitForEmptyQueues) {
    const unsigned int bussize = buses_.size();
    std::vector<std::unique_lock<std::mutex>> locks(bussize);

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

void CanBusManager::resetAllDevices() {
    for(auto bus : buses_) {
        static_cast<CanBus*>(bus)->resetAllDevices();
    }
}

} /* namespace tcan_can */
