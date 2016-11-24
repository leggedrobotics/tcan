/*
 * Bus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan/CanBus.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan {

CanBus::CanBus(CanBusOptions* options):
    Bus<CanMsg>(options),
    devices_(),
    cobIdToFunctionMap_()
{

}

CanBus::~CanBus()
{
    for(auto device : devices_) {
        delete device;
    }
}

void CanBus::handleMessage(const CanMsg& msg) {

    // check if message is a bus error message
    if(msg.getCobId() > 0x20000000) {
        handleBusError(msg);
    }else{
        // Check if CAN message is handled.
        CobIdToFunctionMap::iterator it = cobIdToFunctionMap_.find(msg.getCobId());
        if (it != cobIdToFunctionMap_.end()) {
            if(it->second.first) {
                it->second.first->resetDeviceTimeoutCounter();
            }
            it->second.second(msg); // call function pointer
        } else {
            auto value = msg.getData();
            MELO_WARN("Received CAN message that is not handled: COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                      msg.getCobId(), value[1], value[0], value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]);
        }
    }
}

void CanBus::sanityCheck() {
    bool isMissing = false;
    bool allActive = true;
    for(auto device : devices_) {
        device->sanityCheck();
        isMissing |= device->isMissing();
        allActive &= device->isActive();
    }

    isMissingDevice_ = isMissing;
    allDevicesActive_ = allActive;
}

} /* namespace tcan */

