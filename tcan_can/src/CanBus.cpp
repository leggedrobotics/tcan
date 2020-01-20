/*
 * Bus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan_can/CanBus.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan_can {

CanBus::CanBus(std::unique_ptr<CanBusOptions>&& options):
    tcan::Bus<CanMsg>( std::move(options) ),
    devices_(),
    canFrameIdentifierToFunctionMap_(),
    unmappedMessageCallbackFunction_(std::bind(&CanBus::defaultHandleUnmappedMessage, this, std::placeholders::_1))
{
}

CanBus::~CanBus()
{
    for(auto device : devices_) {
        delete device;
    }
}

void CanBus::handleMessage(const CanMsg& msg) {

    errorMsgFlag_ = false;

    // Check if CAN message is handled.
    auto it = std::find_if(canFrameIdentifierToFunctionMap_.cbegin(), canFrameIdentifierToFunctionMap_.cend(), [&msg](auto p){
        return !((msg.getCobId() ^ p.first.identifier) & p.first.mask);
    });

    if (it != canFrameIdentifierToFunctionMap_.end()) {
        if(it->second.first) {
            it->second.first->resetDeviceTimeoutCounter();
            it->second.first->configureDeviceInternal(msg);
        }
        it->second.second(msg); // call function pointer
    } else {
        unmappedMessageCallbackFunction_(msg);
    }
}

bool CanBus::sanityCheck() {
    bool isMissingOrError = false;
    bool allMissing = true;
    bool allActive = true;
    for(auto device : devices_) {
        isMissingOrError |= !device->sanityCheck();
        allMissing &= device->isMissing();
        allActive &= device->isActive();
    }

    if(!isPassive() && allMissing && static_cast<const CanBusOptions*>(options_.get())->passivateIfNoDevices_) {
        passivate();
        MELO_WARN("All devices missing on bus %s. This bus is now PASSIVE!", options_->name_.c_str());
    }

    isMissingDeviceOrHasError_ = isMissingOrError;
    allDevicesActive_ = allActive;
    allDevicesMissing_ = allMissing;

    return !(isMissingOrError || hasBusError_);
}

void CanBus::resetAllDevices() {
    for(auto device : devices_) {
        device->resetDevice();
    }
}

bool CanBus::defaultHandleUnmappedMessage(const CanMsg& msg) {
    auto value = msg.getData();
    MELO_INFO("Received CAN message on bus %s that is not handled: COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
              options_->name_.c_str(), msg.getCobId(), value[1], value[0], value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7]);

    return true;
}

} /* namespace tcan_can */
