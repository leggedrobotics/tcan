#pragma once

#include "tcan/BusManager.hpp"
#include "tcan_usb/UniversalSerialBus.hpp"
#include "tcan_usb/UsbMsg.hpp"

namespace tcan_usb {

//! Container of all USB buses
class UniversalSerialBusManager : public tcan::BusManager<UsbMsg> {
 public:
    UniversalSerialBusManager();

    ~UniversalSerialBusManager() override;

    UniversalSerialBus* getUniversalSerialBus(const unsigned int index) { return static_cast<UniversalSerialBus*>(buses_[index]); }
};

} /* namespace tcan_usb */
