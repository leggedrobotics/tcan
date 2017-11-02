/*
 * BusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusManager.hpp"
#include "tcan_usb/UniversalSerialBus.hpp"
#include "tcan_usb/UsbMsg.hpp"

namespace tcan_usb {

//! Container of all USB buses
class UniversalSerialBusManager : public tcan::BusManager<UsbMsg> {
 public:
    UniversalSerialBusManager();

    virtual ~UniversalSerialBusManager();

    UniversalSerialBus* getUniversalSerialBus(const unsigned int index) { return static_cast<UniversalSerialBus*>(buses_[index]); }
};

} /* namespace tcan_usb */
