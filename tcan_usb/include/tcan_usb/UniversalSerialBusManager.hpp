/*
 * BusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusManager.hpp"
#include "tcan/UniversalSerialBus.hpp"
#include "tcan/UsbMsg.hpp"

namespace tcan {

//! Container of all USB buses
class UniversalSerialBusManager : public BusManager<UsbMsg> {
 public:
    UniversalSerialBusManager();

    virtual ~UniversalSerialBusManager();

    UniversalSerialBus* getUniversalSerialBus(const unsigned int index) { return static_cast<UniversalSerialBus*>(buses_[index]); }
};

} /* namespace tcan */
