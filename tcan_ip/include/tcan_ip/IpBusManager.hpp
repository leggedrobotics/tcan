/*
 * IpBusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusManager.hpp"
#include "tcan/IpBus.hpp"
#include "tcan/IpMsg.hpp"

namespace tcan {

//! Container of all IP buses
class IpBusManager : public BusManager<IpMsg> {
 public:
	IpBusManager();

    virtual ~IpBusManager();

    IpBus* getIpBus(const unsigned int index) { return static_cast<IpBus*>(buses_[index]); }
};

} /* namespace tcan */
