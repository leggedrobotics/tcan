/*
 * IpBusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusManager.hpp"
#include "tcan_ip/IpBus.hpp"
#include "tcan_ip/IpMsg.hpp"

namespace tcan_ip {

//! Container of all IP buses
class IpBusManager : public tcan::BusManager<IpMsg> {
 public:
	IpBusManager();

    virtual ~IpBusManager();

    IpBus* getIpBus(const unsigned int index) { return static_cast<IpBus*>(buses_[index]); }
};

} /* namespace tcan_ip */
