/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusOptions.hpp"
#include <stdint.h>

namespace tcan {

struct IpBusOptions : public BusOptions {

    IpBusOptions():
    	IpBusOptions(std::string(), 9999)
    {

    }

    IpBusOptions(const std::string& name, const uint16_t port):
        BusOptions(name),
		port_(port),
        maxDeviceTimeoutCounter_(20)
    {

    }

    virtual ~IpBusOptions() { }

    uint16_t port_;

    unsigned int maxDeviceTimeoutCounter_;
};

} /* namespace tcan */
