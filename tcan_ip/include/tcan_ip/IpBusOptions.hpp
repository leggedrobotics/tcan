/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusOptions.hpp"
#include <stdint.h>

namespace tcan_ip {

struct IpBusOptions : public tcan::BusOptions {

	enum class ConnectionType : unsigned int {
		TCP
		//UDP
	};

    IpBusOptions():
    	IpBusOptions(std::string(), 9999)
    {
    }

    IpBusOptions(const std::string& name, const uint16_t port):
        BusOptions(name),
		connectionType_(ConnectionType::TCP),
		port_(port),
        maxDeviceTimeoutCounter_(20)
    {
    }

    ~IpBusOptions() override = default;

    ConnectionType connectionType_;

    uint16_t port_;

    unsigned int maxDeviceTimeoutCounter_;
};

} /* namespace tcan_ip */
