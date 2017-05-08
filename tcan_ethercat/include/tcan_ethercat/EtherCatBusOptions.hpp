/*
 * EtherCatBusOptions.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#pragma once

#include <stdint.h>

#include "tcan/BusOptions.hpp"

namespace tcan_ethercat {

struct EtherCatBusOptions : public tcan::BusOptions {

    enum class ConnectionType : unsigned int {
        TCP
        //UDP
    };

    EtherCatBusOptions():
        EtherCatBusOptions(std::string(), 9999)
    {

    }

    EtherCatBusOptions(const std::string& name, const uint16_t port):
        BusOptions(name),
        connectionType_(ConnectionType::TCP),
        port_(port),
        maxDeviceTimeoutCounter_(20)
    {

    }

    virtual ~EtherCatBusOptions() { }

    ConnectionType connectionType_;

    uint16_t port_;

    unsigned int maxDeviceTimeoutCounter_;
};

} /* namespace tcan_ethercat */
