/*
 * PcanBusOptions.hpp
 *
 *  Created on: Mar 15, 2017
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusOptions.hpp"

namespace tcan_pcanfd {

struct PcanfdBusOptions : public tcan_can::CanBusOptions {
    PcanfdBusOptions():
        PcanfdBusOptions(std::string())
    {
    }

    PcanfdBusOptions(const std::string& interface_name):
        tcan_can::CanBusOptions(interface_name),
        loopback_(false),
        maxMessagesPassed_(10),
        bitrate_(1000000)
    {
    }

    ~PcanfdBusOptions() override = default;

    //! loop back sent messages
    bool loopback_;

    //! max number of messages we are passing to the driver at once
    unsigned int maxMessagesPassed_;

    //! bitrate of the CAN bus (max 1000000)
    unsigned int bitrate_;

    // todo: filter
};

} /* namespace tcan_pcanfd */
