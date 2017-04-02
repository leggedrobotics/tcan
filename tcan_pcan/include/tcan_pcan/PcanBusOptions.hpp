/*
 * PcanBusOptions.hpp
 *
 *  Created on: Mar 15, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include "tcan/BusOptions.hpp"

namespace tcan {

struct PcanBusOptions : public CanBusOptions {
    PcanBusOptions():
        PcanBusOptions(std::string())
    {
    }

    PcanBusOptions(const std::string& interface_name):
        CanBusOptions(interface_name),
        loopback_(false),
        maxMessagesPassed_(10)
    {
    }

    virtual ~PcanBusOptions() { }

    //! loop back sent messages
    bool loopback_;

    //! max number of messages we are passing to the driver at once
    unsigned int maxMessagesPassed_;
    
    //! bitrate of the CAN bus (max 1000000)
    unsigned int bitrate_;
    
    // todo: filter
};

} /* namespace tcan */
