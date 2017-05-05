/*
 * PcanBusOptions.hpp
 *
 *  Created on: Mar 15, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <vector>
#include <linux/can.h> // for can_filter

#include "tcan/BusOptions.hpp"

namespace tcan {

struct PcanBusOptions : public CanBusOptions {
    PcanBusOptions():
        PcanBusOptions(std::string())
    {
    }

    PcanBusOptions(const std::string& interface_name):
        CanBusOptions(interface_name),
        canErrorThrottleTime_(0.0)
    {
    }

    virtual ~PcanBusOptions() { }

    double canErrorThrottleTime_;
};

} /* namespace tcan */
