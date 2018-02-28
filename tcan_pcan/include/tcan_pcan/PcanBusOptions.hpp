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

namespace tcan_pcan {

struct PcanBusOptions : public tcan_can::CanBusOptions {
    PcanBusOptions():
        PcanBusOptions(std::string())
    {
    }

    PcanBusOptions(const std::string& interface_name):
        CanBusOptions(interface_name),
        canErrorThrottleTime_(0.0)
    {
    }

    ~PcanBusOptions() override = default;

    double canErrorThrottleTime_;
};

} /* namespace tcan_pcan */
