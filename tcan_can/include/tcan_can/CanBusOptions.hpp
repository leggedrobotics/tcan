/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusOptions.hpp"

namespace tcan_can {

struct CanBusOptions : public tcan::BusOptions {
    CanBusOptions():
        CanBusOptions(std::string())
    {
    }

    CanBusOptions(const std::string& name):
        BusOptions(name),
        passivateOnBusError_(false),
        passivateIfNoDevices_(false)
    {
    }

    virtual ~CanBusOptions()
    {
    }

    //! If set to true, bus goes to passive mode (no message are sent on the bus) on reception of bus errors
    bool passivateOnBusError_;

    //! If set to true, bus goes to passive mode (no messages are sent on the bus) if all devices are missing.
    bool passivateIfNoDevices_;
};

} /* namespace tcan_can */
