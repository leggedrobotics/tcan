/*
 * DevcieOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <string>

namespace tcan {

class DeviceOptions {
 public:

    DeviceOptions() = delete;

    DeviceOptions(
        const uint32_t nodeId,
        const std::string& name,
        const unsigned int maxDeviceTimeoutCounter = 20):
        nodeId_(nodeId),
        name_(name),
        maxDeviceTimeoutCounter_(maxDeviceTimeoutCounter)
    {

    }

    virtual ~DeviceOptions() { }

    //! CAN node ID of device
    uint32_t nodeId_;

    //! human-readable name of the device
    std::string name_;

    //! counter limit at which the device is considered as timed out (see sanityCheck(..)).  Set 0 to disable.
    // does only work if the device resets deviceTimeoutCounter_ to zero in (at least) one of its message parse functions
    // maxDeviceTimeoutCounter = timeout [s] * looprate [Hz] (looprate = rate of checkSanity(..) calls. In asynchrounous mode this is 10Hz by default (see BusOptions))
    unsigned int maxDeviceTimeoutCounter_;

};

} /* namespace tcan */
