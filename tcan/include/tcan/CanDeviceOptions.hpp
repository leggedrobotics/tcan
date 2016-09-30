/*
 * DevcieOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <string>
#include <stdint.h>

namespace tcan {

class CanDeviceOptions {
 public:

    CanDeviceOptions() = delete;

    /*!
     *
     * @param nodeId                    CAN id of the node
     * @param name                      name of the node
     * @param maxDeviceTimeoutCounter   counter limit at which the device is considered as timed out. Set 0 to disable.
     *                                  If no message was received within this time, the device is considered as missing.
     */
    CanDeviceOptions(
        const uint32_t nodeId,
        const std::string& name,
        const unsigned int maxDeviceTimeoutCounter = 20):
        nodeId_(nodeId),
        name_(name),
        maxDeviceTimeoutCounter_(maxDeviceTimeoutCounter)
    {

    }

    /*!
     *
     * @param nodeId    CAN id of the node
     * @param name      name of the node
     * @param timeout   timeout in seconds. If no message was received within this time, the device is considered as missing
     * @param looprate  loop rate [Hz] of the process calling checkSanity(..). In asynchrounous mode this is 10Hz by default (see BusOptions)
     */
    CanDeviceOptions(
        const uint32_t nodeId,
        const std::string& name,
        const double timeout,
        const double looprate):
        CanDeviceOptions(nodeId, name, static_cast<unsigned int>(timeout*looprate))
    {

    }

    virtual ~CanDeviceOptions() { }

    inline void setDeviceTimeoutCounter(const double timeout, const double looprate) {
        maxDeviceTimeoutCounter_ = static_cast<unsigned int>(timeout*looprate);
    }


    //! CAN node ID of device
    uint32_t nodeId_;

    //! human-readable name of the device
    std::string name_;

    //! counter limit at which the device is considered as timed out (see sanityCheck(..)).  Set 0 to disable.
    // maxDeviceTimeoutCounter = timeout [s] * looprate [Hz] (looprate = rate of checkSanity(..) calls. In asynchrounous mode this is 10Hz by default (see BusOptions))
    unsigned int maxDeviceTimeoutCounter_;

};

} /* namespace tcan */
