/*
 * EtherCatDeviceOptions.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#pragma once

#include <string>
#include <stdint.h>

namespace tcan {

class EtherCatDeviceOptions {
 public:

    EtherCatDeviceOptions() = delete;

    /*!
     *
     * @param address                   address of the device
     * @param name                      name of the device
     * @param maxDeviceTimeoutCounter   counter limit at which the device is considered as timed out. Set 0 to disable.
     *                                  If no message was received within this time, the device is considered as missing.
     */
    EtherCatDeviceOptions(
        const uint32_t address,
        const std::string& name,
        const unsigned int maxDeviceTimeoutCounter = 20):
        address_(address),
        name_(name),
        maxDeviceTimeoutCounter_(maxDeviceTimeoutCounter),
        printConfigInfo_(true)
    {

    }

    /*!
     *
     * @param address   address of the device
     * @param name      name of the device
     * @param timeout   timeout in seconds. If no message was received within this time, the device is considered as missing
     * @param looprate  loop rate [Hz] of the process calling checkSanity(..). In asynchrounous mode this is 10Hz by default (see BusOptions)
     */
    EtherCatDeviceOptions(
        const uint32_t address,
        const std::string& name,
        const double timeout,
        const double looprate):
        EtherCatDeviceOptions(address, name, static_cast<unsigned int>(timeout*looprate))
    {

    }

    virtual ~EtherCatDeviceOptions() { }

    /*!
     * set the maxDeviceTimeoutCounter_
     * @param timeout   timout in seconds
     * @param looprate  looprate at which the sanityCheck(..) function is called [Hz].
     */
    inline void setDeviceTimeoutCounter(const double timeout, const double looprate=10.0) {
        maxDeviceTimeoutCounter_ = static_cast<unsigned int>(timeout*looprate);
    }


    //! address of device
    uint32_t address_;

    //! human-readable name of the device
    std::string name_;

    //! counter limit at which the device is considered as timed out (see sanityCheck(..)).  Set 0 to disable.
    // maxDeviceTimeoutCounter = timeout [s] * looprate [Hz] (looprate = rate of checkSanity(..) calls. In asynchrounous mode this is 10Hz by default (see BusOptions))
    unsigned int maxDeviceTimeoutCounter_;

    //! if true, a message will be printed to the console if configureDevice returned true
    bool printConfigInfo_;
};

} /* namespace tcan */
