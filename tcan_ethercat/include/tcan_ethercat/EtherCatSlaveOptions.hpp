/*
 * EtherCatDeviceOptions.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#pragma once

#include <string>
#include <stdint.h>

namespace tcan_ethercat {

class EtherCatSlaveOptions {
 public:
    /*!
     * Constructor.
     * @param address                   Address of the slave.
     * @param name                      Human-readable name of the slave.
     * @param maxDeviceTimeoutCounter   Counter limit at which the slave is considered as timed out. Set 0 to disable.
     *                                  If no message was received within this time, the slave is considered as missing.
     */
    EtherCatSlaveOptions(
        const uint32_t address,
        const std::string& name,
        const unsigned int maxDeviceTimeoutCounter = 20):
        address_(address),
        name_(name),
        maxDeviceTimeoutCounter_(maxDeviceTimeoutCounter),
        printConfigInfo_(true) {}

    /*!
     * Constructor.
     * @param address   Address of the slave.
     * @param name      Human-readable name of the slave.
     * @param timeout   Timeout in seconds. If no message was received within this time, the slave is considered as missing.
     * @param looprate  Loop rate [Hz] of the process calling checkSanity(..). In asynchronous mode this is 10Hz by default (see BusOptions).
     */
    EtherCatSlaveOptions(
        const uint32_t address,
        const std::string& name,
        const double timeout,
        const double looprate):
        EtherCatSlaveOptions(address, name, static_cast<unsigned int>(timeout*looprate)) {}

    /*!
     * Destructor.
     */
    virtual ~EtherCatSlaveOptions() {}

    /*!
     * Set the maximal slave timeout counter.
     * @param timeout   Timeout in seconds.
     * @param looprate  Looprate at which the sanityCheck(..) function is called [Hz].
     */
    inline void setDeviceTimeoutCounter(const double timeout, const double looprate=10.0) {
        maxDeviceTimeoutCounter_ = static_cast<unsigned int>(timeout*looprate);
    }

    //! Address of slave.
    uint32_t address_ = 0;

    //! Human-readable name of the slave.
    std::string name_;

    //! Counter limit at which the slave is considered as timed out (see sanityCheck(..)). Set 0 to disable.
    // maxDeviceTimeoutCounter = timeout [s] * looprate [Hz] (looprate = rate of checkSanity(..) calls. In asynchronous mode this is 10Hz by default (see BusOptions)).
    unsigned int maxDeviceTimeoutCounter_ = 0;

    //! If true, a message will be printed to the console if configureDevice returned true.
    bool printConfigInfo_ = false;
};

} /* namespace tcan_ethercat */
