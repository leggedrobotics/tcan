/*
 * DeviceCanOpenOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <stdint.h>

#include "tcan_can/CanDeviceOptions.hpp"

namespace tcan_can {

class DeviceCanOpenOptions : public CanDeviceOptions {
 public:

    DeviceCanOpenOptions() = delete;

    DeviceCanOpenOptions(
        const uint32_t nodeId,
        const std::string name,
        const unsigned int maxSdoTimeoutCounter = 1,
        const unsigned int maxSdoSentCounter = 10,
        const uint16_t producerHeartBeatTime = 0,
        const unsigned int maxDeviceTimeoutCounter = 20)
    :
        CanDeviceOptions(nodeId, name, maxDeviceTimeoutCounter),
        maxSdoTimeoutCounter_(maxSdoTimeoutCounter),
        maxSdoSentCounter_(maxSdoSentCounter),
        producerHeartBeatTime_(producerHeartBeatTime)
    {

    }

    virtual ~DeviceCanOpenOptions() { }

    /*!
     * Set the maxSdoTimeoutCounter_
     * @param timeout   timeout in seconds
     * @param looprate  looprate at which the sanityCheck(..) function is called [Hz].
     */
    inline void setSdoTimeoutCounter(const double timeout, const double looprate=10.0) {
        maxSdoTimeoutCounter_ = static_cast<unsigned int>(timeout*looprate);
    }

    //! counter limit at which an SDO is considered as timed out. Set 0 to disable.
    // maxSdoTimeoutCounter = timeout [s] * looprate [Hz] (looprate = rate of checkSanity(..) calls. In asynchronous mode this is 10Hz by default (see BusOptions))
    unsigned int maxSdoTimeoutCounter_;

    //! number of tries of an SDO transmission
    unsigned int maxSdoSentCounter_;

    //! Heartbeat time interval [ms], produced by the device. Set to 0 to disable heartbeat message reception checking.
    uint16_t producerHeartBeatTime_;

};

} /* namespace tcan_can */
