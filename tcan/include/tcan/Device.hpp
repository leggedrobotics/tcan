/*
 * Device.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <string>
#include <stdint.h>
#include <tcan/CanMsg.hpp>
#include <atomic>

#include "tcan/DeviceOptions.hpp"

namespace tcan {
class Bus;


//! A device that is connected via CAN.
class Device {
 public:
    enum State {
        Initializing=0,
        Active,
        Missing
    };

    /*! Constructor
     * @param nodeId	ID of CAN node
     * @param name		human-readable name of the device
     */
    Device() = delete;

    Device(const uint32_t nodeId, const std::string& name):
        Device(new DeviceOptions(nodeId, name))
    {
    }

    Device(DeviceOptions* options):
        options_(options),
        deviceTimeoutCounter_(0),
        state_(Initializing),
        bus_(nullptr)
    {
    }

    //! Destructor
    virtual ~Device()
    {
        delete options_;
    }

    /*! Initialize the device. This function is automatically called by Bus::addDevice(..)
     *   (through initDeviceInternal(..))
     * This function is intended to do some initial device initialization (register messages to be received,
     *   restart remote node, ...)
     * @return true if successfully initialized
     */
    virtual bool initDevice() = 0;

    /*! Configure the device
     * This function is automatically called after reception of a
     * bootup message. (or more general: After reception of any message if the device was missing)
     */
    virtual void configureDevice() = 0;

    /*! Do a sanity check of the device. This function is intended to be called with constant rate
     * and shall check heartbeats, SDO timeouts, ...
     * This function is automatically called if the Bus has asynchronous=true and sanityCheckInterval > 0
     * @return true if everything is ok.
     */
    virtual bool sanityCheck() {
        const bool active = checkDeviceTimeout();
        if(!active) {
            state_ = Missing;
        }
        return active;
    }

    inline uint32_t getNodeId() const { return options_->nodeId_; }
    inline const std::string& getName() const { return options_->name_; }

    inline bool isInitializing() const { return (state_ == Initializing); }
    inline bool isActive() const { return (state_ == Active); }
    inline bool isMissing() const { return (state_ == Missing); }


 public: /// Internal functions
    /*! Initialize the device. This function is automatically called by Bus::addDevice(..).
     * Calls the initDevice() function.
     */
    bool initDeviceInternal(Bus* bus) {
        bus_ = bus;
        return initDevice();
    }

    inline void resetDeviceTimeoutCounter() {
        if(state_ != Active) {
            configureDevice();
            state_ = Active;
        }
        deviceTimeoutCounter_ = 0;
    }

 protected:
    /*!
     * @return False if the device timed out
     */
    inline bool checkDeviceTimeout()
    {
        return !(options_->maxDeviceTimeoutCounter_ != 0 && (deviceTimeoutCounter_++ > options_->maxDeviceTimeoutCounter_) );
        // deviceTimeoutCounter_ is only increased if options_->maxDeviceTimeoutCounter != 0
    }

 protected:
    const DeviceOptions* options_;

    std::atomic<unsigned int> deviceTimeoutCounter_;

    std::atomic<State> state_;

    //!  reference to the CAN bus the device is connected to
    Bus* bus_;
};

} /* namespace tcan */
