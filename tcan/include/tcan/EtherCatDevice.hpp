/*
 * EthernetCatDevice.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#pragma once

#include <stdint.h>
#include <string>
#include <atomic>

#include "tcan/EthernetFrame.hpp"
#include "tcan/EtherCatDeviceOptions.hpp"

#include "message_logger/message_logger.hpp"

namespace tcan {
class EtherCatBus;

//! A device that is connected via EtherCat.
class EtherCatDevice {
 public:
    enum State {
        Initializing=0,
        Active=1,
        Error=-2,
        Missing=-1
    };

    /*! Constructor
     * @param address	address of the device
     * @param name		human-readable name of the device
     */
    EtherCatDevice() = delete;

    EtherCatDevice(const uint32_t address, const std::string& name)
    :   EtherCatDevice(new EtherCatDeviceOptions(address, name)) {
    }

    EtherCatDevice(EtherCatDeviceOptions* options):
        options_(options),
        deviceTimeoutCounter_(0),
        state_(Initializing),
        bus_(nullptr) {
    }

    //! Destructor
    virtual ~EtherCatDevice() {
        delete options_;
    }

    /*! Initialize the device. This function is automatically called by Bus::addDevice(..)
     *   (through initDeviceInternal(..))
     * This function is intended to do some initial device initialization (register messages to be received,
     *   restart remote node, ...)
     * @return true if successfully initialized
     */
    virtual bool initDevice() {
        return true;
    }

    /*!
     * Configure the device
     * This function is automatically called after reception of a
     * bootup message. (or more general: After reception of any message if the device was in state Missing or Initializing)
     * @param msg   received message which caused the call of this function
     * @return      true if device is active
     */
    virtual bool configureDevice(const CanMsg& msg) {
        return true;
    }

    /*! Do a sanity check of the device. This function is intended to be called with constant rate
     * and shall check heartbeats, SDO timeouts, ...
     * This function is automatically called if the Bus has asynchronous=true and sanityCheckInterval > 0
     * @return true if everything is ok.
     */
    virtual void sanityCheck() {
        if(!isMissing()) {
            if(isTimedOut()) {
                state_ = Missing;
                MELO_WARN("Device %s timed out!", getName().c_str());
            }
        }
    }

    inline uint32_t getAddress() const { return options_->address_; }
    inline const std::string& getName() const { return options_->name_; }

    inline bool isInitializing() const { return (state_ == Initializing); }
    inline bool isActive() const { return (state_ == Active); }
    inline bool hasError() const { return (state_ == Error); }
    inline bool isMissing() const { return (state_ == Missing); }

    virtual int getStatus() const { return static_cast<int>(state_.load()); }

    /*!
     * Resets the device to Initializing state
     */
    virtual void resetDevice() { state_ = Initializing; }

 public: /// Internal functions
    /*! Initialize the device. This function is automatically called by Bus::addDevice(..).
     * Calls the initDevice() function.
     */
    inline bool initDeviceInternal(EtherCatBus* bus) {
        bus_ = bus;
        return initDevice();
    }

    inline void configureDeviceInternal(const CanMsg& msg) {
        if(state_ != Active && state_ != Error) {
            if(configureDevice(msg)) {
                state_ = Active;
                if(options_->printConfigInfo_) {
                    MELO_INFO("Device %s configured successfully.", options_->name_.c_str());
                }
            }
        }
    }

    inline void resetDeviceTimeoutCounter() {
        deviceTimeoutCounter_ = 0;
    }

 protected:
    /*!
     * @return True if the device timed out
     */
    inline bool isTimedOut()
    {
        return (options_->maxDeviceTimeoutCounter_ != 0 && (deviceTimeoutCounter_++ > options_->maxDeviceTimeoutCounter_) );
        // deviceTimeoutCounter_ is only increased if options_->maxDeviceTimeoutCounter != 0
    }

 protected:
    const EtherCatDeviceOptions* options_ = nullptr;

    std::atomic<unsigned int> deviceTimeoutCounter_;

    std::atomic<State> state_;

    //!  reference to the EtherCat bus the device is connected to
    EtherCatBus* bus_ = nullptr;
};

} /* namespace tcan */
