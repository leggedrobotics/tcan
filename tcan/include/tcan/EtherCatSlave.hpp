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

#include "tcan/EtherCatSlaveOptions.hpp"
#include "tcan/EtherCatDatagram.hpp"

#include "message_logger/message_logger.hpp"

namespace tcan {

class EtherCatBus;

//! A slave that is connected via EtherCat.
class EtherCatSlave {
 public:
    enum State {
        Initializing=0,
        Active=1,
        Error=-2,
        Missing=-1
    };

    /*! Constructor
     *  @param address address of the device
     *  @param name    human-readable name of the device
     */
    EtherCatSlave() = delete;

    EtherCatSlave(const uint32_t address, const std::string& name)
    :   EtherCatSlave(new EtherCatSlaveOptions(address, name)) {
    }

    EtherCatSlave(EtherCatSlaveOptions* options)
    :   options_(options),
        deviceTimeoutCounter_(0),
        state_(Initializing) {
    }

    //! Destructor
    virtual ~EtherCatSlave() {
        delete options_; // TODO why are the options not (always) destroyed where they are created?
    }

    /*! Initialize the device. This function is automatically called by EtherCatBus::addDevice(..).
     *  (through initDeviceInternal(..))
     *  This function is intended to do some initial device initialization (register messages to be received,
     *  restart remote node, ...)
     *  @return true if successfully initialized
     */
    virtual bool initDevice() = 0;

    /*! Initialize the device's communication. This function is automatically called by EtherCatBus::initializeInterface(..).
     *  This function is intended to set up the communication between master and slave.
     *  @return true if successfully initialized
     */
    virtual bool initializeInterface() = 0;

    /*! Configure the device
     *  This function is automatically called after reception of a
     *  bootup message. (or more general: After reception of any message if the device was in state Missing or Initializing)
     *  @param msg   received message which caused the call of this function
     *  @return      true if device is active
     */
//    virtual bool configureDevice(const CanMsg& msg) {
//        return true;
//    }

    /*! Do a sanity check of the device. This function is intended to be called with constant rate
     *  and shall check heartbeats, SDO timeouts, ...
     *  This function is automatically called if the Bus has asynchronous=true and sanityCheckInterval > 0
     *  @return true if everything is ok.
     */
    virtual void sanityCheck() {
        if(!isMissing()) {
            if(isTimedOut()) {
                state_ = Missing;
                MELO_WARN("Slave %s timed out!", getName().c_str());
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
    /*! Initialize the device. This function is automatically called by EtherCatBus::addDevice(..).
     *  Calls the initDevice() function.
     */
    inline bool initDeviceInternal(EtherCatBus* bus) {
        bus_ = bus;
        return initDevice();
    }

//    inline void configureDeviceInternal(const CanMsg& msg) {
//        if(state_ != Active && state_ != Error) {
//            if(configureDevice(msg)) {
//                state_ = Active;
//                if(options_->printConfigInfo_) {
//                    MELO_INFO("Device %s configured successfully.", options_->name_.c_str());
//                }
//            }
//        }
//    }

    inline void resetDeviceTimeoutCounter() {
        deviceTimeoutCounter_ = 0;
    }

    void syncDistributedClocks(const bool activate);

    template <typename Value>
    void sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value);

    template <typename Value>
    void sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

    void sendSdoReadAndPrint(const uint16_t index, const uint8_t subindex, const bool completeAccess);

 protected:
    /*!
     * @return True if the device timed out
     */
    inline bool isTimedOut() {
        return (options_->maxDeviceTimeoutCounter_ != 0 && (deviceTimeoutCounter_++ > options_->maxDeviceTimeoutCounter_) );
        // deviceTimeoutCounter_ is only increased if options_->maxDeviceTimeoutCounter != 0
    }

 protected:
    const EtherCatSlaveOptions* options_ = nullptr;

    std::atomic<unsigned int> deviceTimeoutCounter_;

    std::atomic<State> state_;

    //! Pointer to the EtherCat bus the device is connected to.
    EtherCatBus* bus_ = nullptr;
};

} /* namespace tcan */
