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

#include "tcan_ethercat/EtherCatSlaveOptions.hpp"
#include "tcan_ethercat/EtherCatDatagram.hpp"

#include "message_logger/message_logger.hpp"

namespace tcan_ethercat {

class EtherCatBus;

//! A slave that is connected via EtherCat.
class EtherCatSlave {
 public:
    enum State {
        Initializing = 0,
        Active = 1,
        Error = -2,
        Missing = -1
    };

    /*! Constructor by address and name.
     *  @param address address of the device
     *  @param name    human-readable name of the device
     */
    EtherCatSlave(const uint32_t address, const std::string& name)
    :   EtherCatSlave(std::unique_ptr<EtherCatSlaveOptions>(new EtherCatSlaveOptions(address, name))) {}

    /*! Constructor by options.
     *  @param options EtherCAT options.
     */
    EtherCatSlave(std::unique_ptr<EtherCatSlaveOptions>&& options)
    :   options_(std::move(options)),
        deviceTimeoutCounter_(0),
        state_(Initializing) {}

    //! Destructor
    virtual ~EtherCatSlave() {}

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

 public: /// Internal functions
    /*! Initialize the device. This function is automatically called by EtherCatBus::addDevice(..).
     *  Calls the initDevice() function.
     */
    inline bool initDeviceInternal(EtherCatBus* bus) {
        bus_ = bus;
        return initDevice();
    }

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
     * Check if the device is timed out.
     * @return True if the device timed out
     */
    inline bool isTimedOut() {
      // deviceTimeoutCounter_ is only increased if options_->maxDeviceTimeoutCounter != 0
        return (options_->maxDeviceTimeoutCounter_ != 0 && (deviceTimeoutCounter_++ > options_->maxDeviceTimeoutCounter_));
    }

 protected:
    const std::unique_ptr<EtherCatSlaveOptions> options_;

    std::atomic<unsigned int> deviceTimeoutCounter_;

    std::atomic<State> state_;

    //! Pointer to the EtherCat bus the device is connected to.
    EtherCatBus* bus_ = nullptr;
};

} /* namespace tcan_ethercat */
