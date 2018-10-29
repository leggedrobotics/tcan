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

    /*!
     * Constructor by address and name.
     * @param address Address of the slave.
     * @param name    Human-readable name of the slave.
     */
    EtherCatSlave(const uint32_t address, const std::string& name)
    :   EtherCatSlave(std::unique_ptr<EtherCatSlaveOptions>(new EtherCatSlaveOptions(address, name))) {}

    /*!
     * Constructor by EtherCAT slave options.
     * @param options EtherCAT options.
     */
    EtherCatSlave(std::unique_ptr<EtherCatSlaveOptions>&& options)
    :   options_(std::move(options)),
        deviceTimeoutCounter_(0),
        state_(Initializing) {}

    /*!
     * Destructor
     */
    virtual ~EtherCatSlave() {}

    /*!
     * Initialize the device. This function is automatically called by EtherCatBus::addDevice(..).
     * (through initDeviceInternal(..))
     * This function is intended to do some initial device initialization (register messages to be received,
     * restart remote node, ...).
     * @return True if successful.
     */
    virtual bool initDevice() = 0;

    /*!
     * Initialize the device's communication. This function is automatically called by EtherCatBus::initializeInterface(..).
     * This function is intended to set up the communication between master and slave.
     * @return True if successful.
     */
    virtual bool initializeInterface() = 0;

    /*!
     * Do a sanity check of the device. This function is intended to be called with constant rate
     * and shall check heartbeats, SDO timeouts, ...
     * This function is automatically called if the Bus has asynchronous=true and sanityCheckInterval > 0
     * @return True if everything is ok.
     */
    virtual bool sanityCheck() {
        if(!isMissing()) {
            if(isTimedOut()) {
                state_ = Missing;
                MELO_WARN("Slave %s timed out!", getName().c_str());
            }
        }

        return !(isMissing() || hasError());
    }

    /*!
     * Get the address of the slave.
     * @return Address of the slave.
     */
    inline uint32_t getAddress() const { return options_->address_; }

    /*!
     * Get the human-readable name of the slave.
     * @return Name of the slave.
     */
    inline const std::string& getName() const { return options_->name_; }

    /*!
     * Check if the slave is initializing.
     * @return True if the slave is initializing.
     */
    inline bool isInitializing() const { return (state_ == Initializing); }

    /*!
     * Check if the slave is active.
     * @return True if the slave is active.
     */
    inline bool isActive() const { return (state_ == Active); }

    /*!
     * Check if the slave has an error.
     * @return True if the slave has an error.
     */
    inline bool hasError() const { return (state_ == Error); }

    /*!
     * Check if the slave is missing.
     * @return True if the slave is missing.
     */
    inline bool isMissing() const { return (state_ == Missing); }

    /*!
     * Get the state of the slave.
     * @return State of the slave.
     */
    virtual int getState() const { return static_cast<int>(state_.load()); }

 public:
    /*!
     * Initialize the device. This function is automatically called by EtherCatBus::addDevice(..).
     * Calls the initDevice() function.
     * @param bus EtherCAT bus this slave belongs to.
     * @return True if successful.
     */
    inline bool initDeviceInternal(EtherCatBus* bus) {
        bus_ = bus;
        return initDevice();
    }

    /*!
     * Reset the slaves timeout counter.
     */
    inline void resetDeviceTimeoutCounter() {
        deviceTimeoutCounter_ = 0;
    }

    /*!
     * Synchronize the distribute clock.
     * @param activate True to activate, false to deactivate.
     */
    void syncDistributedClocks(const bool activate);

    /*!
     * Send a writing SDO.
     * @param index          Index of the SDO.
     * @param subindex       Sub-index of the SDO.
     * @param completeAccess Access all sub-inidices at once.
     * @param value          Value to write.
     * @return True if successful.
     */
    template <typename Value>
    bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value);

    /*!
     * Send a reading SDO.
     * @param index          Index of the SDO.
     * @param subindex       Sub-index of the SDO.
     * @param completeAccess Access all sub-inidices at once.
     * @param value          Return argument, will contain the value which was read.
     * @return True if successful.
     */
    template <typename Value>
    bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

    template <typename T>
    bool changeSdoParameter(const uint16_t index,
                          const uint8_t subindex,
                          const bool completeAccess,
                          const T value,
                          const std::string & name,
                          const bool verbose = true)
    {
      T temp;
      sendSdoWrite(index, subindex, completeAccess, value );
      sendSdoRead(index, subindex, completeAccess, temp);
      if(temp == value) {
          MELO_INFO_STREAM("Set " << name << " to " << value << "!");
      } else{
          MELO_WARN_STREAM("Could not set " << name << " to " << value << ", " << name << " is " << temp << "!");
      }
      return (temp == value);
    }

    template <typename T>
    void printSdoParameter(const uint16_t index,
                           const uint8_t subindex,
                           const bool completeAccess,
                           const std::string & name)
    {
      T temp;
      sendSdoRead(index, subindex, completeAccess, temp);
      MELO_INFO_STREAM(name << " is " << temp << "!");
    }

 protected:
    /*!
     * Check if the device is timed out.
     * @return True if the device timed out.
     */
    inline bool isTimedOut() {
      // deviceTimeoutCounter_ is only increased if options_->maxDeviceTimeoutCounter != 0
        return (options_->maxDeviceTimeoutCounter_ != 0 && (deviceTimeoutCounter_++ > options_->maxDeviceTimeoutCounter_));
    }

 protected:
    //! Pointer to the EtherCAT slave options.
    const std::unique_ptr<EtherCatSlaveOptions> options_;

    //! Slave timeout counter.
    std::atomic<unsigned int> deviceTimeoutCounter_;

    //! State of the slave.
    std::atomic<State> state_;

    //! Pointer to the EtherCat bus the device is connected to.
    EtherCatBus* bus_ = nullptr;
};

} /* namespace tcan_ethercat */
