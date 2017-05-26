/*
 * BusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <vector>

#include "tcan/Bus.hpp"

namespace tcan {

//! Container of all CAN buses
template <class Msg>
class BusManager {
 public:
    BusManager():
        buses_()
    {
    }

    virtual ~BusManager()
    {
        closeBuses();
    }

    bool addBus(Bus<Msg>* bus) {
        buses_.push_back( bus );
        return bus->initBus();
    }
    /*! Gets the number of buses
     * @return	number of buses
     */
    unsigned int getSize() const { return buses_.size(); }

    /*! Read and parse messages from all buses. Call this function in the control loop if synchronous mode is used.
     */
    void readMessagesSynchronous() {
        for(auto bus : buses_) {
            if(!bus->isAsynchronous()) {
                while(bus->readMessage()) {
                }
            }
        }
    }
    /*!
     * Send the messages in the output queue on all buses. Call this function in the control loop if synchronous mode is used.
     * Note that this function may not send all the messages in the output queue if BlockingWrite is disabled (see BusOptions)
     * @return  False if at least one write error occurred
     */
    bool writeMessagesSynchronous() {
        bool sendingData = true;
        bool noError = true;
        while(sendingData) {
            sendingData = false;

            for(auto bus : buses_) {
                if(!bus->isAsynchronous() && bus->getNumOutogingMessagesWithoutLock() > 0) {
                    noError &= bus->writeMessagesWithoutLock();
                    sendingData = true;
                }
            }
        }
        return noError;
    }

    /*! Call sanityCheck(..) on all buses. Call this function in the control loop if synchronous mode is used.
     */
    bool sanityCheckSynchronous() {
        bool allFine = true;
        for(auto bus : buses_) {
            bus->sanityCheck();
            allFine &= bus->allDevicesActive();
        }

        return allFine;
    }

    /*!
     * Check if no device timed out
     * @return  True if at least one device is missing
     */
    bool isMissingDeviceOrHasError() const {
        for(auto bus : buses_) {
            if(bus->isMissingDeviceOrHasError()) {
                return true;
            }
        }
        return false;
    }

    /*!
     * check if we received a message from all devices within timeout
     * @return True if all devices are active
     */
    bool allDevicesActive() const {
        for(auto bus : buses_) {
            if(!(bus->allDevicesActive())) {
                return false;
            }
        }
        return true;
    }


    void closeBuses() {
        for(Bus<Msg>* bus : buses_) {
            bus->stopThreads(false);
        }
        for(Bus<Msg>* bus : buses_) {
            delete bus;
        }

        buses_.clear();
    }

 protected:
    std::vector<Bus<Msg>*> buses_;

};

} /* namespace tcan */
