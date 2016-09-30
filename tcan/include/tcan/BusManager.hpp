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
    void readMessagesSynchrounous() {
        for(auto bus : buses_) {
            if(!bus->isAsynchronous()) {
                while(bus->readMessage()) {
                }
            }
        }
    }
    /*! Send the messages in the output queue on all buses. Call this function in the control loop if synchronous mode is used.
     */
    void writeMessagesSynchronous() {
        bool sendingData = true;

        while(sendingData) {
            sendingData = false;

            for(auto bus : buses_) {
                if(!bus->isAsynchronous()) {
                    sendingData |= bus->writeMessage();
                }
            }
        }
    }
    /*! Call sanityCheck(..) on all buses. Call this function in the control loop if synchronous mode is used.
     */
    bool sanityCheckSynchronous() {
        bool allFine = true;
        for(auto bus : buses_) {
            if(!bus->sanityCheck()) {
                allFine = false;
            }
        }

        return allFine;
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
