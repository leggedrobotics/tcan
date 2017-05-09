/*
 * BusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusManager.hpp"
#include "tcan/CanBus.hpp"
#include "tcan/CanMsg.hpp"

namespace tcan {

//! Container of all CAN buses
class CanBusManager : public BusManager<CanMsg> {
 public:
    CanBusManager();

    virtual ~CanBusManager();

    /*!
     * @param index     Index of the bus
     * @return          Pointer to CanBus instance
     */
    CanBus* getCanBus(const unsigned int index) { return static_cast<CanBus*>(buses_[index]); }


    /*! Send a sync message on all buses
     * @param waitForEmptyQueues     wheter the busmanager should wait until the output message queues of all buses are empty before sending the global SYNC.
     * 			ensures that the sync messages are immediatly sent at the same time and not just appended to a queue.
     * 			Only useful in asynchronous mode.
     */
    void sendSyncOnAllBuses(const bool waitForEmptyQueues=false);

    /*! Send a sync on a single bus
     * @param busIndex  The index of the bus to send the SYNC message on
     */
    void sendSync(const unsigned int busIndex);

    /*! Checks if a error message was received on one of the buses.
     * @return true if a error message was received
     */
    bool getErrorMsgFlag() const;

    /*! Checks if a error message was received on one of the buses and resets the flag.
     * @return true if a error message was received
     */
    bool resetErrorMsgFlag();

    /*!
     * Resets all devices handled by all buses to Initializing state and sends appropriate restart commands to the devices
     */
    void resetAllDevices();

};

} /* namespace tcan */
