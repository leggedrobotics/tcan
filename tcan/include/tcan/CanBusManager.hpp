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

    CanBus* getCanBus(const unsigned int index) { return static_cast<CanBus*>(buses_[index]); }


    /*! Send a sync message on all buses
     * @param	wheter the busmanager should wait until the output message queues of all buses are empty before sending the global SYNC.
     * 			ensures that the sync messages are immediatly sent at the same time and not just appended to a queue.
     * 			Only useful in asynchronous mode.
     */
    void sendSyncOnAllBuses(const bool waitForEmptyQueues=false);

    /*! Send a sync on a single bus
     * @param busIndex  The index of the bus to send the SYNC message on
     */
    void sendSync(const unsigned int busIndex);

};

} /* namespace tcan */
