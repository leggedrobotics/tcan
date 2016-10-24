/*
 * BusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <memory>
#include <vector>

#include "tcan/Bus.hpp"

namespace tcan {

//! Container of all CAN buses
class BusManager {
 public:
    BusManager();

    virtual ~BusManager();

    bool addBus(Bus* bus);

    /*! Gets the number of buses
     * @return	number of buses
     */
    unsigned int getSize() const { return buses_.size(); }

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

    /*! Read and parse messages from all buses. Call this function in the control loop if synchronous mode is used.
     */
    void readMessagesSynchrounous();

    /*! Send the messages in the output queue on all buses. Call this function in the control loop if synchronous mode is used.
     */
    void writeMessagesSynchronous();

    /*! Call sanityCheck(..) on all buses. Call this function in the control loop if synchronous mode is used.
     */
    bool sanityCheckSynchronous();

    /*!
     * Check operational state of all buses.
     * @return  True if all devices on all buses are operational
     */
    bool allBusesOperational() const;

    void closeBuses();

 protected:
    std::vector<Bus*> buses_;

};

} /* namespace tcan */
