#pragma once

#include "tcan/BusManager.hpp"
#include "tcan_can/CanBus.hpp"
#include "tcan_can/CanMsg.hpp"

namespace tcan_can {

//! Container of all CAN buses
class CanBusManager : public tcan::BusManager<CanMsg> {
 public:
    CanBusManager() = default;
    ~CanBusManager() override = default;

    /*!
     * @param index     Index of the bus
     * @return          Pointer to CanBus instance
     */
    inline CanBus* getCanBus(const unsigned int index) { return static_cast<CanBus*>(buses_[index]); }


    /*! Send a sync message on all buses
     * @param waitForEmptyQueues     whether the busmanager should wait until the output message queues of all buses are empty before sending the global SYNC.
     * 			ensures that the sync messages are sent at the same time and not just appended to a queue.
     * 			Only useful in asynchronous mode.
     */
    void sendSyncOnAllBuses(const bool waitForEmptyQueues=false);

    /*! Send a sync on a single bus
     * @param busIndex  The index of the bus to send the SYNC message on
     */
    void sendSync(const unsigned int busIndex);

    /*!
     * Resets all devices handled by all buses to Initializing state and sends appropriate restart commands to the devices
     */
    void resetAllDevices();

};

} /* namespace tcan_can */
