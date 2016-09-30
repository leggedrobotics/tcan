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

    /*! Read and parse messages from all buses. Call this function in the control loop if synchronous mode is used.
     */
    void readMessagesSynchrounous();

    /*! Send the messages in the output queue on all buses. Call this function in the control loop if synchronous mode is used.
     */
    void writeMessagesSynchronous();

    /*! Call sanityCheck(..) on all buses. Call this function in the control loop if synchronous mode is used.
     */
    bool sanityCheckSynchronous();

    void closeBuses();

 protected:
    std::vector<Bus*> buses_;

};

} /* namespace tcan */
