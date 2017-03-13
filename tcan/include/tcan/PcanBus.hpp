/*
 * PcanBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <libpcan.h>
#include "tcan/CanBus.hpp"
#include "tcan/PcanBusOptions.hpp"


namespace tcan {

class PcanBus : public CanBus {
 public:

    PcanBus(const std::string& interface);
    PcanBus(BusOptions* options) = delete;
    PcanBus(PcanBusOptions* options);

    virtual ~PcanBus();

 protected:
    bool initializeInterface();
    bool readData();
    bool writeData(const CanMsg& cmsg);

    /*!
     * Is called on reception of a bus error message. Sets the flag
     * @param msg  reference to the bus error message
     */
    void handleBusError(const can_frame& msg);

 protected:
    Handle handle_;
};

} /* namespace tcan */
