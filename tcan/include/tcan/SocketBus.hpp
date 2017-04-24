/*
 * SocketBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <poll.h>

#include "tcan/CanBus.hpp"
#include "tcan/SocketBusOptions.hpp"

namespace tcan {

class SocketBus : public CanBus {
 public:

    SocketBus(const std::string& interface);
    SocketBus(std::unique_ptr<BusOptions>&& options) = delete;
    SocketBus(std::unique_ptr<SocketBusOptions>&& options);

    virtual ~SocketBus();

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
    pollfd socket_;
};

} /* namespace tcan */
