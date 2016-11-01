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
    SocketBus(BusOptions* options) = delete;
    SocketBus(SocketBusOptions* options);

    virtual ~SocketBus();

 protected:
    bool initializeInterface();
    bool readData();
    bool writeData(const CanMsg& cmsg);

 protected:
    pollfd socket_;
};

} /* namespace tcan */
