/*
 * SocketBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <poll.h>

#include "tcan/Bus.hpp"
#include "tcan/SocketBusOptions.hpp"

namespace tcan {

class SocketBus : public Bus {
 public:

    SocketBus(const std::string& interface);
    SocketBus(BusOptions* options) = delete;
    SocketBus(SocketBusOptions* options);

    virtual ~SocketBus();

 protected:
    virtual bool initializeCanBus();
    virtual bool readCanMessage();
    virtual bool writeCanMessage(const CanMsg& cmsg);

 protected:
    pollfd socket_;
};

} /* namespace tcan */
