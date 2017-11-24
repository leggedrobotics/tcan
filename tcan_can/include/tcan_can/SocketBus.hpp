/*
 * SocketBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <memory>

#include "tcan_can/CanBus.hpp"
#include "tcan_can/SocketBusOptions.hpp"

namespace tcan_can {

class SocketBus : public CanBus {
 public:

    SocketBus(const std::string& interface);
    SocketBus(std::unique_ptr<SocketBusOptions>&& options);

    virtual ~SocketBus();

    int getPollableFileDescriptor() { return socket_; }

protected:
    bool initializeInterface();
    bool readData();
    bool writeData(std::unique_lock<std::mutex>* lock);

    /*!
     * Is called on reception of a bus error message. Sets the flag
     * @param msg  reference to the bus error message
     */
    void handleBusError(const can_frame& msg);

 protected:
    int socket_;
    int recvFlag_;
    int sendFlag_;
};

} /* namespace tcan_can */
