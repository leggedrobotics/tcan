#pragma once

#include "tcan_can/CanBus.hpp"
#include "tcan_can/SocketBusOptions.hpp"

namespace tcan_can {

class SocketBus : public CanBus {
 public:

    SocketBus(const std::string& interface);
    SocketBus(std::unique_ptr<SocketBusOptions>&& options);

    ~SocketBus() override;

    int getPollableFileDescriptor() const override { return socket_; }

protected:
    bool initializeInterface() override;
    bool readData() override;
    bool writeData(std::unique_lock<std::mutex>* lock) override;

    /*!
     * Is called on reception of a bus error message. Sets the flag
     * @param msg  reference to the bus error message
     */
    void handleBusErrorMessage(const can_frame& msg);

 protected:
    int socket_;
    int recvFlag_;
    int sendFlag_;
};

} /* namespace tcan_can */
