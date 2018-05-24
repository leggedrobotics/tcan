/*
 * TcpDevice.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan_ip/IpBus.hpp"

namespace tcan_example {

class TcpConnection : public tcan_ip::IpBus {
 public:
    TcpConnection() = delete;
    TcpConnection(std::unique_ptr<tcan_ip::IpBusOptions>&& options);

    ~TcpConnection() override;

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    void handleMessage(const tcan_ip::IpMsg& msg) override;

 private:

};

} /* namespace tcan_example */
