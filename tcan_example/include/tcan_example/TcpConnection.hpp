/*
 * TcpDevice.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/IpBus.hpp"

namespace tcan_example {

class TcpConnection : public tcan::IpBus {
 public:
    TcpConnection() = delete;
    TcpConnection(tcan::IpBusOptions* options);

    virtual ~TcpConnection();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const tcan::IpMsg& msg);

 private:

};

} /* namespace tcan */

