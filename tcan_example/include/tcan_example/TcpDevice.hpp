/*
 * TcpDevice.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/IpBus.hpp"

namespace tcan_example {

class TcpDevice : public tcan::IpBus {
 public:
	TcpDevice() = delete;
	TcpDevice(tcan::IpBusOptions* options);

    virtual ~TcpDevice();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const tcan::IpMsg& msg);

 private:

};

} /* namespace tcan */

