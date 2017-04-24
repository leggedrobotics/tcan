/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/UniversalSerialBus.hpp"

namespace tcan_example {

class XbeeUsb : public tcan::UniversalSerialBus {
 public:
    XbeeUsb() = delete;
    XbeeUsb(std::unique_ptr<tcan::UniversalSerialBusOptions>&&);

    virtual ~XbeeUsb();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const tcan::UsbMsg& msg);

 private:

};

} /* namespace tcan */

