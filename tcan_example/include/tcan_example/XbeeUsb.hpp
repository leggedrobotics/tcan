/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan_usb/UniversalSerialBus.hpp"

namespace tcan_example {

class XbeeUsb : public tcan_usb::UniversalSerialBus {
 public:
    XbeeUsb() = delete;
    XbeeUsb(std::unique_ptr<tcan_usb::UniversalSerialBusOptions>&&);

    virtual ~XbeeUsb();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const tcan_usb::UsbMsg& msg);

 private:

};

} /* namespace tcan_example */
