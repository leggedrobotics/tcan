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
    XbeeUsb(tcan::UniversalSerialBusOptions* options);

    virtual ~XbeeUsb();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const tcan::UsbMsg& msg);

    void sendMessage(const tcan::UsbMsg& msg) {
        writeData(msg);
    }

 private:

};

} /* namespace tcan */

