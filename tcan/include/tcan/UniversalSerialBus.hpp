/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/Bus.hpp"
#include "tcan/UniversalSerialBusOptions.hpp"
#include "tcan/UsbMsg.hpp"

namespace tcan {

class UniversalSerialBus : public Bus<UsbMsg> {
 public:
    UniversalSerialBus() = delete;
    UniversalSerialBus(UniversalSerialBusOptions* options);

    virtual ~UniversalSerialBus();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const UsbMsg& msg) = 0;

    /*! Do a sanity check of all devices on this bus.
     */
    bool sanityCheck();

 protected:
    bool initializeInterface();
    bool readData();
    bool writeData(const UsbMsg& msg);


 private:
    void configureInterface();

 private:
    int fileDescriptor_;

    unsigned int deviceTimeoutCounter_;
};

} /* namespace tcan */

