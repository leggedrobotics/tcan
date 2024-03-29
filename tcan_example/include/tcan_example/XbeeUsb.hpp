#pragma once

#include "tcan_usb/UniversalSerialBus.hpp"

namespace tcan_example {

class XbeeUsb : public tcan_usb::UniversalSerialBus {
 public:
    XbeeUsb() = delete;
    XbeeUsb(std::unique_ptr<tcan_usb::UniversalSerialBusOptions>&&);

    ~XbeeUsb() override;

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    void handleMessage(const tcan_usb::UsbMsg& msg) override;

 private:

};

} /* namespace tcan_example */
