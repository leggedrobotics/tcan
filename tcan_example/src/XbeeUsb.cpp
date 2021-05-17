#include "tcan_example/XbeeUsb.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan_example {

XbeeUsb::XbeeUsb(std::unique_ptr<tcan_usb::UniversalSerialBusOptions>&& options):
    tcan_usb::UniversalSerialBus(std::move(options))
{

}

XbeeUsb::~XbeeUsb()
{
}

void XbeeUsb::handleMessage(const tcan_usb::UsbMsg& msg) {
    std::cout << " got data: " << msg.getData() << std::endl;


    // clear the error message flag, indicating that the received message is valid.
    // This is used for the passive bus option
    errorMsgFlag_ = false;
}

} /* namespace tcan_usb */
