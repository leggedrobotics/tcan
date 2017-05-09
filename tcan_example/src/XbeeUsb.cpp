/*
 * XbeeUsb.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */


#include "tcan_example/XbeeUsb.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan_example {

XbeeUsb::XbeeUsb(std::unique_ptr<tcan::UniversalSerialBusOptions>&& options):
    tcan::UniversalSerialBus(std::move(options))
{

}

XbeeUsb::~XbeeUsb()
{
}

void XbeeUsb::handleMessage(const tcan::UsbMsg& msg) {
    std::cout << " got data: " << msg.getData() << std::endl;


    // clear the error message flag, indicating that the received message is valid.
    // This is used for the passive bus option
    errorMsgFlag_ = false;
}

} /* namespace tcan */

