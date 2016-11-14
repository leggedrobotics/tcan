/*
 * TcpDevice.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */


#include "tcan_example/TcpDevice.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan_example {

TcpDevice::TcpDevice(tcan::IpBusOptions* options):
    tcan::IpBus(options)
{

}

TcpDevice::~TcpDevice()
{
}

void TcpDevice::handleMessage(const tcan::IpMsg& msg) {
    std::cout << " got data: " << msg.getData() << std::endl;
}

} /* namespace tcan */

