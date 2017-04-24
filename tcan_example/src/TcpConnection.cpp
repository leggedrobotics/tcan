/*
 * TcpConnection.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */


#include "tcan_example/TcpConnection.hpp"
#include "message_logger/message_logger.hpp"

namespace tcan_example {

TcpConnection::TcpConnection(std::unique_ptr<tcan::IpBusOptions>&& options):
    tcan::IpBus(std::move(options))
{

}

TcpConnection::~TcpConnection()
{
}

void TcpConnection::handleMessage(const tcan::IpMsg& msg) {
    std::cout << " got data: " << msg.getData() << std::endl;
}

} /* namespace tcan */

