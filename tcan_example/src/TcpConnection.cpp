#include "message_logger/message_logger.hpp"
#include "tcan_example/TcpConnection.hpp"

namespace tcan_example {

TcpConnection::TcpConnection(std::unique_ptr<tcan_ip::IpBusOptions>&& options):
    tcan_ip::IpBus(std::move(options))
{

}

TcpConnection::~TcpConnection()
{
}

void TcpConnection::handleMessage(const tcan_ip::IpMsg& msg) {
    std::cout << " got data: " << msg.getData() << std::endl;

    // clear the error message flag, indicating that the received message is valid.
    // This is used for the passive bus option
    errorMsgFlag_ = false;
}

} /* namespace tcan_example */
