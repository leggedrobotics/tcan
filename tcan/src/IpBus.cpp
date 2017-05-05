/*
 * Bus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <poll.h>

#include "tcan/IpBus.hpp"
#include "message_logger/message_logger.hpp"


namespace tcan {

constexpr uint16_t maxMessageSize = 512;

IpBus::IpBus(std::unique_ptr<IpBusOptions>&& options):
    Bus<IpMsg>(std::move(options)),
	socket_(-1),
    deviceTimeoutCounter_(0)
{

}

IpBus::~IpBus()
{
    stopThreads(true);

    close(socket_);
}

void IpBus::sanityCheck() {
    const unsigned int maxTimeout = static_cast<const IpBusOptions*>(options_.get())->maxDeviceTimeoutCounter_;
    isMissingDeviceOrHasError_ = (maxTimeout != 0 && (deviceTimeoutCounter_++ > maxTimeout) );
    allDevicesActive_ = !isMissingDeviceOrHasError_;
}


bool IpBus::initializeInterface() {

	const IpBusOptions* options = static_cast<const IpBusOptions*>(options_.get());
	const char* interface = options->name_.c_str();

	/* open socket */
	socket_ = socket(AF_INET, SOCK_STREAM, 0);
	if(socket_ < 0) {
		MELO_FATAL("Opening socket %s failed: %d", interface, socket_);
		return false;
	}

	sockaddr_in server;

    server.sin_addr.s_addr = inet_addr(interface);
    server.sin_family = AF_INET;
    server.sin_port = htons( options->port_ );

    //Connect to remote server
    if (connect(socket_ , (struct sockaddr *)&server , sizeof(server)) < 0) {
    	MELO_ERROR("Failed to connect to host %s:%d: %s", interface, options->port_, strerror(errno));
        return false;
    }

    // set nonblocking
    int flags = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

bool IpBus::readData() {
    pollfd fds = {socket_, POLLIN, 0};

    const int ret = poll( &fds, 1, 1000 );

    if ( ret == -1 ) {
        MELO_ERROR("poll failed on bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
        return false;
    }else if ( ret == 0 || !(fds.revents & POLLIN) ) {
        // poll timed out, without being able to read => return silently
        return false;
    }else{
        uint8_t buf[maxMessageSize];
        const int bytes_read = recv( socket_, &buf, maxMessageSize, 0);

        if(bytes_read<=0) {
            // failed to read something even with poll(..) reporting availabe data => raise error
            MELO_ERROR("read failed on IP interface %s:\n  %s", options_->name_.c_str(), strerror(errno));
            return false;
        } else {
            handleMessage( IpMsg(bytes_read, buf) );
        }
    }

    return true;
}

bool IpBus::writeData(const IpMsg& msg) {
    pollfd fds = {socket_, POLLOUT, 0};

    int ret = poll( &fds, 1, 1000 );

    if ( ret == -1 ) {
        MELO_ERROR("poll failed on bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
        return false;
    }else if ( ret == 0 || !(fds.revents & POLLOUT) ) {
        // poll timed out, without being able to read => raise error
        MELO_WARN("polling for fileDescriptor writeability timed out for bus %s. Overflow?", options_->name_.c_str());
        return false;
    }else{
        if( ( ret = send(socket_, msg.getData(), msg.getLength(), 0) ) != static_cast<int>(msg.getLength())) {
            MELO_ERROR("Error at sending TCP/UDP message on interface %s (return value=%d, length=%d):\n  %s", options_->name_.c_str(), ret, msg.getLength(), strerror(errno));
            return false;
        }
    }
    return true;
}

} /* namespace tcan */

