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

#include "tcan_ip/IpBus.hpp"
#include "message_logger/message_logger.hpp"


namespace tcan_ip {

constexpr uint16_t maxMessageSize = 512;

IpBus::IpBus(std::unique_ptr<IpBusOptions>&& options):
    tcan::Bus<IpMsg>(std::move(options)),
	socket_(-1),
	recvFlag_(0),
	sendFlag_(0),
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
    allDevicesMissing_ = isMissingDeviceOrHasError_.load();
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

    // set read timeout
    if (options_->readTimeout_.tv_sec != 0 || options_->readTimeout_.tv_usec != 0) {
        if(setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (char*)&options->readTimeout_, sizeof(options->readTimeout_)) != 0) {
            MELO_WARN("Failed to set read timeout:\n  %s", strerror(errno));
        }
    }

    // set write timeout
    if (options_->writeTimeout_.tv_sec != 0 || options_->writeTimeout_.tv_usec != 0) {
        if(setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, (char*)&options->writeTimeout_, sizeof(options->writeTimeout_)) != 0) {
            MELO_WARN("Failed to set write timeout:\n  %s", strerror(errno));
        }
    }

    // set nonblocking flags for synchronous mode
    if(!isAsynchronous()) {
        recvFlag_ = MSG_DONTWAIT;
        if(!options_->synchronousBlockingWrite_) {
            sendFlag_ = MSG_DONTWAIT;
        }
    }

    return true;
}

bool IpBus::readData() {

    uint8_t buf[maxMessageSize];
    const int bytes_read = recv( socket_, &buf, maxMessageSize, recvFlag_);

    if(bytes_read <= 0) {
        if(errno != EAGAIN && errno != EWOULDBLOCK) {
            MELO_ERROR("Failed to read data from IP interface %s:\n  %s", options_->name_.c_str(), strerror(errno));
        }
        return false;
    } else {
        handleMessage( IpMsg(bytes_read, buf) );
    }

    return true;
}

bool IpBus::writeData(std::unique_lock<std::mutex>* lock) {

    IpMsg msg = outgoingMsgs_.front();
    if(lock != nullptr) {
        lock->unlock();
    }

    const int ret = send(socket_, msg.getData(), msg.getLength(), sendFlag_);
    if( ret != static_cast<int>(msg.getLength())) {
        if(errno != EAGAIN && errno != EWOULDBLOCK) {
            MELO_ERROR("Error at sending TCP/UDP message on interface %s (return value=%d, length=%d):\n  %s", options_->name_.c_str(), ret, msg.getLength(), strerror(errno));
        }
    }else{
        if(lock != nullptr) {
            lock->lock();
        }
        outgoingMsgs_.pop_front();
        return true;
    }

    return false;
}

} /* namespace tcan_ip */
