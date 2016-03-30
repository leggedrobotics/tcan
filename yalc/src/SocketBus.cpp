/*
 * SocketBus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "yalc/SocketBus.hpp"

SocketBus::SocketBus(const std::string& interface, const unsigned int baudrate):
	Bus(new SocketBusOptions(interface, baudrate)),
	socket_()
{
}

SocketBus::SocketBus(SocketBusOptions* options):
	Bus(options),
	socket_()
{

}

SocketBus::~SocketBus()
{
	closeBus();
}

bool SocketBus::initializeCanBus()
{
	const auto options = static_cast<SocketBusOptions*>(options_);
	const char* interface = options->interface.c_str();

	/* open socket */
	int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(fd < 0) {
		printf("Opening CAN channel %s failed: %d\n", interface, fd);
		return false;
	}


	/* configure socket */
	struct ifreq ifr;
	strcpy(ifr.ifr_name, interface);
	ioctl(fd, SIOCGIFINDEX, &ifr);

	int loopback = options->loopback; /* 0 = disabled, 1 = enabled (default) */
	if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) != 0) {
		printf("Failed to set loopback mode\n");
		perror("setsockopt");
	}
	int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
	if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) != 0) {
		printf("Failed to set reception of own messages option\n");
		perror("setsockopt");
	}

	// CAN error handling
	can_err_mask_t err_mask = CAN_ERR_MASK; //( CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF );
	if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0) {
		printf("Failed to set error mask\n");
		perror("setsockopt");
	}

	// get default bufer sizes
//	int buf_size;
//	socklen_t len;
//	getsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buf_size, &len);
//	printf("sndbuf size: %d (%d)\n", buf_size, len);
//
//	getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buf_size, &len);
//	printf("rcvbuf size: %d (%d)\n", buf_size, len);

	// On some CAN drivers, the txqueuelen of the netdevice cannot be changed (default=10).
	// The buffer size of the socket is larger than those 10 messages, so the write(..) call never blocks but raises a ENOBUFS error when writing to the netdevice.
	// This renders poll(..) useless. Not the writes in the sockets are the limiting pipe but the writes in the underlying netdevice.
	// The recommended way to fix this is to increase the txqueuelen of the netdevice (e.g. command line: ip link set can0 txqueuelen 1000)
	// If this is not possible, the sndbuf size of the socket can be decrease, such that the socket writes are the limiting pipe, leading to blocking socket write(..) calls. (write becomes poll(..)-able)
	// https://www.mail-archive.com/socketcan-users@lists.berlios.de/msg00787.html
	// http://socket-can.996257.n3.nabble.com/Solving-ENOBUFS-returned-by-write-td2886.html
	if(options->sndBufLength != 0) {
		setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &(options->sndBufLength), sizeof(options->sndBufLength));
	}

//	timeval timeout;
//	timeout.tv_sec = 1;
//	timeout.tv_usec = 0;
//	if(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
//		printf("Failed to set read timeout\n");
//		perror("setsockopt");
//	}


	/* bind socket */
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(struct sockaddr_can));
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	//printf("%s at index %d\n", channelname, ifr.ifr_ifindex);
	if(bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		printf("Error in socket %s bind.\n", interface);
		perror("bind");
		return false;
	}

	socket_ = {fd, POLLIN, 0};

	printf("Opened socket %s.\n", interface);

	return true;
}

bool SocketBus::closeBus() {
	close(socket_.fd);
	return true;
}


bool SocketBus::readCanMessage() {

	// poll the socket, even if it is non-blocking, to provide a timeout
	const int ret = poll( &socket_, 1, 1000 );

	if ( ret == -1 ) {
		printf("poll failed");
		perror("poll()");
		return false;
	}else if ( ret == 0 || !(socket_.revents & POLLIN) ) {
		/*******************************************************
		 * no data received, poll timed out
		 *******************************************************/
	}else{
		/*******************************************************
		 * data ready to be read
		 *******************************************************/
		can_frame frame;
		socket_.revents = 0;
		bool dataAvailable=true;
		do {
			const int bytes_read = read( socket_.fd, &frame, sizeof(struct can_frame));
//			printf("CanManager_ bytes read: %i\n", bytes_read);

			if(bytes_read<=0) {
//				printf("read nothing\n");
				dataAvailable=false;
			} else {
				// printf("CanManager:bus_routine: Data received from iBus %i, n. Bytes: %i \n", iBus, bytes_read);

				handleMessage( CANMsg(frame.can_id, frame.can_dlc, frame.data) );
			}
		} while(dataAvailable);

	}

	return true;
}


bool SocketBus::writeCanMessage(const CANMsg& cmsg) {

	// no need to poll(..) the device if the socket is blocking.

	const unsigned int baudRate = static_cast<SocketBusOptions*>(options_)->baudrate;

	can_frame frame;
	frame.can_id = cmsg.getCOBId();
	frame.can_dlc = cmsg.getLength();
	std::copy(cmsg.getData(), &(cmsg.getData()[frame.can_dlc]), frame.data);

	int ret;
	if( ( ret = write(socket_.fd, &frame, sizeof(struct can_frame)) ) != sizeof(struct can_frame)) {
		printf("Error at sending CAN message %x : %d\n", cmsg.getCOBId(), ret);
		perror("write");
		return false;
	}

	return true;
}
