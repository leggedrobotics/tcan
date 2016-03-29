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

	/* **************************
	 * CAN DRIVER SETUP
	 ***************************/
	/* open channel */
	int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(fd < 0) {
		printf("Opening CAN channel %s failed: %d\n", interface, fd);
		return false;
	}

	//Configure the socket:
	struct ifreq ifr;
	strcpy(ifr.ifr_name, interface);
	ioctl(fd, SIOCGIFINDEX, &ifr);

	int loopback = options->loopback; /* 0 = disabled, 1 = enabled (default) */
	setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
	int recv_own_msgs = 0; /* 0 = disabled (default), 1 = enabled */
	setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,&recv_own_msgs, sizeof(recv_own_msgs));

	// CAN error handling
	can_err_mask_t err_mask = CAN_ERR_MASK; //( CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF );
	setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

	//Bind
	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(struct sockaddr_can));
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	//printf("%s at index %d\n", channelname, ifr.ifr_ifindex);
	if(bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		printf("Error in socket %s bind.\n", interface);
		return false;
	}

	//Set nonblocking:
	int flags;
	if (-1 == (flags = fcntl(fd, F_GETFL, 0))) flags = 0;
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);

	socket_ = {fd, POLLIN, 0};

	printf("Opened socket %s.\n", interface);

	return true;
}

bool SocketBus::closeBus() {
	close(socket_.fd);
	return true;
}


bool SocketBus::readCanMessage() {

	const int ret = poll( &socket_, 1, 1000 );

	if ( ret == -1 ) {
		printf("poll failed");
		perror("poll()");
		return false;
	}else if ( ret == 0 || !(socket_.revents & POLLIN) ) {
		/*******************************************************
		 * no data received
		 *******************************************************/
	}else{
		/*******************************************************
		 * READ INCOMING CAN MESSAGES
		 *******************************************************/
		can_frame frame;
		socket_.revents = 0;
		bool dataAvailable=true;
		do {
			int bytes_read = read( socket_.fd, &frame, sizeof(struct can_frame));
			// printf("CanManager_ bytes read: %i\n", bytes_read);

			if(bytes_read<=0) {
				//  printf("CanManager:bus_routine: Data buffer empty or error\n");
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
	const unsigned int baudRate = static_cast<SocketBusOptions*>(options_)->baudrate;
	const bool sleepAfterSending = static_cast<SocketBusOptions*>(options_)->sleepAfterSending;

	can_frame frame;
	frame.can_id = cmsg.getCOBId();
	frame.can_dlc = cmsg.getLength();
	std::copy(cmsg.getData(), &(cmsg.getData()[frame.can_dlc]), frame.data);

	if( write(socket_.fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		printf("Error at sending CAN message %x\n", cmsg.getCOBId());
		perror("write");
		return false;
	}

	// sleep the amount of time the message needs to be transmitted
	if(sleepAfterSending) {
		usleep((cmsg.getLength()*8+44)/baudRate*1000);
	}
	return true;
}
