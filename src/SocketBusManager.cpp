/*!
 * @file 	BusManager.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

// socket includes
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "yalc/SocketBusManager.hpp"
#include "yalc/CANMsg.hpp"

SocketBusManager::SocketBusManager():
	sockets_()
{

}

SocketBusManager::~SocketBusManager()
{
	closeBuses();
}

bool SocketBusManager::initializeBus(const std::string& device)
{
	/* **************************
	 * CAN DRIVER SETUP
	 ***************************/
	/* open channel */
	int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(fd < 0) {
		printf("Opening CAN channel %s failed: %d\n", device.c_str(), fd);
		return false;
	}

	//Configure the socket:
	struct ifreq ifr;
	strcpy(ifr.ifr_name, device.c_str());
	ioctl(fd, SIOCGIFINDEX, &ifr);

	int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
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
		printf("Error in socket %s bind.\n", device.c_str());
		return false;
	}

	//Set nonblocking:
	int flags;
	if (-1 == (flags = fcntl(fd, F_GETFL, 0))) flags = 0;
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);

	sockets_.emplace_back(pollfd{fd, POLLIN, 0});

	return true;
}

bool SocketBusManager::closeBuses() {
	for (unsigned int i=0; i<sockets_.size(); i++) {
		close(sockets_[i].fd);
	}
	return true;
}


bool SocketBusManager::readMessages() {

	const unsigned int numSockets = sockets_.size();

	const int ret = poll( &sockets_[0], numSockets, 1000 );

	if ( ret == -1 ) {
	    printf("poll failed");
		perror("poll()");
		return false;
	}else if ( ret == 0 ) {
		/*******************************************************
		 * no data received
		 *******************************************************/
	}else{
		/*******************************************************
		 * READ INCOMING CAN MESSAGES
		 *******************************************************/
		struct can_frame frame;
		for (unsigned int iBus=0; iBus<numSockets; iBus++) {
			if( sockets_[iBus].revents & POLLIN ) {
				sockets_[iBus].revents = 0;
				bool dataAvailable=true;
				do {
					int bytes_read = read( sockets_[iBus].fd, &frame, sizeof(struct can_frame));
					// printf("CanManager_ bytes read: %i\n", bytes_read);

					if(bytes_read<=0) {
						//  printf("CanManager:bus_routine: Data buffer empty or error\n");
						dataAvailable=false;
					} else {
						// printf("CanManager:bus_routine: Data received from iBus %i, n. Bytes: %i \n", iBus, bytes_read);

						CANMsg cmsg(frame.can_id, frame.can_dlc, frame.data);
						buses_[iBus]->handleCanMessage(cmsg);
					}
				} while(dataAvailable);
			}
		}
	}


	return true;
}
