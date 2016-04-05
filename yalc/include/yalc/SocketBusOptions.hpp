/*
 * SocketBusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#ifndef SOCKETBUSOPTIONS_HPP_
#define SOCKETBUSOPTIONS_HPP_

#include <string>
#include <vector>
#include <linux/can.h>

#include "yalc/BusOptions.hpp"

namespace yalc {

class SocketBusOptions : public BusOptions {
public:

	SocketBusOptions():
		BusOptions(),
		interface(),
		loopback(false),
		sndBufLength(0),
		canErrorMask(CAN_ERR_MASK),
		canFilters()
	{

	}

	SocketBusOptions(const std::string& interface_name):
		BusOptions(),
		interface(interface_name),
		loopback(false),
		sndBufLength(0),
		canErrorMask(CAN_ERR_MASK),
		canFilters()
	{

	}

	virtual ~SocketBusOptions() { }

	//!  name of the network interface to bind to.
	std::string interface;

	//! loop back sent messages
	bool loopback;

	//! length of the socket buffer. 0=default. If the txqueuelen (default=10) of the netdevice cannot be changed
	// (e.g. with "ip link set <interface> txqueuelen 1000" in the command line), set this value to prevent ENOBUFS errors when writing.
	// The minimum length is 1024, set 0 to keep the default
	unsigned int sndBufLength;

	//! error mask. By default, subscribe to all error messages
	// see https://www.kernel.org/doc/Documentation/networking/can.txt
	unsigned int canErrorMask;


	//! vector of can filters to be applied
	// see https://www.kernel.org/doc/Documentation/networking/can.txt
	std::vector<can_filter> canFilters;
};

} /* namespace yalc */

#endif /* SOCKETBUSOPTIONS_HPP_ */
