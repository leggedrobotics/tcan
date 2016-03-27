/*!
 * @file 	Bus.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#ifndef SOCKETBUSOPTIONS_HPP_
#define SOCKETBUSOPTIONS_HPP_

#include <string>

#include "yalc/BusOptions.hpp"

class SocketBusOptions : public BusOptions {
public:

	SocketBusOptions():
		BusOptions(),
		interface(),
		loopback(false),
		baudrate(125)
	{

	}

	SocketBusOptions(const std::string& interface_name, const unsigned int baud_rate):
		BusOptions(),
		interface(interface_name),
		loopback(false),
		baudrate(baud_rate)
	{

	}

	virtual ~SocketBusOptions() { }

	//!  name of the network interface to bind to.
	std::string interface;

	//! loop back sent messages
	bool loopback;

	//! baud rate of the bus in kbps
	unsigned int baudrate;

};

#endif /* SOCKETBUSOPTIONS_HPP_ */
