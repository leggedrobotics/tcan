/*!
 * @file 	Bus.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#ifndef SOCKETBUS_HPP_
#define SOCKETBUS_HPP_

#include <poll.h>
#include <string>

#include "yalc/Bus.hpp"

class BusManager;

class SocketBus : public Bus {
public:

	SocketBus() = delete;
	SocketBus(const std::string& interface);

	virtual ~SocketBus();

	virtual bool initializeBus();
	bool closeBus();

protected:
	virtual bool readCanMessage();
	virtual bool writeCanMessage(const CANMsg& cmsg);

protected:
	std::string interface_;

	pollfd socket_;
	unsigned int baudRate_; // baud rate of the bus [kbps]

};

#endif /* SOCKETBUS_HPP_ */
