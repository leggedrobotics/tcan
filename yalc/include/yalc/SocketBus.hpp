/*
 * SocketBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <poll.h>

#include "yalc/Bus.hpp"
#include "yalc/SocketBusOptions.hpp"

namespace yalc {

class SocketBus : public Bus {
public:

	SocketBus() = delete;
	SocketBus(const std::string& interface);
	SocketBus(SocketBusOptions* options);

	virtual ~SocketBus();

protected:
	virtual bool initializeCanBus();
	virtual bool readCanMessage();
	virtual bool writeCanMessage(const CanMsg& cmsg);

protected:
	pollfd socket_;
};

} /* namespace yalc */
