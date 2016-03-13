/*!
 * @file 	BusManager.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */
#ifndef SOCKETBUSMANAGER_HPP_
#define SOCKETBUSMANAGER_HPP_

#include <vector>
#include <poll.h>

#include "yalc/BusManager.hpp"

struct can_frame;

class SocketBusManager : public BusManager {
public:

	SocketBusManager();

	virtual ~SocketBusManager();

	virtual bool initializeBus(const std::string& device);
	bool closeBuses();
	virtual bool readMessages();

protected:
	std::vector<pollfd> sockets_;

};

#endif /* SOCKETBUSMANAGER_HPP_ */
