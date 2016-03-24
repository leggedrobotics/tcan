/*!
 * @file 	Bus.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#ifndef BUS_HPP_
#define BUS_HPP_

#include <stdint.h>
#include <unordered_map>
#include <memory>
#include <functional>
#include <queue>
#include <mutex>

#include "yalc/CANMsg.hpp"
#include "yalc/Device.hpp"

class BusManager;

class Bus {
public:
	typedef std::shared_ptr<Device> DevicePtr;
	typedef std::unordered_map<uint32_t, std::function<bool(CANMsg)> > CobIdToFunctionMap;

	Bus() = delete;
	Bus(BusManager* bus_manager);

	virtual ~Bus();

	/*! Adds a device to the device vector and calls its initDevice function
	 * @param device	Pointer to the device
	 * @return true if init was successfull
	 */
	bool addDevice(DevicePtr device);

	/*! Adds a can message (identified by its cobId) and a function pointer to its parse function
	 * @param cobId				cobId of the message
	 * @param parseFunction		pointer to the parse function
	 * @return true if successfull
	 */
	bool addCanMessage(const uint32_t cobId, std::function<bool(const CANMsg&)>&& parseFunction);

	/*! Handles outgoing can messages
	 * @param cmsg	reference to the can message
	 */
	void sendCanMessage(const CANMsg& cmsg);


	void handleCanMessage(const CANMsg& cmsg);

	bool popNextCanMessage(CANMsg* msg);

	void setOperational(const bool operational) { isOperational_ = operational; }
	bool getOperational() const { return isOperational_; }

protected:
	// pointer to the bus manager owning this object
	BusManager* busManager_;

	// state of the bus. True if all devices are operational. Sync messages is sent only if bus is operational
	bool isOperational_;

	// interval of sync message sends. set to 0 to disable
	unsigned int syncInterval_;

	// vector containing all devices
	std::vector<DevicePtr> devices_;

	// map mapping COB id to parse functions
	CobIdToFunctionMap cobIdToFunctionMap_;

	std::mutex outgointMsgsMutex_;
	std::queue<CANMsg> outgoingMsgs_;
};

#endif /* BUS_HPP_ */
