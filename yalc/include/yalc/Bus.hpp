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
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "yalc/CANMsg.hpp"
#include "yalc/Device.hpp"

class Bus {
public:
	typedef std::shared_ptr<Device> DevicePtr;
	typedef std::unordered_map<uint32_t, std::function<bool(const CANMsg&)> > CobIdToFunctionMap;

	Bus();

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

	/*! Add a can message to be sent
	 * @param cmsg	reference to the can message
	 */
	void sendMessage(const CANMsg& cmsg);


	void handleMessage(const CANMsg& cmsg);

	void setOperational(const bool operational) { isOperational_ = operational; }
	bool getOperational() const { return isOperational_; }

	// thread loop functions
	void receiveWorker();
	void transmitWorker();


	virtual bool initializeBus() = 0;

protected:
	virtual bool readCanMessage() = 0;
	virtual bool writeCanMessage(const CANMsg& cmsg) = 0;

protected:
	// state of the bus. True if all devices are operational. Sync messages is sent only if bus is operational
	bool isOperational_;

	// vector containing all devices
	std::vector<DevicePtr> devices_;

	// map mapping COB id to parse functions
	CobIdToFunctionMap cobIdToFunctionMap_;

	std::mutex outgointMsgsMutex_;
	std::queue<CANMsg> outgoingMsgs_;

	// threads for message reception and transmission
	std::thread receiveThread_;
	std::thread transmitThread_;
	std::atomic<bool> running_;

	// variables to wake the transmitThread after inserting something to the message output queue
	std::atomic<unsigned int> numMessagesToSend_;
	std::condition_variable condTransmitThread_;
	std::mutex mutexTransmitThread_;
};

#endif /* BUS_HPP_ */
