/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
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
#include "yalc/BusOptions.hpp"

namespace yalc {

class Bus {
public:

	typedef std::function<bool(const CANMsg&)> CallbackPtr;
	typedef std::unordered_map<uint32_t, CallbackPtr> CobIdToFunctionMap;

	Bus() = delete;
	Bus(const bool asynchronous, const unsigned int sanityCheckInterval);
	Bus(BusOptions* options);

	virtual ~Bus();

	/*! Initializes the Bus. Sets up threads and calls initializeCanBus(..).
	 * @return true if init was successfull
	 */
	bool initBus();

	/*! Adds a device to the device vector and calls its initDevice function
	 * @param device	Pointer to the device
	 * @return true if init was successfull
	 */
	bool addDevice(Device* device) {
		devices_.push_back(device);
		return device->initDeviceInternal(this);
	}

	/*! Adds a can message (identified by its cobId) and a function pointer to its parse function
	 * to the map, which is used to assign incoming messages to their callbacks.
	 * @param cobId				cobId of the message
	 * @param parseFunction		pointer to the parse function
	 * @return true if successfull
	 */
	bool addCanMessage(const uint32_t cobId, CallbackPtr&& parseFunction)
	{
		return cobIdToFunctionMap_.emplace(cobId, std::move(parseFunction)).second;
	}

	/*! Add a can message to be sent (added to the output queue)
	 * @param cmsg	reference to the can message
	 */
	void sendMessage(const CANMsg& cmsg) {
		std::lock_guard<std::mutex> guard(outgointMsgsMutex_);
		sendMessageWithoutLock(cmsg);
	}

	/*! Send a sync message on the bus. Is called by BusManager::sendSyncOnAllBuses or directly.
	 */
	void sendSync() {
		sendMessage(CANMsg(0x80, 0, nullptr));
	}

	/*! Send a sync message on the bus without locking the queue.
	 * This function is intended to be used by BusManager::sendSyncOnAllBuses, which locks the queue.
	 */
	void sendSyncWithoutLock() {
		sendMessageWithoutLock(CANMsg(0x80, 0, nullptr));
	}

	/*! Waits until the output queue is empty, locks the queue and returns the lock
	 */
	void waitForEmptyQueue(std::unique_lock<std::mutex>& lock)
	{
		lock = std::unique_lock<std::mutex>(outgointMsgsMutex_);
		condOutputQueueEmpty_.wait(lock, [this]{return outgoingMsgs_.size() > 0 || !running_;});
	}

	/*! write all messages in the queue to the CAN driver. Call this function inside your control loop if
	 * asynchronous mode is not used.
	 */
	void writeMessages();
	/*! read all messages from the CAN driver. Call this function inside your control loop if
	 * asynchronous mode is not used.
	 */
	void readMessages();

	/*! Do a sanity check of all devices. Call this function inside your control loop if
	 * asynchronous mode is not used.
	 */
	bool sanityCheck();

	/*! Internal function. Is called after reception of a message.
	 * Routes the message to the callback.
	 * @param cmsg	reference to the can message
	 */
	void handleMessage(const CANMsg& cmsg);

	inline void setOperational(const bool operational) { isOperational_ = operational; }
	inline bool getOperational() const { return isOperational_; }



protected:
	virtual bool initializeCanBus() = 0;
	virtual bool readCanMessage() = 0;
	virtual bool writeCanMessage(const CANMsg& cmsg) = 0;

	void sendMessageWithoutLock(const CANMsg& cmsg)
	{
		outgoingMsgs_.push( cmsg );	// do not use emplace here. We need a copy of the message in some situations
									// (SDO queue processing). Declaring another sendMessage(..) which uses move
									// semantics does not increase performance as CANMsg has only POD members

		condTransmitThread_.notify_all();
	}

	// thread loop functions
	void receiveWorker();
	void transmitWorker();
	void sanityCheckWorker();

protected:
	// state of the bus. True if all devices are operational.
	bool isOperational_;

	BusOptions* options_;

	// vector containing all devices
	std::vector<Device*> devices_;

	// map mapping COB id to parse functions
	CobIdToFunctionMap cobIdToFunctionMap_;

	// output queue containing all messages to be sent by the transmitThread_
	std::mutex outgointMsgsMutex_;
	std::queue<CANMsg> outgoingMsgs_;

	// threads for message reception and transmission and device sanity checking
	std::thread receiveThread_;
	std::thread transmitThread_;
	std::thread sanityCheckThread_;
	std::atomic<bool> running_;

	// variable to wake the transmitThread after inserting something to the message output queue
	std::condition_variable condTransmitThread_;

	// variable to wait for empty output queues (required for global sync)
	std::condition_variable condOutputQueueEmpty_;
};

} /* namespace yalc */

#endif /* BUS_HPP_ */
