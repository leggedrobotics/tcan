/*!
 * @file 	Bus.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#include <utility>

#include "yalc/Bus.hpp"

Bus::Bus():
	isOperational_(false),
	cobIdToFunctionMap_(),
	outgointMsgsMutex_(),
	outgoingMsgs_(),
	receiveThread_(&Bus::receiveWorker, this),
	transmitThread_(&Bus::transmitWorker, this),
	running_(true),
	numMessagesToSend_(0),
	condTransmitThread_(),
	mutexTransmitThread_()
{
}

Bus::~Bus()
{
	running_ = false;
	condTransmitThread_.notify_all();

	receiveThread_.join();
	transmitThread_.join();
}

bool Bus::addDevice(DevicePtr device) {
	devices_.push_back(device);
	return device->initDeviceInternal(this);
}

bool Bus::addCanMessage(const uint32_t cobId, std::function<bool(const CANMsg&)>&& parseFunction) {
	cobIdToFunctionMap_.emplace(cobId, std::move(parseFunction));
	return true;
}

void Bus::sendMessage(const CANMsg& cmsg) {

	{
		std::lock_guard<std::mutex> guard(outgointMsgsMutex_);
		outgoingMsgs_.push( cmsg ); // do not use emplace here. We need a copy of the message in some situations
									// (SDO queue processing). Declaring another sendMessage(..) which uses move
									// semantics does not increase performance as CANMsg has only POD members
	}

	numMessagesToSend_++;
	condTransmitThread_.notify_all();
}

void Bus::handleMessage(const CANMsg& cmsg) {

	// Check if CAN message is handled.
	CobIdToFunctionMap::iterator it = cobIdToFunctionMap_.find(cmsg.getCOBId());
	if (it != cobIdToFunctionMap_.end()) {

		it->second(cmsg); // call function pointer
	} else {
		auto value = cmsg.getData();
		printf("Received CAN message that is not handled: COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
				cmsg.getCOBId(),
				value[1],
				value[0],
				value[0],
				value[1],
				value[2],
				value[3],
				value[4],
				value[5],
				value[6],
				value[7]
		);
	}
}


void Bus::receiveWorker() {
	while(running_) {
		readCanMessage();
	}
}

void Bus::transmitWorker() {
	std::unique_lock<std::mutex> lock(mutexTransmitThread_);

	while(running_) {
		condTransmitThread_.wait(lock, [this](){return numMessagesToSend_ > 0 || !running_;});
		if(running_) {
			while(numMessagesToSend_ > 0) {
				if(writeCanMessage( outgoingMsgs_.front() )) {
					// only pop the message from the queue if sending was successful
					numMessagesToSend_--;
					std::lock_guard<std::mutex> guard(outgointMsgsMutex_);
					outgoingMsgs_.pop();
				}
			}
		}
	}
}
