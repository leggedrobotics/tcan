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
#include <chrono>

#include "yalc/Bus.hpp"

Bus::Bus(const bool asynchronous, const unsigned int sanityCheckInterval):
	Bus(BusOptions(asynchronous, sanityCheckInterval))
{

}

Bus::Bus(const BusOptions& options):
	isOperational_(false),
	sanityCheckInterval_(options.sanityCheckInterval),
	cobIdToFunctionMap_(),
	outgointMsgsMutex_(),
	outgoingMsgs_(),
	receiveThread_(),
	transmitThread_(),
	sanityCheckThread_(),
	running_(true),
	condTransmitThread_(),
	condOutputQueueEmpty_()
{
	if(options.asynchronous) {
		receiveThread_ = std::thread(&Bus::receiveWorker, this);
		transmitThread_ = std::thread(&Bus::transmitWorker, this);
		// todo: set thread priorities
	}

	if(sanityCheckInterval_) {
		sanityCheckThread_ = std::thread(&Bus::sanityCheckWorker, this);
	}

}

Bus::~Bus()
{
	running_ = false;
	condTransmitThread_.notify_all();

	receiveThread_.join();
	transmitThread_.join();
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

void Bus::writeMessages() {
	bool writeSuccess = false;
	std::unique_lock<std::mutex> lock(outgointMsgsMutex_);

	while(outgoingMsgs_.size() == 0 && running_) {
		condOutputQueueEmpty_.notify_all();
		condTransmitThread_.wait(lock);
	}
	// after the wait function we own the lock. copy data and unlock.
	if(!running_) {
		return;
	}
	CANMsg cmsg = outgoingMsgs_.front();
	lock.unlock();


	writeSuccess = writeCanMessage( lock, cmsg );

	lock.lock();

	if(writeSuccess) {
		// only pop the message from the queue if sending was successful
		outgoingMsgs_.pop();
	}
}

bool Bus::sanityCheck() {
	bool allFine = true;
	for(auto device : devices_) {
		if(!device->sanityCheck()) {
			allFine = false;
		}
	}

	isOperational_ = allFine;
	return allFine;
}


void Bus::receiveWorker() {
	while(running_) {
		readMessages();
	}
}

void Bus::transmitWorker() {
	while(running_) {
		writeMessages();
	}
}

void Bus::sanityCheckWorker() {
	while(running_) {
		sanityCheck();

		std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::milliseconds(sanityCheckInterval_));
	}
}
