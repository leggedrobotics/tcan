/*
 * Bus.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include <utility>
#include <chrono>
#include <pthread.h>

#include "yalc/Bus.hpp"

namespace yalc {

Bus::Bus(const bool asynchronous, const unsigned int sanityCheckInterval):
	Bus(new BusOptions(asynchronous, sanityCheckInterval))
{

}

Bus::Bus(BusOptions* options):
	isOperational_(false),
	options_(options),
	cobIdToFunctionMap_(),
	outgointMsgsMutex_(),
	outgoingMsgs_(),
	receiveThread_(),
	transmitThread_(),
	sanityCheckThread_(),
	running_(false),
	condTransmitThread_(),
	condOutputQueueEmpty_()
{

}

Bus::~Bus()
{
	stopThreads();

	for(Device* device : devices_) {
		delete device;
	}

	delete options_;
}

void Bus::stopThreads() {
	running_ = false;
	condTransmitThread_.notify_all();
	condOutputQueueEmpty_.notify_all();

	if(receiveThread_.joinable()) {
		receiveThread_.join();
	}

	if(transmitThread_.joinable()) {
		transmitThread_.join();
	}

	if(sanityCheckThread_.joinable()) {
		sanityCheckThread_.join();
	}
}

bool Bus::initBus() {

	if(!initializeCanBus()) {
		return false;
	}

	running_ = true;

	if(options_->asynchronous) {
		receiveThread_ = std::thread(&Bus::receiveWorker, this);
		transmitThread_ = std::thread(&Bus::transmitWorker, this);

		sched_param sched;
		sched.sched_priority = options_->priorityReceiveThread;
		if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
			printf("Failed to set receive thread priority\n");
			perror("pthread_setschedparam");
		}

		sched.sched_priority = options_->priorityTransmitThread;
		if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
			printf("Failed to set transmit thread priority\n");
			perror("pthread_setschedparam");
		}

		if(options_->sanityCheckInterval > 0) {
			sanityCheckThread_ = std::thread(&Bus::sanityCheckWorker, this);

			sched.sched_priority = options_->prioritySanityCheckThread;
			if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
				printf("Failed to set receive thread priority\n");
				perror("pthread_setschedparam");
			}
		}
	}

	return true;
}

void Bus::handleMessage(const CanMsg& cmsg) {

	// Check if CAN message is handled.
	CobIdToFunctionMap::iterator it = cobIdToFunctionMap_.find(cmsg.getCobId());
	if (it != cobIdToFunctionMap_.end()) {

		it->second(cmsg); // call function pointer
	} else {
		auto value = cmsg.getData();
		printf("Received CAN message that is not handled: COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
				cmsg.getCobId(),
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


bool Bus::processOutputQueue() {
	bool writeSuccess = false;
	std::unique_lock<std::mutex> lock(outgointMsgsMutex_);

	while(outgoingMsgs_.size() == 0 && running_) {
		condOutputQueueEmpty_.notify_all();
		condTransmitThread_.wait(lock);
	}
	// after the wait function we own the lock. copy data and unlock.

	if(!running_) {
		return true;
	}

	CanMsg cmsg = outgoingMsgs_.front();
	lock.unlock();

	writeSuccess = writeCanMessage( cmsg );

	lock.lock();

	if(writeSuccess) {
		// only pop the message from the queue if sending was successful
		outgoingMsgs_.pop();
	}

	return writeSuccess;
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
		readMessage();
	}

	printf("receive thread terminated\n");
}

void Bus::transmitWorker() {
	while(running_) {
		processOutputQueue();
	}

	printf("transmit thread terminated\n");
}

void Bus::sanityCheckWorker() {
	auto nextLoop = std::chrono::steady_clock::now();

	while(running_) {
		sanityCheck();

		nextLoop += std::chrono::milliseconds(options_->sanityCheckInterval);
		std::this_thread::sleep_until(nextLoop);
	}

	printf("sanityCheck thread terminated\n");
}

} /* namespace yalc */

