/*!
 * @file 	BusManager.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#include <chrono>

#include "yalc/BusManager.hpp"

BusManager::BusManager():
	buses_(),
	globalSyncInterval_(0),
	syncWaitForEmptyQueue_(false),
	receiveThread_(&BusManager::receiveWorker, this),
	transmitThread_(&BusManager::transmitWorker, this),
	running_(true),
	sendingMessages_(false),
	condTransmitThread_(),
	mutexTransmitThread_()
{
	// todo: set thread priorities?

}

BusManager::~BusManager()
{
	running_ = false;
	condTransmitThread_.notify_all();

	receiveThread_.join();
	transmitThread_.join();
}

bool BusManager::addBus(const std::string& interface)
{
	buses_.emplace_back(new Bus(this));

	return initializeBus(interface);
}

void BusManager::notifyTransmitWorker() {
	sendingMessages_ = true;
	condTransmitThread_.notify_all();
}

void BusManager::receiveWorker() {
	while(running_) {
		readMessages();
	}
}

void BusManager::transmitWorker() {
	std::unique_lock<std::mutex> lock(mutexTransmitThread_);
	const unsigned int numBuses = buses_.size();
	CANMsg cmsg;
	bool finished = false;

	while(running_) {
		condTransmitThread_.wait(lock, [this](){return sendingMessages_ || !running_;});
		if(running_) {
			finished = false;
			while(!finished) {
				for(unsigned int iBus=0; iBus<numBuses; iBus++) {
					if(buses_[iBus]->popNextCanMessage(&cmsg)) {

					}
				}
//				usleep();
			}
		}
	}
}
