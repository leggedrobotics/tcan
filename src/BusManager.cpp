/*!
 * @file 	BusManager.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */


#include "yalc/BusManager.hpp"

BusManager::BusManager():
	buses_(),
	globalSyncInterval_(0),
	syncWaitForEmptyQueue_(false),
	receiveThread_(&BusManager::receiveWorker, this),
	transmitThread_(&BusManager::transmitWorker, this),
	running_(true)
{
	// todo: set thread priorities?

}

BusManager::~BusManager()
{
	running_ = false;
	receiveThread_.join();
	transmitThread_.join();
}

bool BusManager::addBus(const std::string& interface)
{
	buses_.emplace_back(new Bus());

	return initializeBus(interface);
}


void BusManager::receiveWorker() {
	while(running_) {
		readMessages();
	}
}

void BusManager::transmitWorker() {
	while(running_) {
		writeMessages();
	}
}
