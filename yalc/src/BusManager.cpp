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
	buses_()
{

}

BusManager::~BusManager()
{

}

bool BusManager::addBus(Bus* bus)
{
	buses_.emplace_back( bus );
	return bus->initializeBus();
}

void BusManager::sendSyncOnAllBuses(const bool waitForEmptyQueues) {
	const unsigned int bussize = buses_.size();
	std::unique_lock<std::mutex> locks[bussize];

	if(waitForEmptyQueues) {
		for(unsigned int i=0; i<bussize; i++) {
			buses_[i]->waitForEmptyQueue(locks[i]);
		}
		// we now own a lock on all output message queues
	}

	for(unsigned int i=0; i<bussize; i++) {
		buses_[i]->sendSync();
	}
}
