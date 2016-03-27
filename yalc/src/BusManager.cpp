/*
 * BusManager.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "yalc/BusManager.hpp"

BusManager::BusManager():
	buses_()
{

}

BusManager::~BusManager()
{
	for(Bus* bus : buses_) {
		delete bus;
	}
}

bool BusManager::addBus(Bus* bus)
{
	buses_.push_back( bus );
	return bus->initBus();
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
		buses_[i]->sendSyncWithoutLock();
	}
}
