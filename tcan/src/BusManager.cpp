/*
 * BusManager.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan/BusManager.hpp"

namespace tcan {

BusManager::BusManager():
	buses_()
{

}

BusManager::~BusManager()
{
	closeBuses();
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


void BusManager::readMessagesSynchrounous() {
	for(auto bus : buses_) {
		if(!bus->isAsynchronous()) {
			while(bus->readMessage()) {
			}
		}
	}
}

void BusManager::writeMessagesSynchronous() {
	bool sendingData = true;

	while(sendingData) {
		sendingData = false;

		for(auto bus : buses_) {
			if(!bus->isAsynchronous()) {
				sendingData |= bus->writeMessage();
			}
		}
	}
}

bool BusManager::sanityCheckSynchronous() {
	bool allFine = true;
	for(auto bus : buses_) {
		if(!bus->sanityCheck()) {
			allFine = false;
		}
	}

	return allFine;
}


void BusManager::closeBuses() {
	for(Bus* bus : buses_) {
		delete bus;
	}

	buses_.clear();
}

} /* namespace tcan */
