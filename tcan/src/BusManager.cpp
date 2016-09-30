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
        bus->stopThreads(false);
    }
    for(Bus* bus : buses_) {
        delete bus;
    }

    buses_.clear();
}

} /* namespace tcan */
