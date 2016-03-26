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
	syncWaitForEmptyQueue_(false)
{
	// todo: set thread priorities?

}

BusManager::~BusManager()
{

}

bool BusManager::addBus(Bus* bus)
{
	buses_.emplace_back( bus );
	bus->initializeBus();

	return true;
}

void BusManager::sendSyncOnAllBuses() {

}
