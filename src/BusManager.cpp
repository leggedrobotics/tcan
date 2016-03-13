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

BusManager::BusManager()
{

}

BusManager::~BusManager()
{
}

bool BusManager::addBus(const std::string& device)
{
	buses_.emplace_back(new Bus());

	return initializeBus(device);
}


int BusManager::getSize() const
{
	return buses_.size();
}
