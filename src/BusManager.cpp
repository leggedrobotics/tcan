/*!
 * @file 	BusManager.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#include "libcanplusplus/BusManager.hpp"

#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <iostream>

BusManager::BusManager()
{

}

BusManager::~BusManager()
{
	buses_.clear();
}

void BusManager::addBus(Bus* bus)
{
	buses_.push_back(bus);
}

int BusManager::getSize()
{
	return buses_.size();
}

Bus*  BusManager::getBus(unsigned int index)
{
	try {
		if (index >= buses_.size()) {
			std::string error = "BusManager: Could not get bus with index "
								+ boost::lexical_cast<std::string>(index) + "!";
			throw std::out_of_range(error);
		}
		return &(buses_[index]);

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	}
	return NULL;
}
