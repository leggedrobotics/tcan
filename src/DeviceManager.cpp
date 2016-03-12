/*!
 * @file 	DeviceManager.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */
#include "libcanplusplus/DeviceManager.hpp"

#include <stdexcept>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <stdio.h>

DeviceManager::DeviceManager(Bus* bus):bus_(bus)
{

}

DeviceManager::~DeviceManager()
{
//	printf("~DeviceManager()\n");
	devices_.clear();
}

void DeviceManager::addDevice(Device* device)
{
	device->setBus(bus_);
	device->addRxPDOs();
	device->addTxPDOs();
	devices_.push_back(device);
}

int DeviceManager::getSize()
{
	return devices_.size();
}

Device* DeviceManager::getDevice(unsigned int index)
{
	try {
		if (index >= devices_.size()) {
			std::string error = "DeviceManager: Could not get device with index "
								+ boost::lexical_cast<std::string>(index) + "!";
			throw std::out_of_range(error);
		}
		return &(devices_[index]);

	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	}
	return NULL;
}

