/*!
 * @file 	Bus.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#include "libcanplusplus/Bus.hpp"

Bus::Bus(int iBus):iBus_(iBus)
{
}

Bus::~Bus()
{
}

PDOManager* Bus::getRxPDOManager()
{
	return rxPDOManager_;
}

PDOManager* Bus::getTxPDOManager()
{
	return txPDOManager_;
}

SDOManager* Bus::getSDOManager()
{
	return SDOManager_;
}

DeviceManager* Bus::getDeviceManager()
{
	return deviceManager_;
}

int Bus::iBus()
{
	return iBus_;
}
