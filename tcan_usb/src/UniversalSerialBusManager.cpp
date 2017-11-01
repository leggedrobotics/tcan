/*
 * BusManager.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan/UniversalSerialBusManager.hpp"

namespace tcan {

UniversalSerialBusManager::UniversalSerialBusManager():
    BusManager<UsbMsg>()
{
}

UniversalSerialBusManager::~UniversalSerialBusManager()
{
}

} /* namespace tcan */
