/*
 * IpBusManager.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan/IpBusManager.hpp"

namespace tcan {

IpBusManager::IpBusManager():
    BusManager<IpMsg>()
{
}

IpBusManager::~IpBusManager()
{
}

} /* namespace tcan */
