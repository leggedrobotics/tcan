/*
 * IpBusManager.cpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#include "tcan_ip/IpBusManager.hpp"

namespace tcan_ip {

IpBusManager::IpBusManager():
    tcan::BusManager<IpMsg>()
{
}

IpBusManager::~IpBusManager()
{
}

} /* namespace tcan_ip */
