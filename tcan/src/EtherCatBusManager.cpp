/*
 * EtherCatBusManager.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#include "tcan/EtherCatBusManager.hpp"

namespace tcan {

EtherCatBusManager::EtherCatBusManager():
    BusManager<EthernetFrame>()
{
}

EtherCatBusManager::~EtherCatBusManager()
{
}

} /* namespace tcan */
