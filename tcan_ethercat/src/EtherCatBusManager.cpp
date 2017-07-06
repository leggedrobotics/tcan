/*
 * EtherCatBusManager.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#include "tcan_ethercat/EtherCatBusManager.hpp"

namespace tcan_ethercat {

EtherCatBusManager::EtherCatBusManager():
    BusManager<EtherCatDatagrams>()
{
}

EtherCatBusManager::~EtherCatBusManager()
{
}

} /* namespace tcan_ethercat */
