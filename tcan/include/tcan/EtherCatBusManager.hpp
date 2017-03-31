/*
 * EtherCatBusManager.hpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */

#pragma once

#include "tcan/BusManager.hpp"
#include "tcan/EtherCatBus.hpp"

namespace tcan {

//! Container of all EtherCat buses
class EtherCatBusManager : public BusManager<EtherCatDatagrams> {
 public:
	EtherCatBusManager();

    virtual ~EtherCatBusManager();

    EtherCatBus* getEtherCatBus(const unsigned int index) { return static_cast<EtherCatBus*>(buses_[index]); }
};

} /* namespace tcan */
