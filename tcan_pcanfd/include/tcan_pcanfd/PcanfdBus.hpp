/*
 * PcanBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/CanBus.hpp"
#include "tcan_pcanfd/PcanfdBusOptions.hpp"

#include <string>


namespace tcan {

class PcanfdBus : public CanBus {
 public:

    PcanfdBus(const std::string& interface);
    PcanfdBus(BusOptions* options) = delete;
    PcanfdBus(PcanfdBusOptions* options);

    virtual ~PcanfdBus();

 protected:
    bool initializeInterface();
    bool readData();
    bool writeData(std::unique_lock<std::mutex>* lock);

 protected:
    int fd_;
};

} /* namespace tcan */
