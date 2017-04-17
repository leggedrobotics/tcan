/*
 * PcanBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/CanBus.hpp"
#include "tcan_pcan/PcanBusOptions.hpp"

#include <string>


namespace tcan {

class PcanBus : public CanBus {
 public:

    PcanBus(const std::string& interface);
    PcanBus(BusOptions* options) = delete;
    PcanBus(PcanBusOptions* options);

    virtual ~PcanBus();

 protected:
    bool initializeInterface();
    bool readData();
    bool writeData(std::unique_lock<std::mutex>* lock);

 protected:
    int fd_;
};

} /* namespace tcan */
