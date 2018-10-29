/*
 * PcanBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Christian Gehring
 *
 * fixme:
 *  WARNING: THIS IMPLEMENTATION IS INCOMPLETE AND DOES NOT SUPPORT ALL BusOptions
 */

#pragma once

#include "tcan_can/CanBus.hpp"
#include "tcan_pcan/PcanBusOptions.hpp"

#include "pcan_driver/libpcan.h"

namespace tcan_pcan {

class PcanBus : public tcan_can::CanBus {
 public:

    PcanBus(const std::string& interface);
    PcanBus(std::unique_ptr<tcan::BusOptions>&& options) = delete;
    PcanBus(std::unique_ptr<PcanBusOptions>&& options);

    ~PcanBus() override;

 protected:
    bool initializeInterface() override;
    bool readData() override;
    bool writeData(std::unique_lock<std::mutex>* lock) override;

    /*!
     * Is called on reception of a bus error message. Sets the flag
     * @param msg  reference to the bus error message
     */
    void handleBusErrorMessage(const can_frame& msg);

 protected:
    HANDLE handle_;
//    pcan_handle pcanHandle_;
};

} /* namespace tcan_pcan */
