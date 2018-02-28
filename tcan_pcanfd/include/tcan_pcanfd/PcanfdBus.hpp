/*
 * PcanBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan_can/CanBus.hpp"
#include "tcan_pcanfd/PcanfdBusOptions.hpp"

#include <string>
#include <memory>


namespace tcan_pcanfd {

class PcanfdBus : public tcan_can::CanBus {
 public:

    PcanfdBus(const std::string& interface);
    PcanfdBus(std::unique_ptr<PcanfdBusOptions>&& options);

    ~PcanfdBus() override;

    int getPollableFileDescriptor() const  override {
        // todo: check if this is actually pollable!
        return fd_;
    }

 protected:
    bool initializeInterface() override;
    bool readData() override;
    bool writeData(std::unique_lock<std::mutex>* lock) override;


 protected:
    int fd_;
};

} /* namespace tcan_pcanfd */
