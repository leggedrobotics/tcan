/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <string>

#include "tcan/BusOptions.hpp"

namespace tcan {

struct UniversalSerialBusOptions : public BusOptions {
    UniversalSerialBusOptions():
        UniversalSerialBusOptions(std::string())
    {

    }

    UniversalSerialBusOptions(const std::string& name):
        BusOptions(name),
        baudrate(115200),
        databits(8),
        parity("None"),
        stopbits(1),
        softwareHandshake(false),
        hardwareHandshake(false)
    {

    }

    virtual ~UniversalSerialBusOptions() { }


    int baudrate;
    int databits;
    std::string parity;
    unsigned int stopbits;
    bool softwareHandshake;
    bool hardwareHandshake;
};

} /* namespace tcan */
