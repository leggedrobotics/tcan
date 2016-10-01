/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/BusOptions.hpp"

namespace tcan {

struct UniversalSerialBusOptions : public BusOptions {
    enum class Parity : unsigned int {
        None,
        Odd,
        Even,
        Mark,
        Space
    };

    UniversalSerialBusOptions():
        UniversalSerialBusOptions(std::string())
    {

    }

    UniversalSerialBusOptions(const std::string& name, const unsigned int bufSize=1024):
        BusOptions(name),
        bufferSize(bufSize),
        baudrate(115200),
        databits(8),
        parity(Parity::None),
        stopbits(1),
        softwareHandshake(false),
        hardwareHandshake(false)
    {

    }

    virtual ~UniversalSerialBusOptions() { }

    unsigned int bufferSize;
    int baudrate;
    int databits;
    Parity parity;
    unsigned int stopbits;
    bool softwareHandshake;
    bool hardwareHandshake;
};

} /* namespace tcan */
