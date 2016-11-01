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
        maxDeviceTimeoutCounter(20),
        bufferSize(bufSize),
        minMessageLength(0),
        baudrate(115200),
        databits(8),
        parity(Parity::None),
        stopbits(1),
        softwareHandshake(false),
        hardwareHandshake(false),
        skipConfiguration(false)
    {

    }

    virtual ~UniversalSerialBusOptions() { }

    unsigned int maxDeviceTimeoutCounter;


    unsigned int bufferSize;
    unsigned int minMessageLength; // the minimum number of bytes to read from the serial interface. Setting this to != 0 sets non-canonical serial mode
    int baudrate;
    int databits;
    Parity parity;
    unsigned int stopbits;
    bool softwareHandshake;
    bool hardwareHandshake;
    bool skipConfiguration;
};

} /* namespace tcan */
