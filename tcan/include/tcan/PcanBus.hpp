/*
 * PcanBus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <libpcan.h>
#include <pcan.h>
#include "tcan/CanBus.hpp"
#include "tcan/PcanBusOptions.hpp"

//#include <unistd.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <sys/ioctl.h>
//
//#define NO_RT 1
//#define __COBALT__ 1
//#include </home/integration/software/peak-linux-driver-8.3.1/lib/src/libprivate.h>


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
    bool writeData(const CanMsg& cmsg);

    /*!
     * Is called on reception of a bus error message. Sets the flag
     * @param msg  reference to the bus error message
     */
    void handleBusError(const can_frame& msg);

 protected:
    HANDLE handle_;
//    pcan_handle pcanHandle_;
};

} /* namespace tcan */
