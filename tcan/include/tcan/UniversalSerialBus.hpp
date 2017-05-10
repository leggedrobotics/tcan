/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <termios.h> // tcgettatr
#include <memory>

#include "tcan/Bus.hpp"
#include "tcan/UniversalSerialBusOptions.hpp"
#include "tcan/UsbMsg.hpp"

namespace tcan {

class UniversalSerialBus : public Bus<UsbMsg> {
 public:
    UniversalSerialBus() = delete;
    UniversalSerialBus(std::unique_ptr<UniversalSerialBusOptions>&& options);

    virtual ~UniversalSerialBus();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const UsbMsg& msg) = 0;

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck();

 protected:
    bool initializeInterface();
    bool readData();
    bool writeData(std::unique_lock<std::mutex>* lock);

 private:
    void configureInterface();
    inline int calculateTimeoutMs(const timeval& tv) {
        // normal infinity timeout is specified with timeout of 0. poll has infinity for negative values, so subtract 1ms
        return (tv.tv_sec*1000 + tv.tv_usec/1000)-1;
    }

 private:
    int fileDescriptor_;

    termios savedAttributes_;

    unsigned int deviceTimeoutCounter_;
};

} /* namespace tcan */

