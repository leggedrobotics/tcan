/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/Bus.hpp"
#include "tcan/IpBusOptions.hpp"
#include "tcan/IpMsg.hpp"

namespace tcan {

class IpBus : public Bus<IpMsg> {
 public:
	IpBus() = delete;
	IpBus(IpBusOptions* options);

    virtual ~IpBus();

    /*! Callback called after reception of a message.
     * @param msg	reference to the usb message
     */
    virtual void handleMessage(const IpMsg& msg) = 0;

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck();

 protected:
    bool initializeInterface();
    bool readData();
    bool writeData(std::unique_lock<std::mutex>* lock);

 private:
    int socket_;
    int recvFlag_;
    int sendFlag_;

    unsigned int deviceTimeoutCounter_;
};

} /* namespace tcan */

