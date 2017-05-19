/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <memory>

#include "tcan/Bus.hpp"
#include "tcan/IpBusOptions.hpp"
#include "tcan/IpMsg.hpp"

namespace tcan {

class IpBus : public Bus<IpMsg> {
 public:
	IpBus() = delete;
	IpBus(std::unique_ptr<IpBusOptions>&& options);

    virtual ~IpBus();

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck();

    virtual int getPollableFileDescriptor() { return socket_; }

protected:
    virtual bool initializeInterface();
    virtual bool readData();
    virtual bool writeData(std::unique_lock<std::mutex>* lock);

 private:
    int socket_;
    int recvFlag_;
    int sendFlag_;

    unsigned int deviceTimeoutCounter_;
};

} /* namespace tcan */

