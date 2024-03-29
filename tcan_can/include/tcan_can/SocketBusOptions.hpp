#pragma once

#include <vector>
#include <linux/can.h> // for can_filter
#include <linux/can/error.h>

#include "tcan/BusOptions.hpp"

namespace tcan_can {

struct SocketBusOptions : public CanBusOptions {
    SocketBusOptions():
        SocketBusOptions(std::string())
    {
    }

    SocketBusOptions(const std::string& interface_name):
        CanBusOptions(interface_name),
        loopback_(false),
        sndBufLength_(0),
        canErrorMask_(CAN_ERR_MASK),
        canFilters_()
    {
    }

    ~SocketBusOptions() override = default;

    //! loop back sent messages
    bool loopback_;

    //! length of the socket buffer. 0=default. If the txqueuelen (default=10) of the netdevice cannot be changed
    // (e.g. with "ip link set <interface> txqueuelen 1000" in the command line), set this value to prevent ENOBUFS errors when writing.
    // The minimum length is 1024, set 0 to keep the default
    unsigned int sndBufLength_;

    //! error mask. By default, subscribe to all error messages. It may be a good idea to disable CAN_ERR_LOSTARB, as this is normal
    // bus behavior.
    // see https://www.kernel.org/doc/Documentation/networking/can.txt
    unsigned int canErrorMask_;


    //! vector of can filters to be applied
    // see https://www.kernel.org/doc/Documentation/networking/can.txt
    std::vector<can_filter> canFilters_;
};

} /* namespace tcan_can */
