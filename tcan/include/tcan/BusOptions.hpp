/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <string>
#include <sys/time.h> // for timeval

namespace tcan {

struct BusOptions {
    enum class Mode : uint8_t {
        Synchronous,
        SemiSynchronous,
        Asynchronous
    };


    BusOptions():
        BusOptions(std::string())
    {

    }

    BusOptions(const std::string& name):
        mode_(Mode::Asynchronous),
        sanityCheckInterval_(100),
        priorityReceiveThread_(99),
        priorityTransmitThread_(98),
        prioritySanityCheckThread_(1),
        maxQueueSize_(1000),
        name_(name),
        startPassive_(false),
        activateBusOnReception_(false),
        synchronousBlockingWrite_(true),
        readTimeout_{1, 0},
        writeTimeout_{1,0},
        errorThrottleTime_(0.0)
    {
    }

    virtual ~BusOptions() { }

    //! Mode to operate the bus in.
    //! Synchronous: It is up to the user to call BusManager's readMessagesSynchronous(), writeMessagesSynchronous() and sanityCheckSynchronous() functions.
    //!              No threads are created.
    //! Semi-synchronous: The BusManager creates threads for receiving messages and sanity check. It us up to the user to call writeMessagesSynchronous()
    //!                   and to call startThreads() after all the buses have been added to the manager (addBus(..)).
    //! Asynchronous: The bus will create threads for receiving, sending and sanity check, no further actions required by the user.
    Mode mode_;

    //! if > 0 and in asynchronous mode, a thread will be created which does a sanity check of the devices. Default is 100 [ms].
    unsigned int sanityCheckInterval_;

    int priorityReceiveThread_;
    int priorityTransmitThread_;
    int prioritySanityCheckThread_;

    //! max size of the output queue
    unsigned int maxQueueSize_;

    //! name of the interface
    std::string name_;

    //! start the bus in passive state (outgoing messages are not sent)
    bool startPassive_;

    //! if true, the bus will automatically switch from passive to active state as soon as a message is received
    bool activateBusOnReception_;

    //! Whether write calls are blocking in synchrounous and semi-synchronous mode. Doing so ensures that all messages in the
    //! output queue are sent when calling BusManager::writeMessagesSynchronous(), but may increase its execution time.
    bool synchronousBlockingWrite_;

    //! timeouts for read and write operations on the interface. Set to zero to leave default (infinity)
    timeval readTimeout_;
    timeval writeTimeout_;

    //! throttle time for MELO outputs
    double errorThrottleTime_;
};

} /* namespace tcan */
