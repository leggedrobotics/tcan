/*
 * BusManager.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <vector>
#include <poll.h>

#include "tcan/Bus.hpp"
#include "tcan/helper_functions.hpp"

namespace tcan {

//! Container of all CAN buses
template <class Msg>
class BusManager {
 public:
    BusManager():
        buses_(),
        receiveThread_(),
        sanityCheckThread_(),
        running_{false},
        sanityCheckInterval_(100)
    {
    }

    virtual ~BusManager()
    {
        closeBuses();
    }

    bool addBus(Bus<Msg>* bus) {
        if(bus->isSemiSynchronous() && running_) {
            MELO_FATAL("Tried to add a semi-synchronous bus after calling startThreads. This is not allowed due to data concurrency!");
        }

        buses_.push_back( bus );
        return bus->initBus();
    }
    /*! Gets the number of buses
     * @return	number of buses
     */
    unsigned int getSize() const { return buses_.size(); }

    /*! Read and parse messages from all buses. Call this function in the control loop if synchronous mode is used.
     */
    void readMessagesSynchronous() {
        for(auto bus : buses_) {
            if(bus->isSynchronous()) {
                while(bus->readMessage()) {
                }
            }
        }
    }
    /*!
     * Send the messages in the output queue on all buses. Call this function in the control loop if synchronous mode is used.
     * Note that this function may not send all the messages in the output queue if BlockingWrite is disabled (see BusOptions)
     * @return  False if at least one write error occurred
     */
    bool writeMessagesSynchronous() {
        bool sendingData = true;
        bool noError = true;
        while(sendingData) {
            sendingData = false;

            for(auto bus : buses_) {
                if(bus->isSynchronous() && bus->getNumOutgoingMessagesWithoutLock() > 0) {
                    noError &= bus->writeMessages( nullptr );
                    sendingData = true;
                }else if(bus->isSemiSynchronous()) {
                    // we need to acquire lock here because the callbacks of incoming messages may put new messages in the output queue
                    std::unique_lock<std::mutex> lock( bus->getOutgoingMsgsMutex() );
                    if(bus->getNumOutgoingMessagesWithoutLock() > 0) {
                        noError &= bus->writeMessages( &lock );
                        sendingData = true;
                    }
                }
            }
        }
        return noError;
    }

    /*! Call sanityCheck(..) on all buses. Call this function in the control loop if synchronous mode is used.
     * @return True if no device is missing or has error nor any bus has any errors
     */
    bool sanityCheckSynchronous() {
        bool allFine = true;
        for(auto bus : buses_) {
            if(bus->isSynchronous()) {
                allFine &= bus->sanityCheck();
            }
        }

        return allFine;
    }

    /*!
     * Check if no device timed out
     * @return  True if at least one device is missing
     */
    bool isMissingDeviceOrHasError() const {
        for(auto bus : buses_) {
            if(bus->isMissingDeviceOrHasError()) {
                return true;
            }
        }
        return false;
    }

    /*!
     * check if we received a message from all devices within timeout
     * @return True if all devices are active
     */
    bool allDevicesActive() const {
        for(auto bus : buses_) {
            if(!(bus->allDevicesActive())) {
                return false;
            }
        }
        return true;
    }

    /*!
     * Checks if a error message was received on one of the buses.
     * @return true if a error message was received
     */
    bool getErrorMsgFlag() const {
        for(auto bus : buses_) {
            if(bus->getErrorMsgFlag()) {
                return true;
            }
        }
        return false;
    }

    /*!
     * Checks if a error message was received on one of the buses and resets the flag.
     * @return true if a error message was received
     */
    bool resetErrorMsgFlag() {
        bool hadBusError = false;
        for(auto bus : buses_) {
            if(bus->resetErrorMsgFlag()) {
                hadBusError = true;
            }
        }
        return hadBusError;
    }

    /*!
     * Close all buses and stop threads associated to them.
     */
    void closeBuses() {
        // tell all threads to stop
        stopThreads(false);
        for(Bus<Msg>* bus : buses_) {
            bus->stopThreads(false);
        }

        // join all threads and destruct buses
        stopThreads(true);
        for(Bus<Msg>* bus : buses_) {
            delete bus;
        }

        buses_.clear();
    }

    /*
     * Start threads for buses which are asynchronous or semi-synchronous.
     */
    void startThreads() {

        for(auto bus : buses_) {
            bus->startThreads();
        }

        if(running_) {
            return;
        }

        bool hasSemiSyncBus = false;
        int priorityReceiveThread = 0;
        int prioritySanityCheckThread = 0;
        sanityCheckInterval_ = 0;

        std::once_flag flag;

        for(auto bus : buses_) {
            if(bus->isSemiSynchronous()) {
                const BusOptions *options = bus->getOptions();

                priorityReceiveThread = std::max(priorityReceiveThread, options->priorityReceiveThread_);
                prioritySanityCheckThread = std::max(prioritySanityCheckThread, options->prioritySanityCheckThread_);

                std::call_once(flag, [&](){ sanityCheckInterval_ = options->sanityCheckInterval_; });

                if (sanityCheckInterval_ < options->sanityCheckInterval_) {
                    sanityCheckInterval_ = options->sanityCheckInterval_;
                    MELO_WARN("Rising sanity check interval for bus manager to %d", sanityCheckInterval_);
                } else if (sanityCheckInterval_ > options->sanityCheckInterval_) {
                    MELO_WARN("Bus manager sanity check interval (%d) is larger than sanity check interval of added bus %s (%d). Devices may wrongly be considered as timed out.",
                              sanityCheckInterval_, options->name_.c_str(), options->sanityCheckInterval_);
                }

                hasSemiSyncBus = true;
            }
        }

        if(hasSemiSyncBus) {
            running_ = true;

            receiveThread_ = std::thread(&BusManager::receiveWorker, this);
            if (!setThreadPriority(receiveThread_, priorityReceiveThread)) {
                MELO_WARN("Failed to set receive thread priority for bus manager\n  %s", strerror(errno));
            }

            if (sanityCheckInterval_ > 0) {
                sanityCheckThread_ = std::thread(&BusManager::sanityCheckWorker, this);
                if (!setThreadPriority(sanityCheckThread_, prioritySanityCheckThread)) {
                    MELO_WARN("Failed to set sanity check thread priority for bus manager\n  %s", strerror(errno));
                }
            }
        }else{
            MELO_INFO("No bus is configured to be semi synchrounous. Not starting threads.");
        }
    }

    /*!
     * Stop all threads associated with buses
     * @param wait  Whether to wait for the threads to stop or return immediately
     */
    void stopThreads(const bool wait=true) {
        running_ = false;

        if(wait) {
            if(receiveThread_.joinable()) {
                receiveThread_.join();
            }

            if(sanityCheckThread_.joinable()) {
                sanityCheckThread_.join();
            }
        }
    }

 protected:

    // thread loop functions
    void receiveWorker() {
        unsigned int numFds = 0;
        pollfd fds[buses_.size()];
        std::vector<unsigned int> busIndices;

        for(unsigned int i=0; i<buses_.size(); ++i) {
            if(buses_[i]->isSemiSynchronous()) {
                fds[numFds] = {buses_[i]->getPollableFileDescriptor(), POLLIN, 0};
                busIndices.push_back(i);
                ++numFds;
            }
        }

        while(running_) {
            int ret = poll( fds, numFds, 500 /*timeout [ms]*/ );

            if ( ret == -1 ) {
                MELO_ERROR("polling for fileDescriptor readability failed in bus manager:\n  %s", strerror(errno));
            }else if ( ret == 0 ) {
                // poll timed out, without being able to read => continue silently
            }else{
                // there is something in the fd ready to be read
                for(unsigned int i=0; i<numFds; ++i) {
                    if(fds[i].revents & POLLIN) {
                        buses_[busIndices[i]]->readMessage();
                    }

                    fds[i].revents = 0;
                }
            }
        }

        MELO_INFO("Receive thread for bus manager terminated");
    }

    void sanityCheckWorker() {
        auto nextLoop = std::chrono::steady_clock::now();

        while(running_) {
            nextLoop += std::chrono::milliseconds(sanityCheckInterval_);
            std::this_thread::sleep_until(nextLoop);

            for(auto bus : buses_) {
                if(bus->isSemiSynchronous()) {
                    bus->sanityCheck();
                }
            }
        }

        MELO_INFO("SanityCheck thread for bus manager terminated");
    }

 protected:
    std::vector<Bus<Msg>*> buses_;

    //! threads for message reception and device sanity checking
    std::thread receiveThread_;
    std::thread sanityCheckThread_;
    std::atomic<bool> running_;

    unsigned int sanityCheckInterval_;
};

} /* namespace tcan */
