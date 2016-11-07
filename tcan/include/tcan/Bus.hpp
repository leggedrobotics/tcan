/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "tcan/BusOptions.hpp"

#include "message_logger/message_logger.hpp"


namespace tcan {

template <class Msg>
class Bus {
 public:

    Bus() = delete;
    Bus(BusOptions* options):
        isMissingDevice_(false),
        allDevicesActive_(false),
        options_(options),
        outgointMsgsMutex_(),
        outgoingMsgs_(),
        receiveThread_(),
        transmitThread_(),
        sanityCheckThread_(),
        running_(false),
        condTransmitThread_(),
        condOutputQueueEmpty_()
    {
    }

    virtual ~Bus()
    {
        stopThreads(true);

        delete options_;
    }


    /*! Initializes the Bus. Sets up threads and calls initializeCanBus(..).
     * @return true if init was successful
     */
    bool initBus() {

        if(!initializeInterface()) {
            return false;
        }

        running_ = true;

        if(options_->asynchronous_) {
            receiveThread_ = std::thread(&Bus::receiveWorker, this);
            transmitThread_ = std::thread(&Bus::transmitWorker, this);

            sched_param sched;
            sched.sched_priority = options_->priorityReceiveThread_;
            if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
                MELO_WARN("Failed to set receive thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
            }

            sched.sched_priority = options_->priorityTransmitThread_;
            if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
                MELO_WARN("Failed to set transmit thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
            }

            if(options_->sanityCheckInterval_ > 0) {
                sanityCheckThread_ = std::thread(&Bus::sanityCheckWorker, this);

                sched.sched_priority = options_->prioritySanityCheckThread_;
                if (pthread_setschedparam(receiveThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
                    MELO_WARN("Failed to set receive thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
                }
            }
        }

        return true;
    }

    /*! Add a can message to be sent (added to the output queue)
     * @param cmsg	reference to the can message
     */
    void sendMessage(const Msg& msg) {
        std::lock_guard<std::mutex> guard(outgointMsgsMutex_);
        sendMessageWithoutLock(msg);
    }

    /*!
     * @return true if no device timed out
     */
    inline bool isMissingDevice() const { return isMissingDevice_; }

    /*!
     * @return true if we received a message from all devices within timeout
     */
    inline bool allDevicesActive() const { return allDevicesActive_; }

    inline bool isAsynchronous() const { return options_->asynchronous_; }

 public: /// Internal functions

    /*! write the message at the front of the queue to the CAN bus
     * @return true if a message was successfully written to the bus
     */
    inline bool writeMessage()
    {
        if(outgoingMsgs_.size() != 0) {
            if(writeData( outgoingMsgs_.front() )) {
                outgoingMsgs_.pop();
                return true;
            }
        }

        return false;
    }

    /*! read and parse a message from the CAN bus
     * @return true if a message was read
     */
    inline bool readMessage()
    {
        return readData();
    }

    /*! Do a sanity check of the bus.
     */
    virtual void sanityCheck() = 0;

    /*!
     * Stops all threads handled by this bus (send, receive, sanity check)
     * @param wait  whether the function shall wait for the the threads to terminate or return immediately.
     */
    void stopThreads(const bool wait=true) {
        running_ = false;
        condTransmitThread_.notify_all();
        condOutputQueueEmpty_.notify_all();

        if(wait) {
            if(receiveThread_.joinable()) {
                receiveThread_.join();
            }

            if(transmitThread_.joinable()) {
                transmitThread_.join();
            }

            if(sanityCheckThread_.joinable()) {
                sanityCheckThread_.join();
            }
        }
    }

    /*! Waits until the output queue is empty, locks the queue and returns the lock
     */
    void waitForEmptyQueue(std::unique_lock<std::mutex>& lock)
    {
        lock = std::unique_lock<std::mutex>(outgointMsgsMutex_);
        condOutputQueueEmpty_.wait(lock, [this]{ return outgoingMsgs_.size() == 0 || !running_; });
    }

 protected:
    /*! Initialized the device driver
     * @return true if successful
     */
    virtual bool initializeInterface() = 0;

    /*! read CAN message from the device driver
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() = 0;

    /*! write CAN message to the device driver
     * @return true if the message was successfully written
     */
    virtual bool writeData(const Msg& msg) = 0;

    /*! Internal function. Is called after reception of a message.
     * Routes the message to the callback.
     * @param cmsg  reference to the can message
     */
    virtual void handleMessage(const Msg& msg) = 0;


    bool processOutputQueue() {
        bool writeSuccess = false;
        std::unique_lock<std::mutex> lock(outgointMsgsMutex_);

        while(outgoingMsgs_.size() == 0 && running_) {
            condOutputQueueEmpty_.notify_all();
            condTransmitThread_.wait(lock);
        }
        // after the wait function we own the lock. copy data and unlock.

        if(!running_) {
            return true;
        }

        Msg msg = outgoingMsgs_.front();
        lock.unlock();

        writeSuccess = writeData( msg );

        if(writeSuccess) {
            // only pop the message from the queue if sending was successful
            lock.lock();
            outgoingMsgs_.pop();
        }

        return writeSuccess;
    }

    void sendMessageWithoutLock(const Msg& msg) {
        if(outgoingMsgs_.size() >= options_->maxQueueSize_) {
            MELO_WARN("Exceeding max queue size on bus %s! Dropping message!", options_->name_.c_str());
        }
        outgoingMsgs_.push( msg ); // do not use emplace here. We need a copy of the message in some situations
        // (SDO queue processing). Declaring another sendMessage(..) which uses move
        // semantics does not increase performance as CANMsg has only POD members

        condTransmitThread_.notify_all();
    }

    // thread loop functions
    void receiveWorker() {
        while(running_) {
            readMessage();
        }

        MELO_INFO("receive thread for bus %s terminated", options_->name_.c_str());
    }

    void transmitWorker() {
        while(running_) {
            processOutputQueue();
        }

        MELO_INFO("transmit thread for bus %s terminated", options_->name_.c_str());
    }

    void sanityCheckWorker() {
        auto nextLoop = std::chrono::steady_clock::now();

        while(running_) {
            nextLoop += std::chrono::milliseconds(options_->sanityCheckInterval_);
            std::this_thread::sleep_until(nextLoop);

            sanityCheck();
        }

        MELO_INFO("sanityCheck thread for bus %s terminated", options_->name_.c_str());
    }

 protected:
    // true if a device timed out. Devices in 'initializing' state are not considered as missing.
    std::atomic<bool> isMissingDevice_;

    // true if all devices are in active state (we received a message within the timeout)
    std::atomic<bool> allDevicesActive_;

    const BusOptions* options_;

    // output queue containing all messages to be sent by the transmitThread_
    std::mutex outgointMsgsMutex_;
    std::queue<Msg> outgoingMsgs_;

    // threads for message reception and transmission and device sanity checking
    std::thread receiveThread_;
    std::thread transmitThread_;
    std::thread sanityCheckThread_;
    std::atomic<bool> running_;

    // variable to wake the transmitThread after inserting something to the message output queue
    std::condition_variable condTransmitThread_;

    // variable to wait for empty output queues (required for global sync)
    std::condition_variable condOutputQueueEmpty_;
};

} /* namespace tcan */

