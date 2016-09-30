/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <stdint.h>
#include <unordered_map>
#include <memory>
#include <functional>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <utility>

#include "tcan/Device.hpp"
#include "tcan/BusOptions.hpp"

#include "message_logger/message_logger.hpp"


namespace tcan {

template <class Msg>
class Bus {
 public:

    Bus() = delete;
    Bus(BusOptions* options):
        isOperational_(false),
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

        for(Device* device : devices_) {
            delete device;
        }

        delete options_;
    }

    /*! Initializes the Bus. Sets up threads and calls initializeCanBus(..).
     * @return true if init was successful
     */
    bool initBus() {

        if(!initializeHardware()) {
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

    /*!
     * in-place construction of a new device
     * @param options   pointer to the option class of the device
     * @return true if successful
     */
    template <class C, typename TOptions>
    std::pair<C*, bool> addDevice(TOptions* options) {
        C* dev = new C(options);
        bool success = addDevice(dev);
        return std::make_pair(dev, success);
    }

    /*! Adds a device to the device vector and calls its initDevice function
     * @param device	Pointer to the device
     * @return true if init was successful
     */
    bool addDevice(Device* device) {
        devices_.push_back(device);
        return device->initDeviceInternal(this);
    }

    /*! Add a can message to be sent (added to the output queue)
     * @param cmsg	reference to the can message
     */
    void sendMessage(const Msg& msg) {
        std::lock_guard<std::mutex> guard(outgointMsgsMutex_);
        sendMessageWithoutLock(msg);
    }

    /*! Waits until the output queue is empty, locks the queue and returns the lock
     */
    void waitForEmptyQueue(std::unique_lock<std::mutex>& lock)
    {
        lock = std::unique_lock<std::mutex>(outgointMsgsMutex_);
        condOutputQueueEmpty_.wait(lock, [this]{ return outgoingMsgs_.size() == 0 || !running_; });
    }

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

    /*! Do a sanity check of all devices on this bus.
     */
    bool sanityCheck() {
        bool allFine = true;
        for(auto device : devices_) {
            if(!device->sanityCheck()) {
                allFine = false;
            }
        }

        isOperational_ = allFine;
        return allFine;
    }

    inline void setOperational(const bool operational) { isOperational_ = operational; }
    inline bool getOperational() const { return isOperational_; }

    inline bool isAsynchronous() const { return options_->asynchronous_; }

    /*!
     * Stops all threads handled by this bus (send, receive, sanity check)
     * @param wait  whether the function shall wait for the the threads to terminate or return immediately.
     */
    void stopThreads(const bool wait) {
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


    /*! Internal function. Is called after reception of a message.
     * Routes the message to the callback.
     * @param cmsg  reference to the can message
     */
    virtual void handleMessage(const Msg& msg) = 0;


 protected:
    /*! Initialized the device driver
     * @return true if successful
     */
    virtual bool initializeHardware() = 0;

    /*! read CAN message from the device driver
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() = 0;

    /*! write CAN message to the device driver
     * @return true if the message was successfully written
     */
    virtual bool writeData(const CanMsg& cmsg) = 0;


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
            sanityCheck();

            nextLoop += std::chrono::milliseconds(options_->sanityCheckInterval_);
            std::this_thread::sleep_until(nextLoop);
        }

        MELO_INFO("sanityCheck thread for bus %s terminated", options_->name_.c_str());
    }

 protected:
    // state of the bus. True if all devices are operational.
    bool isOperational_;

    const BusOptions* options_;

    // vector containing all devices
    std::vector<Device*> devices_;

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

