/*
 * Bus.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <memory>

#include "tcan/BusOptions.hpp"
#include "tcan/helper_functions.hpp"

#include "message_logger/message_logger.hpp"


namespace tcan {

template <class Msg>
class Bus {
 public:

    using MsgQueue = std::deque<Msg>;

    Bus() = delete;
    Bus(std::unique_ptr<BusOptions>&& options):
            hasBusError_{false},
            isMissingDeviceOrHasError_{false},
            allDevicesActive_{false},
            allDevicesMissing_{false},
            isPassive_{options->startPassive_},
            options_(std::move(options)),
            outgoingMsgsMutex_(),
            outgoingMsgs_(),
            receiveThread_(),
            transmitThread_(),
            sanityCheckThread_(),
            running_{false},
            condTransmitThread_(),
            condOutputQueueEmpty_(),
            errorMsgFlagPersistent_{false},
            errorMsgFlag_(false)
    {
    }

    virtual ~Bus()
    {
        stopThreads(true);
    }


    /*!
     * Initializes the Bus. Calls initializeInterface(..).
     * @return true if init was successful
     */
    inline bool initBus() {
        return initializeInterface();
    }

    /*!
     * Do a sanity check of the bus.
     * This function shall check hasBusError_, call sanityCheck() on all its devices and set isMissingDeviceOrHasError_, allDevicesActive_ and
     * allDevicesMissing_ accordingly. It also shall passivate() the bus if all its device(s) are missing (may be configurable with specific bus options)
     * @return true if hasBusError_ and isMissingDeviceOrHasError_ are false
     */
    virtual bool sanityCheck() = 0;

    /*!
     * Starts threads for this bus (send, recieve, sanity check) if it is configured to be asynchronous
     */
    void startThreads() {
        if(isAsynchronous() && !running_) {
            running_ = true;

            receiveThread_ = std::thread(&Bus::receiveWorker, this);
            if(!setThreadPriority(receiveThread_, options_->priorityReceiveThread_)) {
                MELO_WARN("Failed to set receive thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
            }

            transmitThread_ = std::thread(&Bus::transmitWorker, this);
            if (!setThreadPriority(transmitThread_, options_->priorityTransmitThread_)) {
                MELO_WARN("Failed to set transmit thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
            }

            if(options_->sanityCheckInterval_ > 0) {
                sanityCheckThread_ = std::thread(&Bus::sanityCheckWorker, this);
                if (!setThreadPriority(sanityCheckThread_, options_->prioritySanityCheckThread_)) {
                    MELO_WARN("Failed to set sanity check thread priority for bus %s:\n  %s", options_->name_.c_str(), strerror(errno));
                }
            }
        }
    }

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

    /*! Copy a message to be sent to the output queue
     * @param msg	const reference to the message to be sent
     */
    inline bool sendMessage(const Msg& msg) {
        std::lock_guard<std::mutex> guard(outgoingMsgsMutex_);
        return sendMessageWithoutLock(msg);
    }

    /*!
     * Move a massage to be sent to the output queue
     * @param msg   message to be sent
     */
    inline bool emplaceMessage(Msg&& msg) {
        std::lock_guard<std::mutex> guard(outgoingMsgsMutex_);
        return emplaceMessageWithoutLock(std::forward<Msg>(msg));
    }

    /*!
     * Activates the bus and allows sending messages
     */
    inline void activate() {
        isPassive_ = false;
        condTransmitThread_.notify_all(); // kick off transmit thread in case it has been waiting because the bus was passive
    }

    /*!
     * Passivates the bus, thus holding back all outgoing messages until the bus is active again
     */
    inline void passivate() {
        isPassive_ = true;
    }

    /*!
     * @return True if the bus is in passive state
     */
    inline bool isPassive() const { return isPassive_; }

    /*!
     * @return false if no device timed out
     */
    inline bool isMissingDeviceOrHasError() const { return isMissingDeviceOrHasError_; }

    /*!
     * @return true if we received a message from all devices within timeout
     */
    inline bool allDevicesActive() const { return allDevicesActive_; }

    /*!
     * @return true if all devices are missing
     */
    inline bool allDevicesMissing() const { return allDevicesMissing_; }

    /*!
     * @return true if the bus is configured to be asynchronous
     */
    inline bool isAsynchronous() const { return (options_->mode_ == BusOptions::Mode::Asynchronous); }

    /*!
     * @return true if the bus is configured to be semi-synchronous
     */
    inline bool isSemiSynchronous() const { return (options_->mode_ == BusOptions::Mode::SemiSynchronous); }

    /*!
     * @return true if the bus is configured to be synchronous
     */
    inline bool isSynchronous() const { return (options_->mode_ == BusOptions::Mode::Synchronous); }

    /*!
     * @return  number of messages in the output queue. 0 if the bus is passive
     */
    unsigned int getNumOutgoingMessagesWithoutLock() const { return isPassive() ? 0 : outgoingMsgs_.size(); }

    /*!
     * @return  returns the name of the bus
     */
    const std::string& getName() const { return options_->name_; }

    /*!
     * @return a const pointer to the options struct
     */
    const BusOptions* getOptions() const { return options_.get(); }

    /*!
     * @return true if a bus error message has been received
     */
    inline bool getErrorMsgFlag() const { return errorMsgFlagPersistent_; }

    /*!
     * resets the bus error flag and returns its previous value.
     * @return true if a bus error message has been received
     */
    inline bool resetErrorMsgFlag() {
        bool tmp = errorMsgFlagPersistent_;
        errorMsgFlagPersistent_ = false;
        return tmp;
    }


public: /// Internal functions
    /*!
     * write the message(s) at the front of the queue to the CAN bus
     * @param lock
     * @return true if a message was successfully written to the bus or if the bus is passive
     */
    inline bool writeMessages(std::unique_lock<std::mutex>* lock)
    {
        return isPassive_ ? true : writeData( lock );
    }

    /*! read and parse a message from the bus
     * @return true if a message was read
     */
    inline bool readMessage()
    {
        if(readData()) {
            if(isPassive_ && options_->activateBusOnReception_ && !errorMsgFlag_) {
                isPassive_ = false;
                MELO_WARN("Auto-activated bus %s", options_->name_.c_str());
            }
            return true;
        }
        return false;
    }

    /*!
     * Waits until the output queue is empty, locks the queue and returns the lock.
     * This function shall only be called for asynchronous buses.
     */
    void waitForEmptyQueue(std::unique_lock<std::mutex>& lock)
    {
        lock = std::unique_lock<std::mutex>(outgoingMsgsMutex_);
        condOutputQueueEmpty_.wait(lock, [this]{ return getNumOutgoingMessagesWithoutLock() == 0 || !running_; });
    }

    /*! Get a file descriptor, used for polling multiple buses for incoming messages. Required for semi-synchronous buses.
     * @return  valid file descriptor
     */
    virtual int getPollableFileDescriptor() const {
        MELO_FATAL("Bus %s does not support semi-synchronous mode!", getName().c_str());
        return 0;
    }

    inline std::mutex& getOutgoingMsgsMutex() { return outgoingMsgsMutex_; }

 protected:
    /*! Initialized the device driver
     * @return true if successful
     */
    virtual bool initializeInterface() = 0;

    /*! read CAN message from the device driver. This function shall be blocking in asynchronous mode and non-blocking in synchronous and semi-synchronous!
     * It shall set errorMsgFlag_ and errorMsgFlagPersistent_ to true if it successfully read a message but identified it as error message (used for passive bus feature)
     * and set errorMsgFlag_ to false on successful reads of non-error messages. It shall also set hasBusError_ to true if read operations fail due to
     * non-easily recoverable reasons (like buffer-full errors) and to false on succcessful read operations.
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() = 0;

    /*! write CAN message to the device driver.  This function shall be blocking in asynchronous mode and non-blocking in synchronous and semi-synchronous!
     * It shall set hasBusError_ to true if write operations fail due to non-easily recoverable reasons (like buffer-full errors) and to false on succcessful write operations.
     * @param lock      pointer to the lock protecting the output queue, which is in LOCKED state when the function is called.
     *                  Use nullptr if queue is unprotected.
     * @return          True if no error occurred
     */
    virtual bool writeData(std::unique_lock<std::mutex>* lock) = 0;

    /*! Is called after reception of a message, routes the message to the callbacks.
     * @param cmsg  reference to the can message
     */
    virtual void handleMessage(const Msg& msg) = 0;

    inline bool checkOutgoingMsgsSize() const {
        if(outgoingMsgs_.size() >= options_->maxQueueSize_) {
            MELO_WARN_THROTTLE(options_->errorThrottleTime_, "Exceeding max queue size on bus %s! Dropping message!", getName().c_str());
            return false;
        }
        return true;
    }

    inline bool sendMessageWithoutLock(const Msg& msg) {
        if(checkOutgoingMsgsSize()) {
            outgoingMsgs_.push_back( msg );
            condTransmitThread_.notify_all();
            return true;
        }

        return false;
    }

    inline bool emplaceMessageWithoutLock(Msg&& msg) {
        if(checkOutgoingMsgsSize()) {
            outgoingMsgs_.emplace_back( std::forward<Msg>(msg) );
            condTransmitThread_.notify_all();
            return true;
        }

        return false;
    }

    // thread loop functions
    void receiveWorker() {
        while(running_) {
            readMessage();
        }

        MELO_INFO("receive thread for bus %s terminated", options_->name_.c_str());
    }

    void transmitWorker() {
        std::unique_lock<std::mutex> lock(outgoingMsgsMutex_);

        while(running_) {
            while(getNumOutgoingMessagesWithoutLock() == 0 && running_) {
                condOutputQueueEmpty_.notify_all();
                condTransmitThread_.wait(lock);
            }

            // after the wait function we own the lock.

            if(running_) { // check if running_ is still true, otherwise leave loop immediately
                if(!isPassive_) {
                    writeData(&lock);
                }
            }

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
    //! true if a read/write operation on a bus failed, for not easily recoverable reasons (like buffer-full errors)
    std::atomic<bool> hasBusError_;

    //! true if a device is in 'Missing' or 'Error' state.
    std::atomic<bool> isMissingDeviceOrHasError_;

    //! true if all devices are in active state (we received a message within the timeout)
    std::atomic<bool> allDevicesActive_;

    //! true if all devices handeled by this bus are missing
    std::atomic<bool> allDevicesMissing_;

    //! if true, the outgoing messages are not sent to the physical bus
    std::atomic<bool> isPassive_;

    const std::unique_ptr<BusOptions> options_;

    //! output queue containing all messages to be sent by the transmitThread_
    std::mutex outgoingMsgsMutex_;
    MsgQueue outgoingMsgs_;

    //! threads for message reception and transmission and device sanity checking
    std::thread receiveThread_;
    std::thread transmitThread_;
    std::thread sanityCheckThread_;
    std::atomic<bool> running_;

    //! variable to wake the transmitThread after inserting something to the message output queue
    std::condition_variable condTransmitThread_;

    //! variable to wait for empty output queues (required for global sync)
    std::condition_variable condOutputQueueEmpty_;

    //! flag to indicate the reception of an error message. Can be cleared with resetError().
    std::atomic<bool> errorMsgFlagPersistent_;

    //! flag indicating that the last received message was an error message. This flag is reset upon successfull
    // reception of a non-error message. (No need for thread safety, is only used in readMessage(..) and its sub functions)
    bool errorMsgFlag_;
};

} /* namespace tcan */

