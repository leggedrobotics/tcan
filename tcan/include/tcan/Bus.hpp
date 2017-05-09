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

#include "message_logger/message_logger.hpp"


namespace tcan {

template <class Msg>
class Bus {
 public:

    using MsgQueue = std::deque<Msg>;

    Bus() = delete;
    Bus(std::unique_ptr<BusOptions>&& options):
        isMissingDeviceOrHasError_(false),
        allDevicesActive_(false),
        allDevicesMissing_(false),
        isPassive_(options->startPassive_),
        options_(std::move(options)),
        outgoingMsgsMutex_(),
        outgoingMsgs_(),
        receiveThread_(),
        transmitThread_(),
        sanityCheckThread_(),
        running_(false),
        condTransmitThread_(),
        condOutputQueueEmpty_(),
        errorMsgFlagPersistent_(false),
        errorMsgFlag_(false)
    {
    }

    virtual ~Bus()
    {
        stopThreads(true);
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
    inline bool isAsynchronous() const { return options_->asynchronous_; }

    /*!
     * @return  number of messages in the output queue
     */
    unsigned int getNumOutogingMessagesWithoutLock() const { return outgoingMsgs_.size(); }

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

    /*! write the message(s) at the front of the queue to the CAN bus
     * This is a helper function for BusManager::writeMessagesSynchronous(). The output message queue mutex is NOT locked,
     * and if there is something in the queue is NOT checked.
     * @return true if a message was successfully written to the bus
     */
    inline bool writeMessagesWithoutLock()
    {
        return isPassive_ ? true : writeData( nullptr );
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
        lock = std::unique_lock<std::mutex>(outgoingMsgsMutex_);
        condOutputQueueEmpty_.wait(lock, [this]{ return outgoingMsgs_.size() == 0 || !running_; });
    }

 protected:
    /*! Initialized the device driver
     * @return true if successful
     */
    virtual bool initializeInterface() = 0;

    /*! read CAN message from the device driver. This function shall be blocking in asynchrounous mode and non-blocking in synchronous!
     * It shall set errorMsgFlag_ and errorMsgFlagPersistent_ to true if it successfully read a message but identified it as error message (used for passive bus feature)
     * and set errorMsgFlag_ to false on successfull reads of non-error messages.
     * @return true if a message was successfully read and parsed
     */
    virtual bool readData() = 0;

    /*! write CAN message to the device driver.  This function shall be blocking in asynchrounous mode and non-blocking in synchronous!
     * @param lock      pointer to the lock protecting the output queue, which is in LOCKED state when the function is called.
     *                  Use nullptr if queue is unprotected.
     * @return          True if no error occured
     */
    virtual bool writeData(std::unique_lock<std::mutex>* lock) = 0;

    /*! Is called after reception of a message, routes the message to the callbacks.
     * @param cmsg  reference to the can message
     */
    virtual void handleMessage(const Msg& msg) = 0;

    /*! Helper function for transmission thread.
     * @return  True if message(s) were successfully sent.
     */
    bool processOutputQueue() {
        std::unique_lock<std::mutex> lock(outgoingMsgsMutex_);

        while(outgoingMsgs_.size() == 0 && running_) {
            condOutputQueueEmpty_.notify_all();
            condTransmitThread_.wait(lock);
        }
        // after the wait function we own the lock.

        if(!running_) {
            return true;
        }

        return isPassive_ ? true : writeData( &lock );
    }

    inline bool checkOutgoingMsgsSize() const {
        if(outgoingMsgs_.size() >= options_->maxQueueSize_) {
            MELO_WARN_THROTTLE(options_->errorThrottleTime_, "Exceeding max queue size on bus %s! Dropping message!", options_->name_.c_str());
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
    // reception of a non-error message.
    bool errorMsgFlag_;
};

} /* namespace tcan */

