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
#include "tcan/CanMsg.hpp"

namespace tcan {

class Bus {
 public:

    typedef std::function<bool(const CanMsg&)> CallbackPtr;
    typedef std::unordered_map<uint32_t, std::pair<Device*, CallbackPtr>> CobIdToFunctionMap;

    Bus() = delete;
    Bus(BusOptions* options);

    virtual ~Bus();

    /*! Initializes the Bus. Sets up threads and calls initializeCanBus(..).
     * @return true if init was successful
     */
    bool initBus();

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

    /*! Adds a device and callback function for incoming messages identified by its cobId. The timeout counter of the device is
     *  reset on reception of the message (treated as heartbeat).
     * @param cobId             cobId of the message
     * @param device            pointer to the device
     * @param fp                pointer to the parse function
     * @return true if successful
     */
    template <class T, typename Enable=void>
    inline bool addCanMessage(const uint32_t cobId, T* device, bool(std::common_type<T>::type::*fp)(const CanMsg&))
    {
        return cobIdToFunctionMap_.emplace(cobId, std::make_pair(nullptr, std::bind(fp, device, std::placeholders::_1))).second;
    }

    template <class T, typename std::enable_if<std::is_base_of<Device, T>::value>::type>
    inline bool addCanMessage(const uint32_t cobId, T* device, bool(std::common_type<T>::type::*fp)(const CanMsg&))
    {
        return cobIdToFunctionMap_.emplace(cobId, std::make_pair(device, std::bind(fp, device, std::placeholders::_1))).second;
    }

    /*! Add a can message to be sent (added to the output queue)
     * @param cmsg	reference to the can message
     */
    inline void sendMessage(const CanMsg& cmsg) {
        std::lock_guard<std::mutex> guard(outgointMsgsMutex_);
        sendMessageWithoutLock(cmsg);
    }

    /*! Send a sync message on the bus. Is called by BusManager::sendSyncOnAllBuses or directly.
     */
    inline void sendSync() {
        sendMessage(CanMsg(0x80, 0, nullptr));
    }

    /*! Do a sanity check of all devices on this bus.
     */
    bool sanityCheck();

    /*!
     * @return true if no device timed out
     */
    inline bool isMissingDevice() const { return isMissingDevice_; }

    /*!
     * @return true if we received a message from all devices within timeout
     */
    inline bool allDevicesActive() const { return allDevicesActive_; }


    inline bool isAsynchronous() const { return options_->asynchronous_; }

    /*!
     * Stops all threads handled by this bus (send, receive, sanity check)
     * @param wait  whether the function shall wait for the the threads to terminate or return immediately.
     */
    void stopThreads(const bool wait=true);


 public: /// Internal functions

    /*! write the message at the front of the queue to the CAN bus
     * @return true if a message was successfully written to the bus
     */
    inline bool writeMessage()
    {
        if(outgoingMsgs_.size() != 0) {
            if(writeCanMessage( outgoingMsgs_.front() )) {
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
        return readCanMessage();
    }

    /*! Internal function. Is called after reception of a message.
     * Routes the message to the callback.
     * @param cmsg  reference to the can message
     */
    void handleMessage(const CanMsg& cmsg);

    /*! Send a sync message on the bus without locking the queue.
     * This function is intended to be used by BusManager::sendSyncOnAllBuses, which locks the queue.
     */
    inline void sendSyncWithoutLock() {
        sendMessageWithoutLock(CanMsg(0x80, 0, nullptr));
    }

    /*! Waits until the output queue is empty, locks the queue and returns the lock
     */
    inline void waitForEmptyQueue(std::unique_lock<std::mutex>& lock)
    {
        lock = std::unique_lock<std::mutex>(outgointMsgsMutex_);
        condOutputQueueEmpty_.wait(lock, [this]{ return outgoingMsgs_.size() == 0 || !running_; });
    }

 protected:
    /*! Initialized the device driver
     * @return true if successful
     */
    virtual bool initializeCanBus() = 0;

    /*! read CAN message from the device driver
     * @return true if a message was successfully read and parsed
     */
    virtual bool readCanMessage() = 0;

    /*! write CAN message to the device driver
     * @return true if the message was successfully written
     */
    virtual bool writeCanMessage(const CanMsg& cmsg) = 0;


    bool processOutputQueue();

    void sendMessageWithoutLock(const CanMsg& cmsg);

    // thread loop functions
    void receiveWorker();
    void transmitWorker();
    void sanityCheckWorker();

 protected:
    // true if a device timed out. Devices in 'initializing' state are not considered as missing.
    std::atomic<bool> isMissingDevice_;

    // true if all devices are in active state (we received a message within the timeout)
    std::atomic<bool> allDevicesActive_;

    const BusOptions* options_;

    // vector containing all devices
    std::vector<Device*> devices_;

    // map mapping COB id to parse functions
    CobIdToFunctionMap cobIdToFunctionMap_;

    // output queue containing all messages to be sent by the transmitThread_
    std::mutex outgointMsgsMutex_;
    std::queue<CanMsg> outgoingMsgs_;

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

