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
#include <vector>
#include <atomic>

#include "tcan/Bus.hpp"
#include "tcan/CanBusOptions.hpp"
#include "tcan/CanMsg.hpp"
#include "tcan/CanDevice.hpp"

namespace tcan {

class CanBus : public Bus<CanMsg> {
 public:

    typedef std::function<bool(const CanMsg&)> CallbackPtr;
    typedef std::unordered_map<uint32_t, std::pair<CanDevice*, CallbackPtr>> CobIdToFunctionMap;

    CanBus() = delete;
    CanBus(CanBusOptions* options);

    virtual ~CanBus();

    /*!
     * in-place construction of a new device
     * @param options   pointer to the option class of the device
     * @return true if successful
     */
    template <class C, typename TOptions>
    inline std::pair<C*, bool> addDevice(TOptions* options) {
        C* dev = new C(options);
        bool success = addDevice(dev);
        return std::make_pair(dev, success);
    }

    /*! Adds a device to the device vector and calls its initDevice function
     * @param device    Pointer to the device
     * @return true if init was successful
     */
    inline bool addDevice(CanDevice* device) {
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
    template <class T>
    inline bool addCanMessage(const uint32_t cobId, T* device, bool(std::common_type<T>::type::*fp)(const CanMsg&), typename std::enable_if<!std::is_base_of<CanDevice, T>::value>::type* = 0)
    {
        return cobIdToFunctionMap_.emplace(cobId, std::make_pair(nullptr, std::bind(fp, device, std::placeholders::_1))).second;
    }

    template <class T>
    inline bool addCanMessage(const uint32_t cobId, T* device, bool(std::common_type<T>::type::*fp)(const CanMsg&), typename std::enable_if<std::is_base_of<CanDevice, T>::value>::type* = 0)
    {
        return cobIdToFunctionMap_.emplace(cobId, std::make_pair(device, std::bind(fp, device, std::placeholders::_1))).second;
    }

    /*! Send a sync message on the bus. Is called by BusManager::sendSyncOnAllBuses or directly.
     */
    inline void sendSync() {
        sendMessage(CanMsg(0x80, 0, nullptr));
    }

    /*!
     * @return true if a bus error message has been received
     */
    inline bool hadBusError() const { return busErrorFlag_; }

    /*!
     * resets the bus error flag and returns its previous value.
     * @return true if a bus error message has been received
     */
    inline bool resetBusError() {
        bool tmp = busErrorFlag_;
        busErrorFlag_ = false;
        return tmp;
    }

 public:/// INTERNAL FUNCTIONS
    /*! Send a sync message on the bus without locking the queue.
     * This function is intended to be used by BusManager::sendSyncOnAllBuses, which locks the queue.
     */
    inline void sendSyncWithoutLock() {
        sendMessageWithoutLock(CanMsg(0x80, 0, nullptr));
    }

    /*! Is called after reception of a message. Routes the message to the callback.
     * @param cmsg	reference to the can message
     */
    void handleMessage(const CanMsg& cmsg);

    /*! Do a sanity check of all devices on this bus.
     */
    void sanityCheck();

 protected:
    // vector containing all devices
    std::vector<CanDevice*> devices_;

    // map mapping COB id to parse functions
    CobIdToFunctionMap cobIdToFunctionMap_;

    // flag to indicate the reception of a bus error message
    std::atomic<bool> busErrorFlag_;
};

} /* namespace tcan */

