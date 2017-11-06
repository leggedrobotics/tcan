/*
 * DeviceCanOpen.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <stdint.h>
#include <queue>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <memory>

#include "tcan_can/DeviceCanOpenOptions.hpp"
#include "tcan_can/CanDevice.hpp"
#include "tcan_can/CanBus.hpp"
#include "tcan_can/SdoMsg.hpp"


namespace tcan_can {
//! A CANOpen device that is connected via CAN.

class DeviceCanOpen : public CanDevice {
 public:
    static constexpr int TxNMTCmdId = 0x0;
    static constexpr int TxEmcyId = 0x80;
    static constexpr int TxPDO1Id = 0x180;
    static constexpr int TxPDO2Id = 0x280;
    static constexpr int TxPDO3Id = 0x380;
    static constexpr int TxPDO4Id = 0x480;
    static constexpr int TxSDOId = 0x580;
    static constexpr int TxNMTId = 0x700;

    static constexpr int RxPDOSyncId = 0x80;
    static constexpr int RxPDO1Id = 0x200;
    static constexpr int RxPDO2Id = 0x300;
    static constexpr int RxPDO3Id = 0x400;
    static constexpr int RxPDO4Id = 0x500;
    static constexpr int RxSDOId = 0x600;

    enum class NMTStates : uint8_t {
        stopped = 1,
        preOperational = 2,
        operational = 3
    };

    /*! Constructors
     * @param nodeId	ID of CAN node
     * @param name		name of the device
     */
    DeviceCanOpen(const uint32_t nodeId, const std::string& name);
    DeviceCanOpen(std::unique_ptr<DeviceCanOpenOptions>&& options);

    //! Destructor
    virtual ~DeviceCanOpen();

    /*! Do a sanity check of the device. This function is intended to be called with constant rate
     * and shall check heartbeats, SDO timeouts, ...
     * This function is automatically called if the Bus has sanityCheckInterval > 0
     * @return true if everything is ok.
     */
    virtual void sanityCheck();

    /*! Send a PDO message.
     * @param pdoMsg Message to be sent.
     */
    void sendPdo(const CanMsg& pdoMsg);

    /*! Put an SDO at the end of the sdo queue and send automatically on the CAN bus.
     * To receive the answer of read SDO's it is necessary to implement the handleReadSDOAnswer(..) function.
     * @param sdoMsg Message to be sent
     */
    void sendSdo(const SdoMsg& sdoMsg);

    /*! Handle a SDO answer
     * this function is automatically called by parseSDO(..) and provides the possibility to save data from read SDO requests.
     * @param sdoMsg	the SDO response message (Note that sdoMsg is not a complete instance of an SdoMsg, only the members defined in CanMsg are initialized)
     */
    virtual void handleReadSdoAnswer(const SdoMsg& sdoMsg) { }

    /*!
     * This function is automatically called by sanityCheck(..) if a sdo message exceeded the timeout and the number of retries.
     * @param sdoMsg    the SDO message that failed
     */
    virtual void handleTimedoutSdo(const SdoMsg& msg);

    /*!
     * This function is called upon reception of a SDO error.
     * @param request   The message sent to the device ..
     * @param answer    .. the answer received
     */
    virtual void handleSdoError(const SdoMsg& request, const SdoMsg& answer);

    /*! Get the SDO answer and erase it from the SDO answer map if it has been received.
     * @param sdoAnswer SDO answer if it has been found (output parameter).
     * @return true if SDO answer has been found.
     */
    bool getSdoAnswer(SdoMsg& sdoAnswer);

    /*! NMT state requests. Send a NMT CAN message to the device.
     * The following functions also clear the sdo queue and set the nmtState_:
     *    setNmtEnterPreOperational(), setNmtResetRemoteCommunication(), setNmtRestartRemoteDevice()
     * All other functions set the nmtState_ only if heartbeat message is disabled.
     */
    void setNmtEnterPreOperational();
    void setNmtStartRemoteDevice();
    void setNmtStopRemoteDevice();
    void setNmtResetRemoteCommunication();
    void setNmtRestartRemoteDevice();

    /*! CANState accessors
     */
    bool isStopped()		const { return isActive() && (nmtState_ == NMTStates::stopped); }
    bool isPreOperational()	const { return isActive() && (nmtState_ == NMTStates::preOperational); }
    bool isOperational()	const { return isActive() && (nmtState_ == NMTStates::operational); }

    virtual int getStatus() const {
        if(static_cast<int>(state_.load()) <= 0) {
            // device is initializing, missing or has an error
            return static_cast<int>(state_.load());
        }
        return static_cast<int>(nmtState_.load());
    }

    /*!
     * Resets the device to Initializing state and sends the RestartRemoteDevice NMT command.
     */
    virtual void resetDevice();

 public: /// Internal functions
    /*! Parse a heartbeat message
     * @param cmsg   reference to the received message
     */
    bool parseHeartBeat(const CanMsg& cmsg);

    /*! Parse a SDO answer
     * This function removes the SDO from the queue and calls handleReadSDOAnswer() if the SDO is a read response.
     * @param cmsg   reference to the received message
     */
    bool parseSDOAnswer(const CanMsg& cmsg);

 protected:
    /*! Check if the SDO at the front of the SDO queue has timed out. If so, try to resend it a couple of times (see DeviceCanOpenOptions)
     * @return false if no answer was received after a couple of sending attempts.
     */
    bool checkSdoTimeout();

    /*!
     * put the next SDO from the sdo queue into the bus output queue.
     * WARNING: This function does not lock the sdoMsgsMutex_, so its up to the caller to do so.
     */
    void sendNextSdo();

    /*!
     * Acquires lock on the sdo queue mutex and clears the queue
     */
    void clearSdoQueue();


    /*! Get the ID of an SDO answer by index and subIndex.
     * @param index SDO index.
     * @param subIndex SDO subIndex.
     * @return SDO id.
     */
    static uint32_t getSdoAnswerId(const uint16_t index, const uint8_t subIndex);


 protected:
    //! the can state the device is in
    std::atomic<NMTStates> nmtState_;

    std::atomic<unsigned int> sdoTimeoutCounter_;
    std::atomic<unsigned int> sdoSentCounter_;

    std::mutex sdoMsgsMutex_;
    std::queue<SdoMsg> sdoMsgs_;

    // Map from SDO answer id to SDO answer.
    std::mutex sdoAnswerMapMutex_;
    std::unordered_map<uint32_t, SdoMsg> sdoAnswerMap_;
};

} /* namespace tcan_can */
