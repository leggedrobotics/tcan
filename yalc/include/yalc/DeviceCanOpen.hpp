/*!
 * @file 	Device.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICECANOPEN_HPP_
#define DEVICECANOPEN_HPP_

#include <stdint.h>
#include <queue>
#include <mutex>
#include <atomic>

#include "yalc/Device.hpp"
#include "yalc/SDOMsg.hpp"


//! A CANOpen device that is connected via CAN.

class DeviceCanOpen : public Device {
public:
	static constexpr int TxPDO1Id = 0x180;
	static constexpr int TxPDO2Id = 0x280;
	static constexpr int TxPDO3Id = 0x380;
	static constexpr int TxPDO4Id = 0x480;
	static constexpr int TxSDOId = 0x580;
	static constexpr int TxNMT = 0x700;

	static constexpr int RxPDOSyncId = 0x80;
	static constexpr int RxPDO1Id = 0x200;
	static constexpr int RxPDO2Id = 0x300;
	static constexpr int RxPDO3Id = 0x400;
	static constexpr int RxPDO4Id = 0x500;
	static constexpr int RxSDOId = 0x600;

	enum class NMTStates : uint8_t {
		initializing = 0,
		stopped = 1,
		preOperational = 2,
		operational = 3,
		missing = 4 // state to enter if no life sign from the node after a certain time
	};

	/*! Constructors
	 * @param nodeId	ID of CAN node
	 * @param name		name of the device
	 */
	DeviceCanOpen() = delete;
	DeviceCanOpen(const uint32_t nodeId);
	DeviceCanOpen(const uint32_t nodeId, const std::string& name);

	//! Destructor
	virtual ~DeviceCanOpen();

	/*! Do a sanity check of the device. This function is intended to be called with constant rate
	 * and shall check heartbeats, SDO timeouts, ...
	 * @return true if everything is ok.
	 */
	virtual bool sanityCheck();

	/*! Handle a SDO answer
	 * this function is automatically called by parseSDO(..) and provides the possibility to save data from read SDO requests
	 * @param index		index of the SDO
	 * @param subIndex	subIndex of the SDO
	 * @param data		data of the answer to the read request (4 bytes)
	 */
	virtual void handleReadSDOAnswer(const uint16_t index, const uint8_t subIndex, const uint8_t *data) { }


	/*! Parse a heartbeat message
	 * @param cmsg   reference to the received message
	 */
	bool parseHeartBeat(const CANMsg& cmsg);

	/*! Parse a SDO answer
	 * This function removes the SDO from the queue and calls handleReadSDOAnswer() if the SDO is a read response.
	 * @param cmsg   reference to the received message
	 */
	bool parseSDOAnswer(const CANMsg& cmsg);

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
	bool isInitializing()	const { return (nmtState_ == NMTStates::initializing); }
	bool isStopped()		const { return (nmtState_ == NMTStates::stopped); }
	bool isPreOperational()	const { return (nmtState_ == NMTStates::preOperational); }
	bool isOperational()	const { return (nmtState_ == NMTStates::operational); }
	bool isMissing()		const { return (nmtState_ == NMTStates::missing); }

protected:
	void sendSDO(const SDOMsg& sdoMsg);

protected:
	//! the can state the device is in
	std::atomic<NMTStates> nmtState_;

	//! Heartbeat time interval [ms]. Set to 0 to disable heartbeat message reception checking.
	uint16_t producerHeartBeatTime_;

	std::mutex sdoMsgsMutex_;
	std::queue<SDOMsg> sdoMsgs_;
};

#endif /* DEVICECANOPEN_HPP_ */
