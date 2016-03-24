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
#include <string>
#include <memory>
#include <queue>
#include <mutex>
#include <atomic>

#include "yalc/Device.hpp"
#include "yalc/SDOMsg.hpp"



//! A device that is connected via CAN.
/*! This class functions as a base class.
 * It provides functions to add PDOs to the bus manager and
 * an initialization function.
 * @ingroup robotCAN, device
 */
class DeviceCanOpen : public Device {
public:

	typedef std::unique_ptr<SDOMsg> SDOMsgPtr;

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

	/*! Configure the device (send SDOs to initialize it)
	 * This function is automatically called by initDevice(..)
	 * @return true if successfully configured
	 */
	virtual bool configureDevice() = 0;

	/*! Get the next SDO message from the queue
	 * @param  msg	Pointer to the next sdo msg in the queue
	 * @return true if there is a message in the queue
	 */
	bool getNextSDO(SDOMsg* msg);

	/*! Parse a heartbeat message
	 * @param cmsg   reference to the received message
	 */
	bool parseHeartBeat(const CANMsg& cmsg);

	/*! Parse a SDO answer
	 * This function removes the SDO from the queue and calls handleReadSDOAnswer() if the SDO is a read response.
	 * @param cmsg   reference to the received message
	 */
	bool parseSDOAnswer(const CANMsg& cmsg);

	/*! Handle a SDO answer
	 * this function is automatically called by parseSDO(..) and provides the possibility to save data from read SDO requests
	 * @param index		index of the SDO
	 * @param subIndex	subIndex of the SDO
	 * @param data		data of the answer to the read request (4 bytes)
	 */
	virtual void handleReadSDOAnswer(const uint16_t index, const uint8_t subIndex, const uint8_t *data) { }


	/* NMT state requests
	 * setNMTRestartNode() is automatically called after initialization of the can busses in the CanManager
	 * declared as virtual to be able to have "readonly" devices, whose states are not changed (overwrite with an empty function) */
	virtual void sendNMTEnterPreOperational();
	virtual void sendNMTStartRemoteNode();
	virtual void setNMTRestartNode();

	/*! CANState accessors
	 */
	bool isInitializing()	const { return (nmtState_ == NMTStates::initializing); }
	bool isStopped()		const { return (nmtState_ == NMTStates::stopped); }
	bool isPreOperational()	const { return (nmtState_ == NMTStates::preOperational); }
	bool isOperational()	const { return (nmtState_ == NMTStates::operational); }
	bool isMissing()		const { return (nmtState_ == NMTStates::missing); }

protected:
	void sendSDO(SDOMsgPtr sdoMsg);

protected:
	//! the can state the device is in
	std::atomic<NMTStates> nmtState_;

	//! Heartbeat time interval [ms]. Set to 0 to disable heartbeat message reception checking.
	uint16_t producerHeartBeatTime_;

	std::mutex sdoMsgsMutex_;
	std::queue<SDOMsgPtr> sdoMsgs_;
};

#endif /* DEVICECANOPEN_HPP_ */
