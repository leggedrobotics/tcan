/*!
 * @file 	Device.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICE_HPP_
#define DEVICE_HPP_

#include <string>
#include "Bus.hpp"
#include "canopen_pdos.hpp"
class Bus;


//! A device that is connected via CAN.
/*! This class functions as a base class.
 * It provides functions to add PDOs to the bus manager and
 * an initialization function.
 * @ingroup robotCAN, device
 */
class Device {
public:

	enum class CANStates : uint8_t {
		initializing = 0,
		stopped = 1,
		preOperational = 2,
		operational = 3,
		missing = 4 // state to enter if no life sign from the node after a certain time
	};

	/*! Constructor
	 * @param nodeId	ID of CAN node
	 */
	Device(int nodeId);

	Device(int nodeId, const std::string& name);

	//! Destructor
	virtual ~Device();

	/*! Sets the reference to the CAN bus the device is connected to
	 * @param bus	reference to bus
	 */
	void setBus(Bus* bus);

	/*! Adds PDOs to the RxPDO manager
	 *  This function is invoked by the device manager when this device is added.
	 */
	virtual void addRxPDOs() = 0;

	/*! Adds PDOs to the TxPDO manager
	 * This function is invoked by the device manager when this device is added.
	 */
	virtual void addTxPDOs() = 0;

	/*! Initialize the device (send SDOs to initialize it)
	 * This function is automatically called after receiving the bootup message
	 * @return true if successfully initialized
	 */
	virtual bool initDevice() = 0;

	/*! Initialize the heartbeat reception.
	 * This does NOT configure the heartbeat generation on the device. Do that manually in the initDevice function.
	 * It only sets the expected heartbeat time
	 * @param heartBeatTime time in ms at which the producer sends heartbeat messages
	 * @return true if successfully initialized
	 */
	bool initHeartbeat(const unsigned int heartBeatTime);

	/*! Checks if the last heartbeat message has arrived recently and save the CAN-state of the device
	 * @return true if within time window
	 */
	bool checkHeartbeat();

	/* NMT state requests
	 * setNMTRestartNode() is automatically called after initialization of the can busses in the CanManager
	 * declared as virtual to be able to have "readonly" devices, whose states are not changed (overwrite with an empty function) */
	virtual void sendNMTEnterPreOperational();
	virtual void sendNMTStartRemoteNode();
	virtual void setNMTRestartNode();

	/*! CANState accessor
	 * @return the state the device is in
	 */
	CANStates getCANState() const;

	const std::string& getName() const;
	void setName(const std::string& name);

protected:
	void sendSDO(SDOMsg* sdoMsg);
	bool checkSDOResponses(bool& success);

protected:

	//!  reference to the CAN bus the device is connected to
	Bus* bus_;
	//! CAN node ID of device
	int nodeId_;

	std::string name_;

	//! List of SDO messages
	std::vector<SDOMsgPtr> sdos_;

	//! the can state the device is in
	CANStates canState_;

	//! Heartbeat time interval [ms]. Set to 0 to disable heartbeat message reception checking.
	uint16_t producerHeartBeatTime_;

	canopen::TxPDONMT* txPDONMT_;
};

#endif /* DEVICE_HPP_ */
