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
#include <memory>
#include <vector>
#include <queue>
#include <stdint.h>

#include "yalc/PDOMsg.hpp"
#include "yalc/SDOMsg.hpp"
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

    typedef std::unique_ptr<SDOMsg> SDOMsgPtr;

    enum class NMTStates : uint8_t {
		initializing = 0,
		stopped = 1,
		preOperational = 2,
		operational = 3,
		missing = 4 // state to enter if no life sign from the node after a certain time
	};

	/*! Constructor
	 * @param nodeId	ID of CAN node
	 */
	Device(const uint16_t nodeId);
	Device(const uint16_t nodeId, const std::string& name);

	//! Destructor
	virtual ~Device();

	/*! Sets the reference to the CAN bus the device is connected to
	 * @param bus	reference to bus
	 */
	void setBus(Bus* bus);

	/*! Adds PDOs to the RxPDO manager
	 *  This function is invoked by the device manager when this device is added.
	 */
    virtual void addRxPDOs();

	/*! Adds PDOs to the TxPDO manager
	 * This function is invoked by the device manager when this device is added.
	 */
    virtual void addTxPDOs();

	/*! Initialize the device (send SDOs to initialize it)
	 * This function is automatically called after receiving the bootup message
	 * @return true if successfully initialized
	 */
	virtual bool initDevice() = 0;

	virtual void processMsg(const CANMsg& cmsg);

	/* NMT state requests
	 * setNMTRestartNode() is automatically called after initialization of the can busses in the CanManager
	 * declared as virtual to be able to have "readonly" devices, whose states are not changed (overwrite with an empty function) */
	virtual void sendNMTEnterPreOperational();
	virtual void sendNMTStartRemoteNode();
	virtual void setNMTRestartNode();

	/*! CANState accessor
	 * @return the state the device is in
	 */
    NMTStates getNMTState() const { return nmtState_; }
    void setNMTState(const NMTStates state) { nmtState_ = state; }

    uint16_t getNodeId() const { return nodeId_; }

	const std::string& getName() const { return name_; }
	void setName(const std::string& name) { name_ = name; }

protected:
    void sendSDO(SDOMsgPtr sdoMsg);

protected:

	//!  reference to the CAN bus the device is connected to
	Bus* bus_;
	//! CAN node ID of device
	uint16_t nodeId_;

	std::string name_;

	//! the can state the device is in
    NMTStates nmtState_;

	//! Heartbeat time interval [ms]. Set to 0 to disable heartbeat message reception checking.
	uint16_t producerHeartBeatTime_;

    std::vector<PDOMsg> receivePdos_;
    std::vector<PDOMsg> transmitPdos_;
    std::queue<SDOMsgPtr> sdoMsgs_;

private:
    Device(); // declared as private to prevent construction with default constructor
};

#endif /* DEVICE_HPP_ */
