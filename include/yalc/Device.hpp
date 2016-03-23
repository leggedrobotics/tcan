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
#include <stdint.h>
#include <chrono>

class Bus;


//! A device that is connected via CAN.
class Device {
public:

	/*! Constructor
	 * @param nodeId	ID of CAN node
	 */
	Device() = delete;
	Device(const uint32_t nodeId):
		Device(nodeId, std::string())
	{
	}

	Device(const uint32_t nodeId, const std::string& name):
	    bus_(nullptr),
	    nodeId_(nodeId),
	    name_(name),
		msgReceiveTime_()
	{
	}

	//! Destructor
	virtual ~Device()
	{
	}

	/*! Initialize the device
	 * @return true if successfully initialized
	 */
	bool initDevice(Bus* bus)
	{
		bus_ = bus;
		return configureDevice();
	}

	/*! Configure the device (send SDOs to initialize it)
	 * This function is automatically called by initDevice(..)
	 * @return true if successfully configured
	 */
	virtual bool configureDevice() = 0;


    uint32_t getNodeId() const { return nodeId_; }
	const std::string& getName() const { return name_; }

protected:

	//!  reference to the CAN bus the device is connected to
	Bus* bus_;

	//! CAN node ID of device
	uint32_t nodeId_;

	std::string name_;

	// time of the last message reception
	std::chrono::time_point<std::chrono::steady_clock> msgReceiveTime_;

};

#endif /* DEVICE_HPP_ */
