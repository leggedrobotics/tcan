/*
 * Device.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#ifndef DEVICE_HPP_
#define DEVICE_HPP_

#include <string>
#include <stdint.h>

#include "yalc/CANMsg.hpp"

class Bus;


//! A device that is connected via CAN.
class Device {
public:

	/*! Constructor
	 * @param nodeId	ID of CAN node
	 * @param name		human-readable name of the device
	 */
	Device() = delete;
	Device(const uint32_t nodeId):
		Device(nodeId, std::string())
	{

	}

	Device(const uint32_t nodeId, const std::string& name):
		bus_(nullptr),
		nodeId_(nodeId),
		name_(name)
	{
	}

	//! Destructor
	virtual ~Device()
	{
	}

	/*! Initialize the device. This function is automatically called by Bus::addDevice(..)
	 *   (through initDeviceInternal(..))
	 * This function is intended to do some initial device initialization (register messages to be received,
	 *   restart remote node, ...)
	 * @return true if successfully initialized
	 */
	virtual bool initDevice() = 0;

	/*! Configure the device (send SDOs to initialize it)
	 * This function is intended to be called (automatically) after reception of a
	 * specific message from the device (e.g. bootup message in CANOpen)
	 */
	virtual void configureDevice() {

	}

	/*! Do a sanity check of the device. This function is intended to be called with constant rate
	 * and shall check heartbeats, SDO timeouts, ...
	 * @return true if everything is ok.
	 */
	virtual bool sanityCheck() {
		return true;
	}

	/*! Initialize the device. This function is automatically called by Bus::addDevice(..).
	 * Calls the initDevice() function.
	 */
	virtual bool initDeviceInternal(Bus* bus) {
		bus_ = bus;
		return initDevice();
	}

	uint32_t getNodeId() const { return nodeId_; }
	const std::string& getName() const { return name_; }

protected:

	//!  reference to the CAN bus the device is connected to
	Bus* bus_;

	//! CAN node ID of device
	uint32_t nodeId_;

	//! human-readable name of the device
	std::string name_;
};

#endif /* DEVICE_HPP_ */
