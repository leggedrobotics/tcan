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
#include "yalc/DeviceOptions.hpp"

namespace yalc {
class Bus;


//! A device that is connected via CAN.
class Device {
public:

	/*! Constructor
	 * @param nodeId	ID of CAN node
	 * @param name		human-readable name of the device
	 */
	Device() = delete;

	Device(const uint32_t nodeId, const std::string& name):
		Device(new DeviceOptions(nodeId, name))
	{
	}

	Device(DeviceOptions* options):
		options_(options),
		bus_(nullptr)
	{
	}

	//! Destructor
	virtual ~Device()
	{
		delete options_;
	}

	/*! Initialize the device. This function is automatically called by Bus::addDevice(..)
	 *   (through initDeviceInternal(..))
	 * This function is intended to do some initial device initialization (register messages to be received,
	 *   restart remote node, ...)
	 * @return true if successfully initialized
	 */
	virtual bool initDevice() = 0;

	/*! Configure the device (send SDOs)
	 * This function is intended to be (automatically) called after reception of a
	 * specific message from the device (e.g. bootup message in DeviceCanOpen)
	 */
	virtual void configureDevice() {

	}

	/*! Do a sanity check of the device. This function is intended to be called with constant rate
	 * and shall check heartbeats, SDO timeouts, ...
	 * This function is automatically called if the Bus has asynchronous=true and sanityCheckInterval > 0
	 * @return true if everything is ok.
	 */
	virtual bool sanityCheck() {
		return true;
	}

	/*! Initialize the device. This function is automatically called by Bus::addDevice(..).
	 * Calls the initDevice() function.
	 */
	bool initDeviceInternal(Bus* bus) {
		bus_ = bus;
		return initDevice();
	}

	inline uint32_t getNodeId() const { return options_->nodeId_; }
	inline const std::string& getName() const { return options_->name_; }

protected:

	DeviceOptions* options_;

	//!  reference to the CAN bus the device is connected to
	Bus* bus_;
};

} /* namespace yalc */

#endif /* DEVICE_HPP_ */
