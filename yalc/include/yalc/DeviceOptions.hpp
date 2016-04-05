/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#ifndef DEVICEOPTIONS_HPP_
#define DEVICEOPTIONS_HPP_

#include <string>

namespace yalc {

class DeviceOptions {
public:

	DeviceOptions() = delete;

	DeviceOptions(const uint32_t node_id, const std::string& node_name):
		nodeId(node_id),
		name(node_name),
		maxDeviceTimeoutCounter(10)
	{

	}

	virtual ~DeviceOptions() { }

	//! CAN node ID of device
	uint32_t nodeId;

	//! human-readable name of the device
	std::string name;

	//! counter limit at which the device is considered as timed out (see sanityCheck(..)).  Set 0 to disable.
	// does only work if the device resets deviceTimeoutCounter_ to zero in (at least) one of its message parse functions
	// maxDeviceTimeoutCounter = timeout [s] * looprate [Hz] (looprate = rate of checkSanity(..) calls. In asynchrounous mode this is 10Hz by default (see BusOptions))
	unsigned int maxDeviceTimeoutCounter;

};

} /* namespace yalc */

#endif /* DEVICEOPTIONS_HPP_ */
