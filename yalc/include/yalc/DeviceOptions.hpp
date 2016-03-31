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

	DeviceOptions(const uint32_t nodeId, const std::string& name):
		nodeId_(nodeId),
		name_(name),
		deviceTimeoutCounter(0)
	{

	}

	virtual ~DeviceOptions() { }

	//! CAN node ID of device
	uint32_t nodeId_;

	//! human-readable name of the device
	std::string name_;

	//! counter limit at which the device is considered as timed out (see sanityCheck(..)).  Set 0 to disable.
	unsigned int deviceTimeoutCounter;

};

} /* namespace yalc */

#endif /* DEVICEOPTIONS_HPP_ */
