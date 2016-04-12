/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include "tcan/DeviceOptions.hpp"

namespace tcan {

struct DeviceCanOpenOptions : public DeviceOptions {
	DeviceCanOpenOptions() = delete;

	DeviceCanOpenOptions(const uint32_t nodeId, const std::string name):
		DeviceOptions(nodeId, name),
		maxSdoTimeoutCounter(1),
		sdoSendTries(10),
		producerHeartBeatTime(0)
	{

	}

	virtual ~DeviceCanOpenOptions() { }

	//! counter limit at which an SDO is considered as timed out. Set 0 to disable.
	// maxSdoTimeoutCounter = timeout [s] * looprate [Hz] (looprate = rate of checkSanity(..) calls. In asynchrounous mode this is 1Hz by default (see BusOptions))
	unsigned int maxSdoTimeoutCounter;

	//! number of tries of an SDO transmission
	unsigned int sdoSendTries;

	//! Heartbeat time interval [ms], produced by the device. Set to 0 to disable heartbeat message reception checking.
	uint16_t producerHeartBeatTime;

};

} /* namespace tcan */
