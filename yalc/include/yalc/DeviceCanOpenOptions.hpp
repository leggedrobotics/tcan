/*
 * BusOptions.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#ifndef DEVICECANOPENOPTIONS_HPP_
#define DEVICECANOPENOPTIONS_HPP_

#include "yalc/DeviceOptions.hpp"

namespace yalc {

class DeviceCanOpenOptions : public DeviceOptions {
public:

	DeviceCanOpenOptions() = delete;

	DeviceCanOpenOptions(const uint32_t nodeId, const std::string name):
		DeviceOptions(nodeId, name),
		sdoTimeoutCounter(0),
		sdoSendTries(0),
		producerHeartBeatTime(0)
	{

	}

	virtual ~DeviceCanOpenOptions() { }

	//! counter limit at which an SDO is considered as timed out. Set 0 to disable.
	unsigned int sdoTimeoutCounter;

	//! number of tries of an SDO transmission
	unsigned int sdoSendTries;

	//! Heartbeat time interval [ms], produced by the device. Set to 0 to disable heartbeat message reception checking.
	uint16_t producerHeartBeatTime;

};

} /* namespace yalc */

#endif /* DEVICECANOPENOPTIONS_HPP_ */
