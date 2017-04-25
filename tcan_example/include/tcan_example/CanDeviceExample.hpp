/*!
 * @file 	Device.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#pragma once

#include "tcan/DeviceCanOpen.hpp"

#include <stdint.h>
#include <atomic>


namespace tcan {

namespace example_can {
//! An example device that is connected via CAN.

class CanDeviceExampleOptions : public DeviceCanOpenOptions {
public:
	CanDeviceExampleOptions() = delete;
	CanDeviceExampleOptions(const uint32_t nodeId, const std::string& name):
		DeviceCanOpenOptions(nodeId, name),
		someParameter(0)
	{

	}

	unsigned int someParameter;
};


class CanDeviceExample : public DeviceCanOpen {
public:

	/*! Constructors
	 * @param nodeId	ID of CAN node
	 * @param name		name of the device
	 */
	CanDeviceExample() = delete;
	CanDeviceExample(const uint32_t nodeId, const std::string& name);
	CanDeviceExample(CanDeviceExampleOptions* options);

	//! Destructor
	virtual ~CanDeviceExample();

	virtual bool initDevice();

	virtual bool configureDevice(const CanMsg& msg);

	void setCommand(const float value);

	bool parsePdo1(const CanMsg& cmsg);

    void handleReadSdoAnswer(const SdoMsg& sdoMsg);

	float getMeasurement() const { return myMeasurement_; }



protected:
	std::atomic<float> myMeasurement_;
};

} /* namespace example_can */

} /* namespace tcan */
