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

#include <stdint.h>
#include <atomic>

#include "tcan/DeviceCanOpen.hpp"

namespace tcan {

namespace example_can {
//! An example device that is connected via CAN.

class DeviceExampleOptions : public DeviceCanOpenOptions {
public:
	DeviceExampleOptions() = delete;
	DeviceExampleOptions(const uint32_t nodeId, const std::string& name):
		DeviceCanOpenOptions(nodeId, name),
		someParameter(0)
	{

	}

	unsigned int someParameter;
};


class DeviceExample : public DeviceCanOpen {
public:

	/*! Constructors
	 * @param nodeId	ID of CAN node
	 * @param name		name of the device
	 */
	DeviceExample() = delete;
	DeviceExample(const uint32_t nodeId, const std::string& name);
	DeviceExample(DeviceExampleOptions* options);

	//! Destructor
	virtual ~DeviceExample();

	virtual bool initDevice();

	virtual void configureDevice();

	void setCommand(const float value);

	bool parsePdo1(const CanMsg& cmsg);

    void handleReadSdoAnswer(const SdoMsg& sdoMsg);

	float getMeasurement() const { return myMeasurement_; }



protected:
	std::atomic<float> myMeasurement_;
};

} /* namespace example_can */

} /* namespace tcan */
