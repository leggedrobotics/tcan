#pragma once

#include <stdint.h>
#include <atomic>
#include <memory>

#include "tcan_can/DeviceCanOpen.hpp"

namespace tcan_example {

class CanDeviceExampleOptions : public tcan_can::DeviceCanOpenOptions {
public:
	CanDeviceExampleOptions() = delete;
	CanDeviceExampleOptions(const uint32_t nodeId, const std::string& name):
			tcan_can::DeviceCanOpenOptions(nodeId, name),
			someParameter(0)
	{

	}

	unsigned int someParameter;
};


class CanDeviceExample : public tcan_can::DeviceCanOpen {
public:

	/*! Constructors
	 * @param nodeId	ID of CAN node
	 * @param name		name of the device
	 */
	CanDeviceExample() = delete;
	CanDeviceExample(const uint32_t nodeId, const std::string& name);
	CanDeviceExample(std::unique_ptr<CanDeviceExampleOptions>&& options);

	virtual ~CanDeviceExample() = default;

	bool initDevice() override;
	bool configureDevice(const tcan_can::CanMsg& msg) override;

	void setCommand(const float value);

	bool parsePdo1(const tcan_can::CanMsg& cmsg);

	void handleReadSdoAnswer(const tcan_can::SdoMsg& sdoMsg);

	float getMeasurement() const { return myMeasurement_; }

protected:
	std::atomic<float> myMeasurement_;
};


} /* namespace tcan_example */
