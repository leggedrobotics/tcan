/*!
 * @file 	Device.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef DEVICEEXAMPLE_HPP_
#define DEVICEEXAMPLE_HPP_

#include <stdint.h>
#include <atomic>

#include "yalc/DeviceCanOpen.hpp"


//! An example device that is connected via CAN.

class DeviceExample : public DeviceCanOpen {
public:

	/*! Constructors
	 * @param nodeId	ID of CAN node
	 * @param name		name of the device
	 */
	DeviceExample() = delete;
	DeviceExample(const uint32_t nodeId);
	DeviceExample(const uint32_t nodeId, const std::string& name);

	//! Destructor
	virtual ~DeviceExample();

	virtual bool initDevice();

	virtual void configureDevice();

	void setCommand(const float value);

	bool parsePDO1(const CANMsg& cmsg);

	/*! Handle a SDO answer
	 * this function is automatically called by parseSDO(..) and provides the possibility to save data from read SDO requests
	 * @param index		index of the SDO
	 * @param subIndex	subIndex of the SDO
	 * @param data		data of the answer to the read request (4 bytes)
	 */
	virtual void handleReadSDOAnswer(const uint16_t index, const uint8_t subIndex, const uint8_t *data);

protected:
	std::atomic<float> myMeasurement_;
};

#endif /* DEVICEEXAMPLE_HPP_ */
