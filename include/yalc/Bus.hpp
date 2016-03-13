/*!
 * @file 	Bus.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#ifndef BUS_HPP_
#define BUS_HPP_

#include <stdint.h>
#include <unordered_map>
#include <memory>

#include "yalc/CANMsg.hpp"
#include "yalc/Device.hpp"

class Bus {
public:
	typedef std::unique_ptr<Device> DevicePtr;
	typedef std::unordered_map<uint16_t, DevicePtr> MapNodeIdToDevice;

    Bus();

	virtual ~Bus();

	bool addDevice(DevicePtr device);

	void handleCanMessage(const CANMsg& cmsg);

    void setOperational(const bool operational) { isOperational_ = operational; }

    bool getOperational() const { return isOperational_; }

protected:
    // state of the bus. True if all devices are operational. Sync messages is sent only if bus is operational
    bool isOperational_;

    // interval of sync message sends. set to 0 to disable
    unsigned int syncInterval_;

    MapNodeIdToDevice mapNodeIdToDevice_;
};

#endif /* BUS_HPP_ */
