/*!
 * @file 	Bus.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, bus
 *
 */

#include "yalc/Bus.hpp"

Bus::Bus():
	isOperational_(false),
	syncInterval_(0),
	cobIdToFunctionMap_(),
	outgoingMsgs_()
{
}

Bus::~Bus()
{
}

bool Bus::addDevice(DevicePtr device) {
	devices_.emplace_back(std::move(device));
	return devices_[devices_.size()-1]->initDevice(this);
}

bool Bus::addCanMessage(const uint32_t cobId, std::function<bool(const CANMsg&)>&& parseFunction) {
	cobIdToFunctionMap_.emplace(cobId, std::move(parseFunction));
	return true;
}

void Bus::handleCanMessage(const CANMsg& cmsg) {

	// Check if CAN message is handled.
	CobIdToFunctionMap::iterator it = cobIdToFunctionMap_.find(cmsg.getCOBId());
	if (it != cobIdToFunctionMap_.end()) {

		it->second(cmsg);
	} else {
		auto value = cmsg.getData();
		printf("Received CAN message that is not handled: COB_ID: 0x%02X, code: 0x%02X%02X, message: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
				cmsg.getCOBId(),
				value[1],
				value[0],
				value[0],
				value[1],
				value[2],
				value[3],
				value[4],
				value[5],
				value[6],
				value[7]
		);
	}
}

void Bus::sendCanMessage(const CANMsg& cmsg) {
	outgoingMsgs_.emplace(std::move(cmsg));
}
