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
	mapNodeIdToDevice_()
{
}

Bus::~Bus()
{
}

bool Bus::addDevice(DevicePtr device) {
	mapNodeIdToDevice_.emplace(device->getNodeId(), std::move(device));
	return true;
}

void Bus::handleCanMessage(const CANMsg& cmsg) {

	if(cmsg.getCOBId() == 0x80) { /* sync message */


	}else if(cmsg.getCOBId() == 0x0) { /* NMT message */

		MapNodeIdToDevice::iterator it = mapNodeIdToDevice_.find( cmsg.getValue()[1] );
		if (it != mapNodeIdToDevice_.end() && cmsg.getValue()[0] == 0x01) {
			it->second->setNMTState( Device::NMTStates::operational );
		}

	}else{

		// extract node id, which is stored in the lower 7 bits of the cob id
		const uint16_t nodeId = static_cast<uint16_t>(cmsg.getCOBId() & 0x7f);

		// Check if CAN message is handled.
		MapNodeIdToDevice::iterator it = mapNodeIdToDevice_.find(nodeId);
		if (it != mapNodeIdToDevice_.end()) {

			it->second->processMsg(cmsg);
		} else {
			auto value = cmsg.getValue();
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
}
