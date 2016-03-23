/*!
 * @file 	DeviceCanOpen.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include <stdio.h>
#include "yalc/DeviceCanOpen.hpp"
#include "yalc/canopen_sdos.hpp"

DeviceCanOpen::DeviceCanOpen(const uint32_t nodeId, const std::string& name):
    Device(nodeId, name),
	nmtState_(NMTStates::initializing),
	producerHeartBeatTime_(0),
    sdoMsgs_()
{

}

DeviceCanOpen::DeviceCanOpen(const uint32_t nodeId):
	DeviceCanOpen(nodeId, std::string())
{
}

DeviceCanOpen::~DeviceCanOpen()
{

}

bool DeviceCanOpen::getNextSDO(SDOMsg* msg) {

	if(sdoMsgs_.empty()) {
		return false;
	}

	msg = sdoMsgs_.front().get();
	return true;
}

bool DeviceCanOpen::parseHeartBeat(const CANMsg& cmsg) {

	if(cmsg.getLength() != 1) {
		printf("Invalid Heartbeat message length from nodeId %x: %d\n", nodeId_, cmsg.getLength());
		return false;
	}

	switch(cmsg.readuint8(0)) {
	case 0x0: // bootup
	case 0x7F: // pre-operational
		nmtState_ = NMTStates::preOperational; // devices should switch automatically to pre-operational after bootup
		break;

	case 0x04: // stopped
		nmtState_ = NMTStates::stopped;
		break;

	case 0x05: // operational
		nmtState_ = NMTStates::operational;
		break;

	default:
		printf("Invalid Heartbeat message data from nodeId %x: %x\n", nodeId_, cmsg.readuint8(0));
		return false;
		break;
	}

	return true;
}

bool DeviceCanOpen::parseSDOAnswer(const CANMsg& cmsg) {
	const uint8_t responseMode = cmsg.readuint8(0);
	const uint16_t index = cmsg.readuint16(1);
    const uint8_t subindex = cmsg.readuint8(3);
    SDOMsgPtr& sdo = sdoMsgs_.front();

    if(sdo->getIndex() == index && sdo->getSubIndex() == subindex) {

    	if(responseMode == 0x43 || responseMode == 0x4B || responseMode == 0x4F) { // read responses (4, 2 or 1 byte)
    		handleReadSDOAnswer( index, subindex, &(cmsg.getData()[4]) );
    	}else if(responseMode == 0x80) { // error response
    		printf("Received SDO error. COB=%x / index=%x / subindex=%x / data=%x\n", cmsg.getCOBId(), index, subindex, cmsg.readuint32(4));
    	}

    	{
    		sdoMsgs_.pop();
    	}
    }else{
    	printf("Received unexpected SDO answer. COB=%x / index=%x / subindex=%x / data=%x\n", cmsg.getCOBId(), index, subindex, cmsg.readuint32(4));
    	return false;
    }

    return true;
}


void DeviceCanOpen::sendSDO(SDOMsgPtr sdoMsg) {
    sdoMsgs_.emplace(std::move(sdoMsg));
}

void DeviceCanOpen::sendNMTEnterPreOperational() {
    // swap with an empty queue to clear it
    std::queue<SDOMsgPtr>().swap(sdoMsgs_);
//    sendSDO(new canopen::SDONMTEnterPreOperational(0, 0, nodeId_));
}

void DeviceCanOpen::sendNMTStartRemoteNode() {
    // swap with an empty queue to clear it
    std::queue<SDOMsgPtr>().swap(sdoMsgs_);
//    sendSDO(new canopen::SDONMTStartRemoteNode(0, 0, nodeId_));
}

void DeviceCanOpen::setNMTRestartNode() {
    // swap with an empty queue to clear it
    std::queue<SDOMsgPtr>().swap(sdoMsgs_);
//    sendSDO(new canopen::SDONMTResetNode(0, 0, nodeId_));
    nmtState_ = NMTStates::initializing;
}

