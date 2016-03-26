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
#include "yalc/Bus.hpp"

DeviceCanOpen::DeviceCanOpen(const uint32_t nodeId, const std::string& name):
	Device(nodeId, name),
	nmtState_(NMTStates::initializing),
	producerHeartBeatTime_(0),
	sdoMsgsMutex_(),
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

bool DeviceCanOpen::sanityCheck() {

	return true;
}


bool DeviceCanOpen::parseHeartBeat(const CANMsg& cmsg) {

	if(cmsg.getLength() != 1) {
		printf("Invalid Heartbeat message length from nodeId %x: %d\n", nodeId_, cmsg.getLength());
		return false;
	}

	switch(cmsg.readuint8(0)) {
	case 0x0: // bootup
		nmtState_ = NMTStates::preOperational; // devices should switch automatically to pre-operational after bootup
		configureDevice();
		break;

	case 0x7F: // pre-operational
		nmtState_ = NMTStates::preOperational;
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
	const SDOMsg& sdo = sdoMsgs_.front();

	if(sdo.getIndex() == index && sdo.getSubIndex() == subindex) {

		if(responseMode == 0x43 || responseMode == 0x4B || responseMode == 0x4F) { // read responses (4, 2 or 1 byte)
			handleReadSDOAnswer( index, subindex, &(cmsg.getData()[4]) );
		}else if(responseMode == 0x80) { // error response
			const int32_t error = cmsg.readint32(4);
			printf("Received SDO error: %s. COB=%x / index=%x / subindex=%x / error=%x\n", SDOMsg::getErrorName(error).c_str(), cmsg.getCOBId(), index, subindex, error);
			// todo: further error handling
		}

		{
			std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
			sdoMsgs_.pop();

			// put next SDO message into the bus output queue
			if(sdoMsgs_.size() > 0) {
				bus_->sendMessage(sdoMsgs_.front());
			}
		}
	}else{
		printf("Received unexpected SDO answer. COB=%x / index=%x / subindex=%x / data=%x\n", cmsg.getCOBId(), index, subindex, cmsg.readuint32(4));
		return false;
	}

	return true;
}


void DeviceCanOpen::sendSDO(const SDOMsg& sdoMsg) {

	std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
	sdoMsgs_.push(sdoMsg);

	if(sdoMsgs_.size() == 1) {
		// sdo queue was empty before, so put the new message in the bus output queue
		bus_->sendMessage(sdoMsgs_.front());
	}

}

void DeviceCanOpen::setNmtEnterPreOperational() {
	{
		std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
		// swap with an empty queue to clear it
		std::queue<SDOMsg>().swap(sdoMsgs_);
	}
	bus_->sendMessage( CANMsg(0, 2, {0x80, static_cast<uint8_t>(nodeId_)}) );
	// todo: wait some time?

	// the remote device will not tell us in which state it is if heartbeat message is disabled
	//   => assume that the state switch will be successful
	if(producerHeartBeatTime_ == 0) {
		nmtState_ == NMTStates::preOperational;
	}
}

void DeviceCanOpen::setNmtStartRemoteDevice() {
	bus_->sendMessage( CANMsg(0, 2, {0x1, static_cast<uint8_t>(nodeId_)}) );

	// the remote device will not tell us in which state it is if heartbeat message is disabled
	//   => assume that the state switch will be successful
	if(producerHeartBeatTime_ == 0) {
		nmtState_ == NMTStates::operational;
	}
}

void DeviceCanOpen::setNmtStopRemoteDevice() {
	bus_->sendMessage( CANMsg(0, 2, {0x2, static_cast<uint8_t>(nodeId_)}) );

	// the remote device will not tell us in which state it is if heartbeat message is disabled
	//   => assume that the state switch will be successful
	if(producerHeartBeatTime_ == 0) {
		nmtState_ == NMTStates::stopped;
	}
}

void DeviceCanOpen::setNmtResetRemoteCommunication() {
	{
		std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
		// swap with an empty queue to clear it
		std::queue<SDOMsg>().swap(sdoMsgs_);
	}
	bus_->sendMessage( CANMsg(0, 2, {0x82, static_cast<uint8_t>(nodeId_)}) );

	nmtState_ = NMTStates::initializing;
}

void DeviceCanOpen::setNmtRestartRemoteDevice() {
	{
		std::lock_guard<std::mutex> guard(sdoMsgsMutex_);
		// swap with an empty queue to clear it
		std::queue<SDOMsg>().swap(sdoMsgs_);
	}
	bus_->sendMessage( CANMsg(0, 2, {0x81, static_cast<uint8_t>(nodeId_)}) );

	nmtState_ = NMTStates::initializing;
}

