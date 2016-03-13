/*!
 * @file 	Device.cpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#include <stdio.h>
#include <chrono>
#include "yalc/Device.hpp"
#include "yalc/Bus.hpp"
#include "yalc/canopen_sdos.hpp"

Device::Device(const uint16_t nodeId, const std::string& name):
    bus_(nullptr),
    nodeId_(nodeId),
    name_(name),
    nmtState_(NMTStates::initializing),
    producerHeartBeatTime_(0),
    receivePdos_(),
    transmitPdos_(),
    sdoMsgs_()
{

}

Device::Device(const uint16_t nodeId):
    Device(nodeId, std::string())
{
}

Device::~Device()
{

}

void Device::setBus(Bus* bus)
{
    bus_ = bus;
}

void Device::addRxPDOs() {

}

void Device::addTxPDOs() {

}

void Device::processMsg(const CANMsg& cmsg) {
	// identify message type by its cob id (error msg, PDO1, PDO2, ..., SDO, heartbeat)
	// last 7 bits are node id
	const uint16_t msgType = cmsg.getCOBId() >> 7;

	switch(msgType) {
	case 0xE: // heartbeat

		break;

	case 0x1: // error msg

		break;

	default:
		if(true) {

		}
		break;
	}
}

void Device::sendSDO(SDOMsgPtr sdoMsg) {
    sdoMsgs_.emplace(std::move(sdoMsg));
}

void Device::sendNMTEnterPreOperational() {
    // swap with an empty queue to clear it
    std::queue<SDOMsgPtr>().swap(sdoMsgs_);
//    sendSDO(new canopen::SDONMTEnterPreOperational(0, 0, nodeId_));
}

void Device::sendNMTStartRemoteNode() {
    // swap with an empty queue to clear it
    std::queue<SDOMsgPtr>().swap(sdoMsgs_);
//    sendSDO(new canopen::SDONMTStartRemoteNode(0, 0, nodeId_));
}

void Device::setNMTRestartNode() {
    // swap with an empty queue to clear it
    std::queue<SDOMsgPtr>().swap(sdoMsgs_);
//    sendSDO(new canopen::SDONMTResetNode(0, 0, nodeId_));
    nmtState_ = NMTStates::initializing;
}

