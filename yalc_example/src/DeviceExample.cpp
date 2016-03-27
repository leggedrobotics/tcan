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

#include "yalc_example/DeviceExample.hpp"
#include "yalc/Bus.hpp"
#include "yalc/canopen_sdos.hpp"

namespace example_can {

DeviceExample::DeviceExample(const uint32_t nodeId, const std::string& name):
	DeviceCanOpen(nodeId, name),
	myMeasurement_(0.f)
{

}

DeviceExample::DeviceExample(const uint32_t nodeId):
	DeviceExample(nodeId, std::string())
{
}

DeviceExample::~DeviceExample()
{

}

class mySDO : public SDOMsg {
public:
	mySDO(uint32_t nodeid, int value):
		SDOMsg(nodeid, SDOMsg::Command::WRITE_4_BYTE, 0x1010, 0x00, value) {

	}
};

bool DeviceExample::initDevice() {

	bus_->addCanMessage(DeviceCanOpen::TxSDOId + nodeId_, std::bind(&DeviceCanOpen::parseSDOAnswer, this, std::placeholders::_1));
	bus_->addCanMessage(DeviceCanOpen::TxNMT + nodeId_, std::bind(&DeviceCanOpen::parseHeartBeat, this, std::placeholders::_1));
	bus_->addCanMessage(DeviceCanOpen::TxPDO1Id + nodeId_, std::bind(&DeviceExample::parsePDO1, this, std::placeholders::_1));

	setNmtRestartRemoteDevice();
	return true;
}

void DeviceExample::configureDevice() {
	setNmtEnterPreOperational();
	sendSDO(mySDO(nodeId_, 0x40));
	setNmtStartRemoteDevice();
}

void DeviceExample::setCommand(const float value) {
	CANMsg cmsg(DeviceCanOpen::RxPDO1Id + nodeId_);
	cmsg.write(static_cast<uint32_t>(value), 0);

	bus_->sendMessage(cmsg);
}

bool DeviceExample::parsePDO1(const CANMsg& cmsg) {
	// variable is atomic - no need for mutexes
	myMeasurement_ = cmsg.readint32(0);

	printf("recieved PDO1 message\n");
	return true;
}


void DeviceExample::handleReadSDOAnswer(const uint16_t index, const uint8_t subIndex, const uint8_t *data) {
	switch(index) {
	default:
		printf("received SDO read answer");
		break;
	}
}

} /* namespace example_can */
