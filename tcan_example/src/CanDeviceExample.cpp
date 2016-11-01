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

#include "tcan_example/CanDeviceExample.hpp"
#include "tcan/Bus.hpp"
#include "tcan/canopen_sdos.hpp"

namespace tcan {

namespace example_can {

CanDeviceExample::CanDeviceExample(const uint32_t nodeId, const std::string& name):
	CanDeviceExample(new CanDeviceExampleOptions(nodeId, name))

{

}

CanDeviceExample::CanDeviceExample(CanDeviceExampleOptions* options):
	DeviceCanOpen(options),
	myMeasurement_(0.f)
{

}

CanDeviceExample::~CanDeviceExample()
{

}

class mySdo : public SdoMsg {
public:
	mySdo(uint32_t nodeid, int value):
		SdoMsg(nodeid, SdoMsg::Command::WRITE_4_BYTE, 0x1010, 0x00, value) {

	}
};

bool CanDeviceExample::initDevice() {

	bus_->addCanMessage(DeviceCanOpen::TxSDOId + getNodeId(), this, &DeviceCanOpen::parseSDOAnswer);
	bus_->addCanMessage(DeviceCanOpen::TxNMTId + getNodeId(), this, &DeviceCanOpen::parseHeartBeat);
	bus_->addCanMessage(DeviceCanOpen::TxPDO1Id + getNodeId(), this, &CanDeviceExample::parsePdo1);

	setNmtRestartRemoteDevice();
	return true;
}

void CanDeviceExample::configureDevice() {
	// device is in pre-operational state when this function is called
	printf("configureDevice called\n");
	setNmtEnterPreOperational();
	sendSdo(mySdo(getNodeId(), static_cast<const CanDeviceExampleOptions*>(options_)->someParameter));
	setNmtStartRemoteDevice();
}

void CanDeviceExample::setCommand(const float value) {
	CanMsg cmsg(DeviceCanOpen::RxPDO1Id + getNodeId());
	cmsg.write(static_cast<uint32_t>(value), 0);

	bus_->sendMessage(cmsg);
}

bool CanDeviceExample::parsePdo1(const CanMsg& cmsg) {
	// variable is atomic - no need for mutexes
	myMeasurement_ = cmsg.readint32(0);

	printf("recieved PDO1 message\n");
	return true;
}


void CanDeviceExample::handleReadSdoAnswer(const SdoMsg& sdoMsg) {
    switch(sdoMsg.getIndex()) {
    default:
        int data = sdoMsg.readint32(4); // note that the data starts at byte 4 in a sdo message. byte 0-3 contain cmd,index and subindex
        printf("received SDO read answer: %d", data);
        break;
    }
}

} /* namespace example_can */

} /* namespace tcan */
