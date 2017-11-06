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

#include "tcan/Bus.hpp"
#include "tcan_can/canopen_sdos.hpp"
#include "tcan_example/CanDeviceExample.hpp"

namespace tcan_example {

namespace example_can {

// put this definition in a separate header file
class mySdo : public SdoMsg {
public:
	mySdo(uint32_t nodeid, int value):
			SdoMsg(nodeid, SdoMsg::Command::WRITE_4_BYTE, 0x1010, 0x00, value) {

	}
};


CanDeviceExample::CanDeviceExample(const uint32_t nodeId, const std::string& name):
	CanDeviceExample(std::unique_ptr<CanDeviceExampleOptions>(new CanDeviceExampleOptions(nodeId, name)))

{

}

CanDeviceExample::CanDeviceExample(std::unique_ptr<CanDeviceExampleOptions>&& options):
	DeviceCanOpen(std::move(options)),
	myMeasurement_(0.f)
{

}

CanDeviceExample::~CanDeviceExample()
{

}

bool CanDeviceExample::initDevice() {

	bus_->addCanMessage(DeviceCanOpen::TxSDOId + getNodeId(), this, &DeviceCanOpen::parseSDOAnswer);
	bus_->addCanMessage(DeviceCanOpen::TxNMTId + getNodeId(), this, &DeviceCanOpen::parseHeartBeat);
	bus_->addCanMessage(DeviceCanOpen::TxPDO1Id + getNodeId(), this, &CanDeviceExample::parsePdo1);

	setNmtRestartRemoteDevice();
	return true;
}

bool CanDeviceExample::configureDevice(const CanMsg& msg) {
    // msg contains the received message which caused the call of this function. You can check if it is an
    // appropriate "bootup message" of your device

	// canopen device should already be in pre-operational state when this function is called (as per canopen standard)
    // explicitly set it to preoperational, just to make sure..
	printf("configureDevice called\n");
	setNmtEnterPreOperational();
	sendSdo(mySdo(getNodeId(), static_cast<const CanDeviceExampleOptions*>(options_.get())->someParameter));
	setNmtStartRemoteDevice();

	// when returning true, the device is considered as beeing 'active'. return false to leave it in 'missing'
	// state and wait for another message.
	return true;
}

void CanDeviceExample::setCommand(const float value) {
	// you should normally only send PDOs if the device is operational. However, for demonstration purposes, we want the software
	// to send the PDOs even if the device is missing
//	if(!isOperational()) {
//		return;
//	}

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

} /* namespace tcan_example */
