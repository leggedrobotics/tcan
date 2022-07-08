#include <iostream>

#include "tcan/Bus.hpp"
#include "tcan_example/CanDeviceExample.hpp"

namespace tcan_example {

// cleaner approach would be to put this definition in a separate header file
class mySdo : public tcan_can::SdoMsg {
public:
    mySdo(uint32_t nodeid, int value):
            SdoMsg(nodeid, SdoMsg::Command::WRITE_4_BYTE, 0x1010, 0x00, value)
    {
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

bool CanDeviceExample::initDevice() {
    // This function is automatically called when the device is added to the bus (with addDevice(..))
    // register the messages here with addCanMessage(..) and optionally restart the device with setNmtRestartRemoteDevice()

    bus_->addCanMessage(DeviceCanOpen::TxSDOId + getNodeId(), this, &DeviceCanOpen::parseSDOAnswer); // register callback for read SDO answers
    bus_->addCanMessage(DeviceCanOpen::TxNMTId + getNodeId(), this, &DeviceCanOpen::parseHeartBeat); // register callback for heartbeat messages
    bus_->addCanMessage(DeviceCanOpen::TxPDO1Id + getNodeId(), this, &CanDeviceExample::parsePdo1); // register callback for Rx PDO 1

    setNmtRestartRemoteDevice();
    return true;
}

bool CanDeviceExample::configureDevice(const tcan_can::CanMsg& /*msg*/) {
    // This function is automatically called on reception of the first message from this device ("bootup message")
    // Configure the device here by sending the SDOs with sendSdo(..). Set the device to operational mode by calling setNmtStartRemoteDevice()
    // parameter 'msg' contains the received message which caused the call of this function. You can check if it is an
    // appropriate "bootup message" of your device

    // canopen device should already be in pre-operational state when this function is called (as per canopen standard)
    // explicitly set it to preoperational, just to make sure..
    std::cout << "configureDevice called\n";
    setNmtEnterPreOperational();
    sendSdo(mySdo(getNodeId(), static_cast<const CanDeviceExampleOptions*>(options_.get())->someParameter));
    setNmtStartRemoteDevice();

    // when returning true, the device is considered as beeing 'active'. return false to leave it in 'missing' or 'initializing'
    // state and wait for another message.
    return true;
}

void CanDeviceExample::setCommand(const float value) {
    // you should normally only send PDOs if the device is operational. However, for demonstration purposes, we want the software
    // to send the PDOs even if the device is missing
//	if(!isOperational()) {
//		return;
//	}

    tcan_can::CanMsg cmsg(DeviceCanOpen::RxPDO1Id + getNodeId());
    cmsg.write(static_cast<uint32_t>(value), 0);

    bus_->sendMessage(cmsg);
}

bool CanDeviceExample::parsePdo1(const tcan_can::CanMsg& cmsg) {
    // This function is automatically called on reception of a message registered in the initDevice() function
    // Parse the CAN message and save the interesting values to member variables. Ensuring thread safety is up to the user!

    // variable is atomic - no need for mutexes
    myMeasurement_ = cmsg.readint32(0);

    std::cout << "recieved PDO1 message\n";
    return true;
}


void CanDeviceExample::handleReadSdoAnswer(const tcan_can::SdoMsg& sdoMsg) {
    // This function is automatically called on reception of a read SDO answer
    switch(sdoMsg.getIndex()) {
        default:
            int data = sdoMsg.readint32(4); // note that the data starts at byte 4 in a sdo message. byte 0-3 contain cmd,index and subindex
            std::cout << "received SDO read answer: " <<  data << "\n";
            break;
    }
}

} /* namespace tcan_example */
