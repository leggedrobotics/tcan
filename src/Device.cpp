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
#include "libcanplusplus/Device.hpp"
#include "libcanplusplus/canopen_sdos.hpp"

Device::Device(int nodeId, const std::string& name)
:bus_(nullptr),
 nodeId_(nodeId),
 name_(name),
 canState_(CANStates::initializing),
 producerHeartBeatTime_(0),
 txPDONMT_(new canopen::TxPDONMT(nodeId_))
{

}

Device::Device(int nodeId)
:Device(nodeId, std::string())
{
}

Device::~Device()
{

}

void Device::setBus(Bus* bus)
{
	bus_ = bus;
	bus_->getTxPDOManager()->addPDO(txPDONMT_);
}


void Device::sendSDO(SDOMsg* sdoMsg) {
  SDOMsgPtr sdo(sdoMsg);
  SDOManager* SDOManager = bus_->getSDOManager();
  sdos_.push_back(sdo);
  SDOManager->addSDO(sdo);
}

const std::string& Device::getName() const {
  return name_;
}

void Device::setName(const std::string& name) {
  name_ = name;
}

Device::CANStates Device::getCANState() const {
	return canState_;
}

bool Device::initHeartbeat(const unsigned int heartBeatTime) {

	producerHeartBeatTime_ = heartBeatTime;

	return true;
}

bool Device::checkHeartbeat() {

	if(canState_ == CANStates::initializing && txPDONMT_->isBootup()) {
		canState_ = CANStates::preOperational;
		initDevice();
	}else if(txPDONMT_->isPreOperational()) {
		canState_ = CANStates::preOperational;
	}else if(txPDONMT_->isOperational()) {
		canState_ = CANStates::operational;
	}else if(txPDONMT_->isStopped()){
		canState_ = CANStates::stopped;
	}

	// Check if heartbeat supervision is active.
	if (producerHeartBeatTime_ == 0) {
		// Heartbeat supervision is not active.  It is fine.
		return true;
	}

	// Check if device is initialized.
	if (canState_ != CANStates::operational) {
		// Device is not initialized. It is fine.
		return true;
	}

	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - txPDONMT_->getTime()).count() <= static_cast<int64_t>(producerHeartBeatTime_);
}


void Device::sendNMTEnterPreOperational() {
	sdos_.clear();
	sendSDO(new canopen::SDONMTEnterPreOperational(0, 0, nodeId_));
}

void Device::sendNMTStartRemoteNode() {
	sdos_.clear();
	sendSDO(new canopen::SDONMTStartRemoteNode(0, 0, nodeId_));
}

void Device::setNMTRestartNode() {
	sdos_.clear();
	sendSDO(new canopen::SDONMTResetNode(0, 0, nodeId_));
	canState_ = CANStates::initializing;
}



bool Device::checkSDOResponses(bool& success) {
  // Check if SDOS are processed
  bool done = true;
  // If one of the SDOs could not be sent, this flag will be false:
  success = true;

  for (auto& sdo : sdos_) {
    // Check if SDO was received or has a timeout
    if (!sdo->getIsReceived() &&  !sdo->hasTimeOut()) {
      done = false;
    }
    if(sdo->hasTimeOut()) {
      success = false;
    }
  }
  if (done) {
    sdos_.clear();
  }
  return done;
}
