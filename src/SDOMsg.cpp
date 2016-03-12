/*!
 * @file 	SDOMsg.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#include "libcanplusplus/SDOMsg.hpp"
#include <stdio.h>

SDOMsg::SDOMsg(int inSDOSMID, int outSDOSMID, int nodeId, int index, int subIndex)
:nodeId_(nodeId),
 timeout_(0),
 index_(index),
 subIndex_(subIndex),
 isSent_(false),
 isReceived_(false),
 isWaiting_(false),
 isQueuing_(false)
{
	inputMsg_ = new CANOpenMsg(0x580 + nodeId_, inSDOSMID);
	outputMsg_ = new CANOpenMsg(0x600 + nodeId_, outSDOSMID);

}

SDOMsg::~SDOMsg()
{
	delete inputMsg_;
	delete outputMsg_;

}

CANOpenMsg* SDOMsg::getOutputMsg()
{
	return outputMsg_;
}

CANOpenMsg* SDOMsg::getInputMsg()
{

	return inputMsg_;
}

bool SDOMsg::hasTimeOut()
{
	const int maxTimeOut = 10;

	if (timeout_ >= maxTimeOut) {
		return true;
	}
	return false;
}

bool SDOMsg::getIsSent()
{
	return isSent_;
}

bool SDOMsg::getIsReceived()
{
	return isReceived_;
}


bool SDOMsg::getIsWaiting()
{
	return isWaiting_;
}

void SDOMsg::setIsWaiting(bool isWaiting)
{
	isWaiting_ = isWaiting;
}

bool SDOMsg::getIsQueuing()
{
	return isQueuing_;
}

void SDOMsg::setIsQueuing(bool isQueuing)
{
	isQueuing_ = isQueuing;
}

void SDOMsg::sendMsg(CANMsg *canDataDes)
{

	if (isSent_) {
		outputMsg_->setFlag(0);
	}
	outputMsg_->getCANMsg(canDataDes);
	isSent_ = true;
	isQueuing_ = false;
	isWaiting_ = true;
}
void SDOMsg::receiveMsg(CANMsg *canDataMeas)
{

	if (canDataMeas->flag) {
		isReceived_ = true;
		isWaiting_ = false;
		inputMsg_->setCANMsg(canDataMeas);
		processReceivedMsg();
	} else {
		timeout_++;
	}

	const int maxTimeOut = 10;
	if (timeout_ >= maxTimeOut) {
		isWaiting_ = false;
	}
}

void SDOMsg::processReceivedMsg()
{

}

