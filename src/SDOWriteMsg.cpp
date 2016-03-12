/*!
 * @file 	SDOWriteMsg.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#include "libcanplusplus/SDOWriteMsg.hpp"

SDOWriteMsg::SDOWriteMsg(int inSMID,
		int outSMID,
		int nodeId,
		char length,
		int index,
		int subindex,
		int data)
:SDOMsg(inSMID, outSMID, nodeId, index, subindex),
 length_(length),
 data_(data)
{
	int Length[8] = {1, 1, 1, 1, 1, 1, 1, 1};
	int Value[8] = {length_,
					(index & 0x00ff),
					(index & 0xff00)>>8,
					subindex,
					(data_ & 0x000000ff),
					(data_ & 0x0000ff00)>>8,
					(data_ & 0x00ff0000)>>16,
					(data_ & 0xff000000)>>24};
	outputMsg_->setLength(Length);
	outputMsg_->setValue(Value);
	outputMsg_->setFlag(1);

}

SDOWriteMsg::~SDOWriteMsg()
{

}

void SDOWriteMsg::processReceivedMsg()
{

}
