/*!
 * @file 	SDOWriteMsg.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#ifndef SDOWRITEMSG_HPP_
#define SDOWRITEMSG_HPP_

#include "libcanplusplus/SDOMsg.hpp"


//! Service Data Writing Object Message
/*!	Derive a class from this class to write/send data to a CAN node via SDO
 * @ingroup robotCAN
 */
class SDOWriteMsg : public SDOMsg {
public:
	/*! Constructor
	 * @param inSMID	shared memory index of input message
	 * @param outSMID	shared memory index of output message
	 * @param nodeId	ID of CAN node
	 * @param length	length/size of data (number of bytes)
	 * @param index		index of object (2bytes)
	 * @param subindex	sub index of object (1byte)
	 * @param data		data to be sent
	 */
	SDOWriteMsg(int inSMID,
				int outSMID,
				int nodeId,
				char length,
				int index,
				int subindex,
				int data);

	//! Destructor
	virtual ~SDOWriteMsg();

protected:

	//! length/size of data (number of bytes)
	char length_;

	//! data that has to be written
	int data_;

	//! Hook function that is invoked when the input message is received
	virtual void processReceivedMsg();
};

#endif /* SDOWRITEMSG_HPP_ */
