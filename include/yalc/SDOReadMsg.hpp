/*!
 * @file 	SDOReadMsg.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#ifndef SDOREADMSG_HPP_
#define SDOREADMSG_HPP_


#include "libcanplusplus/SDOMsg.hpp"



//! Service Data Reading Object Message
/*!	Derive a class from this class to read data from a CAN node via SDO
 * @ingroup robotCAN
 */
class SDOReadMsg : public SDOMsg {
public:
	/*! Constructor
	 * @param inSMID	shared memory index of input message
	 * @param outSMID	shared memory index of output message
	 * @param nodeId	ID of CAN node
	 * @param index		index of object (2bytes)
	 * @param subindex	sub index of object (1byte)
	 */
	SDOReadMsg(int inSMID,
			int outSMID,
			int nodeId,
			int index,
			int subindex);

	//! Destructor
	virtual ~SDOReadMsg();

protected:

	//! Hook function that is invoked when the input message is received
	virtual void processReceivedMsg();

};

#endif /* SDOREADMSG_HPP_ */
