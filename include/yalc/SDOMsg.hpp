/*!
 * @file 	SDOMsg.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#ifndef SDOMSG_HPP_
#define SDOMSG_HPP_

#include "libcanplusplus/CANOpenMsg.hpp"
#include <boost/shared_ptr.hpp>


//! Service Data Object Message Container
/*! It contains an output CANOpen message that needs to be sent to the CAN node and
 *  an input CANOpen message that stores the answer received from the CAN node.
 *  It also stores information about the status of the SDO that is
 *  	queuing:	not yet processed by the SDO manager
 *  	sent:		output message was sent to CAN node
 *  	waiting:	is waiting for a response of the CAN node
 *  	received:	has received a response from the CAN node
 *  	timeout:	has not received a response after a while
 *
 * @ingroup robotCAN
 */
class SDOMsg {
public:
	/*! Constructor
	 *
	 * @param inSMID	index of shared memory of input message
	 * @param outSMID   index of shared memory of output message
	 * @param nodeId	ID of the CAN node
	 */
	SDOMsg(int inSMID, int outSMID, int nodeId, int index = 0, int subIndex = 0);

	//! Destructor
	virtual ~SDOMsg();

	/*! Gets the reference to the output message
	 * @return	 output message
	 */
	CANOpenMsg* getOutputMsg();

	/*! Gets the reference to the input message
	 * @return	 input message
	 */
	CANOpenMsg* getInputMsg();

	/*! Gets flag
	 * @return true if input message was not received in a certain time
	 */
	bool hasTimeOut();

	/*! Gets flag
	 * @return true if output message was sent to the node
	 */
	bool getIsSent();

	/*! Gets flag
	 * @return	true if input message was received from node
	 */
	bool getIsReceived();

	/*! Gets flag
	 * @return true if SDO is pending for receiving an answer from the node
	 */
	bool getIsWaiting();

	/*! Gets flag
	 * @return true if SDO is queuing to be processed by the SDO manager, i.e. the
	 * 	message neither was sent nor is waiting for an input message
	 */
	bool getIsQueuing();

	/*! The SDO manager sets the isWaiting flag to true as soon as it has sent
	 * the output message
	 * @param isWaiting
	 */
	virtual void setIsWaiting(bool isWaiting);

	/*! Sets the flag that the SDO was added to the SDOManager
	 * @param isQueuing
	 */
	virtual void setIsQueuing(bool isQueuing);

	/*! Gets the output CAN message that needs to be sent to the CAN node
	 * @param[out] canDataDes	output CAN message
	 */
	void sendMsg(CANMsg *canDataDes);

	/*! Sets the input CAN message that was received from the CAN node
	 * @param[in] canDataMeas	input CAN message
	 */
	void receiveMsg(CANMsg *canDataMeas);

	inline uint8_t readuint8() const
	{
	  uint8_t value;
	  value  = ((uint8_t)inputMsg_->getValue()[4]);
	  return value;
	}

  inline uint16_t readuint16() const
  {
      uint16_t value;
      value  = ((uint16_t)inputMsg_->getValue()[5] << 8);
      value |= ((uint16_t)inputMsg_->getValue()[4]);
      return value;
  }

  inline int16_t readint16() const
  {
      int16_t value;
      value  = ((int16_t)inputMsg_->getValue()[5] << 8);
      value |= ((int16_t)inputMsg_->getValue()[4]);
      return value;
  }

  inline int32_t readint32() const
  {
      int32_t value;
      value  = ((int32_t)inputMsg_->getValue()[7] << 24);
      value |= ((int32_t)inputMsg_->getValue()[6] << 16);
      value |= ((int32_t)inputMsg_->getValue()[5] << 8);
      value |= ((int32_t)inputMsg_->getValue()[4]);
      return value;
  }

  //! getters for index and subindex for message verfication
  int getIndex() {return index_;}
  int getSubIndex() {return subIndex_;}


protected:
	//! Hook function that is invoked when a message is received
	virtual void processReceivedMsg();

	//! CAN node ID
	int nodeId_;

	//! timeout counter
	int timeout_;

	//! Index
	int index_;

	//! SubIndex
	int subIndex_;

	//! if true, SDO manager has sent SDO through CAN network
	bool isSent_;

	//! if true, response from node is received
	bool isReceived_;

	//! if true, SDO was sent and SDO manager is waiting for a response of the node
	bool isWaiting_;

	//! if true, SDO was added to the SDO manager, but not yet sent
	bool isQueuing_;

	//! input CAN message that is received from the CAN node
	CANOpenMsg* inputMsg_;

	//! output CAN message that will be sent to the CAN node
	CANOpenMsg* outputMsg_;

};

//! Boost shared point of an SDO message
typedef boost::shared_ptr<SDOMsg> SDOMsgPtr;

#endif /* SDOMSG_HPP_ */
