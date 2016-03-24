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

#include <stdint.h>

#include "yalc/CANMsg.hpp"


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
class SDOMsg : public CANMsg {
public:

	enum class Command : uint8_t {
		READ=0x40,
		WRITE_1_BYTE=0x2f,
		WRITE_2_BYTE=0x2b,
		WRITE_4_BYTE=0x23
	};

	/*! Constructor
	 *
	 * @param nodeId	ID of the CAN node
	 * @param index     index to be read/write
	 * @param subIndex  subindex to be read/write
	 * @param command   SDO command (read or write)
	 */
	SDOMsg() = delete;
	SDOMsg(const uint32_t nodeId, const uint16_t index, const uint8_t subIndex, const Command command, const uint32_t data):
		CANMsg(nodeId, 8, {
				static_cast<uint8_t>(command),
				static_cast<uint8_t>(index & 0xff),
				static_cast<uint8_t>((index >> 8) & 0xff),
				subIndex,
				static_cast<uint8_t>((data >> 0) & 0xff),
				static_cast<uint8_t>((data >> 8) & 0xff),
				static_cast<uint8_t>((data >> 16) & 0xff),
				static_cast<uint8_t>((data >> 24) & 0xff)
		}),
		isSent_(false)
	{

	}

	//! Destructor
	virtual ~SDOMsg()
	{

	}

	/*! Gets sent status
	 * @return true if output message was sent to the node
	 */
	bool getIsSent() const { return isSent_; }
	void setIsSent(const bool sent) { isSent_ = sent; }


	//! getters for index and subindex for answer verfication
	inline uint16_t getIndex() const { return readuint16(1); }
	inline uint8_t getSubIndex() const { return readuint8(3); }

private:
	//! if true, SDO request was sent through CAN network
	bool isSent_;
};

#endif /* SDOMSG_HPP_ */
