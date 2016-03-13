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

#include "yalc/PDOMsg.hpp"


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
class SDOMsg : public PDOMsg {
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
    SDOMsg(const uint16_t nodeId, const uint16_t index, const uint8_t subIndex, const Command command, const uint32_t data);

	//! Destructor
    virtual ~SDOMsg();

    virtual void receiveMsg();

    /*! Gets timeout status
	 * @return true if input message was not received in a certain time
	 */
    bool hasTimedOut();

    /*! Gets sent status
	 * @return true if output message was sent to the node
	 */
    bool getIsSent() const { return isSent_; }

    /*! Gets receive status
	 * @return	true if input message was received from node
	 */
    bool getIsReceived() const { return isReceived_; }

    //! getters for index and subindex for message verfication
    uint16_t getIndex() const { return readuint16(1); }
    uint8_t getSubIndex() const { return readuint8(3); }
\
protected:
    //! if true, SDO request was sent through CAN network
	bool isSent_;

	//! if true, response from node is received
	bool isReceived_;

private:
    SDOMsg(); // declared as private to prevent construction with default constructor
};

#endif /* SDOMSG_HPP_ */
