/*!
 * @file 	CANOpenMsg.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#ifndef CANMsg_HPP_
#define CANMsg_HPP_

#include <algorithm>
#include <stdint.h>

//! General CANOpen message container
/*! The message can be either a sent or received message.
 * It can be either a SDO or PDO message.
 * This class converts a stack of data to a CAN message and vice versa.
 *
 * To send a message, the following information is needed:
 * 	COBId: the Communication Object Identifier
 * 	flag: if true, the CAN message will be sent
 * 	value[8]: a stack of max. 8 values, each element of the array represents maximal a 32bit value
 *  length[8]: the length of each value in the stack in bytes (0-4)
 *
 * As an example, a PDO with a velocity command of 4bytes and a operation mode of 1byte
 * needs to be sent to the node with ID 1:
 * 	COBId=0x200+1
 * 	flag=true
 * 	value[0] = velocity
 * 	value[1] = operation_mode
 * 	value[2-7] = 0
 * 	length[0] = 4
 * 	length[1] = 1
 * 	length[2-7] = 0
 *
 * The stack of values is converted into a stream of unsigned chars by the function
 * getCANMsg().
 *
 * @ingroup robotCAN
 */

class CANMsg {
public:
	/*! Constructor
     * @param	COBId	Communication Object Identifier
     */
    CANMsg(const uint16_t COBId):
        COBId_(COBId),
        flag_(false),
        length_{0},
        value_{0, 0, 0, 0, 0, 0, 0, 0}
    {

    }

    CANMsg(const uint16_t COBId, const uint8_t length, const uint8_t* value):
    	COBId_(COBId),
		flag_(false),
		length_(length),
		value_{0, 0, 0, 0, 0, 0, 0, 0}
    {
    	std::copy(&value[0], &value[8], value_);
    }

	//! Destructor
    virtual ~CANMsg() { }

    /*! Gets the Communication Object Identifier
     *
     * @return COBId
     */
    uint16_t getCOBId() const { return COBId_; }

    /*! Gets flag whether the message will be sent or the message is received
     * @return
     */
    bool getFlag() const { return flag_; }

    /*! Gets the stack of values
     *
     * @return reference to value_[8]
     */
    const uint8_t* getValue() const { return value_; }

    /*! Gets the lengths of the values in the stack
     * @return reference to length
     */
    uint8_t getLength() { return length_; }

    /*! Sets the flag if the message needs to be sent
     * @param flag	if true message is sent
     */
    void setFlag(const bool flag) { flag_ = flag; }

    /*! Sets the stack of values
     * @param value	 array of length 8
     */
    void setValue(const uint8_t* value) {
    	std::copy(&value[0], &value[8], value_);
    }

    /*! Length of the values in the stack
     * @param length array of length 8
     */
    void setLength(const uint8_t length) { length_ = length; }

    /*! Sets the Communication Object Identifier
     * @param COBId	Communication Object Identifier
     */
    void setCOBId(const uint8_t COBId) { COBId_ = COBId; }


protected:
    //! Communication Object Identifier
    uint16_t COBId_;

    //! if true, the message will be sent or the message is received
    bool flag_;

    //! the message data length
    uint8_t length_;

    /*! Data of the CAN message
     */
    uint8_t value_[8];

private:
    CANMsg(); // declared as private to prevent construction with default constructor
};

#endif /* CANMsg_HPP_ */
