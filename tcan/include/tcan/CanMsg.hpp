/*
 * CanMsg.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <algorithm>
#include <stdint.h>
#include <initializer_list>

namespace tcan {

//! General CANOpen message container

class CanMsg {
public:
	/*! Constructor
	 * @param	COBId	Communication Object Identifier
	 */
	CanMsg() = delete;

	CanMsg(const uint32_t CobId):
		CobId_(CobId),
		length_{0},
		data_{0, 0, 0, 0, 0, 0, 0, 0}
	{

	}

	CanMsg(const uint32_t CobId, const uint8_t length, const uint8_t* data):
		CobId_(CobId),
		length_(length),
		data_{0, 0, 0, 0, 0, 0, 0, 0}
	{
		std::copy(&data[0], &data[length], data_);
	}

	CanMsg(const uint32_t CobId, const uint8_t length, const std::initializer_list<uint8_t> data):
		CobId_(CobId),
		length_(length),
		data_{0, 0, 0, 0, 0, 0, 0, 0}
	{
		std::copy(data.begin(), data.end(), data_);
	}

	//! Destructor
	virtual ~CanMsg() { }

	/*! Gets the Communication Object Identifier
	 *
	 * @return COBId
	 */
	inline uint32_t getCobId() const { return CobId_; }


	/*! Gets the stack of values
	 *
	 * @return reference to data_[8]
	 */
	inline const uint8_t* getData() const { return data_; }

	/*! Gets the lengths of the values in the stack
	 * @return reference to length
	 */
	inline uint8_t getLength() const { return length_; }


	/*! Sets the stack of values
	 * @param value	 array of length 8
	 */
	inline void setData(const uint8_t length, const uint8_t* data) {
		length_ = length;
		std::copy(&data[0], &data[length], data_);
	}

	inline void write(const int32_t value, const uint8_t pos)
	{
		data_[3 + pos] = static_cast<uint8_t>((value >> 24) & 0xFF);
		data_[2 + pos] = static_cast<uint8_t>((value >> 16) & 0xFF);
		data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
		data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

		length_ = pos+4;
	}

	inline void write(const uint32_t value, const uint8_t pos)
	{
		data_[3 + pos] = static_cast<uint8_t>((value >> 24) & 0xFF);
		data_[2 + pos] = static_cast<uint8_t>((value >> 16) & 0xFF);
		data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
		data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

		length_ = pos+4;
	}

	inline void write(const int16_t value, const uint8_t pos)
	{
		data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
		data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

		length_ = pos+2;
	}

	inline void write(const uint16_t value, const uint8_t pos)
	{
		data_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
		data_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);

		length_ = pos+2;
	}

	inline void write(const int8_t value, const uint8_t pos)
	{
		data_[0 + pos] = static_cast<uint8_t>(value);

		length_ = pos+1;
	}

	inline void write(const uint8_t value, const uint8_t pos)
	{
		data_[0 + pos] = value;

		length_ = pos+1;
	}

	inline int32_t readint32(uint8_t pos) const
	{
		return (static_cast<int32_t>(data_[3 + pos]) << 24)
				| (static_cast<int32_t>(data_[2 + pos]) << 16)
				| (static_cast<int32_t>(data_[1 + pos]) << 8)
				| (static_cast<int32_t>(data_[0 + pos]));
	}

	inline uint32_t readuint32(uint8_t pos) const
	{
		return (static_cast<uint32_t>(data_[3 + pos]) << 24)
				| (static_cast<uint32_t>(data_[2 + pos]) << 16)
				| (static_cast<uint32_t>(data_[1 + pos]) << 8)
				| (static_cast<uint32_t>(data_[0 + pos]));
	}

	inline int16_t readint16(uint8_t pos) const
	{
		return (static_cast<int16_t>(data_[1 + pos]) << 8)
				| (static_cast<int16_t>(data_[0 + pos]));
	}

	inline uint16_t readuint16(uint8_t pos) const
	{
		return (static_cast<uint16_t>(data_[1 + pos]) << 8)
				| (static_cast<uint16_t>(data_[0 + pos]));
	}

	inline int8_t readint8(uint8_t pos) const
	{
		return static_cast<int8_t>(data_[0 + pos]);
	}

	inline uint8_t readuint8(uint8_t pos) const
	{
		return data_[0 + pos];
	}

private:
	//! Communication Object Identifier
	uint32_t CobId_;

	//! the message data length
	uint8_t length_;

	/*! Data of the CAN message
	 */
	uint8_t data_[8];
};

} /* namespace tcan */
