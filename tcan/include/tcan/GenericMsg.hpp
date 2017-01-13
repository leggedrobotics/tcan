/*
 * GenericMsg.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <algorithm> // copy(..)

namespace tcan {

//! General message container

class GenericMsg {
 public:
	GenericMsg():
        length_(0),
        data_(nullptr)
    {
    }

	/*!
	 * Constructor copying data to internal buffer
	 * @param length    data length
	 * @param data      data to be copied
	 */
	GenericMsg(const unsigned int length, const uint8_t* data):
        length_(length),
        data_(new uint8_t[length_])
    {
        std::copy(&data[0], &data[length_], data_);
    }

	GenericMsg(const std::string msg):
        length_(msg.length()),
        data_(new uint8_t[length_])
    {
        std::copy(&msg.c_str()[0], &msg.c_str()[length_], data_);
    }

	GenericMsg(const GenericMsg& other):
        length_(other.length_),
        data_(new uint8_t[length_])
    {
        std::copy(&other.data_[0], &other.data_[length_], data_);
    }

	GenericMsg(GenericMsg&& other):
	    length_(other.length_),
	    data_(other.data_)
	{
	    other.length_ = 0;
	    other.data_ = nullptr;
	}

    virtual ~GenericMsg() {
        if(data_) {
            delete[] data_;
        }
    }

    inline void operator=(const GenericMsg& other) {
        if(data_) {
            delete[] data_;
        }
        length_ = other.length_;
        data_ = new uint8_t[length_];
        std::copy(&other.data_[0], &other.data_[length_], data_);
    }

    inline void emplaceData(const unsigned int length, uint8_t* data) {
        length_ = length;
        data_ = data;
        data = nullptr;
    }

    inline unsigned int getLength() const { return length_; }
    inline const uint8_t* getData() const { return data_; }

 private:
    unsigned int length_;
    uint8_t* data_;

};

} /* namespace tcan */
