/*!
 * @file  PDOMsg.hpp
 * @author  Christian Gehring
 * @date  May, 2015
 * @version 1.0
 */

#pragma once

#include <chrono>
#include <stdint.h>

#include "yalc/CANMsg.hpp"

//! PDO message
/*!
 * @ingroup robotCAN
 */


namespace canopen {
constexpr int TxPDO1Id = 0x180;
constexpr int TxPDO2Id = 0x280;
constexpr int TxPDO3Id = 0x380;
constexpr int TxPDO4Id = 0x480;
constexpr int TxSDOId = 0x580;
constexpr int TxNMT = 0x700;

constexpr int RxPDOSyncId = 0x80;
constexpr int RxPDO1Id = 0x200;
constexpr int RxPDO2Id = 0x300;
constexpr int RxPDO3Id = 0x400;
constexpr int RxPDO4Id = 0x500;
constexpr int RxSDOId = 0x600;
}

class PDOMsg: public CANMsg {
public:
    PDOMsg(const uint16_t COBId):
        CANMsg(COBId),
        eventTime_()
    {

    }
    virtual ~PDOMsg() {

    }

    /*! Function that is invoked by upon message reception
     *  Calls processMsg()
     */
    virtual void receiveMsg() {
        eventTime_ = std::chrono::steady_clock::now();
        processMsg();
    }

    /*! Hook function that is invoked by receiveMsg()
     *  Allows to process an incoming message
     */
    virtual void processMsg();

    inline void write(int32_t value, uint8_t pos)
    {
        value_[3 + pos] = static_cast<uint8_t>((value >> 24) & 0xFF);
        value_[2 + pos] = static_cast<uint8_t>((value >> 16) & 0xFF);
        value_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
        value_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);
    }

    inline void write(uint32_t value, uint8_t pos)
    {
        value_[3 + pos] = static_cast<uint8_t>((value >> 24) & 0xFF);
        value_[2 + pos] = static_cast<uint8_t>((value >> 16) & 0xFF);
        value_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
        value_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);
    }

    inline void write(int16_t value, uint8_t pos)
    {
        value_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
        value_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);
    }

    inline void write(uint16_t value, uint8_t pos)
    {
        value_[1 + pos] = static_cast<uint8_t>((value >> 8) & 0xFF);
        value_[0 + pos] = static_cast<uint8_t>((value >> 0) & 0xFF);
    }

    inline void write(int8_t value, uint8_t pos)
    {
        value_[0 + pos] = static_cast<uint8_t>(value);
    }

    inline void write(uint8_t value, uint8_t pos)
    {
        value_[0 + pos] = value;
    }

    inline int32_t readint32(uint8_t pos) const
    {
        return (static_cast<int32_t>(value_[3 + pos]) << 24)
                | (static_cast<int32_t>(value_[2 + pos]) << 16)
                | (static_cast<int32_t>(value_[1 + pos]) << 8)
                | (static_cast<int32_t>(value_[0 + pos]));
    }

    inline uint32_t readuint32(uint8_t pos) const
    {
        return (static_cast<uint32_t>(value_[3 + pos]) << 24)
                | (static_cast<uint32_t>(value_[2 + pos]) << 16)
                | (static_cast<uint32_t>(value_[1 + pos]) << 8)
                | (static_cast<uint32_t>(value_[0 + pos]));
    }

    inline int16_t readint16(uint8_t pos) const
    {
        return (static_cast<int16_t>(value_[1 + pos]) << 8)
                | (static_cast<int16_t>(value_[0 + pos]));
    }

    inline uint16_t readuint16(uint8_t pos) const
    {
        return (static_cast<uint16_t>(value_[1 + pos]) << 8)
                | (static_cast<uint16_t>(value_[0 + pos]));
    }

    inline int8_t readint8(uint8_t pos) const
    {
        return static_cast<int8_t>(value_[0 + pos]);
    }

    inline uint8_t readuint8(uint8_t pos) const
    {
        return value_[0 + pos];
    }

    inline const std::chrono::time_point<std::chrono::steady_clock>& getEventTime() const {
        return eventTime_;
    }

protected:
    //! send or receive time of the message
    std::chrono::time_point<std::chrono::steady_clock> eventTime_;

private:
    PDOMsg(); // declared as private to prevent construction with default constructor

};
