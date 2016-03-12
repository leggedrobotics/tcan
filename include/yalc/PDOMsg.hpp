/*!
 * @file  PDOMsg.hpp
 * @author  Christian Gehring
 * @date  May, 2015
 * @version 1.0
 */

#pragma once

#include "libcanplusplus/CANOpenMsg.hpp"
#include <stdint.h>

//! PDO message
/*!
 * @ingroup robotCAN
 */
class PDOMsg: public CANOpenMsg {
public:
  PDOMsg(int COBId, int SMId): CANOpenMsg(COBId, SMId) {

  }
  virtual ~PDOMsg() {

  }

  inline void write(int32_t value, uint8_t pos)
  {
    value_[3 + pos] = (uint8_t)((value >> 24) & 0xFF);
    value_[2 + pos] = (uint8_t)((value >> 16) & 0xFF);
    value_[1 + pos] = (uint8_t)((value >> 8) & 0xFF);
    value_[0 + pos] = (uint8_t)((value >> 0) & 0xFF);
  }

  inline void write(uint32_t value, uint8_t pos)
  {
    value_[3 + pos] = (uint8_t)((value >> 24) & 0xFF);
    value_[2 + pos] = (uint8_t)((value >> 16) & 0xFF);
    value_[1 + pos] = (uint8_t)((value >> 8) & 0xFF);
    value_[0 + pos] = (uint8_t)((value >> 0) & 0xFF);
  }

  inline void write(int16_t value, uint8_t pos)
  {
    value_[1 + pos] = (uint8_t)((value >> 8) & 0xFF);
    value_[0 + pos] = (uint8_t)((value >> 0) & 0xFF);
  }

  inline void write(uint16_t value, uint8_t pos)
  {
    value_[1 + pos] = (uint8_t)((value >> 8) & 0xFF);
    value_[0 + pos] = (uint8_t)((value >> 0) & 0xFF);
  }

  inline void write(int8_t value, uint8_t pos)
  {
    value_[0 + pos] = (uint8_t)((value >> 0) & 0xFF);
  }

  inline void write(uint8_t value, uint8_t pos)
  {
    value_[0 + pos] = (uint8_t)((value >> 0) & 0xFF);
  }

  inline int32_t readint32(uint8_t pos) const
  {
    int32_t value;
    value  = ((int32_t)value_[3 + pos] << 24);
    value |= ((int32_t)value_[2 + pos] << 16);
    value |= ((int32_t)value_[1 + pos] << 8);
    value |= ((int32_t)value_[0 + pos]);
    return value;
  }

  inline uint32_t readuint32(uint8_t pos) const
  {
    uint32_t value;
    value  = ((uint32_t)value_[3 + pos] << 24);
    value |= ((uint32_t)value_[2 + pos] << 16);
    value |= ((uint32_t)value_[1 + pos] << 8);
    value |= ((uint32_t)value_[0 + pos]);
    return value;
  }

  inline int16_t readint16(uint8_t pos) const
  {
    int16_t value;
    value  = ((int16_t)value_[1 + pos] << 8);
    value |= ((int16_t)value_[0 + pos]);
    return value;
  }

  inline uint16_t readuint16(uint8_t pos) const
  {
    uint16_t value;
    value  = ((uint16_t)value_[1 + pos] << 8);
    value |= ((uint16_t)value_[0 + pos]);
    return value;
  }

  inline int8_t readint8(uint8_t pos) const
  {
    int8_t value;
    value  = (int8_t)value_[0 + pos];
    return value;
  }

  inline uint8_t readuint8(uint8_t pos) const
  {
    uint8_t value;
    value  = (uint8_t)value_[0 + pos];
    return value;
  }

};
