/*
 * canopen_pdos.hpp
 *
 *  Created on: Apr 16, 2015
 *      Author: gech
 */

#pragma once

#include <chrono>

#include "CANOpenMsg.hpp"

namespace canopen {

//////////////////////////////////////////////////////////////////////////////
class RxPDOSync: public CANOpenMsg {
public:
  RxPDOSync(int SMId):CANOpenMsg(0x80, SMId) {
    flag_ = 1;
  };
  virtual ~RxPDOSync() {};
};

//////////////////////////////////////////////////////////////////////////////
class TxPDONMT: public CANOpenMsg {
public:
  TxPDONMT(int nodeId):CANOpenMsg(TxNMT+nodeId, 0),
  state_(-1),
  timeReceived_()
  {
    //0x00 - Bootup; 0x04 - Stopped; 0x05 - Operational; 0x7F - Pre-Operational.
  };

  virtual ~TxPDONMT()
  {
  };

  virtual void processMsg()
  {
    timeReceived_ = std::chrono::steady_clock::now();
    state_ = (uint8_t)(value_[0]);
  };

  bool isBootup() const
  {
    return (state_ == 0x00);
  };

  bool isStopped() const
  {
    return (state_ == 0x04);
  };

  bool isOperational() const
  {
    return (state_ == 0x05);
  };

  bool isPreOperational() const
  {
    return (state_ ==  0x7F);
  };

  uint8_t getState() const {
	  return state_;
  }

  const std::chrono::time_point<std::chrono::steady_clock>& getTime() const {
    return timeReceived_;
  }
private:
  uint8_t state_;
  std::chrono::time_point<std::chrono::steady_clock> timeReceived_;

};
} // namespace canopen
