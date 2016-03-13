/*
 * canopen_pdos.hpp
 *
 *  Created on: Apr 16, 2015
 *      Author: gech
 */

#pragma once

#include "PDOMsg.hpp"

namespace canopen {

//////////////////////////////////////////////////////////////////////////////
class RxPDOSync: public PDOMsg {
public:
    RxPDOSync():PDOMsg(RxPDOSyncId) {
        flag_ = 1;
    }

    virtual ~RxPDOSync() {}
};

//////////////////////////////////////////////////////////////////////////////
class TxPDONMT: public PDOMsg {
public:
    TxPDONMT(int nodeId):
        PDOMsg(TxNMT+nodeId),
        state_(-1)
    {

    }

    virtual ~TxPDONMT()
    {
    }

    virtual void processMsg()
    {
        state_ = (uint8_t)(value_[0]);
    }

    bool isBootup() const
    {
        return (state_ == 0x00);
    }

    bool isStopped() const
    {
        return (state_ == 0x04);
    }

    bool isOperational() const
    {
        return (state_ == 0x05);
    }

    bool isPreOperational() const
    {
        return (state_ ==  0x7F);
    }

    uint8_t getState() const {
        return state_;
    }
private:
    //0x00 - Bootup; 0x04 - Stopped; 0x05 - Operational; 0x7F - Pre-Operational.
    uint8_t state_;

};

} // namespace canopen
