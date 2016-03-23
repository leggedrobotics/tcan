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

    inline const std::chrono::time_point<std::chrono::steady_clock>& getEventTime() const {
        return eventTime_;
    }

protected:
    //! send or receive time of the message
    std::chrono::time_point<std::chrono::steady_clock> eventTime_;

private:
    PDOMsg(); // declared as private to prevent construction with default constructor

};
