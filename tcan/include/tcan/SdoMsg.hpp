/*
 * SdoMsg.hpp
 *
 *  Created on: Mar 27, 2016
 *      Author: Philipp Leemann
 */

#pragma once

#include <stdint.h>
#include <tcan/CanMsg.hpp>
#include <string>


namespace tcan {

//! Service Data Object Message Container
class SdoMsg : public CanMsg {
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
    SdoMsg():
        CanMsg(0),
        requiresAnswer_(true)
    {

    }

    SdoMsg(const uint32_t nodeId, const Command command, const uint16_t index, const uint8_t subIndex, const uint32_t data):
        CanMsg(0x600 + nodeId, 8, {
                static_cast<uint8_t>(command),
                static_cast<uint8_t>(index & 0xff),
                static_cast<uint8_t>((index >> 8) & 0xff),
                subIndex,
                static_cast<uint8_t>((data >> 0) & 0xff),
                static_cast<uint8_t>((data >> 8) & 0xff),
                static_cast<uint8_t>((data >> 16) & 0xff),
                static_cast<uint8_t>((data >> 24) & 0xff)
    }),
    requiresAnswer_(true)
    {

    }

    // special constructor for NMT messages
    SdoMsg(const uint8_t nodeId, const uint8_t nmtState):
        CanMsg(0x0, 2, {nmtState, nodeId}),
        requiresAnswer_(false)
    {

    }

    //! Destructor
    virtual ~SdoMsg()
    {

    }

    //! getters for index and subindex for answer verfication
    inline uint8_t getCommandByte() const { return readuint8(0); }
    inline uint16_t getIndex() const { return readuint16(1); }
    inline uint8_t getSubIndex() const { return readuint8(3); }

    inline bool getRequiresAnswer() const { return requiresAnswer_; }

    static std::string getErrorName(const int32_t error) {
        std::string name;
        switch (error) {
            case 0x00000000:
                name = std::string{"No Communication Error"};
                break;
            case 0x05030000:
                name = std::string{"Toggle Error"};
                break;
            case 0x05040000:
                name = std::string{"SDO Time Out"};
                break;
            case 0x05040001:
                name = std::string{"Client / Server Specifier Error"};
                break;
            case 0x05040005:
                name = std::string{"Out of Memory Error"};
                break;
            case 0x06010000:
                name = std::string{"Access Error"};
                break;
            case 0x06010001:
                name = std::string{"Write Only"};
                break;
            case 0x06010002:
                name = std::string{"Read Only"};
                break;
            case 0x06020000:
                name = std::string{"Object does not exist Error"};
                break;
            case 0x06040041:
                name = std::string{"PDO mapping Error"};
                break;
            case 0x06040042:
                name = std::string{"PDO Length Error"};
                break;
            case 0x06040043:
                name = std::string{"General Parameter Error"};
                break;
            case 0x06040047:
                name = std::string{"General internal Incompatibility Error"};
                break;
            case 0x06060000:
                name = std::string{"Hardware Error"};
                break;
            case 0x06070010:
                name = std::string{"Service Parameter Error"};
                break;
            case 0x06070012:
                name = std::string{"Service Parameter too long Error"};
                break;
            case 0x06070013:
                name = std::string{"Service Parameter too short Error"};
                break;
            case 0x06090011:
                name = std::string{"Object Subindex Error"};
                break;
            case 0x06090030:
                name = std::string{"Value Range Error"};
                break;
            case 0x06090031:
                name = std::string{"Value too high Error"};
                break;
            case 0x06090032:
                name = std::string{"Value too low Error"};
                break;
            case 0x06090036:
                name = std::string{"Maximum less Minimum Error"};
                break;
            case 0x08000000:
                name = std::string{"General Error"};
                break;
            case 0x08000020:
                name = std::string{"Transfer or store Error"};
                break;
            case 0x08000021:
                name = std::string{"Local Control Error"};
                break;
            case 0x08000022:
                name = std::string{"Wrong Device State"};
                break;
            default:
                name = std::string{""};
                break;
        }
        return name;
    }

 private:
    //! if true, message will stay in the SDO queue until answer was received or timed out.
    bool requiresAnswer_;
};

} /* namespace tcan */
