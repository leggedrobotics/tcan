/*!
 * @file 	SDOMsg.cpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */
#include "yalc/SDOMsg.hpp"

SDOMsg::SDOMsg(const uint16_t nodeId, const uint16_t index, const uint8_t subIndex, const Command command, const uint32_t data):
    PDOMsg(nodeId),
    isSent_(false),
    isReceived_(false)
{
    value_[0] = static_cast<uint8_t>(command);
    value_[1] = (index & 0xff);
    value_[2] = ((index >> 8) & 0xff);
    value_[3] = subIndex;
    value_[4] = ((data >> 0) & 0xff);
    value_[5] = ((data >> 8) & 0xff);
    value_[6] = ((data >> 16) & 0xff);
    value_[7] = ((data >> 24) & 0xff);
}

SDOMsg::~SDOMsg()
{

}

bool SDOMsg::hasTimedOut()
{
    // timeout after 1sec
    if(std::chrono::duration_cast<std::chrono::microseconds>(getEventTime() - std::chrono::steady_clock::now()).count() > 1000000) {
        return true;
    }
	return false;
}

void SDOMsg::receiveMsg()
{
    isReceived_ = true;
    processMsg();
}
