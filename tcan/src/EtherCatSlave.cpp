/*
 * EthernetCatDevice.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */


#include "../include/tcan/EtherCatSlave.hpp"

#include "tcan/EtherCatBus.hpp"


namespace tcan {


void EtherCatSlave::syncDistributedClocks(const bool activate) {
    bus_->syncDistributedClocks(options_->address_, activate);
}

template <typename Value>
void EtherCatSlave::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
    bus_->sendSdoWrite(options_->address_, index, subindex, completeAccess, value);
}

template <typename Value>
void EtherCatSlave::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
    bus_->sendSdoRead(options_->address_, index, subindex, completeAccess, value);
}

void EtherCatSlave::sendSdoReadAndPrint(const uint16_t index, const uint8_t subindex, const bool completeAccess) {
    bus_->sendSdoReadAndPrint(options_->address_, index, subindex, completeAccess);
}


template void EtherCatSlave::sendSdoWrite<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value);
template void EtherCatSlave::sendSdoWrite<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value);
template void EtherCatSlave::sendSdoWrite<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value);
template void EtherCatSlave::sendSdoWrite<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value);
template void EtherCatSlave::sendSdoWrite<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value);
template void EtherCatSlave::sendSdoWrite<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value);
template void EtherCatSlave::sendSdoWrite<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value);
template void EtherCatSlave::sendSdoWrite<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value);

template void EtherCatSlave::sendSdoRead<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value);
template void EtherCatSlave::sendSdoRead<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value);
template void EtherCatSlave::sendSdoRead<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value);
template void EtherCatSlave::sendSdoRead<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value);
template void EtherCatSlave::sendSdoRead<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value);
template void EtherCatSlave::sendSdoRead<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value);
template void EtherCatSlave::sendSdoRead<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value);
template void EtherCatSlave::sendSdoRead<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value);


} /* namespace tcan */
