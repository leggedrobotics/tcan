/*
 * EthernetCatDevice.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */


#include "tcan/EtherCatDevice.hpp"
#include "tcan/EtherCatBus.hpp"


namespace tcan {


void EtherCatDevice::syncDistributedClocks(const bool activate) {
    bus_->syncDistributedClocks(options_->address_, activate);
}

template <typename Value>
void EtherCatDevice::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
    bus_->sendSdoWrite(options_->address_, index, subindex, completeAccess, value);
}

template <typename Value>
void EtherCatDevice::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
    bus_->sendSdoRead(options_->address_, index, subindex, completeAccess, value);
}

void EtherCatDevice::sendSdoReadAndPrint(const uint16_t index, const uint8_t subindex, const bool completeAccess) {
    bus_->sendSdoReadAndPrint(options_->address_, index, subindex, completeAccess);
}


template void EtherCatDevice::sendSdoWrite<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value);
template void EtherCatDevice::sendSdoWrite<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value);
template void EtherCatDevice::sendSdoWrite<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value);
template void EtherCatDevice::sendSdoWrite<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value);
template void EtherCatDevice::sendSdoWrite<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value);
template void EtherCatDevice::sendSdoWrite<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value);
template void EtherCatDevice::sendSdoWrite<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value);
template void EtherCatDevice::sendSdoWrite<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value);

template void EtherCatDevice::sendSdoRead<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value);
template void EtherCatDevice::sendSdoRead<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value);
template void EtherCatDevice::sendSdoRead<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value);
template void EtherCatDevice::sendSdoRead<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value);
template void EtherCatDevice::sendSdoRead<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value);
template void EtherCatDevice::sendSdoRead<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value);
template void EtherCatDevice::sendSdoRead<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value);
template void EtherCatDevice::sendSdoRead<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value);


} /* namespace tcan */
