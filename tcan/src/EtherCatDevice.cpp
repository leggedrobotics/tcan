/*
 * EthernetCatDevice.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */


#include "tcan/EtherCatDevice.hpp"
#include "tcan/EtherCatBus.hpp"


namespace tcan {


template <typename Value>
void EtherCatDevice::sendSdoWrite(uint16_t index, uint8_t subindex, bool completeAccess, Value value) {
    bus_->sendSdoWrite(options_->address_, index, subindex, completeAccess, value);
}

template <typename Value>
void EtherCatDevice::sendSdoRead(uint16_t index, uint8_t subindex, bool completeAccess, Value& value) {
    bus_->sendSdoRead(options_->address_, index, subindex, completeAccess, value);
}

void EtherCatDevice::sendSdoReadAndPrint(uint16_t index, uint8_t subindex, bool completeAccess) {
    bus_->sendSdoReadAndPrint(options_->address_, index, subindex, completeAccess);
}


template void EtherCatDevice::sendSdoWrite<int8_t>(uint16_t index, uint8_t subindex, bool completeAccess, int8_t value);
template void EtherCatDevice::sendSdoWrite<int16_t>(uint16_t index, uint8_t subindex, bool completeAccess, int16_t value);
template void EtherCatDevice::sendSdoWrite<int32_t>(uint16_t index, uint8_t subindex, bool completeAccess, int32_t value);
template void EtherCatDevice::sendSdoWrite<int64_t>(uint16_t index, uint8_t subindex, bool completeAccess, int64_t value);
template void EtherCatDevice::sendSdoWrite<uint8_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint8_t value);
template void EtherCatDevice::sendSdoWrite<uint16_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint16_t value);
template void EtherCatDevice::sendSdoWrite<uint32_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint32_t value);
template void EtherCatDevice::sendSdoWrite<uint64_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint64_t value);

template void EtherCatDevice::sendSdoRead<int8_t>(uint16_t index, uint8_t subindex, bool completeAccess, int8_t& value);
template void EtherCatDevice::sendSdoRead<int16_t>(uint16_t index, uint8_t subindex, bool completeAccess, int16_t& value);
template void EtherCatDevice::sendSdoRead<int32_t>(uint16_t index, uint8_t subindex, bool completeAccess, int32_t& value);
template void EtherCatDevice::sendSdoRead<int64_t>(uint16_t index, uint8_t subindex, bool completeAccess, int64_t& value);
template void EtherCatDevice::sendSdoRead<uint8_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint8_t& value);
template void EtherCatDevice::sendSdoRead<uint16_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint16_t& value);
template void EtherCatDevice::sendSdoRead<uint32_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint32_t& value);
template void EtherCatDevice::sendSdoRead<uint64_t>(uint16_t index, uint8_t subindex, bool completeAccess, uint64_t& value);


} /* namespace tcan */
