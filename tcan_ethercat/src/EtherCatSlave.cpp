/*
 * EthernetCatDevice.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: Remo Diethelm
 */


#include "tcan_ethercat/EtherCatSlave.hpp"
#include "tcan_ethercat/EtherCatBus.hpp"


namespace tcan_ethercat {


void EtherCatSlave::syncDistributedClocks(const bool activate) {
    bus_->syncDistributedClocks(options_->address_, activate);
}

template <typename Value>
bool EtherCatSlave::sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value) {
    return bus_->sendSdoWrite(options_->address_, index, subindex, completeAccess, value);
}

template <typename Value>
bool EtherCatSlave::sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value) {
    return bus_->sendSdoRead(options_->address_, index, subindex, completeAccess, value);
}

template bool EtherCatSlave::sendSdoWrite<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int8_t value);
template bool EtherCatSlave::sendSdoWrite<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int16_t value);
template bool EtherCatSlave::sendSdoWrite<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int32_t value);
template bool EtherCatSlave::sendSdoWrite<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const int64_t value);
template bool EtherCatSlave::sendSdoWrite<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint8_t value);
template bool EtherCatSlave::sendSdoWrite<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint16_t value);
template bool EtherCatSlave::sendSdoWrite<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint32_t value);
template bool EtherCatSlave::sendSdoWrite<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const uint64_t value);
template bool EtherCatSlave::sendSdoWrite<float>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value);
template bool EtherCatSlave::sendSdoWrite<double>(const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value);

template bool EtherCatSlave::sendSdoRead<int8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value);
template bool EtherCatSlave::sendSdoRead<int16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value);
template bool EtherCatSlave::sendSdoRead<int32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value);
template bool EtherCatSlave::sendSdoRead<int64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value);
template bool EtherCatSlave::sendSdoRead<uint8_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value);
template bool EtherCatSlave::sendSdoRead<uint16_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint16_t& value);
template bool EtherCatSlave::sendSdoRead<uint32_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint32_t& value);
template bool EtherCatSlave::sendSdoRead<uint64_t>(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint64_t& value);
template bool EtherCatSlave::sendSdoRead<float>(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value);
template bool EtherCatSlave::sendSdoRead<double>(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value);


} /* namespace tcan_ethercat */
