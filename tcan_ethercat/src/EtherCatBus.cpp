/*
 * EtherCatBus.cpp
 *
 *  Created on: Mai 29, 2017
 *      Author: Remo Diethelm
 */

#include "tcan_ethercat/EtherCatBus.hpp"

namespace tcan_ethercat {

template <>
void EtherCatBus::sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value) {
    sendSdoWrite(slave, index, subindex, completeAccess, convertFloatToInt32(value));
}

template <>
void EtherCatBus::sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value) {
    sendSdoWrite(slave, index, subindex, completeAccess, convertDoubleToInt64(value));
}

template <>
void EtherCatBus::sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value) {
    int32_t valueInt = 0;
    sendSdoRead(slave, index, subindex, completeAccess, valueInt);
    value = convertInt32ToFloat(valueInt);
}

template <>
void EtherCatBus::sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value) {
    int64_t valueInt = 0;
    sendSdoRead(slave, index, subindex, completeAccess, valueInt);
    value = convertInt64ToDouble(valueInt);
}

} /* namespace tcan_ethercat */

