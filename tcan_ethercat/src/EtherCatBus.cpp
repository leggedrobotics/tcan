/*
 * EtherCatBus.cpp
 *
 *  Created on: Mai 29, 2017
 *      Author: Remo Diethelm
 */

#include "tcan_ethercat/conversions.hpp"
#include "tcan_ethercat/EtherCatBus.hpp"

namespace tcan_ethercat {

template <>
bool EtherCatBus::sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const float value) {
    return sendSdoWrite(slave, index, subindex, completeAccess, convertFloatToInt32(value));
}

template <>
bool EtherCatBus::sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const double value) {
    return sendSdoWrite(slave, index, subindex, completeAccess, convertDoubleToInt64(value));
}

template <>
bool EtherCatBus::sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value) {
    int32_t valueInt = 0;
    if (!sendSdoRead(slave, index, subindex, completeAccess, valueInt))
      return false;
    value = convertInt32ToFloat(valueInt);
    return true;
}

template <>
bool EtherCatBus::sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value) {
    int64_t valueInt = 0;
    if (!sendSdoRead(slave, index, subindex, completeAccess, valueInt))
      return false;
    value = convertInt64ToDouble(valueInt);
    return true;
}

} /* namespace tcan_ethercat */

