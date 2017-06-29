/*
 * conversions.hpp
 *
 *  Created on: Mai 29, 2017
 *      Author: Remo Diethelm
 */

#pragma once

#include <cstdint>
#include <cstring>


namespace tcan_ethercat {

static int32_t convertFloatToInt32(const float value) {
  int32_t valueInt = 0;
  memcpy(&valueInt, &value, sizeof(float));
  return valueInt;
}

static float convertInt32ToFloat(const int32_t value) {
  float valueFloat = 0.0;
  memcpy(&valueFloat, &value, sizeof(float));
  return valueFloat;
}

static uint32_t convertFloatToUInt32(const float value) {
  uint32_t valueInt = 0;
  memcpy(&valueInt, &value, sizeof(float));
  return valueInt;
}

static float convertUInt32ToFloat(const uint32_t value) {
  float valueFloat = 0.0;
  memcpy(&valueFloat, &value, sizeof(float));
  return valueFloat;
}

static int64_t convertDoubleToInt64(const double value) {
  int64_t valueInt = 0;
  memcpy(&valueInt, &value, sizeof(double));
  return valueInt;
}

static double convertInt64ToDouble(const int64_t value) {
  double valueDouble = 0.0;
  memcpy(&valueDouble, &value, sizeof(double));
  return valueDouble;
}

static uint64_t convertDoubleToUInt64(const double value) {
  uint64_t valueInt = 0;
  memcpy(&valueInt, &value, sizeof(double));
  return valueInt;
}

static double convertUInt64ToDouble(const uint64_t value) {
  double valueDouble = 0.0;
  memcpy(&valueDouble, &value, sizeof(double));
  return valueDouble;
}

} /* namespace tcan_ethercat */

