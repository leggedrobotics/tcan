#pragma once

// STL
#include <stdint.h>

namespace tcan {

template <typename Value>
void readValue(const uint8_t* data, const uint16_t pos, Value& value) {
  value = 0;
  const uint16_t len = sizeof(Value);
  for (uint16_t i = 0; i < len; i++) {
    value |= (static_cast<Value>(data[pos+i]) << i*8);
  }
}

template <typename Value>
Value readValue(const uint8_t* data, const uint16_t pos) {
  Value value = 0;
  readValue(data, pos, value);
  return value;
}

}
