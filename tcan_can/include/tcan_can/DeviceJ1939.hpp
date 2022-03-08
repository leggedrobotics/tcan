#pragma once

#include "tcan_can/CanBus.hpp"
#include "tcan_can/CanDevice.hpp"
#include "tcan_can/CanFrameIdentifier.hpp"
#include "tcan_can/J1939PgnParser.hpp"

namespace tcan_can {

class DeviceJ1939 : public CanDevice {
   public:
    static const uint32_t CAN_ID_PGN_MASK = 0x3ffff00u;
    static const uint32_t extendedFlag = 1UL << 31;

    using CanDevice::CanDevice;
    ~DeviceJ1939() override = default;
    bool initDevice() override;
    bool configureDevice(const CanMsg& /*msg*/) { return true; }

  protected:
   void addParser(J1939PgnParser& parser) { pgnMap_.emplace(parser.pgn_, &parser); }

  private:
   bool parseMessage(const CanMsg& msg);
   std::map<uint32_t, J1939PgnParser*> pgnMap_;
};

}  // namespace tcan_can