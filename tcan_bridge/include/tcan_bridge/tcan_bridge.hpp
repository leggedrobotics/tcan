#pragma once

#include <string>

#include <tcan_can/CanBusManager.hpp>

namespace tcan_bridge
{

class TcanBridge
{
public:
 TcanBridge() = delete;
 TcanBridge(std::string interfaceName);
 virtual ~TcanBridge() = default;

protected:
 tcan_can::CanBusManager canManager_;
};

}  // namespace tcan_bridge
