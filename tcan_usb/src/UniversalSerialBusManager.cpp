#include "tcan_usb/UniversalSerialBusManager.hpp"

namespace tcan_usb {

UniversalSerialBusManager::UniversalSerialBusManager():
    tcan::BusManager<UsbMsg>()
{
}

UniversalSerialBusManager::~UniversalSerialBusManager()
{
}

} /* namespace tcan_usb */
