#include "tcan_ip/IpBusManager.hpp"

namespace tcan_ip {

IpBusManager::IpBusManager():
    tcan::BusManager<IpMsg>()
{
}

IpBusManager::~IpBusManager()
{
}

} /* namespace tcan_ip */
