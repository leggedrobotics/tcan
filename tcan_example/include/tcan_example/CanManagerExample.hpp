#pragma once

#include <unordered_map>

#include "tcan_can/CanBusManager.hpp"
#include "tcan_can/CanBus.hpp"
#include "tcan_can/SocketBus.hpp"

#include "tcan_example/CanDeviceExample.hpp"

namespace tcan_example {

class CanManagerExample : public tcan_can::CanBusManager {
public:
    enum class BusId : unsigned int {
        BUS1=0,
        BUS2=1,
        BUS3=2
    };

    enum class DeviceExampleId : unsigned int {
        EXAMPLE_DEVICE_1=0
    };

    enum class NodeId : unsigned int{
        EXAMPLE_DEVICE_1=0x1

    };

    typedef std::unordered_map<unsigned int, tcan_can::CanBus*> BusContainer;
    typedef std::unordered_map<unsigned int, CanDeviceExample*> DeviceExampleContainer;

    CanManagerExample() = default;
    ~CanManagerExample() override;

    void init();

    void addDeviceExample(const BusId busId, const DeviceExampleId deviceId, const NodeId nodeId);

    void addSocketBus(const BusId busId, std::unique_ptr<tcan_can::SocketBusOptions>&& options);

    bool parseIncomingSyncBus1(const tcan_can::CanMsg& cmsg);
    bool parseIncomingSyncBus2(const tcan_can::CanMsg& cmsg);
    bool parseIncomingSyncBus3(const tcan_can::CanMsg& cmsg);

    DeviceExampleContainer& getDeviceExampleContainer() {
        return deviceExampleContainer_;
    }

protected:
    BusContainer busContainer_;
    DeviceExampleContainer deviceExampleContainer_;
};

} /* namespace tcan_example */