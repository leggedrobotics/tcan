#include <string>
#include <unistd.h>
#include <iostream>
#include <functional>

#include "yalc/BusManager.hpp"
#include "yalc/SocketBus.hpp"
#include "yalc_example/DeviceExample.hpp"
#include "m545_utils/containers/MultiKeyContainer.hpp"

class CanManager : public BusManager {
public:
	enum class BusId : unsigned int {
		BUS1=0
	};

	enum class DeviceExampleId : unsigned int {
		EXAMPLE_DEVICE_1=0
	};

	enum class NodeId : unsigned int{
		EXAMPLE_DEVICE_1=0x1
	};

	typedef robotUtils::MultiKeyContainer<Bus*, BusId> BusContainer;
	typedef robotUtils::MultiKeyContainer<example_can::DeviceExample*, DeviceExampleId> DeviceExampleContainer;

	CanManager():
		BusManager(),
		busContainer_(),
		deviceExampleContainer_()
	{
		addSocketBus(BusId::BUS1, "can0");
		buses_.at(static_cast<unsigned int>(BusId::BUS1))->addCanMessage(DeviceCanOpen::RxPDOSyncId, std::bind(&CanManager::parseIncomingSync, this, std::placeholders::_1));

		addDeviceExample(BusId::BUS1, DeviceExampleId::EXAMPLE_DEVICE_1, NodeId::EXAMPLE_DEVICE_1);
	}

	void addDeviceExample(const BusId busId, const DeviceExampleId deviceId, const NodeId nodeId) {
		const unsigned int iBus = static_cast<unsigned int>(busId);
		auto device = new example_can::DeviceExample(static_cast<uint32_t>(nodeId));
		buses_.at(iBus)->addDevice( device );
		deviceExampleContainer_.insert(std::make_tuple("EXAMPLE_DEVICE_1", static_cast<unsigned int>(deviceId), deviceId), device);
	}

	void addSocketBus(const BusId busId, const std::string& interface) {
		SocketBusOptions* options = new SocketBusOptions();
		options->loopback = true;
		options->interface = interface;
		options->baudrate = 1000;

		auto bus = new SocketBus(options);
		addBus( bus );
		busContainer_.insert(std::make_tuple("BUS1", static_cast<unsigned int>(busId), busId), bus);
	}

	bool parseIncomingSync(const CANMsg& cmsg) {
		std::cout << "received SYNC message" << std::endl;
	}

	DeviceExampleContainer& getDeviceExampleContainer() {
		return deviceExampleContainer_;
	}

protected:
	BusContainer busContainer_;
	DeviceExampleContainer deviceExampleContainer_;
};

int main() {
	CanManager canManager_;

	while(true) {
		auto device = canManager_.getDeviceExampleContainer().at(CanManager::DeviceExampleId::EXAMPLE_DEVICE_1);
		std::cout << "Measurement=" << device->getMeasurement() << std::endl;
		device->setCommand(0.3465f);
		canManager_.sendSyncOnAllBuses(true);
		sleep(1);
	}
	return 0;
}
