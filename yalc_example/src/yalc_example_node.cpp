#include <string>
#include <unistd.h>
#include <iostream>
#include <functional>
#include <chrono>
#include <signal.h>

#include "yalc/BusManager.hpp"
#include "yalc/SocketBus.hpp"
#include "yalc_example/DeviceExample.hpp"
#include "m545_utils/containers/MultiKeyContainer.hpp"

namespace yalc {
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

		for(unsigned int i; i<30; i++) {
			addDeviceExample(BusId::BUS1, static_cast<DeviceExampleId>(i), static_cast<NodeId>(i+1));
		}
	}

	void addDeviceExample(const BusId busId, const DeviceExampleId deviceId, const NodeId nodeId) {
		const unsigned int iBus = static_cast<unsigned int>(busId);
		auto device = new example_can::DeviceExample(static_cast<uint32_t>(nodeId));
		buses_.at(iBus)->addDevice( device );
		deviceExampleContainer_.insert(std::make_tuple("EXAMPLE_DEVICE" + std::to_string(static_cast<unsigned int>(deviceId)), static_cast<unsigned int>(deviceId), deviceId), device);
	}

	void addSocketBus(const BusId busId, const std::string& interface) {
		SocketBusOptions* options = new SocketBusOptions();
		options->interface = interface;

		auto bus = new SocketBus(options);
		addBus( bus );
		busContainer_.insert(std::make_tuple("BUS1", static_cast<unsigned int>(busId), busId), bus);
	}

	bool parseIncomingSync(const CANMsg& cmsg) {
		std::cout << "received SYNC message" << std::endl;

		return true;
	}

	DeviceExampleContainer& getDeviceExampleContainer() {
		return deviceExampleContainer_;
	}

protected:
	BusContainer busContainer_;
	DeviceExampleContainer deviceExampleContainer_;
};

} /* namespace yalc */

bool g_running = true;

void signal_handler(int) {
	g_running = false;
}

int main() {
	signal(SIGINT, signal_handler);
	yalc::CanManager canManager_;

	auto nextStep = std::chrono::steady_clock::now();

	while(g_running) {
		for(auto device : canManager_.getDeviceExampleContainer()) {
//			std::cout << "Measurement=" << device->getMeasurement() << std::endl;
			device->setCommand(12345.f);
		}
		canManager_.sendSyncOnAllBuses(true);

//		std::cout << "sleeping..\n\n";
//		sleep(5);
		nextStep += std::chrono::microseconds(10000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
