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
		BUS1=0,
		BUS2=1
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

		for(unsigned int i=0; i<30; i++) {
			addDeviceExample(BusId::BUS1, static_cast<DeviceExampleId>(i), static_cast<NodeId>(i+1));
		}

		addSocketBus(BusId::BUS2, "can1");

		for(unsigned int i=30; i<40; i++) {
			addDeviceExample(BusId::BUS2, static_cast<DeviceExampleId>(i), static_cast<NodeId>(i+1));
		}
	}

	virtual ~CanManager()
	{
		// close Buses (especially their threads!) here, so that the receiveThread does not try to call a callback of a already destructed object (parseIncomingSync(..) in this case)
		closeBuses();
	}

	void addDeviceExample(const BusId busId, const DeviceExampleId deviceId, const NodeId nodeId) {
		const unsigned int iBus = static_cast<unsigned int>(busId);
		const std::string name = "EXAMPLE_DEVICE" + std::to_string(static_cast<unsigned int>(deviceId));

		auto options = new example_can::DeviceExampleOptions(static_cast<uint32_t>(nodeId), name);
		options->someParameter = 37;
		options->maxDeviceTimeoutCounter = 1000;

		auto device = new example_can::DeviceExample(options); // or example_can::DeviceExample(nodeId, name);
		buses_.at(iBus)->addDevice( device );
		deviceExampleContainer_.insert(std::make_tuple(name, static_cast<unsigned int>(deviceId), deviceId), device);
	}

	void addSocketBus(const BusId busId, const std::string& interface) {
		const unsigned int iBus = static_cast<unsigned int>(busId);

		SocketBusOptions* options = new SocketBusOptions();
		options->interface = interface;
		options->loopback = true;
		options->sndBufLength = 0;
		// add (multiple) can filters like this {can_id, can_msg}:
		// options->canFilters.push_back({0x123, CAN_SFF_MASK});

		auto bus = new SocketBus(options);
		if(!addBus( bus )) {
			std::cout << "failed to add Bus " << interface << std::endl;
			exit(-1);
		}

		busContainer_.insert(std::make_tuple("BUS" + std::to_string(iBus), iBus, busId), bus);
	}

	bool parseIncomingSync(const CANMsg& cmsg) {
		for(auto device : deviceExampleContainer_) {
			device->setCommand(0.f);
		}

//		std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;
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
			device->setCommand(0.f);
		}
		canManager_.sendSyncOnAllBuses(true);

//		std::cout << "sleeping..\n\n";
//		sleep(5);
		nextStep += std::chrono::microseconds(10000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
