#include <string>
#include <unistd.h>
#include <iostream>
#include <functional>
#include <chrono>
#include <signal.h>
#include <unordered_map>

#include "tcan/CanBusManager.hpp"
#include "tcan/SocketBus.hpp"

#include "tcan_example/CanDeviceExample.hpp"

//#define USE_SYNCHRONOUS_MODE

namespace tcan {
class CanManager : public CanBusManager {
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

	typedef std::unordered_map<unsigned int, CanBus*> BusContainer;
	typedef std::unordered_map<unsigned int, example_can::CanDeviceExample*> DeviceExampleContainer;

	CanManager():
		CanBusManager(),
		busContainer_(),
		deviceExampleContainer_()
	{
	    // add a CAN bus
		addSocketBus(BusId::BUS1, "can0");

		// add some devices to the bus
		for(unsigned int i=0; i<30; i++) {
			addDeviceExample(BusId::BUS1, static_cast<DeviceExampleId>(i), static_cast<NodeId>(i+1));
		}

		// add a custom callback function
        getCanBus(static_cast<unsigned int>(BusId::BUS1))->addCanMessage(DeviceCanOpen::RxPDOSyncId, this, &CanManager::parseIncomingSync);

		// add a second bus
		addSocketBus(BusId::BUS2, "can1");

		// add some devices to the second bus
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
		const std::string name = "EXAMPLE_DEVICE" + std::to_string(static_cast<unsigned int>(deviceId));

		std::unique_ptr<example_can::CanDeviceExampleOptions> options(new example_can::CanDeviceExampleOptions(static_cast<uint32_t>(nodeId), name));
		options->someParameter = 37;
		options->maxDeviceTimeoutCounter_ = 1000;

		auto ret_pair = getCanBus(static_cast<unsigned int>(busId))->addDevice<example_can::CanDeviceExample>( std::move(options) );
		deviceExampleContainer_.insert({static_cast<unsigned int>(deviceId), ret_pair.first});
	}

	void addSocketBus(const BusId busId, const std::string& interface) {
	    std::unique_ptr<SocketBusOptions> options(new SocketBusOptions());
#ifdef USE_SYNCHRONOUS_MODE
		options->asynchronous = false;
#endif
		options->name_ = interface;
		options->loopback_ = true;
		options->sndBufLength_ = 0;
		// add (multiple) can filters like this {can_id, can_msg}:
		// options->canFilters.push_back({0x123, CAN_SFF_MASK});
//		options->canErrorMask = 0;

		auto bus = new SocketBus(std::move(options));
		if(!addBus( bus )) {
			std::cout << "failed to add Bus " << interface << std::endl;
			exit(-1);
		}

		busContainer_.insert({static_cast<unsigned int>(busId), bus});
	}

	bool parseIncomingSync(const CanMsg& cmsg) {
		for(auto device : deviceExampleContainer_) {
			device.second->setCommand(0.f);
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

} /* namespace tcan */

bool g_running = true;

void signal_handler(int) {
	g_running = false;
}

int main() {
	signal(SIGINT, signal_handler);
	tcan::CanManager canManager_;

	auto nextStep = std::chrono::steady_clock::now();

	while(g_running) {
#ifdef USE_SYNCHRONOUS_MODE
		canManager_.readMessagesSynchronous();

		canManager_.sanityCheckSynchronous();
#endif
		for(auto device : canManager_.getDeviceExampleContainer()) {
			std::cout << "Measurement=" << device.second->getMeasurement() << std::endl;
			device.second->setCommand(0.f);
		}
#ifdef USE_SYNCHRONOUS_MODE
		canManager_.writeMessagesSynchronous();
      canManager_.sendSyncOnAllBuses();
		canManager_.writeMessagesSynchronous();
#else
		canManager_.sendSyncOnAllBuses(true);
#endif

		nextStep += std::chrono::microseconds(10000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
