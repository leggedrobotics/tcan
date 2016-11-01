#include <string>
#include <unistd.h>
#include <iostream>
#include <functional>
#include <chrono>
#include <signal.h>

#include "tcan/CanBusManager.hpp"
#include "tcan/SocketBus.hpp"

#include "robot_utils/containers/MultiKeyContainer.hpp"

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

	typedef robot_utils::MultiKeyContainer<CanBus*, BusId> BusContainer;
	typedef robot_utils::MultiKeyContainer<example_can::CanDeviceExample*, DeviceExampleId> DeviceExampleContainer;

	CanManager():
		CanBusManager(),
		busContainer_(),
		deviceExampleContainer_()
	{
		addSocketBus(BusId::BUS1, "can0");
		getCanBus(static_cast<unsigned int>(BusId::BUS1))->addCanMessage(DeviceCanOpen::RxPDOSyncId, this, &CanManager::parseIncomingSync);

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

		auto options = new example_can::CanDeviceExampleOptions(static_cast<uint32_t>(nodeId), name);
		options->someParameter = 37;
		options->maxDeviceTimeoutCounter_ = 1000;

		auto ret_pair = getCanBus(iBus)->addDevice<example_can::CanDeviceExample>( options );
		deviceExampleContainer_.insert(std::make_tuple(name, static_cast<unsigned int>(deviceId), deviceId), ret_pair.first);
	}

	void addSocketBus(const BusId busId, const std::string& interface) {
		const unsigned int iBus = static_cast<unsigned int>(busId);

		SocketBusOptions* options = new SocketBusOptions();
#ifdef USE_SYNCHRONOUS_MODE
		options->asynchronous = false;
#endif
		options->name_ = interface;
		options->loopback_ = true;
		options->sndBufLength_ = 0;
		// add (multiple) can filters like this {can_id, can_msg}:
		// options->canFilters.push_back({0x123, CAN_SFF_MASK});
//		options->canErrorMask = 0;

		auto bus = new SocketBus(options);
		if(!addBus( bus )) {
			std::cout << "failed to add Bus " << interface << std::endl;
			exit(-1);
		}

		busContainer_.insert(std::make_tuple("BUS" + std::to_string(iBus), iBus, busId), bus);
	}

	bool parseIncomingSync(const CanMsg& cmsg) {
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
		canManager_.readMessagesSynchrounous();

		canManager_.sanityCheckSynchronous();
#endif
		for(auto device : canManager_.getDeviceExampleContainer()) {
//			std::cout << "Measurement=" << device->getMeasurement() << std::endl;
			device->setCommand(0.f);
		}
#ifdef USE_SYNCHRONOUS_MODE
		canManager_.writeMessagesSynchronous();
#endif
#ifdef USE_SYNCHRONOUS_MODE
		canManager_.sendSyncOnAllBuses();
		canManager_.writeMessagesSynchronous();
#else
		canManager_.sendSyncOnAllBuses(true);
#endif


//		std::cout << "sleeping..\n\n";
//		sleep(5);
		nextStep += std::chrono::microseconds(10000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
