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

#include "message_logger/message_logger.hpp"

namespace tcan {
class CanManager : public CanBusManager {
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

	typedef std::unordered_map<unsigned int, CanBus*> BusContainer;
	typedef std::unordered_map<unsigned int, example_can::CanDeviceExample*> DeviceExampleContainer;

	CanManager():
		CanBusManager(),
		busContainer_(),
		deviceExampleContainer_()
	{

	}

	virtual ~CanManager()
	{
		// close Buses (especially their threads!) here, so that the receiveThread does not try to call a callback of a already destructed object (parseIncomingSync(..) in this case)
		closeBuses();
	}

	void init() {
		// add a CAN bus, asynchronous
		SocketBusOptions options;
		options.mode_ = tcan::BusOptions::Mode::Asynchronous;
		options.name_ = "can0";
		options.loopback_ = true;
		options.canErrorMask_ = CAN_ERR_MASK & ~CAN_ERR_LOSTARB; // report all errors but 'arbitration lost'
		// add (multiple) can filters like this {can_id, can_msg}:
		// options->canFilters.push_back({0x123, CAN_SFF_MASK});

		addSocketBus(BusId::BUS1, std::unique_ptr<SocketBusOptions>(new SocketBusOptions(options)));

		// add some devices to the bus
		for(unsigned int i=0; i<30; i++) {
			addDeviceExample(BusId::BUS1, static_cast<DeviceExampleId>(i), static_cast<NodeId>(i+1));
		}

		// add a custom callback function
		getCanBus(static_cast<unsigned int>(BusId::BUS1))->addCanMessage(DeviceCanOpen::RxPDOSyncId, this, &CanManager::parseIncomingSyncBus1);

		// add a second bus, semi-synchornous
		options.mode_ = tcan::BusOptions::Mode::SemiSynchronous;
		options.name_ = "can1";
		addSocketBus(BusId::BUS2, std::unique_ptr<SocketBusOptions>(new SocketBusOptions(options)));

		// add some devices to the second bus
		for(unsigned int i=30; i<40; i++) {
			addDeviceExample(BusId::BUS2, static_cast<DeviceExampleId>(i), static_cast<NodeId>(i+1));
		}

		getCanBus(static_cast<unsigned int>(BusId::BUS2))->addCanMessage(DeviceCanOpen::RxPDOSyncId, this, &CanManager::parseIncomingSyncBus2);

		// add a third bus, asynchronous
		options.mode_ = tcan::BusOptions::Mode::Synchronous;
		options.name_ = "can2";
		addSocketBus(BusId::BUS3, std::unique_ptr<SocketBusOptions>(new SocketBusOptions(options)));

		// add some devices to the third bus
		for(unsigned int i=40; i<50; i++) {
			addDeviceExample(BusId::BUS3, static_cast<DeviceExampleId>(i), static_cast<NodeId>(i+1));
		}

		getCanBus(static_cast<unsigned int>(BusId::BUS3))->addCanMessage(DeviceCanOpen::RxPDOSyncId, this, &CanManager::parseIncomingSyncBus3);

		// start the threads for semi-synchronous buses
		startThreads();
	}

	void addDeviceExample(const BusId busId, const DeviceExampleId deviceId, const NodeId nodeId) {
		const std::string name = "EXAMPLE_DEVICE" + std::to_string(static_cast<unsigned int>(deviceId));

		std::unique_ptr<example_can::CanDeviceExampleOptions> options(new example_can::CanDeviceExampleOptions(static_cast<uint32_t>(nodeId), name));
		options->someParameter = 37;
		options->maxDeviceTimeoutCounter_ = 10;

		auto ret_pair = getCanBus(static_cast<unsigned int>(busId))->addDevice<example_can::CanDeviceExample>( std::move(options) );
		deviceExampleContainer_.insert({static_cast<unsigned int>(deviceId), ret_pair.first});
	}

	void addSocketBus(const BusId busId, std::unique_ptr<SocketBusOptions>&& options) {

		auto bus = new SocketBus(std::move(options));
		if(!addBus( bus )) {
			MELO_FATAL_STREAM("failed to add bus " << bus->getName())
		}

		busContainer_.insert({static_cast<unsigned int>(busId), bus});
	}

	bool parseIncomingSyncBus1(const CanMsg& cmsg) {
		std::cout << "Bus1: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;
		return true;
	}

	bool parseIncomingSyncBus2(const CanMsg& cmsg) {
		std::cout << "Bus2: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;
		return true;
	}

	bool parseIncomingSyncBus3(const CanMsg& cmsg) {
		std::cout << "Bus3: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;
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
	canManager_.init();

	auto nextStep = std::chrono::steady_clock::now();

	while(g_running) {
		// these two calls affect synchronous buses only, BUS3 in this case
		canManager_.readMessagesSynchronous();
		canManager_.sanityCheckSynchronous();

		for(auto device : canManager_.getDeviceExampleContainer()) {
//			MELO_INFO_STREAM("Measurement " << device.second->getName() << " = " << device.second->getMeasurement());
			device.second->setCommand(0.f);
		}

		// write the messages on the synchronous and semi-synchronous buses.
		canManager_.writeMessagesSynchronous();

      	canManager_.sendSyncOnAllBuses(true); 	// call this function after writeMessagesSynchronous() for synchronous and semi-synchonous buses,
												// to ensure that the output queues are empty such that the SYNC messages can be put
												// on all buses at the same time
		canManager_.writeMessagesSynchronous(); // now write the SYNC messages to the synchronous and semi-synchronous buses

		nextStep += std::chrono::microseconds(100000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
