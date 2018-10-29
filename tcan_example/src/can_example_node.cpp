#include <chrono>
#include <signal.h>

#include "tcan_example/CanDeviceExample.hpp"
#include "tcan_example/CanManagerExample.hpp"

#include "message_logger/message_logger.hpp"

std::atomic<bool> g_running{true};

void signal_handler(int) {
	g_running = false;
}

int main() {
	signal(SIGINT, signal_handler);
	tcan_example::CanManagerExample canManager_;
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
