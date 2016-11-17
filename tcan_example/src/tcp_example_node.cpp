#include <signal.h>

#include "robot_utils/containers/MultiKeyContainer.hpp"

#include "tcan/IpBusManager.hpp"
#include "tcan/IpBusOptions.hpp"

#include "tcan_example/TcpDevice.hpp"

//#define USE_SYNCHRONOUS_MODE

namespace tcan_example {
class TcpManager : public tcan::IpBusManager {
public:
	enum class IpId : unsigned int {
		DEVICE1=0
	};

	typedef robot_utils::MultiKeyContainer<TcpDevice*, IpId> DeviceContainer;

	TcpManager():
	    tcan::IpBusManager(),
		deviceContainer_()
	{
		addDevice(IpId::DEVICE1, "192.168.0.100", 9999);
	}

	virtual ~TcpManager()
	{
		// close Buses (especially their threads!) here, so that the receiveThread does not try to call a callback of a already destructed object (parseIncomingSync(..) in this case)
		closeBuses();
	}

	void addDevice(const IpId ipId, const std::string& host, const unsigned int port) {
		const unsigned int iBus = static_cast<unsigned int>(ipId);

		tcan::IpBusOptions* options = new tcan::IpBusOptions(host, port);
#ifdef USE_SYNCHRONOUS_MODE
		options->asynchronous = false;
#endif

		auto device = new TcpDevice(options);
		if(!addBus( device )) {
			std::cout << "failed to add Bus " << host << std::endl;
			exit(-1);
		}

		deviceContainer_.insert(std::make_tuple("BUS" + std::to_string(iBus), iBus, ipId), device);
	}

	DeviceContainer getDeviceContainer() { return deviceContainer_; }

protected:
	DeviceContainer deviceContainer_;
};

} // namespace tcan_example

bool g_running = true;

void signal_handler(int) {
	g_running = false;
}

int main() {
	signal(SIGINT, signal_handler);
	tcan_example::TcpManager myManager_;

	auto nextStep = std::chrono::steady_clock::now();

	while(g_running) {
#ifdef USE_SYNCHRONOUS_MODE
		myManager_.readMessagesSynchrounous();

		myManager_.sanityCheckSynchronous();
#endif
		for(auto device : myManager_.getDeviceContainer()) {
//			std::cout << "Measurement=" << device->getMeasurement() << std::endl;
			device->sendMessage(tcan::IpMsg("hi"));
		}
#ifdef USE_SYNCHRONOUS_MODE
		myManager_.writeMessagesSynchronous();
#endif

//		std::cout << "sleeping..\n\n";
//		sleep(5);
		nextStep += std::chrono::microseconds(1000000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
