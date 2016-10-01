#include <signal.h>

#include "robot_utils/containers/MultiKeyContainer.hpp"

#include "tcan/UniversalSerialBusManager.hpp"
#include "tcan/UniversalSerialBusOptions.hpp"

#include "tcan_example/XbeeUsb.hpp"

//#define USE_SYNCHRONOUS_MODE

namespace tcan_example {
class UsbManager : public tcan::UniversalSerialBusManager {
public:
	enum class UsbId : unsigned int {
		XBEE1=0
	};

	typedef robot_utils::MultiKeyContainer<XbeeUsb*, UsbId> UsbContainer;

	UsbManager():
	    tcan::UniversalSerialBusManager(),
		usbContainer_()
	{
		addUsb(UsbId::XBEE1, "/dev/ttyUSB0");
	}

	virtual ~UsbManager()
	{
		// close Buses (especially their threads!) here, so that the receiveThread does not try to call a callback of a already destructed object (parseIncomingSync(..) in this case)
		closeBuses();
	}

	void addUsb(const UsbId usbId, const std::string& interface) {
		const unsigned int iBus = static_cast<unsigned int>(usbId);

		tcan::UniversalSerialBusOptions* options = new tcan::UniversalSerialBusOptions();
#ifdef USE_SYNCHRONOUS_MODE
		options->asynchronous = false;
#endif
		options->name_ = interface;

		auto bus = new XbeeUsb(options);
		if(!addBus( bus )) {
			std::cout << "failed to add Bus " << interface << std::endl;
			exit(-1);
		}

		usbContainer_.insert(std::make_tuple("BUS" + std::to_string(iBus), iBus, usbId), bus);
	}

	UsbContainer getUsbContainer() { return usbContainer_; }

protected:
	UsbContainer usbContainer_;
};

} // namespace tcan_example

bool g_running = true;

void signal_handler(int) {
	g_running = false;
}

int main() {
	signal(SIGINT, signal_handler);
	tcan_example::UsbManager usbManager_;

	auto nextStep = std::chrono::steady_clock::now();

	while(g_running) {
#ifdef USE_SYNCHRONOUS_MODE
	    usbManager_.readMessagesSynchrounous();

	    usbManager_.sanityCheckSynchronous();
#endif
		for(auto xbee : usbManager_.getUsbContainer()) {
//			std::cout << "Measurement=" << device->getMeasurement() << std::endl;
//		    xbee->sendMessage(tcan::UsbMsg("hi"));
		}
#ifdef USE_SYNCHRONOUS_MODE
		usbManager_.writeMessagesSynchronous();
#endif

//		std::cout << "sleeping..\n\n";
//		sleep(5);
		nextStep += std::chrono::microseconds(10000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
