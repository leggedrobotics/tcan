#include <signal.h>
#include <unordered_map>

#include "tcan_usb/UniversalSerialBusManager.hpp"
#include "tcan_usb/UniversalSerialBusOptions.hpp"

#include "tcan_example/XbeeUsb.hpp"

//#define USE_SYNCHRONOUS_MODE

namespace tcan_example {
class UsbManager : public tcan_usb::UniversalSerialBusManager {
public:
	enum class UsbId : unsigned int {
		XBEE1=0
	};

	typedef std::unordered_map<unsigned int, XbeeUsb*> UsbContainer;

	UsbManager():
	    tcan_usb::UniversalSerialBusManager(),
		usbContainer_()
	{
		addUsb(UsbId::XBEE1, "/dev/ttyUSB1");
	}

	virtual ~UsbManager()
	{
		// close Buses (especially their threads!) here, so that the receiveThread does not try to call a callback of a already destroyed object (parseIncomingSync(..) in this case)
		closeBuses();
	}

	void addUsb(const UsbId usbId, const std::string& interface) {
		std::unique_ptr<tcan_usb::UniversalSerialBusOptions> options(new tcan_usb::UniversalSerialBusOptions());
#ifdef USE_SYNCHRONOUS_MODE
		options->mode = UniversalSerialBusOptions::Mode::Synchronous;
#endif
		options->name_ = interface;

		auto bus = new XbeeUsb(std::move(options));
		if(!addBus( bus )) {
			std::cout << "failed to add Bus " << interface << std::endl;
			exit(-1);
		}

		usbContainer_.insert({static_cast<unsigned int>(usbId), bus});
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
	    usbManager_.readMessagesSynchronous();

	    usbManager_.sanityCheckSynchronous();
#endif
		for(auto xbee : usbManager_.getUsbContainer()) {
		    xbee.second->emplaceMessage(tcan_usb::UsbMsg("hi"));
		    // as an alternative, sendMessage(..) can be used, if emplacing the message is not appropriate
		}
#ifdef USE_SYNCHRONOUS_MODE
		usbManager_.writeMessagesSynchronous();
#endif

		nextStep += std::chrono::microseconds(1000000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
