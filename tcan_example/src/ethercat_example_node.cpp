#include <signal.h>
#include <unordered_map>

#include "tcan/EtherCatBus.hpp"
#include "tcan/EtherCatBusManager.hpp"

#define USE_SYNCHRONOUS_MODE

/*
namespace tcan_example {
class TcpManager : public tcan::IpBusManager {
public:
	enum class IpId : unsigned int {
		DEVICE1=0
	};

	typedef std::unordered_map<unsigned int, TcpConnection*> ConnectionContainer;

	TcpManager():
	    tcan::IpBusManager(),
	    connectionContainer_()
	{
		addConnection(IpId::DEVICE1, "192.168.0.100", 9999);
	}

	virtual ~TcpManager()
	{
		// close Buses (especially their threads!) here, so that the receiveThread does not try to call a callback of a already destructed object (parseIncomingSync(..) in this case)
		closeBuses();
	}

	void addConnection(const IpId ipId, const std::string& host, const unsigned int port) {
		tcan::IpBusOptions* options = new tcan::IpBusOptions(host, port);
#ifdef USE_SYNCHRONOUS_MODE
		options->asynchronous = false;
#endif

		auto connection = new TcpConnection(options);
		if(!addBus( connection )) {
			std::cout << "failed to add Bus " << host << std::endl;
			exit(-1);
		}

		connectionContainer_.insert({static_cast<unsigned int>(ipId), connection});
	}

	ConnectionContainer getConnectionContainer() { return connectionContainer_; }

protected:
	ConnectionContainer connectionContainer_;
};

} // namespace tcan_example
*/

bool g_running = true;

void signal_handler(int) {
	g_running = false;
}

int main() {
  const bool asynchronous = false;

	signal(SIGINT, signal_handler);

  tcan::EtherCatDevice device(1, "? M:0000009a I:00030924");

	tcan::EtherCatBusOptions* busOptions = new tcan::EtherCatBusOptions(); // TODO: Why are the options destroyed in the bus destructor?
	busOptions->name_ = "enp0s31f6";
	busOptions->asynchronous_ = asynchronous;
	tcan::EtherCatBus bus(busOptions);
  bus.addDevice(&device);

  tcan::EtherCatBusManager busManager;
	if (!busManager.addBus(&bus)) {
	  MELO_ERROR_STREAM("Bus could not be added.");
	  return 0;
	}
  std::cout << "bus added and initialized" << std::endl;

	auto nextStep = std::chrono::steady_clock::now();

	while(g_running) {
	  std::cout << "in loop" << std::endl;
    if (!asynchronous) {
      busManager.readMessagesSynchrounous();
      busManager.sanityCheckSynchronous();
    }
//	  device.emplaceMessage(tcan::IpMsg("hi"));
		// as an alternative, sendMessage(..) can be used, if emplacing the message is not appropriate
    if (!asynchronous) {
      std::cout << " writeMessagesSynchronous " << std::endl;
      busManager.writeMessagesSynchronous();
    }

		nextStep += std::chrono::microseconds(100000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
