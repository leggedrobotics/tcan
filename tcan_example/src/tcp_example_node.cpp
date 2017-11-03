#include <signal.h>
#include <unordered_map>

#include "tcan_ip/IpBusManager.hpp"
#include "tcan_ip/IpBusOptions.hpp"

#include "tcan_example/TcpConnection.hpp"

//#define USE_SYNCHRONOUS_MODE

namespace tcan_example {
class TcpManager : public tcan_ip::IpBusManager {
public:
	enum class IpId : unsigned int {
		DEVICE1=0
	};

	typedef std::unordered_map<unsigned int, TcpConnection*> ConnectionContainer;

	TcpManager():
	    tcan_ip::IpBusManager(),
	    connectionContainer_()
	{
		addConnection(IpId::DEVICE1, "192.168.0.100", 9999);
	}

	virtual ~TcpManager()
	{
		closeBuses();
	}

	void addConnection(const IpId ipId, const std::string& host, const unsigned int port) {
	    std::unique_ptr<tcan_ip::IpBusOptions> options(new tcan_ip::IpBusOptions(host, port));
#ifdef USE_SYNCHRONOUS_MODE
		options->asynchronous = false;
#endif

		auto connection = new TcpConnection(std::move(options));
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
		myManager_.readMessagesSynchronous();

		myManager_.sanityCheckSynchronous();
#endif
		for(auto device : myManager_.getConnectionContainer()) {
			device.second->emplaceMessage(tcan_ip::IpMsg("hi"));
			// as an alternative, sendMessage(..) can be used, if emplacing the message is not appropriate
		}
#ifdef USE_SYNCHRONOUS_MODE
		myManager_.writeMessagesSynchronous();
#endif

		nextStep += std::chrono::microseconds(1000000);
		std::this_thread::sleep_until( nextStep );
	}
	return 0;
}
