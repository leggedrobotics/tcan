#include "yalc/BusManager.hpp"
#include "yalc/SocketBus.hpp"
#include "yalc_example/DeviceExample.hpp"

class CanManager : public BusManager {
public:
	CanManager() {
		addBus( new SocketBus("can0") );
		addDeviceExample();
	}

	void addDeviceExample() {

	}
};

int main() {
	CanManager canManager_;
	return 0;
}
