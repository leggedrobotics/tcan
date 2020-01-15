#include <gtest/gtest.h>

#include <tcan_can/SocketBus.hpp>

struct BarDevice : public tcan_can::CanDevice {
	template<typename... Args>
	explicit BarDevice(Args&&... args) : tcan_can::CanDevice(std::forward<Args>(args)...) {};
	bool initDevice() override { return true; }
	bool configureDevice(const tcan_can::CanMsg& msg) override { return true; }

	bool callMe(const tcan_can::CanMsg& msg) {
		isCalled = true;
		return true;
	}

	bool wasCalled() {
		auto wasCalled = isCalled;
		isCalled = false;
		return wasCalled;
	}

private:
	bool isCalled = false;
};

TEST(can_bus, handle_exact_cob) {
	tcan_can::SocketBus bus { std::make_unique<tcan_can::SocketBusOptions>("Foo") };
	BarDevice dev {0x123, "Bar"};

	bus.addCanMessage(0xdefacedu, &dev, &BarDevice::callMe);

	bus.handleMessage(tcan_can::CanMsg{0xbaadf00du});
	ASSERT_FALSE(dev.wasCalled());
	bus.handleMessage(tcan_can::CanMsg{0xdefacedu});
	ASSERT_TRUE(dev.wasCalled());
}

TEST(can_bus, handle_cob_mask) {
	auto mask = tcan_can::CanBus::CobMatcher {0x00FF00FF, 0x00120034};

	tcan_can::SocketBus bus { std::make_unique<tcan_can::SocketBusOptions>("Foo") };
	BarDevice dev {0x123, "Bar"};

	bus.addCanMessage(mask, &dev, &BarDevice::callMe);

	bus.handleMessage(tcan_can::CanMsg{0xdefacedu});
	ASSERT_FALSE(dev.wasCalled());

	bus.handleMessage(tcan_can::CanMsg{0xab12cd34});
	ASSERT_TRUE(dev.wasCalled());

	bus.handleMessage(tcan_can::CanMsg{0x00000000});
	ASSERT_FALSE(dev.wasCalled());

	bus.handleMessage(tcan_can::CanMsg{0x00120034});
	ASSERT_TRUE(dev.wasCalled());
}

int main(int argc, char* argv[]) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}