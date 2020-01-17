#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <tcan_can/J1939CanMsg.hpp>

tcan_can::J1939CanMsg msg(uint32_t cob) {
	return tcan_can::J1939CanMsg {tcan_can::CanMsg {cob } };
}

TEST(j1939_can_msg, construct) {
	auto msg = tcan_can::CanMsg(0xdefacedu, {0x10, 0x20});

	auto sae = tcan_can::J1939CanMsg {msg};

	EXPECT_EQ(0xdefacedu, msg.getCobId());
	ASSERT_EQ(2, msg.getLength());

	std::vector<uint8_t> d;
	d.assign(msg.getData(), msg.getData() + msg.getLength());
	ASSERT_THAT(d, testing::ElementsAre(0x10, 0x20));
}

TEST(j1939_can_msg, pgn) {
	EXPECT_EQ(
		0xfe31u,
		msg( 0x18fe3185 ).getParameterGroupNumber() );

	EXPECT_EQ(
		0x1fe31u,
		msg( 0x1dfe3185 ).getParameterGroupNumber() );

	EXPECT_EQ(
		msg(0x18fe385).getParameterGroupCanId(),
		msg(0x18fe385).getCobId() & tcan_can::J1939CanMsg::CAN_ID_PGN_MASK );
}

TEST(j1939_can_msg, priority) {
	EXPECT_EQ(
		6,
		msg( 0x18fe3185 ).getPriority() );

	EXPECT_EQ(
		3,
		msg( 0xcfe3185 ).getPriority() );
}

TEST(j1939_can_msg, pf) {
	EXPECT_EQ(
		0xFE,
		msg( 0x18fe3185 ).getPduFormat() );

	EXPECT_EQ(
		0x3E,
		msg( 0x183e3185 ).getPduFormat() );
}

TEST(j1939_can_msg, ps) {
	EXPECT_EQ(
		49,
		msg( 0x18fe3185 ).getPduSpecific() );
}

TEST(j1939_can_msg, dp_edp) {
	EXPECT_TRUE(msg( 0x18fe3186u | 0x1u << 24u ).getDataPage());
	EXPECT_FALSE(msg( 0x18fe3186 ).getDataPage());

	EXPECT_TRUE(msg( 0x18fe3185u | 0x1u << 25u ).getExtendedDataPage());
	EXPECT_FALSE(msg( 0x18fe3185 ).getExtendedDataPage());
}

TEST(j1939_can_msg, src) {
	EXPECT_EQ(0x85, msg( 0x18fe3185 ).getSourceAddress());
}

int main(int argc, char* argv[]) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}