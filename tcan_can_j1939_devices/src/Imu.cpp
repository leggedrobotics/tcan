#include "tcan_can_j1939_devices/Imu.hpp"

namespace impl {

// TODO (kersimon): Maybe move readuin24 to tcan_can
uint32_t readuint24(const tcan_can::CanMsg& msg, uint8_t pos) {
    assert(pos + 3u <= msg.getLength());
    return (static_cast<uint32_t>(msg.getData()[2 + pos]) << 16) | (static_cast<uint32_t>(msg.getData()[1 + pos]) << 8) |
           (static_cast<uint32_t>(msg.getData()[0 + pos]));
}

double accelerationFromUint16(uint16_t v) {
    return v * 0.01 - 320.;
    return tcan_can::utils::scaledMessageFromRaw(v, 0.01, -320.);
}

double angleFromUint32(uint32_t v) {
    const double degrees = tcan_can::utils::scaledMessageFromRaw(v, 1. / 32768., -250.);
    return degrees / 180 * M_PI;
}

double angularRateFromUint16(uint16_t v) {
    return tcan_can::utils::scaledMessageFromRaw(v, 1 / 128., -250.);
}

}  // namespace impl

namespace tcan_can_j1939_devices {

bool Imu::initDevice() {
    addPgn(0xF02D, this, &Imu::parseAccelerations);
    addPgn(0xF029, this, &Imu::parseOrientation);
    addPgn(0xF02A, this, &Imu::parseAngularRate);

    return true;
}

bool Imu::parseAccelerations(const tcan_can::CanMsg& msg) {
    // Parsed in order yxz on purpose
    // CanMsg in z-up coordinate system

    accelerations_.y_ = impl::accelerationFromUint16(msg.readuint16(0));
    accelerations_.x_ = impl::accelerationFromUint16(msg.readuint16(2));
    accelerations_.z_ = impl::accelerationFromUint16(msg.readuint16(4));
    return true;
}

bool Imu::parseOrientation(const tcan_can::CanMsg& msg) {
    // Parsed in order yxz on purpose
    // CanMsg in z-down coordinate system, hence inverting y and z
    rotation_.y_ = -impl::angleFromUint32(impl::readuint24(msg, 0));
    rotation_.x_ = impl::angleFromUint32(impl::readuint24(msg, 3));
    rotation_.z_ = -0.;
    return true;
}

bool Imu::parseAngularRate(const tcan_can::CanMsg& msg) {
    // CanMsg in z-down coordinate system, hence inverting y and z
    angularRates_.y_ = -impl::angularRateFromUint16(msg.readuint16(0));
    angularRates_.x_ = impl::angularRateFromUint16(msg.readuint16(2));
    angularRates_.z_ = -impl::angularRateFromUint16(msg.readuint16(4));
    return true;
}

}  // namespace tcan_can_j1939_devices