#pragma once

#include "tcan_can_j1939/DeviceJ1939.hpp"
#include "tcan_can_j1939/messages/AccelerationSensor.hpp"
#include "tcan_can_j1939/messages/AngularRateInformation.hpp"
#include "tcan_can_j1939/messages/SlopeSensorInformation2.hpp"
#include "tcan_can_j1939/unit_conversions.hpp"

namespace tcan_can_j1939 {
namespace devices {

class Imu : public DeviceJ1939 {
   public:
    template <typename... Args>
    Imu(Args&&... args) : DeviceJ1939(std::forward<Args>(args)...) {
        addParser(accelerationSensor_);
        addParser(angularRateInformation_);
        addParser(slopeSensorInformation2_);
    }
    ~Imu() override = default;

    // All accelerations in m/s^2, in a z-up coordinate system
    double getLateralAcceleration() const { return accelerationSensor_.lateralAcceleration_; }
    double getLongitudinalAcceleration() const { return accelerationSensor_.longitudinalAcceleration_; }
    double getVerticalAcceleration() const { return accelerationSensor_.verticalAcceleration; }

    // All angles in rad, defined in a z-down coordinate system (pitch positive when driving uphill)
    // All angles signify the angle between the corresponding axis and a horizontal ground plane
    double getPitchAngle() const { return radiansFromDegrees(slopeSensorInformation2_.pitchAngle_); }
    double getRollAngle() const { return radiansFromDegrees(slopeSensorInformation2_.rollAngle_); }

    // All rates in rad/s, defined in a z-down coordinate system (pitch-rate positive when driving further uphill, yaw-rate positive when turning clockwise)
    double getPitchRate() const { return radiansFromDegrees(angularRateInformation_.pitchRate_); }
    double getRollRate() const { return radiansFromDegrees(angularRateInformation_.rollRate_); }
    double getYawRate() const { return radiansFromDegrees(angularRateInformation_.yawRate_); }

   private:
    messages::AccelerationSensor accelerationSensor_;
    messages::SlopeSensorInformation2 slopeSensorInformation2_;
    messages::AngularRateInformation angularRateInformation_;
};

}  // namespace devices
}  // namespace tcan_can_j1939