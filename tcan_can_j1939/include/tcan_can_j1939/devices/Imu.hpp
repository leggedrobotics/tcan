#pragma once

#include "tcan_can_j1939/DeviceJ1939.hpp"
#include "tcan_can_j1939/messages/AccelerationSensor.hpp"
#include "tcan_can_j1939/messages/AngularRateInformation.hpp"
#include "tcan_can_j1939/messages/SlopeSensorInformation2.hpp"

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

    double getRollAngle() const { return slopeSensorInformation2_.rollAngle_; }
    double getPitchAngle() const { return slopeSensorInformation2_.pitchAngle_; }
    double getPitchRate() const { return angularRateInformation_.pitchRate_; }
    double getRollRate() const { return angularRateInformation_.rollRate_; }

   private:
    messages::AccelerationSensor accelerationSensor_;
    messages::SlopeSensorInformation2 slopeSensorInformation2_;
    messages::AngularRateInformation angularRateInformation_;
};

}  // namespace devices
}  // namespace tcan_can_j1939