#pragma once

#include "tcan_can_j1939/DeviceJ1939.hpp"
#include "tcan_can_j1939/unit_conversions.hpp"
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

    /**
     * @return roll angle in radians
     */
    double getRollAngle() const { return radiansFromDegrees(slopeSensorInformation2_.rollAngle_); }
    /**
     * @return pitch angle in radians
     */
    double getPitchAngle() const { return radiansFromDegrees(slopeSensorInformation2_.pitchAngle_); }
    /**
     * @return pitch rate in radians/second
     */
    double getPitchRate() const { return radiansFromDegrees(angularRateInformation_.pitchRate_); }
    /**
     * @return roll rate in radians/second
     */
    double getRollRate() const { return radiansFromDegrees(angularRateInformation_.rollRate_); }

   private:
    messages::AccelerationSensor accelerationSensor_;
    messages::SlopeSensorInformation2 slopeSensorInformation2_;
    messages::AngularRateInformation angularRateInformation_;
};

}  // namespace devices
}  // namespace tcan_can_j1939