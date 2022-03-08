#include <tcan_can/DeviceJ1939.hpp>

#include "tcan_can_j1939_devices/message/AccelerationSensor.hpp"
#include "tcan_can_j1939_devices/message/AngularRateInformation.hpp"
#include "tcan_can_j1939_devices/message/SlopeSensorInformation2.hpp"

namespace tcan_can_j1939_devices {

class Imu : public tcan_can::DeviceJ1939 {
   public:
    template <typename... Args>
    Imu(Args&&... args) : tcan_can::DeviceJ1939(std::forward<Args>(args)...) {
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
    AccelerationSensor accelerationSensor_;
    SlopeSensorInformation2 slopeSensorInformation2_;
    AngularRateInformation angularRateInformation_;
};

}  // namespace tcan_can_j1939_devices