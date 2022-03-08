#include <tcan_can/DeviceJ1939.hpp>

namespace tcan_can_j1939_devices {

class Imu : public tcan_can::DeviceJ1939 {
   public:
    using tcan_can::DeviceJ1939::DeviceJ1939;
    ~Imu() override = default;

    bool initDevice() override;
    bool configureDevice(const tcan_can::CanMsg& /*msg*/) override { return true; }
    bool parseAccelerations(const tcan_can::CanMsg& msg);
    bool parseOrientation(const tcan_can::CanMsg& msg);
    bool parseAngularRate(const tcan_can::CanMsg& msg);

    double getRoll() const { return rotation_.y_; }
    double getPitch() const { return rotation_.x_; }
    double getPitchVelocity() const { return angularRates_.x_; }
    double getRollVelocity() const { return angularRates_.y_; }

   private:
    struct Vector3d {
        double x_{};
        double y_{};
        double z_{};
    };
    Vector3d accelerations_;
    Vector3d rotation_;  // Rotation is defined as the angle between vehicle axis and ground plane, hence no true Euler angles
    Vector3d angularRates_;
};

}  // namespace tcan_can_j1939_devices