#pragma once


// tcan ethercat
#include <tcan_ethercat/EtherCatSlave.hpp>

// tcan ethercat example
#include "tcan_ethercat_example/Dsp402.hpp"


namespace tcan_ethercat_example {


struct AnydriveIndata
{
    Dsp402Statusword statusword;

    int16_t mode_of_operation_display = 0;
    int16_t measured_temperature = 0;
    int16_t measured_motor_voltage = 0;

    int64_t measured_motor_position = 0;
    int64_t measured_gear_position = 0;
    int64_t measured_joint_position = 0;
    int32_t measured_motor_current = 0;
    int32_t measured_motor_velocity = 0;
    int32_t measured_gear_velocity = 0;
    int32_t measured_joint_velocity = 0;
    int32_t measured_joint_acceleration = 0;
    int32_t measured_joint_torque = 0;
};

struct AnydriveOutdata
{
    Dsp402Controlword controlword;

    int16_t mode_of_operation = 0;

    int32_t desired_motor_current = 0;
    int32_t desired_joint_velocity = 0;
    int32_t desired_joint_torque = 0;
    int64_t desired_joint_position = 0;
    int16_t control_gain_a = 0;
    int16_t control_gain_b = 0;
    int16_t control_gain_c = 0;
    int16_t control_gain_d = 0;
};


inline AnydriveOutdata createOutdata(Dsp402Command command, double torque)
{
    // Set controlword data
    static Dsp402Controlword controlword;
    switch(command)
    {
        case Dsp402Command::SWITCH_ON:
            controlword.bits.switch_on = 1;
            break;

        case Dsp402Command::SHUTDOWN:
            controlword.bits.switch_on = 0;
            break;

        case Dsp402Command::DISABLE_VOLTAGE:
            controlword.bits.enable_voltage = 0;
            break;

        case Dsp402Command::ENABLE_VOLTAGE:
            controlword.bits.enable_voltage = 1;
            break;

        case Dsp402Command::QUICK_STOP:
            controlword.bits.quick_stop = 1;
            break;

        case Dsp402Command::DISABLE_OPERATION:
            controlword.bits.enable_operation = 0;
            break;

        case Dsp402Command::ENABLE_OPERATION:
            controlword.bits.enable_operation = 1;
            break;

        case Dsp402Command::FAULT_RESET:
            controlword.bits.fault_reset = 1;
            break;

        case Dsp402Command::HALT:
            controlword.bits.halt = 1;
            break;

        case Dsp402Command::HALT_RESET:
            controlword.bits.halt = 0;
            break;

        case Dsp402Command::CLEAR_CONTROLWORD:
            controlword.all = 0;
            break;
    }

    // Covert torque data to INT16 from double
    double rated_current = 20000.0;
    double rated_torque = (20000.0*0.27)*0.001;
    int16_t torque_data = (int16_t)(torque/rated_torque*1000.0);

    // Copy to internal struct
    AnydriveOutdata data;
//    data.controlword.all = controlword.all;
//    data.torque = torque_data;
    return data;
}

template <typename Value>
void readValue(const uint8_t* data, const uint16_t pos, Value& value) {
  value = 0;
  const uint16_t len = sizeof(Value);
  for (uint16_t i = 0; i < len; i++) {
    value |= (static_cast<Value>(data[pos+i]) << i*8);
  }
}

inline AnydriveIndata createIndata(const tcan_ethercat::EtherCatDatagram& datagram)
{
    // Get data
    uint8_t databuffer[56];
    memcpy(&databuffer[0], datagram.getData(), datagram.getDataLength());

    // Store data
    AnydriveIndata data;
    readValue(&databuffer[0], 0, data.statusword.all);
    readValue(&databuffer[0], 2, data.mode_of_operation_display);
    readValue(&databuffer[0], 4, data.measured_temperature);
    readValue(&databuffer[0], 6, data.measured_motor_voltage);
    readValue(&databuffer[0], 8, data.measured_motor_position);
    readValue(&databuffer[0], 16, data.measured_gear_position);
    readValue(&databuffer[0], 24, data.measured_joint_position);
    readValue(&databuffer[0], 32, data.measured_motor_current);
    readValue(&databuffer[0], 36, data.measured_motor_velocity);
    readValue(&databuffer[0], 40, data.measured_gear_velocity);
    readValue(&databuffer[0], 44, data.measured_joint_velocity);
    readValue(&databuffer[0], 48, data.measured_joint_acceleration);
    readValue(&databuffer[0], 52, data.measured_joint_torque);
    return data;
}



class Anydrive : public tcan_ethercat::EtherCatSlave {
 public:
    Anydrive(const uint32_t address, const std::string& name);

    void printStatusInfo();

    bool initDevice();
    bool initializeInterface();
};


} // tcan_example
