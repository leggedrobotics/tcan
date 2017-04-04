#pragma once


// tcan
#include <tcan/EtherCatDevice.hpp>


namespace tcan_example {

enum class Dsp402Command
{
    SWITCH_ON           = 0x00,
    SHUTDOWN            ,
    DISABLE_VOLTAGE     ,
    ENABLE_VOLTAGE      ,
    QUICK_STOP          ,
    DISABLE_OPERATION   ,
    ENABLE_OPERATION    ,
    FAULT_RESET         ,
    HALT                ,
    HALT_RESET          ,
    CLEAR_CONTROLWORD
};

// Note: Bit field ordering depends on the Endianness which is implementation dependent.
struct Dsp402StatuswordBits
{
    uint16_t ready_to_switch_on:1;
    uint16_t switched_on:1;
    uint16_t operation_enabled:1;
    uint16_t fault:1;
    uint16_t voltage_enabled:1;
    uint16_t quick_stop:1;
    uint16_t switch_on_disabled:1;
    uint16_t warning:1;
    uint16_t manufacturer_specific_0:1;
    uint16_t remote:1;
    uint16_t operation_mode_specific_0:1;
    uint16_t internal_limit_active:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t operation_mode_specific_2:1;
    uint16_t manufacturer_specific_1:2;
};

union Dsp402Statusword
{
    uint16_t all;
    Dsp402StatuswordBits bits;
};

struct Dsp402ControlwordBits
{
    uint16_t switch_on:1;
    uint16_t enable_voltage:1;
    uint16_t quick_stop:1;
    uint16_t enable_operation:1;
    uint16_t operation_mode_specific_0:3;
    uint16_t fault_reset:1;
    uint16_t halt:1;
    uint16_t operation_mode_specific_1:1;
    uint16_t rsrvd:1;
    uint16_t manufacturer_specific:5;
};

union Dsp402Controlword
{
    uint16_t all;
    Dsp402ControlwordBits bits;
};


inline void printStatusword(Dsp402Statusword statusword)
{
    // Printout
    printf("\n\n");
    printf("statusword.ready_to_switch_on         = %d\n", statusword.bits.ready_to_switch_on);
    printf("statusword.switched_on                = %d\n", statusword.bits.switched_on);
    printf("statusword.operation_enabled          = %d\n", statusword.bits.operation_enabled);
    printf("statusword.fault                      = %d\n", statusword.bits.fault);
    printf("statusword.volt_enabled               = %d\n", statusword.bits.voltage_enabled);
    printf("statusword.quick_stop                 = %d\n", statusword.bits.quick_stop);
    printf("statusword.switch_on_disabled         = %d\n", statusword.bits.switch_on_disabled);
    printf("statusword.warning                    = %d\n", statusword.bits.warning);
    printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_0);
    printf("statusword.remote                     = %d\n", statusword.bits.remote);
    printf("statusword.operation_mode_specific_0  = %d\n", statusword.bits.operation_mode_specific_0);
    printf("statusword.internal_limit_active      = %d\n", statusword.bits.internal_limit_active);
    printf("statusword.operation_mode_specific_1  = %d\n", statusword.bits.operation_mode_specific_1);
    printf("statusword.operation_mode_specific_2  = %d\n", statusword.bits.operation_mode_specific_2);
    printf("statusword.manufacturer_specific      = %d\n", statusword.bits.manufacturer_specific_1);
}

inline void printControlword(Dsp402Controlword controlword)
{
    // Printout
    printf("\n\n");
    printf("controlword.switch_on                 = %d\n", controlword.bits.switch_on);
    printf("controlword.enable_voltage            = %d\n", controlword.bits.enable_voltage);
    printf("controlword.quick_stop                = %d\n", controlword.bits.quick_stop);
    printf("controlword.enable_operation          = %d\n", controlword.bits.enable_operation);
    printf("controlword.operation_mode_specific_0 = %d\n", controlword.bits.operation_mode_specific_0);
    printf("controlword.fault_reset               = %d\n", controlword.bits.fault_reset);
    printf("controlword.halt                      = %d\n", controlword.bits.halt);
    printf("controlword.operation_mode_specific_1 = %d\n", controlword.bits.operation_mode_specific_1);
    printf("controlword.manufacturer_specific     = %d\n", controlword.bits.manufacturer_specific);
}






struct ElmoTwitterIndata
{
    Dsp402Statusword statusword;

    int position;
    int velocity;

    int digitalin;

    int busvoltage;
    int motorcurrent;
};

struct ElmoTwitterOutdata
{
    Dsp402Controlword controlword;

    int torque;
};


inline ElmoTwitterOutdata createOutdata(Dsp402Command command, double torque)
{
    // Set controlword data
    static Dsp402Controlword controlword = {0};
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
    ElmoTwitterOutdata data;
    data.controlword.all = controlword.all;
    data.torque = torque_data;
    return data;
}

inline ElmoTwitterIndata createIndata(const tcan::EtherCatDatagram& datagram)
{
    // Get data
    char databuffer[21];
    memcpy(&databuffer[0], datagram.getData(), datagram.getDataLength());

    // Store data
    ElmoTwitterIndata data;
    data.statusword.all = ((databuffer[13] << 8 ) & 0xff00) | (databuffer[12] & 0xff);
    data.position = ((databuffer[3] << 24) & 0xff000000) | ((databuffer[2] << 16) & 0x00ff0000) | ((databuffer[1] << 8) & 0x0000ff00) | ((databuffer[0] << 0) & 0x000000ff);
    data.digitalin = ((databuffer[7] << 24) & 0xff000000) | ((databuffer[6] << 16) & 0x00ff0000) | ((databuffer[5] << 8) & 0x0000ff00) | ((databuffer[4] << 0) & 0x000000ff);
    data.velocity = ((databuffer[11] << 24) & 0xff000000) | ((databuffer[10] << 16) & 0x00ff0000) | ((databuffer[9] << 8) & 0x0000ff00) | ((databuffer[8] << 0) & 0x000000ff);
    data.busvoltage = ((databuffer[18] << 24) & 0xff000000) | ((databuffer[17] << 16) & 0x00ff0000) | ((databuffer[16] << 8) & 0x0000ff00) | ((databuffer[15] << 0) & 0x000000ff);
    data.motorcurrent = ((databuffer[20] << 8) & 0xff00) | ((databuffer[19] << 0) & 0x00ff);
    return data;
}



class ElmoTwitter : public tcan::EtherCatDevice {
 public:
    ElmoTwitter(const uint32_t address, const std::string& name);

    void dumpSlaveStatusInfo();
    void configureSlave();
};


} // tcan_example
