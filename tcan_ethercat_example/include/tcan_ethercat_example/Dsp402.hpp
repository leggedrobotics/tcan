#pragma once


// tcan ethercat
#include "tcan_ethercat/EtherCatSlave.hpp"


namespace tcan_ethercat_example {

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
    uint16_t all = 0;
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
    uint16_t all = 0;
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



} // tcan_ethercat_example
