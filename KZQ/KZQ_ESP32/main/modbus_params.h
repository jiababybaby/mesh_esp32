/*=====================================================================================
 * Description:
 *   The Modbus parameter structures used to define Modbus instances that
 *   can be addressed by Modbus protocol. Define these structures per your needs in
 *   your application. Below is just an example of possible parameters.
 *====================================================================================*/
#ifndef _DEVICE_PARAMS
#define _DEVICE_PARAMS

// This file defines structure of modbus parameters which reflect correspond modbus address space
// for each modbus register type (coils, discreet inputs, holding registers, input registers)
#pragma pack(push, 1)
typedef struct
{
    uint8_t discrete_input0:1;
    uint8_t discrete_input1:1;
    uint8_t discrete_input2:1;
    uint8_t discrete_input3:1;
    uint8_t discrete_input4:1;
    uint8_t discrete_input5:1;
    uint8_t discrete_input6:1;
    uint8_t discrete_input7:1;
    uint8_t discrete_input_port1:8;
} discrete_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t coils_port0;
    uint8_t coils_port1;
} coil_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    float input_data0;
    float input_data1;
    float input_data2;
    float input_data3;
} input_reg_params_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct 
{
    uint16_t WATER_ADDR;
    uint16_t WATER_BAU;
    int16_t POWER_C;
    int16_t POWER_PROTO;
    int16_t POWER_BAU;
    int16_t POWER_ADDR;
    uint16_t WATER_DATA[2];//用水量，使用两个寄存器，四个字节
    int16_t POWER_V[2];//电压，使用两个寄存器，四个字节
    int16_t POWER_I[2];//电流，使用两个寄存器，四个字节
    int16_t POWER_P[2];//瞬时总有功功率，使用两个寄存器，四个字节
    int16_t POWER_Q[2];//瞬时总无功功率，使用两个寄存器，四个字节
    int16_t POWER_S[2];//瞬时总视在功率，使用两个寄存器，四个字节
    int16_t POWER_PF[2];//总功功率因数，使用两个寄存器，四个字节
    int16_t POWER_F[2];//频率，使用两个寄存器，四个字节
    int16_t POWER_EP[2];//有功总电能，使用两个寄存器，四个字节
    uint16_t test_regs[150];
} holding_reg_params_t;
#pragma pack(pop)

extern holding_reg_params_t holding_reg_params;
extern input_reg_params_t input_reg_params;
extern coil_reg_params_t coil_reg_params;
extern discrete_reg_params_t discrete_reg_params;

#endif // !defined(_DEVICE_PARAMS)
