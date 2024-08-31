/**
 * @file        bsp_ms5611.h
 * @author      sephony
 * @date        2024-09-01
 * @version     1.0.0
 * @brief       Bsp Library for MS5611 Pressure-Sensor Driver
 * @details     Based on MS5611 sensor, supporting IIC & SPI connection
 * @note        URL: https://github.com/sephony/MS5611
 *
 * @attention
 *	MIT License
 *
 *	Copyright (c) 2024 sephony
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 *------------------------------------------------------------------------------
 */
#ifndef __BSP_MS5611_H
#define __BSP_MS5611_H

/* Standard libarary */
#include "math.h"
#include "stdbool.h"
#include "stdlib.h"

/* User Inlcude */
#include "bsp_interface.h"
#include "filter.h"

/**
 *------------------------------------------------------------------------------
 * MS5611-01BA datasheet:
 *
 * MODE         SPI    I2C    pin     MS5611
 *                                  +--------+
 *              VCC    VCC    VCC---| o      |
 *              GND    GND    GND---| o      |
 *              SCK    SCL    SCL---| o      |
 *             MOSI    SDA    SDA---| o      |
 *               CS    CSB    CSB---| o      |
 *             MISO           SDO---| o L    |   L = led
 *              LOW   HIGH     PS---| o      |   PS = protocol select
 *                                  +--------+
 *
 *          IIC mode:
 *              PS to VCC   ==>  I2C  (Don`t forget the pull-up resistance)
 *              CSB to VCC  ==>  0x76 (0b1110110)
 *              CSB to GND  ==>  0x77 (0b1110111)
 *              SDO x
 *
 *          SPI mode:
 *              PS to GND  ==>  SPI
 *
 * @note There will be a heat accumulation phenomenon in SPI mode.
 *------------------------------------------------------------------------------
 * Read factory data from PROM
 * C0: is 0 is normal, 1 means IIC communication has problems or the delay after
 *     hardware reset is not enough
 * C1: Pressure sensitivity                          SENS_T1    uint16_t
 * C2: Pressure compensation                         OFF_T1     uint16_t
 * C3: Temperature pressure sensitivity coefficient  TCS        uint16_t
 * C4: Temperature coefficient pressure compensation TCO        uint16_t
 * C5: Reference temperature                         T_REF      uint16_t
 * C6: Temperature coefficient                       TEMPSENS   uint16_t
 * C7: CRC check
 *------------------------------------------------------------------------------
 * Read digital pressure and temperature data
 *
 * D1: Digital pressure value                                   uint32_t
 * D2: Digital temperature value                                uint32_t
 *------------------------------------------------------------------------------
 * Calculated temperature
 *
 * dT: The difference between actual temperature and reference temperature
 *                              dT=D2-C5*2^8                    int32_t
 * TEMP: Actual temperature     TEMP=2000+dT*C6/2^23            int32_t 2007(20.07℃)
 *------------------------------------------------------------------------------
 * Calculated temperature compensation pressure
 *
 * OFF: Actual temperature compensation
 *                              OFF=C2*2^16+(C4*dT)/2^7         int64_t
 * SENS: Sensitivity at actual temperature
 *                              SENS=C1*2^15+(C3*dT)/2^8        int64_t
 * P: Temperature compensation pressure
 *                              P=(D1*SENS/2^21-OFF)/2^15       int32_t 100009(1000.09mbar=100kPa)
 *------------------------------------------------------------------------------
 */

#define MS5611_CSB 0

#if MS5611_CSB == 0
#define MS5611_ADDR 0x77  // 0x 0111 0111	(The 7th bit of the address is the CSB complement.)
#else
#define MS5611_ADDR 0x76  // 0x 0111 0110
#endif
#define MS5611_WRITE ((MS5611_ADDR << 1))  // The last bit 0 is for writing and 1 is for reading
#define MS5611_READ ((MS5611_ADDR << 1) | 1)

/* The device returns a 24-bit result after an ADC read command
    and a 16-bit result after a PROM read command. */
#define MS5611_CMD_READ_ADC 0x00  // 0x 0000 0000
#define MS5611_CMD_RESET 0x1E     // 0x 0001 1110

#define MS5611_CMD_CONVERT_D1 0x40           // 0x 0100 0000	(OSR:Over Sampling Ratio)
#define MS5611_CMD_CONVERT_D1_OSR_256 0x40   // 0x 0100 0000
#define MS5611_CMD_CONVERT_D1_OSR_512 0x42   // 0x 0100 0010
#define MS5611_CMD_CONVERT_D1_OSR_1024 0x44  // 0x 0100 0100
#define MS5611_CMD_CONVERT_D1_OSR_2048 0x46  // 0x 0100 0110
#define MS5611_CMD_CONVERT_D1_OSR_4096 0x48  // 0x 0100 1000

#define MS5611_CMD_CONVERT_D2 0x50           // 0x 0101 0000
#define MS5611_CMD_CONVERT_D2_OSR_256 0x50   // 0x 0101 0000
#define MS5611_CMD_CONVERT_D2_OSR_512 0x52   // 0x 0101 0010
#define MS5611_CMD_CONVERT_D2_OSR_1024 0x54  // 0x 0101 0100
#define MS5611_CMD_CONVERT_D2_OSR_2048 0x56  // 0x 0101 0110
#define MS5611_CMD_CONVERT_D2_OSR_4096 0x58  // 0x 0101 1000 9.04 mSec conversion time ( 110.62 Hz)

#define MS5611_CMD_READ_PROM 0xA0  // 0x 1010 xxx0	(0xA0 ~ 0xAE)
#define MS5611_PROM_CRC 0xAE

typedef enum {
    MS5611_OK = 0x00U,
    MS5611_INTERFACE_DRIVER_ERROR = 0x01U,
    MS5611_INTERFACE_SYS_ERROR = 0x02U,
    MS5611_INIT_ERROR = 0x03U
} MS5611_StatusTypeDef;

typedef enum {
    MS5611_MODE_IIC = 0,
    MS5611_MODE_SPI = 1,
} MS5611_DRIVER_MODE;

typedef enum {
    MS5611_ONLY_PRESSURE = 0,
    MS5611_MIXED = 1,
} MS5611_SOLVE_MODE;

typedef enum {
    MS5611_OSR_ULTRA_LOW = 256,   //  1 ms
    MS5611_OSR_LOW = 512,         //  2 ms
    MS5611_OSR_STANDARD = 1024,   //  3 ms
    MS5611_OSR_HIGH = 2048,       //  5 ms
    MS5611_OSR_ULTRA_HIGH = 4096  // 10 ms
} MS5611_Osr;

typedef struct MS5611_TypeDef bsp_ms5611_t;
typedef struct MS5611_InitTypeDef bsp_ms5611_init_t;
typedef struct MS5611_DataTypeDef bsp_ms5611_data_t;

struct MS5611_InitTypeDef {
    MS5611_DRIVER_MODE driver_mode;
    MS5611_SOLVE_MODE solve_mode;
    MS5611_Osr overSamplingRate;
    void (*setCS)(bool state);
};

struct MS5611_DataTypeDef {
    uint16_t C[8];
    uint32_t D1;
    uint32_t D2;
    int32_t dT;
    int32_t TEMP;
    int64_t OFF;
    int64_t SENS;
    int32_t P;
    double pressure;
    double temperature;
    double pressureOffset;
    double temperatureOffset;
    double height;
    double relativeHeight;
    double H0;
    uint32_t id;
};

struct MS5611_TypeDef {
    // member function
    bool (*init)(bsp_ms5611_t* const ms5611x,
                 const bsp_ms5611_init_t* MS5611_InitStructure);
    bool (*reset)(bsp_ms5611_t* ms5611x);
    void (*read)(bsp_ms5611_t* ms5611x);
    double (*getHeight)(bsp_ms5611_t* ms5611x);
    void (*convert)(bsp_ms5611_t* ms5611x, uint8_t addr);
    bool (*checkCRC)(bsp_ms5611_t* ms5611x);
    void (*setTemperatureOffset)(bsp_ms5611_t* ms5611x, double offset);
    void (*setPressureOffset)(bsp_ms5611_t* ms5611x, double offset);

    // member variable
    bsp_ms5611_init_t config;
    bsp_ms5611_data_t data;
    FilterTypeDef Filter;
    // external interface
    interface_t interface;
};

uint8_t MS5611_Init(bsp_ms5611_t* const ms5611,
                    const bsp_ms5611_init_t* initStruct,
                    const interface_t* interface);
bsp_ms5611_t* MS5611_new(const bsp_ms5611_init_t* initStruct,
                         const interface_t* interface);
void MS5611_delete(bsp_ms5611_t* self);

extern bsp_ms5611_t ms5611;
extern bsp_ms5611_init_t ms5611_InitStructure;

#endif /* __BSP_MS5611_H */
