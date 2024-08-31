
/**
 * @file        bsp_ms5611.c
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
#include "bsp_ms5611.h"

/* function declaration */
// public funtion
static bool bsp_ms5611_init(bsp_ms5611_t* const ms5611x, const bsp_ms5611_init_t* initStruct);
static bool bsp_ms5611_reset(bsp_ms5611_t* ms5611x);
static void bsp_ms5611_read(bsp_ms5611_t* ms5611x);
static double bsp_ms5611_getHeight(bsp_ms5611_t* ms5611x);
static void bsp_ms5611_convert(bsp_ms5611_t* ms5611x, uint8_t addr);
static bool bsp_ms5611_checkCRC(bsp_ms5611_t* ms5611x);
static void bsp_ms5611_setTemperatureOffset(bsp_ms5611_t* ms5611x, double offset);
static void bsp_ms5611_setPressureOffset(bsp_ms5611_t* ms5611x, double offset);
// private funtion
static void bsp_ms5611_command(bsp_ms5611_t* ms5611x, uint8_t command);
static uint16_t bsp_ms5611_readProm(bsp_ms5611_t* ms5611x, uint8_t reg);
static uint32_t bsp_ms5611_readADC(bsp_ms5611_t* ms5611x);

/* function definition */
static bool
bsp_ms5611_init(bsp_ms5611_t* ms5611x, const bsp_ms5611_init_t* initStruct) {
    ms5611x->config = *initStruct;
    ms5611x->config.overSamplingRate = MS5611_OSR_STANDARD;
    switch (initStruct->driver_mode) {
    case MS5611_MODE_SPI:
        ms5611x->interface.driver.spi.setCS(1);
        break;
    case MS5611_MODE_IIC:
#if MS5611_CSB == 0
        ms5611x->config.setCS(0);
#else
        ms5611x->config.setCS(1);
#endif
        break;
    default:
        break;
    }
    if (!bsp_ms5611_reset(ms5611x)) return false;
    for (int i = 0; i < 100; i++) {
        bsp_ms5611_read(ms5611x);
        ms5611x->data.H0 += ms5611x->data.height;
    }
    ms5611x->data.H0 /= 100;
    return true;
}

static bool bsp_ms5611_reset(bsp_ms5611_t* ms5611x) {
    bsp_ms5611_command(ms5611x, MS5611_CMD_RESET);
    ms5611x->interface.sys.delay.ms(200);
    for (uint8_t reg = 0; reg < 8; ++reg) {
        uint16_t temp = bsp_ms5611_readProm(ms5611x, reg);
        ms5611x->data.C[reg] = temp;
        ms5611x->data.id <<= 4;
        ms5611x->data.id ^= temp;
    }
    return bsp_ms5611_checkCRC(ms5611x);
}

static void bsp_ms5611_read(bsp_ms5611_t* ms5611x) {
    bsp_ms5611_convert(ms5611x, MS5611_CMD_CONVERT_D1);
    ms5611x->data.D1 = bsp_ms5611_readADC(ms5611x);
    bsp_ms5611_convert(ms5611x, MS5611_CMD_CONVERT_D2);
    ms5611x->data.D2 = bsp_ms5611_readADC(ms5611x);

    ms5611x->data.dT = ms5611x->data.D2 - (((uint32_t)ms5611x->data.C[5]) << 8);
    ms5611x->data.TEMP = 2000 + (int64_t)ms5611x->data.dT * ms5611x->data.C[6] / 8388608;
    ms5611x->data.OFF = (int64_t)ms5611x->data.C[2] * 65536 + ms5611x->data.dT * (int64_t)ms5611x->data.C[4] / 128;
    ms5611x->data.SENS = (int64_t)ms5611x->data.C[1] * 32768 + ms5611x->data.dT * (int64_t)ms5611x->data.C[3] / 256;

    if (ms5611x->data.TEMP < 2000) {
        int32_t aux = (2000 - ms5611x->data.TEMP) * (2000 - ms5611x->data.TEMP);
        int32_t T2 = (int64_t)ms5611x->data.dT * ms5611x->data.dT / 2147483647;
        int64_t OFF2 = (5 * aux) / 2;
        int64_t SENS2 = (5 * aux) / 4;

        if (ms5611x->data.TEMP < -1500) {
            aux = (ms5611x->data.TEMP + 1500) * (ms5611x->data.TEMP + 1500);
            OFF2 += 7 * aux;
            SENS2 += (11 * aux) / 2;
        }
        ms5611x->data.TEMP -= T2;
        ms5611x->data.OFF -= OFF2;
        ms5611x->data.SENS -= SENS2;
    }
    ms5611x->data.P = (int32_t)((ms5611x->data.D1 * ms5611x->data.SENS / 2097152 - ms5611x->data.OFF) / 32768);

    ms5611x->data.temperature = (double)ms5611x->data.TEMP * 0.01 + ms5611x->data.temperatureOffset;
    ms5611x->data.pressure = (double)ms5611x->data.P * 0.01 + ms5611x->data.pressureOffset;
    bsp_ms5611_getHeight(ms5611x);
}

static double bsp_ms5611_getHeight(bsp_ms5611_t* ms5611x) {
    switch (ms5611x->config.solve_mode) {
    case MS5611_ONLY_PRESSURE:
        ms5611x->data.height = 44330.0 * (1.0 - pow(ms5611x->data.pressure / 1013.25, 1 / 5.255));  // barometric
        break;
    case MS5611_MIXED:
        ms5611x->data.height = (pow((1013.25 / ms5611x->data.pressure), 1.0 / 5.257) - 1) * (ms5611x->data.temperature + 273.15) / 0.65;  // hypsometric
        break;
    }
    ms5611x->data.relativeHeight = ms5611x->data.height - ms5611x->data.H0;
    ms5611x->Filter.LPF_50.input = ms5611x->data.relativeHeight;
    Butterworth50HzLPF(&(ms5611x->Filter.LPF_50));
    ms5611x->data.relativeHeight = ms5611x->Filter.LPF_50.input;
    return ms5611x->data.height;
}

static void bsp_ms5611_convert(bsp_ms5611_t* ms5611x, uint8_t addr) {
    // Relationship between ADC conversion time and oversampling rate
    uint16_t delay_time[5] = {600, 1200, 2300, 4600, 9100};
    uint8_t index = 0;
    switch (ms5611x->config.overSamplingRate) {
    case MS5611_OSR_ULTRA_LOW:
        index = 0;
        break;
    case MS5611_OSR_LOW:
        index = 1;
        break;
    case MS5611_OSR_STANDARD:
        index = 2;
        break;
    case MS5611_OSR_HIGH:
        index = 3;
        break;
    case MS5611_OSR_ULTRA_HIGH:
        index = 4;
        break;
    default:
        break;
    }
    uint8_t offset = index * 2;
    bsp_ms5611_command(ms5611x, addr + offset);
    ms5611x->interface.sys.delay.us(delay_time[index]);
}

static bool bsp_ms5611_checkCRC(bsp_ms5611_t* ms5611x) {
    uint32_t res = 0;
    uint8_t zero = 1;
    uint8_t crc = ms5611x->data.C[7] & 0xF;
    ms5611x->data.C[7] &= 0xFF00;

    for (uint8_t i = 0; i < 8; i++) {
        if (ms5611x->data.C[i] != 0)
            zero = 0;
    }
    if (zero) return false;
    for (uint8_t i = 0; i < 16; i++) {
        if (i & 1)
            res ^= ((ms5611x->data.C[i >> 1]) & 0x00FF);
        else
            res ^= (ms5611x->data.C[i >> 1] >> 8);
        for (uint8_t j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    ms5611x->data.C[7] |= crc;
    if (crc == ((res >> 12) & 0xF))
        return true;
    return false;
}

static void bsp_ms5611_setTemperatureOffset(bsp_ms5611_t* ms5611x, double offset) {
    ms5611x->data.temperatureOffset = offset;
}

static void bsp_ms5611_setPressureOffset(bsp_ms5611_t* ms5611x, double offset) {
    ms5611x->data.pressureOffset = offset;
}
/*********************************************************************************/
static void bsp_ms5611_command(bsp_ms5611_t* ms5611x, uint8_t command) {
    switch (ms5611x->config.driver_mode) {
    case MS5611_MODE_IIC:
        ms5611x->interface.driver.iic.writeByte(MS5611_ADDR, command);
        break;
    case MS5611_MODE_SPI:
        ms5611x->interface.driver.spi.setCS(0);
        ms5611x->interface.driver.spi.readWriteByte(command);
        ms5611x->interface.driver.spi.setCS(1);
        break;
    }
}

static uint16_t bsp_ms5611_readProm(bsp_ms5611_t* ms5611x, uint8_t reg) {
    uint8_t offset = reg * 2;
    uint16_t PROM_data = 0;
    uint8_t rxData[2] = {0};
    switch (ms5611x->config.driver_mode) {
    case MS5611_MODE_IIC:
        ms5611x->interface.driver.iic.writeByte(MS5611_ADDR, MS5611_CMD_READ_PROM + offset);
        ms5611x->interface.driver.iic.readBytes(MS5611_ADDR, rxData, 2);
        PROM_data |= *rxData;
        PROM_data <<= 8;
        PROM_data |= *(rxData + 1);
        break;
    case MS5611_MODE_SPI:
        ms5611x->interface.driver.spi.setCS(0);
        // At the first time, MISO returns 0xFE, only used for command transmission
        ms5611x->interface.driver.spi.readWriteByte(MS5611_CMD_READ_PROM + offset);
        PROM_data |= ms5611x->interface.driver.spi.readWriteByte(0x00);
        PROM_data <<= 8;
        PROM_data |= ms5611x->interface.driver.spi.readWriteByte(0x00);
        ms5611x->interface.driver.spi.setCS(1);
        break;
    }
    return PROM_data;
}

static uint32_t bsp_ms5611_readADC(bsp_ms5611_t* ms5611x) {
    uint32_t ADC_data = 0;
    uint8_t rxData[3] = {0};
    switch (ms5611x->config.driver_mode) {
    case MS5611_MODE_IIC:
        ms5611x->interface.driver.iic.writeByte(MS5611_ADDR, MS5611_CMD_READ_ADC);
        ms5611x->interface.driver.iic.readBytes(MS5611_ADDR, rxData, 3);
        ADC_data |= *rxData;
        ADC_data <<= 8;
        ADC_data |= *(rxData + 1);
        ADC_data <<= 8;
        ADC_data |= *(rxData + 2);
        break;
    case MS5611_MODE_SPI:
        ms5611x->interface.driver.spi.setCS(0);
        // At the first time, MISO returns 0xFE, only used for command transmission
        ms5611x->interface.driver.spi.readWriteByte(MS5611_CMD_READ_ADC);
        ADC_data |= ms5611x->interface.driver.spi.readWriteByte(0x00);
        ADC_data <<= 8;
        ADC_data |= ms5611x->interface.driver.spi.readWriteByte(0x00);
        ADC_data <<= 8;
        ADC_data |= ms5611x->interface.driver.spi.readWriteByte(0x00);
        ms5611x->interface.driver.spi.setCS(1);
        break;
    }
    return ADC_data;
}

/******************************************************************************/
bsp_ms5611_t*
MS5611_new(const bsp_ms5611_init_t* initStruct,
           const interface_t* interface) {
    bsp_ms5611_t* self = (bsp_ms5611_t*)calloc(1, sizeof(bsp_ms5611_t));
    if (self != NULL) {
        if (MS5611_Init(self, initStruct, interface) != MS5611_OK) {
            MS5611_delete(self);
            return NULL;
        }
    }
    return self;
}

void MS5611_delete(bsp_ms5611_t* self) {
    if (self != NULL) {
        free(self);
    }
}

uint8_t MS5611_Init(bsp_ms5611_t* const ms5611,
                    const bsp_ms5611_init_t* initStruct,
                    const interface_t* interface) {
    bsp_ms5611_t* self = ms5611;

    self->init = bsp_ms5611_init;
    self->reset = bsp_ms5611_reset;
    self->read = bsp_ms5611_read;
    self->getHeight = bsp_ms5611_getHeight;
    self->convert = bsp_ms5611_convert;
    self->checkCRC = bsp_ms5611_checkCRC;
    self->setTemperatureOffset = bsp_ms5611_setTemperatureOffset;
    self->setPressureOffset = bsp_ms5611_setPressureOffset;

    interface_t temp = *interface;

    if (initStruct->driver_mode == MS5611_MODE_IIC) {
        if (temp.driver.iic.writeByte != NULL &&
            temp.driver.iic.readByte != NULL &&
            temp.driver.iic.readBytes != NULL) {
            self->interface.driver.iic = temp.driver.iic;
        } else {
            return MS5611_INTERFACE_DRIVER_ERROR;
        }
    } else if (initStruct->driver_mode == MS5611_MODE_SPI) {
        if (temp.driver.spi.readWriteByte != NULL &&
            temp.driver.spi.setCS != NULL) {
            self->interface.driver.spi = temp.driver.spi;
        } else {
            return MS5611_INTERFACE_DRIVER_ERROR;
        }
    }
    if (temp.sys.delay.ms != NULL && temp.sys.delay.us != NULL) {
        self->interface.sys.delay = temp.sys.delay;
    } else {
        return MS5611_INTERFACE_SYS_ERROR;
    }

    if (!bsp_ms5611_init(self, initStruct)) return MS5611_INIT_ERROR;
    return MS5611_OK;
}
/******************************************************************************/
/* User Defined */
bsp_ms5611_t ms5611;
/*
 *  PB6     ------> I2C1_SCL
 *  PB7     ------> I2C1_SDA
 *  PA5     ------> SPI1_SCK
 *  PA6     ------> SPI1_MISO
 *  PA7     ------> SPI1_MOSI
 */
bsp_ms5611_init_t ms5611_InitStructure = {
    .driver_mode = MS5611_MODE_IIC,
    .solve_mode = MS5611_ONLY_PRESSURE,
    .overSamplingRate = MS5611_OSR_ULTRA_HIGH,
    .setCS = bsp_SPI_SetCS};
