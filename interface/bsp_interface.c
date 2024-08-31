/**
 * @file        bsp_interface.c
 * @author      sephony
 * @date        2024-09-01
 * @version     1.0.0
 * @brief       Bsp Library for abstract interface
 * @details     Include common communication protocols, some system functions
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
 * -----------------------------------------------------------------------------
 */
#include "bsp_interface.h"

#ifndef USER_DEFINED_INTERFACE
#if defined(USE_HAL_DRIVER)
void bsp_IIC_WriteByte(uint16_t address, uint8_t command) {
    if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)address << 1, &command, 1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }
}

uint8_t bsp_IIC_ReadByte(uint16_t address) {
    uint8_t rxData;
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)address << 1, &rxData, 1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }
    return rxData;
}

void bsp_IIC_ReadBytes(uint16_t address, uint8_t* rxData, uint16_t size) {
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)address << 1, rxData, size, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }
}

uint8_t bsp_SPI_ReadWriteByte(uint8_t txData) {
    uint8_t rxData;
    if (HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }
    return rxData;
}

void bsp_SPI_SetCS(bool state) {
    HAL_GPIO_WritePin(CS_MS5611_1_GPIO_Port, CS_MS5611_1_Pin, state);
}

void bsp_delay_init(void) {
}
#ifdef __DELAY_H
void bsp_delay_us(uint32_t nus) {
    delay_us(nus);
}

void bsp_delay_ms(uint32_t nms) {
    delay_ms(nms);
}
#else
void bsp_delay_us(uint32_t nus) {
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; /* LOAD的值，一般为1ms的重载值 */
    ticks = nus * g_fac_us;          /* 需要的节拍数（重载值） */

#if SYS_SUPPORT_OS /* 如果需要支持OS */
    delay_osschedlock(); /* 锁定 OS 的任务调度器 */
#endif

    told = SysTick->VAL; /* 刚进入时的计数器值 */
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow; /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break; /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }
}

void bsp_delay_ms(uint32_t nms) {
    HAL_Delay(nms);
}
#endif
/* 需要的节拍数（重载值） */
#elif defined(USE_STANDARD_DRIVER)

#else
void bsp_IIC_WriteByte(uint16_t address, uint8_t command) {
}

uint8_t bsp_IIC_ReadByte(uint16_t address) {
}

void bsp_IIC_ReadBytes(uint16_t address, uint8_t* rxData, uint16_t size) {
}

uint8_t bsp_SPI_ReadWriteByte(uint8_t txData) {
}

void bsp_SPI_SetCS(bool state) {
}

void bsp_delay_init(void) {
}

void bsp_delay_us(uint32_t nus) {
}

void bsp_delay_ms(uint32_t nms) {
}

#endif
#else
void bsp_IIC_WriteByte(uint16_t address, uint8_t command) {
    IIC_PinTypeDef iic = {
        .SCL = {GPIOB, GPIO_PIN_12},
        .SDA = {GPIOB, GPIO_PIN_13},
    };
    IIC_Start(&iic);
    IIC_Send_Byte(&iic, (address << 1));
    IIC_Wait_Ack(&iic);
    IIC_Send_Byte(&iic, command);
    IIC_Wait_Ack(&iic);
    IIC_Stop(&iic);
}

uint8_t bsp_IIC_ReadByte(uint16_t address) {
    IIC_PinTypeDef iic = {
        .SCL = {GPIOB, GPIO_PIN_12},
        .SDA = {GPIOB, GPIO_PIN_13},
    };
    uint8_t dataBuffer = 0;
    IIC_Start(&iic);
    IIC_Send_Byte(&iic, ((address << 1) | 1));
    IIC_Wait_Ack(&iic);
    dataBuffer |= IIC_Read_Byte(&iic, 0);
    IIC_Stop(&iic);
    return dataBuffer;
}

void bsp_IIC_ReadBytes(uint16_t address, uint8_t* rxData, uint16_t size) {
    IIC_PinTypeDef iic = {
        .SCL = {GPIOB, GPIO_PIN_12},
        .SDA = {GPIOB, GPIO_PIN_13},
    };
    IIC_Start(&iic);
    IIC_Send_Byte(&iic, ((address << 1) | 1));
    IIC_Wait_Ack(&iic);
    while (size) {
        *rxData = IIC_Read_Byte(&iic, size == 1 ? 0 : 1);

        size--;
        rxData++;
    }
    IIC_Stop(&iic);
}

uint8_t bsp_SPI_ReadWriteByte(uint8_t txData) {
}

void bsp_SPI_SetCS(bool state) {
    PIN_OUT({GPIOB, GPIO_PIN_14}) = state;
}

void bsp_delay_init(void) {
}

void bsp_delay_us(uint32_t nus) {
}

void bsp_delay_ms(uint32_t nms) {
}
#endif

iic_driver_interface_t iic_driver = {
    // .init = NULL,
    // .deInit = NULL,
    .writeByte = bsp_IIC_WriteByte,
    .readBytes = bsp_IIC_ReadBytes};

spi_driver_interface_t spi_driver = {
    // .init = NULL,
    // .deInit = NULL,
    .readWriteByte = bsp_SPI_ReadWriteByte,
    .setCS = bsp_SPI_SetCS};

delay_function_t delay_func = {
    // .init = NULL,
    .us = bsp_delay_us,
    .ms = bsp_delay_ms};

interface_t bsp_interface = {
    .driver = {
        .iic = {
            // .init = NULL,
            // .deInit = NULL,
            .writeByte = bsp_IIC_WriteByte,
            .readByte = bsp_IIC_ReadByte,
            .readBytes = bsp_IIC_ReadBytes},
        .spi = {// .init = NULL,
                // .deInit = NULL,
                .readWriteByte = bsp_SPI_ReadWriteByte,
                .setCS = bsp_SPI_SetCS}

    },

    .sys = {.delay = {// .init = NULL,
                      .us = bsp_delay_us,
                      .ms = bsp_delay_ms}}

};
