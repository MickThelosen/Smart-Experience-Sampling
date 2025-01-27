//
// Created by marijn on 9/25/24.
//

#ifndef STM32WB07_SPI_H
#define STM32WB07_SPI_H

#include "stm32wb07.h"
#include "stm32wb0x_hal.h"

uint32_t SPI_write8(SPI_HandleTypeDef* spi, const uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t SPI_read8(SPI_HandleTypeDef* spi, uint8_t* buffer, uint32_t size, uint32_t timeout);

#endif //STM32WB07_SPI_H
