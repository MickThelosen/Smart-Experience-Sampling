//
// Created by marijn on 11/4/24.
//

#include "SPI.h"


/*!<
 * usage
 * */
uint32_t SPI_write8(SPI_HandleTypeDef* spi, const uint8_t* buffer, uint32_t size, uint32_t timeout) {
	spi->Instance->CR1 |= 0x00000040UL;
	uint64_t start = uwTick;
	uint32_t i = 0;
	for (; i < size; i++) {
		while (!(spi->Instance->SR & 0x00000002UL))	{ if ( uwTick - start > timeout) { goto SPI_master_write8_end; } }
		*((volatile uint8_t*)&spi->Instance->DR) = buffer[i];
	}
	while (spi->Instance->SR & 0x00000080UL)			{ if ( uwTick - start > timeout) { goto SPI_master_write8_end; } }
	i++; SPI_master_write8_end:
	spi->Instance->CR1 &= ~0x00000040UL;
	return i;
}

uint32_t SPI_read8(SPI_HandleTypeDef* spi, uint8_t* buffer, uint32_t size, uint32_t timeout) {
	spi->Instance->CR1 |= 0x00000040UL;
	uint64_t start = uwTick;
	uint32_t i = 0;

	while (spi->Instance->SR & 0x00000001UL) {
		(void)(volatile uint8_t)spi->Instance->DR;  // flush buffer
	}

	for (; i < size; i++) {
		while (!(spi->Instance->SR & 0x00000002UL))	{ if ( uwTick - start > timeout) { goto SPI_master_read8_end; } }
		*((volatile uint8_t*)&spi->Instance->DR) = 0;
		while (!(spi->Instance->SR & 0x00000001UL))	{ if ( uwTick - start > timeout) { goto SPI_master_read8_end; } }
		buffer[i] = (volatile uint8_t)spi->Instance->DR;
	}
	while (spi->Instance->SR & 0x00000080UL)			{ if ( uwTick - start > timeout) { goto SPI_master_read8_end; } }
	i++; SPI_master_read8_end:
	spi->Instance->CR1 &= ~0x00000040UL;
	return i;
}

// TODO: 22.5.8
