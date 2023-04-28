/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2023 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "spi.h"
#include "spi_bus.h"

#include <libopencm3/stm32/rcc.h>

void bmp_spi_init(const spi_bus_e bus)
{
	if (bus == SPI_BUS_EXTERNAL) {
		rcc_periph_clock_enable(RCC_SPI2);
		rcc_periph_reset_pulse(RST_SPI2);
	} else {
		rcc_periph_clock_enable(RCC_SPI1);
		rcc_periph_reset_pulse(RST_SPI1);
	}

	const uint32_t controller = bus == SPI_BUS_EXTERNAL ? EXT_SPI : AUX_SPI;
	spi_init_master(controller, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_enable(controller);
}

void bmp_spi_deinit(const spi_bus_e bus)
{
	const uint32_t controller = bus == SPI_BUS_EXTERNAL ? EXT_SPI : AUX_SPI;
	spi_disable(controller);

	if (bus == SPI_BUS_EXTERNAL)
		rcc_periph_clock_disable(RCC_SPI2);
	else
		rcc_periph_clock_disable(RCC_SPI1);
}

void bmp_spi_chip_select(const spi_device_e device, const bool select)
{
	switch (device) {
	case SPI_DEVICE_INT_FLASH:
		ispi_flash_select(select);
		break;
	case SPI_DEVICE_EXT_FLASH:
		espi_select(select);
		break;
	case SPI_DEVICE_SDCARD:
		ispi_sdcard_select(select);
		break;
	case SPI_DEVICE_DISPLAY:
		ispi_display_select(select);
		break;
	}
}

uint8_t bmp_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
	if (bus == SPI_BUS_EXTERNAL)
		return espi_xfer(value);
	return ispi_xfer(value);
}
