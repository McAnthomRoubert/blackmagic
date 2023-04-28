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

#ifndef PLATFORMS_STM32_SPI_BUS_H
#define PLATFORMS_STM32_SPI_BUS_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/spi.h>

#include "platform.h"
#include "gpio.h"

/* Functions prefixed ispi are for the internal SPI bus and espi are for the external SPI bus. */

static inline void ispi_flash_select(const bool value)
{
	gpio_set_val(AUX_PORT, AUX_FCS, !value);
}

static inline void ispi_sdcard_select(const bool value)
{
	gpio_set_val(AUX_PORT, AUX_SDCS, !value);
}

static inline void ispi_display_select(const bool value)
{
	gpio_set_val(AUX_PORT, AUX_DCS, !value);
}

static inline uint8_t ispi_xfer(const uint8_t value)
{
	return spi_xfer(AUX_SPI, value);
}

static inline void espi_select(const bool value)
{
	gpio_set_val(EXT_SPI_CS_PORT, EXT_SPI_CS, !value);
}

static inline uint8_t espi_xfer(const uint8_t value)
{
	return spi_xfer(EXT_SPI, value);
}

#endif /* PLATFORMS_STM32_SPI_BUS_H */
