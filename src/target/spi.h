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

#ifndef TARGET_SPI_H
#define TARGET_SPI_H

#define SPI_FLASH_OPCODE_MASK      0x00ffU
#define SPI_FLASH_OPCODE(x)        ((x)&SPI_FLASH_OPCODE_MASK)
#define SPI_FLASH_DUMMY_MASK       0x0700U
#define SPI_FLASH_DUMMY_SHIFT      8U
#define SPI_FLASH_DUMMY_LEN(x)     (((x) << SPI_FLASH_DUMMY_SHIFT) & SPI_FLASH_DUMMY_MASK)
#define SPI_FLASH_OPCODE_MODE_MASK 0x0800U
#define SPI_FLASH_OPCODE_ONLY      (0U << 11U)
#define SPI_FLASH_OPCODE_3B_ADDR   (1U << 11U)
#define SPI_FLASH_DATA_MASK        0x1000U
#define SPI_FLASH_DATA_IN          (0U << 12U)
#define SPI_FLASH_DATA_OUT         (1U << 12U)

#define SPI_FLASH_OPCODE_SECTOR_ERASE 0x20U
#define SPI_FLASH_CMD_WRITE_ENABLE    (SPI_FLASH_OPCODE_ONLY | SPI_FLASH_DUMMY_LEN(0) | SPI_FLASH_OPCODE(0x06U))
#define SPI_FLASH_CMD_PAGE_PROGRAM \
	(SPI_FLASH_OPCODE_3B_ADDR | SPI_FLASH_DATA_OUT | SPI_FLASH_DUMMY_LEN(0) | SPI_FLASH_OPCODE(0x02))
#define SPI_FLASH_CMD_SECTOR_ERASE (SPI_FLASH_OPCODE_3B_ADDR | SPI_FLASH_DUMMY_LEN(0))
#define SPI_FLASH_CMD_CHIP_ERASE   (SPI_FLASH_OPCODE_ONLY | SPI_FLASH_DUMMY_LEN(0) | SPI_FLASH_OPCODE(0x60U))
#define SPI_FLASH_CMD_READ_STATUS \
	(SPI_FLASH_OPCODE_ONLY | SPI_FLASH_DATA_IN | SPI_FLASH_DUMMY_LEN(0) | SPI_FLASH_OPCODE(0x05U))
#define SPI_FLASH_CMD_READ_JEDEC_ID \
	(SPI_FLASH_OPCODE_ONLY | SPI_FLASH_DATA_IN | SPI_FLASH_DUMMY_LEN(0) | SPI_FLASH_OPCODE(0x9fU))
#define SPI_FLASH_CMD_READ_SFDP \
	(SPI_FLASH_OPCODE_3B_ADDR | SPI_FLASH_DATA_IN | SPI_FLASH_DUMMY_LEN(8) | SPI_FLASH_OPCODE(0x5aU))

#define SPI_FLASH_STATUS_BUSY          0x01U
#define SPI_FLASH_STATUS_WRITE_ENABLED 0x02U

#endif /* TARGET_SPI_H */
