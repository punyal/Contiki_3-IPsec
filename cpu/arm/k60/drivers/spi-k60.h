/*
 * Copyright (c) 2014, Eistec AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Mulle platform port of the Contiki operating system.
 *
 */

/**
 * \file
 *         SPI driver for Kinetis K60.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#ifndef CPU_ARM_K60_SPI_H_
#define CPU_ARM_K60_SPI_H_

#include <stddef.h>
#include "K60.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Number of CTAR registers in SPI module, see K60 Reference Manual for details. */
#define NUM_CTAR 2

typedef enum spi_ctar {
  SPI_CTAR_0 = 0,
  SPI_CTAR_1 = 1,
} spi_ctar_t;

typedef enum spi_bus {
  SPI_0 = 0,
  SPI_1 = 1,
  SPI_2 = 2
} spi_bus_t;

/**
 * SPI bus configuration structure.
 */
typedef struct spi_config {
  uint32_t sck_freq; /**< desired SCK frequency */
  uint32_t tcsc_freq; /**< reciprocal (i.e. frequency, 1/t) of tCSC, Time between CS line assertion and first SCK flank */
  uint32_t tasc_freq; /**< reciprocal of tASC, Time between last SCK flank and CS line deassertion */
  uint32_t tdt_freq; /**< reciprocal of tDT, minimum time between CS line deassertion and CS line re-assertion for the next frame */
  uint8_t frame_size; /**< Frame size, in bits */
  uint8_t cpol; /**< CPOL, SCK SPI clock polarity */
  uint8_t cpha; /**< CPHA, SCK SPI clock phase */
} spi_config_t;

typedef enum spi_transfer_flag {
  SPI_TRANSFER_DONE = 0,
  SPI_TRANSFER_CONT = 1
} spi_transfer_flag_t;

/**
 * Request exclusive access to the bus.
 *
 * This is used to ensure that different drivers do not attempt to use the bus
 * at the same time.
 *
 * \note This function will block until the bus is free.
 *
 * \param spi_num The SPI module number.
 */
void spi_acquire_bus(const spi_bus_t spi_num);

/**
 * Release exclusive access of the bus.
 *
 * \param spi_num The SPI module number.
 */
void spi_release_bus(const spi_bus_t spi_num);

/**
 * Test if the bus lock is currently being held by someone.
 *
 * \param spi_num The SPI module number.
 */
int spi_is_busy(const spi_bus_t spi_num);

/**
 * Initialize the SPI hardware module.
 *
 * \param spi_num The SPI module number.
 */
void spi_hw_init_master(const spi_bus_t spi_num);

/**
 * Perform a series of writes followed by a series of reads to/from the SPI bus.
 *
 * \param spi_num SPI module number.
 * \param ctas The CTAR register to use for timing information (0 or 1)
 * \param cs The chip select pins to assert. Bitmask, not PCS number.
 * \param cont Whether to keep asserting the chip select pin after the series of transfers ends.
 * \param data_out Pointer to data to send.
 * \param data_in Pointer to store received data.
 * \param count_out Number of bytes to write.
 * \param count_in Number of bytes to read.
 */
int spi_transfer_blocking(const spi_bus_t spi_num, const spi_ctar_t ctas, const uint32_t cs,
             const spi_transfer_flag_t cont, const uint8_t *data_out,
             uint8_t *data_in, size_t count_out, size_t count_in);

/**
 * Enable the SPI hardware module (enable clock gate).
 *
 * \param spi_num The SPI module number.
 */
void spi_start(const spi_bus_t spi_num);

/**
 * Disable the SPI hardware module (disable clock gate).
 *
 * \param spi_num The SPI module number.
 */
void spi_stop(const spi_bus_t spi_num);

/**
 * Set transfer parameters for the bus.
 *
 * This will update the appropriate CTAR register with proper scaler and
 * prescaler values based on the current clock settings.
 *
 * Remember to call this every time the bus clock changes.
 *
 * Never call this function while the SPI bus has an active transfer going.
 *
 * The struct pointed to by config is expected to remain a valid configuration
 * in order for spi_refresh_params to work properly.
 *
 * \param spi_num The SPI module number.
 * \param ctas Index of the chosen CTAR register.
 * \param config Pointer to bus configuration information.
 */
void spi_set_params(const spi_bus_t spi_num, const spi_ctar_t ctas, const spi_config_t* config);

/**
 * Refresh the latest configuration for the SPI bus.
 *
 * This should be run after changing the bus clock frequency in order to set new
 * prescaler and scaler settings in the SPI hardware module.
 */
void spi_refresh_params(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CPU_ARM_K60_SPI_H_ */
