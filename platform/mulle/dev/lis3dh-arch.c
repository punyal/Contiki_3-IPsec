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
 *         Platform specific functions for LIS3DH accelerometer on the Mulle platform.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "lis3dh.h"
#include "K60.h"
#include "config-board.h"
#include "interrupt.h"
#include "power-control.h"
#include "spi-k60.h"

/* Convenience macro */
#define LIS3DH_CHIP_SELECT_PIN_MASK (1 << LIS3DH_CHIP_SELECT_PIN)

/**
 * Write a single byte to the LIS3DH.
 *
 * \param addr The target register.
 * \param value The value to write.
 */
void
lis3dh_write_byte(const lis3dh_reg_addr_t addr, const uint8_t value)
{
  uint8_t data_out[2];

  /* Address field */
  data_out[0] = (addr & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_WRITE_MASK | LIS3DH_SPI_SINGLE_MASK;
  /* value field */
  data_out[1] = value;

  /* Acquire exclusive access to the bus. */
  spi_acquire_bus(LIS3DH_SPI_NUM);
  /* Perform the transaction */
  spi_transfer_blocking(LIS3DH_SPI_NUM, LIS3DH_CTAS, LIS3DH_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, &data_out[0], NULL, sizeof(data_out)/sizeof(data_out[0]), 0);
  /* Release the bus for other threads. */
  spi_release_bus(LIS3DH_SPI_NUM);
}
/**
 * Read a single byte from the LIS3DH.
 *
 * \param addr The source register.
 * \return The value of the register.
 */
uint8_t
lis3dh_read_byte(const lis3dh_reg_addr_t addr)
{
  uint8_t data_out = (addr & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_READ_MASK | LIS3DH_SPI_SINGLE_MASK;
  uint8_t data_in;

  /* Acquire exclusive access to the bus. */
  spi_acquire_bus(LIS3DH_SPI_NUM);
  /* Perform the transaction */
  spi_transfer_blocking(LIS3DH_SPI_NUM, LIS3DH_CTAS, LIS3DH_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, &data_out, &data_in, 1, 1);
  /* Release the bus for other threads. */
  spi_release_bus(LIS3DH_SPI_NUM);

  return data_in;
}

/**
 * Read a 16-bit integer from the LIS3DH.
 *
 * \param lsb_addr The lower address of the two source registers.
 * \return The value of the register, byte order depends on the big/little endian setting of the LIS3DH.
 * \note The BLE bit of CTRL_REG4 will affect the byte order of the return value.
 */
int16_t
lis3dh_read_int16(const lis3dh_reg_addr_t lsb_addr)
{
  /* Address field */
  uint8_t data_out = (lsb_addr & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_READ_MASK | LIS3DH_SPI_MULTI_MASK;
  /* value field */
  uint8_t data_in[2];

  int16_t ret;

  /*
   * We will do a multi byte read from the LIS3DH.
   * After the first read the address will be automatically increased by one.
   */

  /* Acquire exclusive access to the bus. */
  spi_acquire_bus(LIS3DH_SPI_NUM);
  /* Perform the transaction */
  spi_transfer_blocking(LIS3DH_SPI_NUM, LIS3DH_CTAS, LIS3DH_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, &data_out, &data_in[0], 1, 2);
  /* Release the bus for other threads. */
  spi_release_bus(LIS3DH_SPI_NUM);

  ret = (int16_t)((data_in[1] << 8) | data_in[0]);

  return ret;
}
/**
 * Read multiple bytes from the LIS3DH.
 *
 * \param start_address The lower address of the source registers.
 * \param buffer A buffer to write the read values into.
 * \param count Number of bytes to read, must not be greater than 31.
 */
void
lis3dh_memcpy_from_device(const lis3dh_reg_addr_t start_address,
                          uint8_t *buffer, uint8_t count)
{
  /* Address field */
  uint8_t data_out = (start_address & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_READ_MASK | LIS3DH_SPI_MULTI_MASK;

  /* Acquire exclusive access to the bus. */
  spi_acquire_bus(LIS3DH_SPI_NUM);
  /* Perform the transaction */
  spi_transfer_blocking(LIS3DH_SPI_NUM, LIS3DH_CTAS, LIS3DH_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, &data_out, buffer, 1, count);
  /* Release the bus for other threads. */
  spi_release_bus(LIS3DH_SPI_NUM);
}
/**
 * Write multiple bytes to the LIS3DH.
 *
 * \param start_address The lower address of the target registers.
 * \param buffer A buffer to read the values from.
 * \param count Number of bytes to write.
 */
void
lis3dh_memcpy_to_device(const lis3dh_reg_addr_t start_address,
                        const uint8_t *buffer, uint8_t count)
{
  /* Address field */
  uint8_t data_out = (start_address & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_WRITE_MASK | LIS3DH_SPI_MULTI_MASK;

  /* Acquire exclusive access to the bus. */
  spi_acquire_bus(LIS3DH_SPI_NUM);
  /* Send the address */
  spi_transfer_blocking(LIS3DH_SPI_NUM, LIS3DH_CTAS, LIS3DH_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_CONT, &data_out, NULL, 1, 0);
  /* Send the data */
  spi_transfer_blocking(LIS3DH_SPI_NUM, LIS3DH_CTAS, LIS3DH_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, buffer, NULL, count, 0);
  /* Release the bus for other threads. */
  spi_release_bus(LIS3DH_SPI_NUM);
}
