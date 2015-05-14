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
 *         Driver for the DS18B20 temperature sensor.
 *
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 *
 */

#include "onewire.h"
#include "ds18b20.h"
#include "stdbool.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) PRINTF(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/**
 * Initialize the DS18B20 driver.
 *
 * Not much to do here yet.
 */
void
ds18b20_init(void)
{
}
/**
 * Tell a DS18B20 sensor to initiate a temperature conversion.
 *
 * \param id The ROM code of the sensor, 0 for SKIP ROM.
 *
 * \note Parasite power is not supported yet!
 */
void
ds18b20_convert_temperature(const ow_rom_code_t id)
{
  static const ds18b20_cmd_t cmd = DS18B20_CONVERT_TEMPERATURE;
  uint8_t status = 0;

  ow_skip_or_match_rom(id);
  ow_write_bytes((const uint8_t *)&cmd, 1);
  /* perform a read of the conversion status in order to make sure the write
   * has finished before we leave this function. */
  /* The sensor will ignore the command if STOP mode is entered before the UART
   * has finished writing the 1-wire command. */
  ow_read_bytes(&status, 1);
}
/**
 * Read the scratchpad of a DS18B20 sensor.
 *
 * \param id The ROM code of the sensor, 0 for SKIP ROM.
 * \param dest A destination buffer, must be at least 9 bytes long.
 *
 * \return 0 if the CRC is correct, non-zero otherwise.
 */
uint8_t
ds18b20_read_scratchpad(const ow_rom_code_t id, ds18b20_scratchpad_t *dest)
{
  static const ds18b20_cmd_t cmd = DS18B20_READ_SCRATCHPAD;
  int16_t buf;
  int i;

  ow_skip_or_match_rom(id);
  ow_write_bytes((const uint8_t *)&cmd, 1);
  ow_read_bytes((uint8_t *)dest, DS18B20_SCRATCHPAD_SIZE);
  PRINTF("Scratchpad: ");
  for(i = 0; i < DS18B20_SCRATCHPAD_SIZE; ++i) {
    PRINTF("%02x", ((uint8_t *)dest)[i]);
  }
  PRINTF("\n");
  PRINTF("CRC: %x (should be %x)\n", dest->crc, ow_compute_crc((uint8_t *)dest, DS18B20_SCRATCHPAD_SIZE - 1));
  buf = (dest->temp_msb << 8) | dest->temp_lsb;
  /* ds18b20: */
  //~ PRINTF("Temp (celsius): %d.%d\n", (buf >> 4), (buf & 0x0f) * 625);
  /* ds1820, ds18s20: */
  PRINTF("Temp (celsius): %d.%d\n", (buf >> 1), (buf & 0x01) * 5);
  return ow_compute_crc((uint8_t *)dest, DS18B20_SCRATCHPAD_SIZE);
}

/**
 * Parse the scratchpad contents of a DS18B20 sensor and return the degrees as float.
 *
 * \param scratch Scratchpad contents, use ds18b20_read_scratchpad() to fill.
 *
 * \note Check the return value of ds18b20_read_scratchpad for error handling,
 *       this function only does some simple arithmetic on the given values.
 *
 * \return a float temperature in celsius.
 */
float ds18b20_parse_scratchpad_float(const ds18b20_scratchpad_t *scratch)
{
  int16_t buf;
  float ret = 0.0;
  buf = (scratch->temp_msb << 8) | scratch->temp_lsb;
  PRINTF("Temp (celsius): %d.%d\n", (buf >> 4), (buf & 0x0f)*625);

  /* float transformation */
  ret = buf / 16.0f;

  return ret;
}

/**
 * Parse the scratchpad of a DS18S20 or DS1820 sensor and return the degrees as float.
 *
 * \param scratch Scratchpad contents, use ds18b20_read_scratchpad() to fill.
 *
 * \note Check the return value of ds18b20_read_scratchpad for error handling,
 *       this function only does some simple arithmetic on the given values.
 *
 * \return a float temperature in celsius.
 */
float ds18s20_parse_scratchpad_float(const ds18b20_scratchpad_t *scratch)
{
  /* Increase resolution by using extra information, as per DS1820 data sheet. */
  int16_t buf;
  float ret = 0.0;
  buf = (scratch->temp_msb << 8) | scratch->temp_lsb;

  /* Truncate lsbit */
  buf &= ~(0x01);
  buf <<= 3;
  /* Now at same scale as ds18b20 (but not same accuracy!) */
  /* Increase accuracy by the method described in the data sheet. */
  ret = (16 * (int16_t)(scratch->count_per_c - scratch->count_remain));
  ret /= scratch->count_per_c;
  ret += buf / 4;

  PRINTF("DS1820 Temp (celsius): %d.%d\n", (buf >> 4), (buf & 0x0f) * 625);

  /* float transformation */
  ret = buf / 16.0f;

  return ret;
}
