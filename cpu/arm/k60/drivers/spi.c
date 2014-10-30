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
 *         SPI bus driver for Kinetis K60.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#include <stdint.h>
#include "spi-k60.h"
#include "K60.h"
#include "synchronization.h"
#include "power-modes.h"

#define SPI_IDLE_DATA (0xffff)

/* one config per CTAR instance */
/* This is an array of pointers to the current SPI bus configurations. The saved
 * pointers are used to be able to update the prescaler and scaler settings
 * whenever the bus clock frequency changes. */
static const spi_config_t *spi_conf[NUM_SPI][NUM_CTAR] = {{NULL}};

static lock_t spi_lock[NUM_SPI] = {2, 2, 2}; /* Block access by default until spi_hw_init_master has been run */

static volatile uint8_t spi_waiting_flag[NUM_SPI] = {0};

/**
 * Find the prescaler and scaler settings that will yield a delay timing
 * as close as possible (but not shorter than) the target delay, given the
 * module runs at module_clock Hz.
 *
 * Hardware properties (delay configuration):
 * Possible prescalers: 1, 3, 5, 7
 * Possible scalers: 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536
 *
 * delay = (1/f_BUS) x prescaler x scaler
 *
 * Because we want to do this using only integers, the target_freq parameter is
 * the reciprocal of the delay time.
 */
static int find_closest_delay_scalers(const uint32_t module_clock, const uint32_t target_freq, uint8_t *closest_prescaler, uint8_t *closest_scaler) {
  uint8_t i;
  uint8_t k;
  int freq;
  int prescaler;
  int scaler;
  static const uint8_t num_scalers = 16;
  static const uint8_t num_prescalers = 4;

  int closest_frequency = -1;

  /* Test all combinations until we arrive close to the target clock */
  for (i = 0; i < num_prescalers; ++i)
  {
    for (k = 0; k < num_scalers; ++k)
    {
      prescaler = (i * 2) + 1;
      scaler = (1 << (k + 1)); /* 2^(k+1) */
      freq = module_clock / (prescaler * scaler);
      if (freq <= target_freq)
      {
        /* Found closest lower frequency at this prescaler setting,
         * compare to the best result */
        if (closest_frequency < freq)
        {
          closest_frequency = freq;
          *closest_scaler = k;
          *closest_prescaler = i;
        }
        break;
      }
    }
  }
  if (closest_frequency < 0)
  {
    /* Error, no solution found, this line is never reachable with current
     * hardware settings unless a _very_ low target clock is requested.
     * (scaler_max * prescaler_max) = 458752 */
    return -1;
  }

  return 0;
}


/**
 * Find the prescaler and scaler settings that will yield a clock frequency
 * as close as possible (but not above) the target frequency, given the module
 * runs at module_clock Hz.
 *
 * Hardware properties (Baud rate configuration):
 * Possible prescalers: 2, 3, 5, 7
 * Possible scalers: 2, 4, 6 (sic!), 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768
 *
 * SCK baud rate = (f_BUS/PBR) x [(1+DBR)/BR]
 *
 * where PBR is the prescaler, BR is the scaler, DBR is the Double BaudRate bit.
 *
 * \note We are not using the DBR bit because it may affect the SCK duty cycle.
 */
static int find_closest_baudrate_scalers(const uint32_t module_clock, const uint32_t target_clock, uint8_t *closest_prescaler, uint8_t *closest_scaler)
{
  uint8_t i;
  uint8_t k;
  int freq;
  static const uint8_t num_scalers = 16;
  static const uint8_t num_prescalers = 4;
  static const int br_scalers[16] = {
        2,     4,     6,     8,    16,    32,    64,   128,
      256,   512,  1024,  2048,  4096,  8192, 16384, 32768
  };
  static const int br_prescalers[4] = {2, 3, 5, 7};

  int closest_frequency = -1;

  /* Test all combinations until we arrive close to the target clock */
  for (i = 0; i < num_prescalers; ++i)
  {
    for (k = 0; k < num_scalers; ++k)
    {
      freq = module_clock / (br_scalers[k] * br_prescalers[i]);
      if (freq <= target_clock)
      {
        /* Found closest lower frequency at this prescaler setting,
         * compare to the best result */
        if (closest_frequency < freq)
        {
          closest_frequency = freq;
          *closest_scaler = k;
          *closest_prescaler = i;
        }
        break;
      }
    }
  }
  if (closest_frequency < 0)
  {
    /* Error, no solution found, this line is never reachable with current
     * hardware settings unless a _very_ low target clock is requested.
     * (scaler_max * prescaler_max) = 229376 => target_min@100MHz = 435 Hz*/
    return -1;
  }

  return 0;
}

void spi_acquire_bus(const spi_bus_t spi_num) {
  lock_acquire(&spi_lock[spi_num]);
}

void spi_release_bus(const spi_bus_t spi_num) {
  lock_release(&spi_lock[spi_num]);
}

int spi_is_busy(const spi_bus_t spi_num) {
  return (spi_lock[spi_num] != 0);
}

void
spi_hw_init_master(const spi_bus_t spi_num) {
  /* Clear lock variable */
  spi_lock[spi_num] = 0;

  /* Block access to the bus */
  spi_acquire_bus(spi_num);

  /* enable clock gate */
  spi_start(spi_num);

  /* Clear MDIS to enable the module. */
  BITBAND_REG(SPI[spi_num]->MCR, SPI_MCR_MDIS_SHIFT) = 0;

  /* Enable master mode, select chip select signal polarity */
  /* XXX: Hard-coded chip select active low */
  /* Disable FIFOs, this can be improved in the future */
  SPI[spi_num]->MCR = SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1F) | SPI_MCR_DIS_RXF_MASK | SPI_MCR_DIS_TXF_MASK;

  /* Enable interrupts for TCF flag */
  BITBAND_REG(SPI[spi_num]->RSER, SPI_RSER_TCF_RE_SHIFT) = 1;
  NVIC_EnableIRQ(SPI0_IRQn);

  /* disable clock gate */
  spi_stop(spi_num);

  /* Allow access to the bus */
  spi_release_bus(spi_num);
}

void spi_set_params(const spi_bus_t spi_num, const uint8_t ctas, const spi_config_t* config) {
  uint32_t ctar = 0;
  uint8_t br_prescaler = 0xff;
  uint8_t br_scaler = 0xff;
  uint8_t prescaler_tmp = 0xff;
  uint8_t scaler_tmp = 0xff;

  /* All of the SPI modules run on the Bus clock, see K60 Ref Manual. 3.9.4.2 SPI clocking */
  /* Find the baud rate divisors */
  find_closest_baudrate_scalers(SystemBusClock, config->sck_freq, &br_prescaler, &br_scaler);
  ctar |= SPI_CTAR_PBR(br_prescaler) | SPI_CTAR_BR(br_scaler);

  /* Find the other delay divisors */
  /* tCSC */
  if (config->tcsc_freq > 0) {
    if (find_closest_delay_scalers(SystemBusClock, config->tcsc_freq,
          &prescaler_tmp, &scaler_tmp) == 0) {
      ctar |= SPI_CTAR_PCSSCK(prescaler_tmp) | SPI_CTAR_CSSCK(scaler_tmp);
    } else {
      /* failed to find a solution */
      DEBUGGER_BREAK(BREAK_INVALID_PARAM);
    }
  } else {
    /* default: copy BR scaler */
    ctar |= SPI_CTAR_PCSSCK(br_prescaler) | SPI_CTAR_CSSCK(br_scaler);
  }

  /* tASC */
  if (config->tasc_freq > 0) {
    if (find_closest_delay_scalers(SystemBusClock, config->tasc_freq,
          &prescaler_tmp, &scaler_tmp) == 0) {
      ctar |= SPI_CTAR_PASC(prescaler_tmp) | SPI_CTAR_ASC(scaler_tmp);
    } else {
      /* failed to find a solution */
      DEBUGGER_BREAK(BREAK_INVALID_PARAM);
    }
  } else {
    /* default: copy BR scaler */
    ctar |= SPI_CTAR_PASC(br_prescaler) | SPI_CTAR_ASC(br_scaler);
  }

  /* tDT */
  if (config->tdt_freq > 0) {
    if (find_closest_delay_scalers(SystemBusClock, config->tdt_freq,
          &prescaler_tmp, &scaler_tmp) == 0) {
      ctar |= SPI_CTAR_PDT(prescaler_tmp) | SPI_CTAR_DT(scaler_tmp);
    } else {
      /* failed to find a solution */
      DEBUGGER_BREAK(BREAK_INVALID_PARAM);
    }
  } else {
    /* default: copy BR scaler */
    ctar |= SPI_CTAR_PDT(br_prescaler) | SPI_CTAR_DT(br_scaler);
  }

  /* FMSZ+1 equals the frame size */
  ctar |= SPI_CTAR_FMSZ(config->frame_size - 1);

  if (config->cpol != 0) {
    ctar |= SPI_CTAR_CPOL_MASK;
  }
  if (config->cpha != 0) {
    ctar |= SPI_CTAR_CPHA_MASK;
  }

  SPI[spi_num]->CTAR[ctas] = ctar;
  spi_conf[spi_num][ctas] = config;
}

void spi_refresh_params(void) {
  int spi_num;
  int ctas;

  for (spi_num = 0; spi_num < NUM_SPI; ++spi_num) {
    for (ctas = 0; ctas < NUM_CTAR; ++ctas) {
      if (spi_conf[spi_num][ctas] != NULL) {
        spi_set_params(spi_num, ctas, spi_conf[spi_num][ctas]);
      }
    }
  }
}

int spi_transfer_blocking(const spi_bus_t spi_num, const uint8_t ctas, const uint32_t cs,
             const spi_transfer_flag_t cont, const uint8_t *data_out,
             uint8_t *data_in, size_t count_out, size_t count_in)
{
  SPI_Type *spi_dev = SPI[spi_num];
  /* Frame size in bits */
  unsigned short frame_size = ((SPI[spi_num]->CTAR[ctas] & SPI_CTAR_FMSZ_MASK) >> SPI_CTAR_FMSZ_SHIFT) + 1;

  /* Array stride in bytes */
  /* unsigned int stride = (frame_size / 8) + 1; */

  uint32_t common_flags = SPI_PUSHR_CTAS(ctas) | SPI_PUSHR_PCS(cs);

  uint32_t spi_pushr;

  /* this may yield unaligned memory accesses because of uint8_t* casted to
   * uint16_t*. Using the DMA module will avoid this. */
  if (frame_size > 8) {
    /* Error! not implemented yet */
    DEBUGGER_BREAK(BREAK_INVALID_PARAM);
    while(1);
  }

  while (count_out > 0) {
    spi_pushr = common_flags;
    spi_pushr |= SPI_PUSHR_TXDATA(*data_out);
    ++data_out; /* or += stride */
    --count_out;

    /* See if we are at the end yet */
    if((count_out > 0) || (count_in > 0) || (cont == SPI_TRANSFER_CONT)) {
      spi_pushr |= SPI_PUSHR_CONT_MASK;
    }

    /* Clear transfer complete flag */
    spi_dev->SR |= SPI_SR_TCF_MASK;

    /* Set waiting flag */
    spi_waiting_flag[spi_num] = 1;

    /* Shift a frame out/in */
    spi_dev->PUSHR = spi_pushr;

    /* Wait for transfer complete */
    COND_WAIT(spi_waiting_flag[spi_num] != 0);

    /* Do a dummy read in order to allow for the next byte to be read */
    uint8_t tmp = spi_dev->POPR;
    (void)tmp; /* Suppress warning about unused value. */

  }

  /* Prepare read */
  spi_pushr = common_flags;
  spi_pushr |= SPI_PUSHR_TXDATA(SPI_IDLE_DATA);
  spi_pushr |= SPI_PUSHR_CONT_MASK;

  /* Do a number of reads */
  while (count_in > 0) {
    --count_in;
    /* See if we are at the end yet */
    if((count_in < 1) && (cont != SPI_TRANSFER_CONT)) {
      /* Disable CS line after the next transfer */
      spi_pushr &= ~(SPI_PUSHR_CONT_MASK);
    }

    /* Clear transfer complete flag */
    spi_dev->SR |= SPI_SR_TCF_MASK;

    /* Set waiting flag */
    spi_waiting_flag[spi_num] = 1;

    /* Shift a frame out/in */
    spi_dev->PUSHR = spi_pushr;

    /* Wait for transfer complete */
    COND_WAIT(spi_waiting_flag[spi_num] != 0);

    (*data_in) = spi_dev->POPR;
    ++data_in; /* or += stride */
  }

  return 0;
}

void spi_start(const spi_bus_t spi_num)
{
  /* Enable clock gate for the correct SPI hardware module */
  switch(spi_num) {
    case 0:
      BITBAND_REG(SIM->SCGC6, SIM_SCGC6_SPI0_SHIFT) = 1;
      break;
    case 1:
      BITBAND_REG(SIM->SCGC6, SIM_SCGC6_SPI1_SHIFT) = 1;
      break;
    case 2:
      BITBAND_REG(SIM->SCGC3, SIM_SCGC3_SPI2_SHIFT) = 1;
      break;
  }
}

void spi_stop(const spi_bus_t spi_num)
{
  /* Enable clock gate for the correct SPI hardware module */
  switch(spi_num) {
    case 0:
      BITBAND_REG(SIM->SCGC6, SIM_SCGC6_SPI0_SHIFT) = 0;
      break;
    case 1:
      BITBAND_REG(SIM->SCGC6, SIM_SCGC6_SPI1_SHIFT) = 0;
      break;
    case 2:
      BITBAND_REG(SIM->SCGC3, SIM_SCGC3_SPI2_SHIFT) = 0;
      break;
  }
}

/** \todo SPI: Handle all flags properly in ISR, remove assumption on TCF. */

/**
 * ISR for handling bus transfer complete interrupts.
 */
void _isr_spi0(void) {
  /* Clear status flag by writing a 1 to it */
  BITBAND_REG(SPI0->SR, SPI_SR_TCF_SHIFT) = 1;
  spi_waiting_flag[0] = 0;
}

void _isr_spi1(void) {
  /* Clear status flag by writing a 1 to it */
  BITBAND_REG(SPI1->SR, SPI_SR_TCF_SHIFT) = 1;
  spi_waiting_flag[1] = 0;
}
