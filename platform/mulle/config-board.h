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
 *         Board configuration defines for Mulle platform.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#ifndef MULLE_CONFIG_BOARD_H_
#define MULLE_CONFIG_BOARD_H_

#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Disable hardware watchdog, for debugging purposes, don't use this on production builds. */
#define DISABLE_WDOG    1

/**
 * CPU silicon revision (some registers are moved or added between revisions 1 and 2)
 */
#if !defined(MULLE_SERIAL)
/* Default to revision 2 unless the serial number is specified in the build. */
#define K60_CPU_REV 2
#elif defined(MULLE_SERIAL) && \
  (MULLE_SERIAL >= 200) && \
  (MULLE_SERIAL <= 219)
/* Only Mulles with serial numbers 200 through 219 have revision 1.x silicon
 * (revision 1.4, 4N30D mask set), see the sticker on the CPU top on the Mulle */
#define K60_CPU_REV 1
#else
/* Newer boards have the revision 2 silicon */
#define K60_CPU_REV 2
#endif

/**
 * Voltage reference high for ADC computations (millivolts).
 */
#define MULLE_ADC_VREFH_MILLIVOLTS 3300u

/**
 * Voltage reference low for ADC computations (millivolts).
 */
#define MULLE_ADC_VREFL_MILLIVOLTS 0u

/**
 * Total span of ADC measurement (millivolts).
 */
#define MULLE_ADC_VREFHL_SCALE_MILLIVOLTS ((MULLE_ADC_VREFH_MILLIVOLTS) - (MULLE_ADC_VREFL_MILLIVOLTS))

/**
 * Which channel should perform Vbat measurements
 */
#define MULLE_ADC_VBAT_ADC_NUM 1

#define MULLE_ADC_VBAT_CHANNEL ADC_CH_DAD0

#define MULLE_ADC_VCHR_ADC_NUM 1

#define MULLE_ADC_VCHR_CHANNEL ADC_CH_AD19

/**
 * UART module used for debug printf.
 */
#define BOARD_DEBUG_UART_NUM 1

/**
 * Baud rate of debug UART.
 */
#define BOARD_DEBUG_UART_BAUD 921600

/**
 * PORT module containing the TX pin of the debug UART.
 */
#define BOARD_DEBUG_UART_TX_PIN_PORT PORTC

/**
 * PORT module containing the RX pin of the debug UART.
 */
#define BOARD_DEBUG_UART_RX_PIN_PORT PORTC

/**
 * Pin number within the PORT module of the TX pin of the debug UART.
 */
#define BOARD_DEBUG_UART_TX_PIN_NUMBER 4

/**
 * Pin number within the PORT module of the RX pin of the debug UART.
 */
#define BOARD_DEBUG_UART_RX_PIN_NUMBER 3

/**
 * Function number in the PORT mux for the TX pin of the debug UART.
 */
#define BOARD_DEBUG_UART_TX_PIN_MUX 3

/**
 * Function number in the PORT mux for the RX pin of the debug UART.
 */
#define BOARD_DEBUG_UART_RX_PIN_MUX 3

/**
 * Number of UART modules in CPU.
 */
#define NUM_UARTS 5

/**
 * Number of SPI modules in CPU.
 */
#define NUM_SPI 3

/**
 * @name SLIP configuration
 */
/** @{ */
#ifndef BOARD_SLIP_UART_NUM
/**
 * UART module used for SLIP communications.
 */
#define BOARD_SLIP_UART_NUM 1
#endif

#if BOARD_SLIP_UART_NUM == 0
/**
 * UART module used for SLIP communications.
 *
 * This string is passed to open() during slip_init_arch().
 * This is usually the module name within double-quotes e.g. "UART0"
 */
#define BOARD_SLIP_UART_NAME "UART0"

/**
 * @brief PORT module used for SLIP UART TX pin
 */
#define BOARD_SLIP_TX_PORT PORTA
/**
 * @brief PORT module used for SLIP UART RX pin
 */
#define BOARD_SLIP_RX_PORT PORTA

/**
 * @brief SLIP UART TX pin
 */
#define BOARD_SLIP_TX_PIN 14
/**
 * @brief SLIP UART RX pin
 */
#define BOARD_SLIP_RX_PIN 15

/**
 * @brief SLIP UART TX pin alternate function number
 */
#define BOARD_SLIP_TX_AF 3
/**
 * @brief SLIP UART RX pin alternate function number
 */
#define BOARD_SLIP_RX_AF 3

#elif BOARD_SLIP_UART_NUM == 1

#define BOARD_SLIP_UART_NAME "UART1"
#define BOARD_SLIP_TX_PORT PORTC
#define BOARD_SLIP_RX_PORT PORTC
#define BOARD_SLIP_TX_PIN 3
#define BOARD_SLIP_RX_PIN 4
#define BOARD_SLIP_TX_AF 3
#define BOARD_SLIP_RX_AF 3

#endif /* BOARD_SLIP_UART_NUM */

/** @} */

/**
 * RTC crystal load capacitance configuration bits.
 */
/* enable 12pF load capacitance, might need adjusting.. */
#define BOARD_RTC_LOAD_CAP_BITS (RTC_CR_SC8P_MASK | RTC_CR_SC4P_MASK)

/**
 * PIT channel to use for clock_delay_usec and clock_delay_msec.
 *
 * Note: Make sure this channel is not used elsewhere (asynchronously).
 */
#define BOARD_DELAY_PIT_CHANNEL 1

#define PIT_ISR_GLUE2(CHANNEL) (_isr_pit ## CHANNEL)
#define PIT_ISR_GLUE(CHANNEL) PIT_ISR_GLUE2(CHANNEL)
/**
 * PIT channel interrupt used by clock_delay_usec and clock_delay_msec.
 */
#define BOARD_DELAY_PIT_ISR PIT_ISR_GLUE(BOARD_DELAY_PIT_CHANNEL)

#define PIT_IRQn_GLUE2(CHANNEL) (PIT ## CHANNEL ## _IRQn)
#define PIT_IRQn_GLUE(CHANNEL) PIT_IRQn_GLUE2(CHANNEL)

#define BOARD_DELAY_PIT_IRQn PIT_IRQn_GLUE(BOARD_DELAY_PIT_CHANNEL)

/*
 * The onboard LIS3DH is connected to SPI0.
 * SPI0_PCS0 is the active low CS signal.
 */
#define LIS3DH_SPI_NUM 0
#define LIS3DH_CHIP_SELECT_PIN 0
/*
 * See spi-config.c for the CTAR configuration
 */
#define LIS3DH_CTAS 1

/*
 * The onboard AT86RF212/AT86RF230 is connected to SPI0.
 * SPI0_PCS1 is the active low CS signal.
 */
#define AT86RF212_SPI_NUM 0
#define AT86RF212_CHIP_SELECT_PIN 1
/*
 * See spi-config.c for the CTAR configuration
 */
#define AT86RF212_CTAS 0

/*
 * The onboard M25P16 flash memory is connected to SPI0.
 * SPI0_PCS2 is the active low CS signal.
 */
#define FLASH_SPI_NUM 0
#define FLASH_CHIP_SELECT_PIN 2
/*
 * See spi-config.c for the CTAR configuration
 */
#define FLASH_CTAS 1

/*
 * The onboard FM25L04 FRAM memory is connected to SPI0.
 * SPI0_PCS2 is the active low CS signal.
 */
#define FRAM_SPI_NUM 0
#define FRAM_CHIP_SELECT_PIN 3
/*
 * See spi-config.c for the CTAR configuration
 */
#define FRAM_CTAS 1

/*
 * 1-wire bus configuration.
 *
 * The hardware needs some external components:
 *  - 5k-ish pull-up resistor between 3.3V (or VSEC) and the RX pin.
 *  - Jumper between RX and TX pin.
 *
 * There is a way in software to loop-back connect RX and TX pins inside the CPU
 * package, but using this functionality disables the external electrical
 * signal, which makes it impossible to use this for any communications other
 * than software testing. Therefore we use an external jumper between the RX and
 * TX lines instead.
 * The TX line will be configured as an open-drain output in order to let the
 * 1-wire slave to drive the RX pin low when communicating back.
 */

/**
 * UART module number assigned to 1-wire bus driver.
 *
 * Only used if ow_init() is called by the application, this is not included in
 * Mulle contiki-main default startup procedure.
 */
#define ONEWIRE_UART_NUM 0

/**
 * \brief Clock frequency of the chosen UART module.
 *
 * \todo handle 1-wire clocking with runtime configured core clock frequency.
 */
#define ONEWIRE_UART_MODULE_FREQUENCY F_BUS

/** 1-wire ISR function name. This should be the IRQ handler for the UARTx STATUS IRQ. */
#define ONEWIRE_ISR_FUNC _isr_uart0_status
/** 1-wire IRQ number. This should be the IRQn for the UARTx STATUS IRQ, see MK60D10.h */
#define ONEWIRE_IRQn UART0_RX_TX_IRQn
/** 1-wire TX pin port module. */
#define ONEWIRE_TX_PORT PORTA
/** 1-wire TX pin number. */
#define ONEWIRE_TX_PIN 14
/** Function index number of UART TX function for the given pin. */
#define ONEWIRE_TX_PCR_FUNC 3
/** 1-wire RX pin port module*/
#define ONEWIRE_RX_PORT PORTA
/** 1-wire TX pin number. */
#define ONEWIRE_RX_PIN 15
/** Function index number of UART RX function for the given pin. */
#define ONEWIRE_RX_PCR_FUNC 3

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* !defined(MULLE_CONFIG_BOARD_H_) */
