/*
 * Configuration for SPI0 bus.
 */

/*
 * Onboard devices connected:
 *  AT86RF212: max sck frequency 7.5 MHz (async) 8000000 (sync), CPOL = 0, CPHA = 0, but seems to work with CPOL=1, CPHA=1 as well.
 *  LIS3DH: max sck frequency 10 MHz, CPOL = 1, CPHA = 1
 *  M25P16: max sck frequency 25 MHz, (CPOL=0, CPHA=0) or (CPOL=1, CPHA=1)
 *  FM25L04B: max sck frequency 20 MHz, (CPOL=0, CPHA=0) or (CPOL=1, CPHA=1)
 *
 * The AT86RF212 is allocated to CTAR0, the rest at CTAR1.
 * FIXME: The memories could be driven faster.
 */

#include "spi-k60.h"

static const spi_config_t spi0_conf[NUM_CTAR] = {
  { .sck_freq = 7500000, .frame_size = 8, .cpol = 0, .cpha = 0},
  { .sck_freq = 10000000, .frame_size = 8, .cpol = 1, .cpha = 1}
  };

/* SPI1 is only used by expansion boards. */
/*
static const spi_config_t spi1_conf[NUM_CTAR] = {
  { .sck_freq = 1000000, .frame_size = 8, .cpol = 0, .cpha = 0},
  { .sck_freq = 10000000, .frame_size = 8, .cpol = 1, .cpha = 1}
  };
*/

/* Set port mux for SPI0 bus */
static void port_init_spi0(void) {
  /* Turn on port */
  BITBAND_REG(SIM->SCGC5, SIM_SCGC5_PORTD_SHIFT) = 1;
  /* Set up mux */
  /** \todo Update SPI pin mapping to more dynamic format. (remove magic numbers) */
  PORTD->PCR[0] = PORT_PCR_MUX(2); /* SPI0_PCS0 */
  PORTD->PCR[1] = PORT_PCR_MUX(2); /* SPI0_SCLK */
  PORTD->PCR[2] = PORT_PCR_MUX(2); /* SPI0_MOSI */
  PORTD->PCR[3] = PORT_PCR_MUX(2); /* SPI0_MISO */
  PORTD->PCR[4] = PORT_PCR_MUX(2); /* SPI0_PCS1 */
  PORTD->PCR[5] = PORT_PCR_MUX(2); /* SPI0_PCS2 */
  PORTD->PCR[6] = PORT_PCR_MUX(2); /* SPI0_PCS3 */
}

/* Board or project specific SPI initialization procedure. */
void
board_spi_init(void)
{
  int i;
  port_init_spi0();
  /* SPI0 is used for onboard peripherals */
  spi_hw_init_master(0);
  spi_start(0);
  for (i = 0; i < NUM_CTAR; ++i) {
    spi_set_params(0, i, &spi0_conf[i]);
  }
  spi_stop(0);
}

