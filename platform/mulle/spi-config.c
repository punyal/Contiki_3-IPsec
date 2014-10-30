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

/* For AT86RF212 we need some extra delays for the tASC, tCSC, tDT times. */
/* CS signal deassertion to assertion delay, t8, tDT > 250 ns => 12 DT divider at 48 MHz bus */
/* CS signal assertion to SCK delay, t1, tCSC > 180 ns => 8.64 CSC divider at 48 MHz bus */
/* last SCK rising edge to CS signal deassertion, t9, (tASC + (1/BR)/2) > 250 ns => tASC > 183 ns => 8.8 ASC divider at 48 MHz bus */
/* LSB last byte to MSB next byte, t5, (tCSC+tASC) > 250 ns => already fulfilled if the above conditions are satisfied. */

static const spi_config_t spi0_conf[NUM_CTAR] = {
  {
    .sck_freq =  7500000, /* 7.5 MHz max bus frequency according to AT86RF212 data sheet. */
    .frame_size = 8,
    .cpol = 0,
    .cpha = 0,
    .tcsc_freq = 5555555, /* It looks silly, but this is correct. 1/180e-9 */
    .tasc_freq = 5454545, /* It looks silly, but this is correct. 1/183e-9 */
    .tdt_freq  = 4000000, /* 1/250e-9 */
  },
  {
    .sck_freq = 10000000,
    .frame_size = 8,
    .cpol = 1,
    .cpha = 1,
    .tcsc_freq = 0, /* Use same as BR divider */ /* TODO: Review data sheets for safe timings */
    .tasc_freq = 0, /* Use same as BR divider */ /* TODO: Review data sheets for safe timings */
    .tdt_freq  = 0, /* Use same as BR divider */ /* TODO: Review data sheets for safe timings */
  }
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

