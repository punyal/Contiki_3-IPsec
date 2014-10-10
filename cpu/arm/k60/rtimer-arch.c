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
 *         K60 specific rtimer library implementation.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#include "rtimer-arch.h"
#include "K60.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Write some garbage to CNR to latch the current value */
/* A bug in the MK60D10.h header causes errors since the CNR field is const
 * declared, cast to volatile uint32_t* as a workaround. */
#define LPTMR0_LATCH_CNR() (*((volatile uint32_t*)(&(LPTMR0->CNR))) = 0)

/* The period length of the rtimer, in rtimer ticks */
#define RTIMER_PERIOD ((LPTMR_CNR_COUNTER_MASK >> LPTMR_CNR_COUNTER_SHIFT) + 1)

/* Number of ticks into the future for us to not consider the rtimer as immediately expired */
/* at 32768 Hz (LPTMR maximum), 1 tick == 30.51 us */
/* at 4096 Hz (msp430 default), 1 tick == 244.1 us */
#define RTIMER_SCHEDULE_MARGIN (5)

/** Offset between current counter/timer and t=0 (boot instant) */
static volatile rtimer_clock_t offset;
static volatile rtimer_clock_t target;

/*
 * Initialize the clock module.
 *
 * Generates interrupt from external 32kHz crystal.
 */
/* TODO(Henrik) move to platform, init may differ between platforms. */
void
rtimer_arch_init(void) {
  offset = 0;
  /* Setup Low Power Timer (LPT) */

  /* Enable LPT clock gate */
  BITBAND_REG(SIM->SCGC5, SIM_SCGC5_LPTIMER_SHIFT) = 1;

  /* Disable timer to change settings. */
  /* Logic is reset when the timer is disabled, TCF flag is also cleared on disable. */
  LPTMR0->CSR = 0x00;
  /* Prescaler bypass, LPTMR is clocked directly by ERCLK32K. */
  LPTMR0->PSR = (LPTMR_PSR_PBYP_MASK | LPTMR_PSR_PCS(0b10));
  LPTMR0->CMR = LPTMR_CMR_COMPARE(32767);
  /* Enable free running mode. */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TFC_SHIFT) = 1;
  /* Enable interrupts. */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TIE_SHIFT) = 1;
  /* Enable timer */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TEN_SHIFT) = 1;

  /* Enable LPT interrupt */
  NVIC_EnableIRQ(LPTimer_IRQn);

  PRINTF("rtimer_arch_init: Done\n");
}

void
rtimer_arch_schedule(rtimer_clock_t t) {
  rtimer_clock_t now = rtimer_arch_now();
  if (t < (now + RTIMER_SCHEDULE_MARGIN)) {
    /* Already happened */
    t = now + RTIMER_SCHEDULE_MARGIN;
  }
  target = t; /* Update stored target time, read from ISR */
  if (t > now + RTIMER_PERIOD) {
    /* We can not reach this time in one period, wrap around */
    t = now + RTIMER_PERIOD;
  }
  /* The reference manual states that the CMR register should not be written
   * while the timer is enabled unless the TCF (interrupt flag) is set.
  /* It seems like modifying the CMR variable without stopping (against the
   * reference manual's recommendations) cause sporadic failures of the
   * interrupt to trigger. It seems to happen at random. */
  /* The downside is that the CNR register is reset when the LPTMR is disabled,
   * we need a new offset computation. */
  offset = rtimer_arch_now();
  /* Disable timer in order to modify the CMR register. */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TEN_SHIFT) = 0;
  /* Set timer value */
  /* t and offset are 32 bit ints, CMR_COMPARE is only 16 bit wide,
   * the MSBs will be cut off, so to cope with this we add an additional check
   * for the MSBs in the ISR. */
  LPTMR0->CMR = LPTMR_CMR_COMPARE(t - offset);
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TEN_SHIFT) = 1;
}

rtimer_clock_t
rtimer_arch_now(void) {
  rtimer_clock_t cnr;
  LPTMR0_LATCH_CNR();
  cnr = LPTMR0->CNR; /* Read the counter value */
  return offset + cnr;
}

/* Interrupt handler for rtimer triggers */
void
_isr_lpt(void)
{
  rtimer_clock_t now = rtimer_arch_now();

  /* Clear timer compare flag by writing a 1 to it */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TCF_SHIFT) = 1;

  if (target > now) {
    /* Overflow, schedule the next period */
    rtimer_arch_schedule(target);
  } else {
    rtimer_run_next();
  }
}
