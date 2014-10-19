/**
 * \file
 *         Clock module implementation using rtimers.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#include "contiki.h"
#include "contiki-conf.h"
#include "sys/clock.h"
#include "sys/etimer.h"

#include "sys/etimer.h"
#include "sys/rtimer.h"

#include "K60.h"
#include "power-modes.h"

#define DEBUG 0
#if DEBUG
#include "stdio.h"
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MAX_TICKS (~((clock_time_t)0) / 2)

static volatile clock_time_t current_tick;
static volatile unsigned long current_seconds = 0;
static volatile unsigned long second_countdown = CLOCK_SECOND;

static volatile uint8_t waiting_flag = 0;

static struct rtimer rt_clock;

/* This is based on the MC1322x clock module rtimer implementation. */
/* the typical clock things like incrementing current_tick and etimer checks */
/* are performed as a periodically scheduled rtimer */
static void
rt_do_clock(struct rtimer *t, void *ptr)
{
  rtimer_set(t, RTIMER_TIME(t) + (RTIMER_SECOND/CLOCK_SECOND), 1,
             (rtimer_callback_t)rt_do_clock, ptr);

  current_tick++;

  if(--second_countdown == 0) {
    current_seconds++;
    second_countdown = CLOCK_SECOND;
  }

  if(etimer_pending() &&
     (etimer_next_expiration_time() - current_tick - 1) > MAX_TICKS) {
    etimer_request_poll();
  }
}

/*
 * Get the system time.
 */
clock_time_t
clock_time(void)
{
  return current_tick;
}

/*
 * Get the system time in seconds.
 */
unsigned long
clock_seconds(void)
{
  return current_seconds;
}

/*
 * Set the system time in seconds.
 */
void
clock_set_seconds(unsigned long sec)
{
  current_seconds = sec;
}

#if 0
/*
 * (Deprecated) Delay the CPU.
 */
void
clock_delay(unsigned int delay)
{
}
#endif /* 0 */

/* clock_wait is untested and is not currently needed by any Mulle code.
 * Remove this #if 0 when a use case arises. */
#if 0
/*
 * Delay the CPU for a number of clock ticks.
 */
void
clock_wait(clock_time_t delay)
{
  clock_time_t target = clock_time() + delay;
  while (target > clock_time())
  {
    /* Wait for event (timer interrupt or anything else) */
    /* Make sure that we do not enter STOP mode since then any peripherals
     * currently in use may halt, e.g. UART and SPI buses. */
    power_mode_wait(); /* power_mode_wait clears the DEEPSLEEP bit before executing WFI/WFE. */
  }
}
#endif /* 0 */

/**
 * Delay the CPU by delay microseconds.
 *
 * \param delay_us microsecond delay.
 */
void
clock_delay_usec(uint16_t delay_us) {
  /* Don't hang on zero µs sleep. */
  if (delay_us == 0) return;

  /* Enable PIT peripheral clock */
  BITBAND_REG(SIM->SCGC6, SIM_SCGC6_PIT_SHIFT) = 1;

  /* Disable timer, disable interrupts */
  BITBAND_REG(PIT->CHANNEL[BOARD_DELAY_PIT_CHANNEL].TCTRL, PIT_TCTRL_TEN_SHIFT) = 0;
  BITBAND_REG(PIT->CHANNEL[BOARD_DELAY_PIT_CHANNEL].TCTRL, PIT_TCTRL_TIE_SHIFT) = 0;
  /* Clear interrupt flag */
  BITBAND_REG(PIT->CHANNEL[BOARD_DELAY_PIT_CHANNEL].TFLG, PIT_TFLG_TIF_SHIFT) = 1;

  /* Enable interrupts, disable chaining, disable timer */
  PIT->CHANNEL[BOARD_DELAY_PIT_CHANNEL].TCTRL = PIT_TCTRL_TIE_MASK;

  /* It would be possible to get a better rounding if we allowed a division
   * here, but using a runtime division makes this all sorts of complicated.
   * In order to get a proper division we would have to extend to a 64 bit int
   * because of overflows in the multiplication between delay_us and
   * PIT_module_frequency when PIT_module_frequency > 65535, at least if all
   * possible delay_us values should be allowed. */
  /* Set up timer */
  PIT->CHANNEL[BOARD_DELAY_PIT_CHANNEL].LDVAL =
      PIT_LDVAL_TSV(PIT_ticks_per_usec * (uint32_t)delay_us);

  /* Set flag, will be cleared by ISR when timer fires. */
  waiting_flag = 1;

  /* Enable timer */
  BITBAND_REG(PIT->CHANNEL[BOARD_DELAY_PIT_CHANNEL].TCTRL, PIT_TCTRL_TEN_SHIFT) = 1;

  while(waiting_flag) {
    /* Don't go to deeper sleep modes as that will stop the PIT module clock. */
    power_mode_wait();
  }
}


/*
 * Initialize the clock module.
 */
/* rtimer MUST have been already initialized */
void
clock_init(void)
{
  rtimer_set(&rt_clock, RTIMER_NOW() + RTIMER_SECOND/CLOCK_SECOND, 1, (rtimer_callback_t)rt_do_clock, NULL);

  /* Enable PIT peripheral clock */
  BITBAND_REG(SIM->SCGC6, SIM_SCGC6_PIT_SHIFT) = 1;

  /* Reset PIT logic to a known state */
  /* The MCR really only controls the MDIS (module disable) and FRZ (debug
   * freeze) bits of the module. */
  PIT->MCR = 0x00;

  /* Enable clock_delay_usec PIT IRQ in NVIC. */
  NVIC_EnableIRQ(BOARD_DELAY_PIT_IRQn);
}

/* PIT interrupt handler for clock_delay_usec */
void BOARD_DELAY_PIT_ISR(void) {
  /* Clear interrupt flag */
  BITBAND_REG(PIT->CHANNEL[BOARD_DELAY_PIT_CHANNEL].TFLG, PIT_TFLG_TIF_SHIFT) = 1;

  /* Clear flag set by clock_delay_usec() */
  waiting_flag = 0;
}
