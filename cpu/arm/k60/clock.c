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

static struct rtimer rt_clock;

/* This is based on the MC1322x clock module rtimer implementation. */
/* the typical clock things like incrementing current_tick and etimer checks */
/* are performed as a periodically scheduled rtimer */
void
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

/*
 * Initialize the clock module.
 */
/* rtimer MUST have been already initialized */
void
clock_init(void)
{
  rtimer_set(&rt_clock, RTIMER_NOW() + RTIMER_SECOND/CLOCK_SECOND, 1, (rtimer_callback_t)rt_do_clock, NULL);
}
