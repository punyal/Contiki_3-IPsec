#include <stddef.h>

#include "K60.h"
#include "config-board.h"
#include "config-clocks.h"
#include "uart.h"
#include "interrupt.h"
#include "llwu.h"
#include "ringbuf.h"

typedef int (*rx_callback_t)(unsigned char);

static rx_callback_t rx_callback[NUM_UARTS] = { NULL };


#ifdef UART_CONF_DEFAULT_TXBUFSIZE
#define UART_DEFAULT_TXBUFSIZE UART_CONF_DEFAULT_TXBUFSIZE
#else
#define UART_DEFAULT_TXBUFSIZE 8
#endif
#ifdef UART0_CONF_TXBUFSIZE
#define UART0_TXBUFSIZE UART0_CONF_TXBUFSIZE
#else
#define UART0_TXBUFSIZE UART_DEFAULT_TXBUFSIZE
#endif
#ifdef UART1_CONF_TXBUFSIZE
#define UART1_TXBUFSIZE UART1_CONF_TXBUFSIZE
#else
#define UART1_TXBUFSIZE UART_DEFAULT_TXBUFSIZE
#endif
#ifdef UART2_CONF_TXBUFSIZE
#define UART2_TXBUFSIZE UART2_CONF_TXBUFSIZE
#else
#define UART2_TXBUFSIZE UART_DEFAULT_TXBUFSIZE
#endif
#ifdef UART3_CONF_TXBUFSIZE
#define UART3_TXBUFSIZE UART3_CONF_TXBUFSIZE
#else
#define UART3_TXBUFSIZE UART_DEFAULT_TXBUFSIZE
#endif
#ifdef UART4_CONF_TXBUFSIZE
#define UART4_TXBUFSIZE UART4_CONF_TXBUFSIZE
#else
#define UART4_TXBUFSIZE UART_DEFAULT_TXBUFSIZE
#endif

static struct ringbuf uart_txbuf[NUM_UARTS];

#if UART0_CONF_ENABLE
static uint8_t uart0_txbuf_data[UART0_TXBUFSIZE];
#endif
#if UART1_CONF_ENABLE
static uint8_t uart1_txbuf_data[UART1_TXBUFSIZE];
#endif
#if UART2_CONF_ENABLE
static uint8_t uart2_txbuf_data[UART2_TXBUFSIZE];
#endif
#if UART3_CONF_ENABLE
static uint8_t uart3_txbuf_data[UART3_TXBUFSIZE];
#endif
#if UART4_CONF_ENABLE
static uint8_t uart4_txbuf_data[UART4_TXBUFSIZE];
#endif
#if UART5_CONF_ENABLE
static uint8_t uart5_txbuf_data[UART5_TXBUFSIZE];
#endif

static inline void tx_irq_handler(const unsigned int uart_num) {
  static uint8_t transmitting[NUM_UARTS] = {0};
  UART_Type *uart_dev = UART[uart_num];
  if((uart_dev->C2 & UART_C2_TCIE_MASK) && (uart_dev->S1 & UART_S1_TC_MASK)) {
    if (transmitting[uart_num] != 0) {
      /* transmission complete, allow STOP modes again */
      LLWU_UNINHIBIT_STOP();
      transmitting[uart_num] = 0;
    }
    /* Disable transmission complete interrupt */
    uart_dev->C2 &= ~(UART_C2_TCIE_MASK);
  }

  if((uart_dev->C2 & UART_C2_TIE_MASK) && (uart_dev->S1 & UART_S1_TDRE_MASK)) {
    int ret;
    ret = ringbuf_get(&uart_txbuf[uart_num]);
    if (ret < 0) {
      /* Empty buffer, disable this interrupt */
      uart_dev->C2 &= ~(UART_C2_TIE_MASK);
      /* Enable transmission complete interrupt. */
      uart_dev->C2 |= UART_C2_TCIE_MASK;
    } else {
      /* queue next byte */
      uart_dev->D = (uint8_t)(ret & 0xff);
      if (transmitting[uart_num] == 0) {
        LLWU_INHIBIT_STOP();
        transmitting[uart_num] = 1;
      }
    }
  }
}

static inline void rx_irq_handler(const unsigned int uart_num) {
  UART_Type *uart_dev = UART[uart_num];
  static uint8_t receiving[NUM_UARTS] = {0};
  if(uart_dev->S1 & UART_S1_RDRF_MASK) {
    volatile uint8_t c = uart_dev->D; /* RDRF flag is cleared by first reading S1, then reading D */
    if (rx_callback[uart_num] != NULL) {
      (rx_callback[uart_num])(c);
    }
  }
  if ((uart_dev->S2 & UART_S2_RAF_MASK) == 0) {
    /* Receiver idle */
    if (receiving[uart_num] != 0) {
      receiving[uart_num] = 0;
      LLWU_UNINHIBIT_STOP();
    }
  }

  if((uart_dev->S2 & UART_S2_RXEDGIF_MASK)) {
    /* Woken up by edge detect */
    if (receiving[uart_num] == 0) {
      LLWU_INHIBIT_STOP();
      receiving[uart_num] = 1;
    }
    /* Clear RX wake-up flag by writing a 1 to it */
    uart_dev->S2 = UART_S2_RXEDGIF_MASK;
  }
}

/**
 * Enable the clock gate to an UART module
 *
 * This is a convenience function mapping UART module number to SIM_SCG register.
 *
 * \param uart_num UART module number
 */
void
uart_module_enable(const unsigned int uart_num)
{
  switch (uart_num) {
    case 0:
      BITBAND_REG(SIM->SCGC4, SIM_SCGC4_UART0_SHIFT) = 1;
      break;
    case 1:
      BITBAND_REG(SIM->SCGC4, SIM_SCGC4_UART1_SHIFT) = 1;
      break;
    case 2:
      BITBAND_REG(SIM->SCGC4, SIM_SCGC4_UART2_SHIFT) = 1;
      break;
    case 3:
      BITBAND_REG(SIM->SCGC4, SIM_SCGC4_UART3_SHIFT) = 1;
      break;
    case 4:
      BITBAND_REG(SIM->SCGC1, SIM_SCGC1_UART4_SHIFT) = 1;
      break;
    case 5:
      BITBAND_REG(SIM->SCGC1, SIM_SCGC1_UART5_SHIFT) = 1;
      break;
    default:
      /* Unknown UART module!! */
      DEBUGGER_BREAK(BREAK_INVALID_PARAM);
      return;
  }
}

/**
 * Initialize UART.
 *
 * This is based on the example found in the CodeWarrior samples provided by
 * Freescale.
 *
 * \param uart_num UART module number
 * \param module_clk_hz Module clock (in Hz) of the given UART, if zero: Use current module frequency.
 * \param baud Desired target baud rate of the UART.
 */
void
uart_init(const unsigned int uart_num, uint32_t module_clk_hz, const uint32_t baud)
{
  volatile UART_Type *uart_dev = UART[uart_num];
  uint16_t sbr;
  uint16_t brfa;
  if (module_clk_hz == 0) {
    switch (uart_num) {
      case 0:
      case 1:
        module_clk_hz = SystemSysClock;
        break;
      case 2:
      case 3:
      case 4:
        module_clk_hz = SystemBusClock;
        break;
      default:
        DEBUGGER_BREAK(BREAK_INVALID_PARAM);
        return;
    }
  }

  /* Enable the clock to the selected UART */
  uart_module_enable(uart_num);

  /* Compute new SBR value */
  sbr = UART_SBR(module_clk_hz, baud);
  /* Compute new fine-adjust value */
  brfa = UART_BRFA(module_clk_hz, baud);

  /* Make sure that the transmitter and receiver are disabled while we
   * change settings.
   */
  uart_dev->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

  /* Configure the UART for 8-bit mode, no parity */
  uart_dev->C1 = 0;  /* We need all default settings, so entire register is cleared */

  /* Replace SBR bits in BDH, BDL registers */
  /* High bits */
  uart_dev->BDH = (uart_dev->BDH & ~(UART_BDH_SBR_MASK)) | UART_BDH_SBR(sbr >> 8);
  /* Low bits */
  uart_dev->BDL = (uart_dev->BDL & ~(UART_BDL_SBR_MASK)) | UART_BDL_SBR(sbr);
  /* Fine adjust */
  uart_dev->C4 = (uart_dev->C4 & ~(UART_C4_BRFA_MASK)) | UART_C4_BRFA(brfa);

  /* Enable FIFO buffers */
  uart_dev->PFIFO |= UART_PFIFO_RXFE_MASK | UART_PFIFO_TXFE_MASK;
  /* Set level to trigger TX interrupt whenever there is space in the TXFIFO (sizeof(TXFIFO) - 1) */
  if ((uart_dev->PFIFO & UART_PFIFO_TXFIFOSIZE_MASK) != 0) {
    uart_dev->TWFIFO = UART_TWFIFO_TXWATER((2 << ((uart_dev->PFIFO & UART_PFIFO_TXFIFOSIZE_MASK) >> UART_PFIFO_TXFIFOSIZE_SHIFT)) - 1);
  }
  else {
    /* Missing hardware support */
    uart_dev->TWFIFO = 0;
  }
  /* Trigger RX interrupt when there is 1 byte or more in the RXFIFO */
  uart_dev->RWFIFO = 1;
  /* Clear all hardware buffers now */
  uart_dev->CFIFO = UART_CFIFO_RXFLUSH_MASK | UART_CFIFO_TXFLUSH_MASK;

  /* Enable transmitter */
  uart_dev->C2 |= UART_C2_TE_MASK;

  /* Set up ring buffer and enable interrupt */
  switch (uart_num) {
#if UART0_CONF_ENABLE
    case 0:
      ringbuf_init(&uart_txbuf[uart_num], &uart0_txbuf_data[0], sizeof(uart0_txbuf_data));
      NVIC_EnableIRQ(UART0_RX_TX_IRQn);
      break;
#endif
#if UART1_CONF_ENABLE
    case 1:
      ringbuf_init(&uart_txbuf[uart_num], &uart1_txbuf_data[0], sizeof(uart1_txbuf_data));
      NVIC_EnableIRQ(UART1_RX_TX_IRQn);
      break;
#endif
#if UART2_CONF_ENABLE
    case 2:
      ringbuf_init(&uart_txbuf[uart_num], &uart2_txbuf_data[0], sizeof(uart2_txbuf_data));
      NVIC_EnableIRQ(UART2_RX_TX_IRQn);
      break;
#endif
#if UART3_CONF_ENABLE
    case 3:
      ringbuf_init(&uart_txbuf[uart_num], &uart3_txbuf_data[0], sizeof(uart3_txbuf_data));
      NVIC_EnableIRQ(UART3_RX_TX_IRQn);
      break;
#endif
#if UART4_CONF_ENABLE
    case 4:
      ringbuf_init(&uart_txbuf[uart_num], &uart4_txbuf_data[0], sizeof(uart4_txbuf_data));
      NVIC_EnableIRQ(UART4_RX_TX_IRQn);
      break;
#endif
#if UART5_CONF_ENABLE
    case 5:
      ringbuf_init(&uart_txbuf[uart_num], &uart5_txbuf_data[0], sizeof(uart5_txbuf_data));
      NVIC_EnableIRQ(UART5_RX_TX_IRQn);
      break;
#endif
  }
}

/*
 * Send char on UART.
 */
void
uart_putchar(const unsigned int uart_num, const char ch)
{
  UART_Type *uart_dev = UART[uart_num];
  /* Try to push to ring buffer until it succeeds, ringbuf_put will return 0
   * when there is no space left. */
  while(ringbuf_put(&uart_txbuf[uart_num], ch) == 0) {
    /* Ringbuffer full, retry. */
    /* TODO: sleep or something */
  }
  /* Successfully added char to ring buffer */
  /* Enable transmitter interrupt, txbuf to UART data register data transfer is
   * performed by the interrupt service routine. */
  uart_dev->C2 |= UART_C2_TIE_MASK;
  return;
}

/*
 * Send string to UART1.
 */
void
uart_putstring(const unsigned int uart_num, const char *str)
{
  const char *p = str;
  while (*p) {
    uart_putchar(uart_num, *p++);
  }
}

void
uart_enable_rx_interrupt(const unsigned int uart_num)
{
  UART_Type *uart_dev = UART[uart_num];
  uart_dev->C2 |= UART_C2_RIE_MASK; /* Enable RDRF interrupt */
  uart_dev->BDH |= UART_BDH_RXEDGIE_MASK; /* Enable edge detect interrupt */
  uart_dev->C2 |= UART_C2_RE_MASK; /* Enable receiver */
  LLWU_INHIBIT_LLS(); /* LLS will disable receiver edge detection */
}

void
uart_disable_rx_interrupt(const unsigned int uart_num)
{
  UART_Type *uart_dev = UART[uart_num];
  uart_dev->C2 &= ~(UART_C2_RIE_MASK); /* Disable RDRF interrupt */
  uart_dev->BDH &= ~(UART_BDH_RXEDGIE_MASK); /* Disable edge detect interrupt */
  uart_dev->C2 &= ~(UART_C2_RE_MASK); /* Disable receiver */
  LLWU_UNINHIBIT_LLS(); /* LLS will disable receiver edge detection */
}

void
uart_set_rx_callback(const unsigned int uart_num, rx_callback_t callback)
{
  rx_callback[uart_num] = callback;
}

#if UART0_CONF_ENABLE
void
_isr_uart0_status()
{
  tx_irq_handler(0);
  rx_irq_handler(0);
}
#endif /* UART0_CONF_ENABLE */

#if UART1_CONF_ENABLE
void
_isr_uart1_status()
{
  tx_irq_handler(1);
  rx_irq_handler(1);
}
#endif /* UART1_CONF_ENABLE */

#if UART2_CONF_ENABLE
void
_isr_uart2_status()
{
  tx_irq_handler(2);
  rx_irq_handler(2);
}
#endif /* UART2_CONF_ENABLE */

#if UART3_CONF_ENABLE
void
_isr_uart3_status()
{
  tx_irq_handler(3);
  rx_irq_handler(3);
}
#endif /* UART3_CONF_ENABLE */

#if UART4_CONF_ENABLE
void
_isr_uart4_status()
{
  tx_irq_handler(4);
  rx_irq_handler(4);
}
#endif /* UART4_CONF_ENABLE */

#if UART5_CONF_ENABLE
void
_isr_uart5_status()
{
  tx_irq_handler(5);
  rx_irq_handler(5);
}
#endif /* UART5_CONF_ENABLE */
