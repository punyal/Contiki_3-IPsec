/* Author: Simon Aittamaa */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "K60.h"
#include "spi-k60.h"

#include "flash.h"

#include "clock.h"
#include "udelay.h"

/* ************************************************************************** */
/* #define FLASH_DEBUG */
#ifdef FLASH_DEBUG
#include <stdio.h>
#define DEBUG(...) printf(__VA_ARGS__)
#else
#define DEBUG(...) (void)(0);
#endif

/* ************************************************************************** */

/* By using a union we can make this byte sequence endian agnostic and still
 * perform a 32-bit compare operation on this and the received id. */
static const union {
    uint8_t u8[4];
    uint32_t u32;
} expected_jedec_id = { .u8 = {0x20, 0x20, 0x15, 0x00}}; /** \todo Mulle flash: Handle more chip IDs. */

/* ************************************************************************** */
typedef enum flash_cmd {
  FLASH_CMD_WREN = 0x06,
  FLASH_CMD_WRDI = 0x04,
  FLASH_CMD_RDID = 0x9f,
  FLASH_CMD_RDSR = 0x05,
  FLASH_CMD_WRSR = 0x01,
  FLASH_CMD_EN4B = 0xb7,
  FLASH_CMD_EX4B = 0xe9,
  FLASH_CMD_READ = 0x03,
  FLASH_CMD_FAST_READ = 0x0b,
  FLASH_CMD_RDSFDP = 0x5a,
  FLASH_CMD_2READ = 0xbb,
  FLASH_CMD_DREAD = 0x3b,
  FLASH_CMD_4READ = 0xeb,
  FLASH_CMD_QREAD = 0x6b,
  FLASH_CMD_4PP = 0x38,
  FLASH_CMD_SE = 0xd8,
  /* FLASH_CMD_BE		= 0xd8, */
  FLASH_CMD_BE32K = 0x52,
  FLASH_CMD_CE = 0xc7,
  FLASH_CMD_PP = 0x02,
  FLASH_CMD_CP = 0xad,
  FLASH_CMD_DP = 0xb9,
  FLASH_CMD_RDP = 0xab,
  FLASH_CMD_RES = 0xab,
  FLASH_CMD_REMS = 0x90,
  FLASH_CMD_REMS2 = 0xef,
  FLASH_CMD_REMS4 = 0xdf,
  FLASH_CMD_ENSO = 0xb1,
  FLASH_CMD_EXSO = 0xc1,
  FLASH_CMD_RDSCUR = 0x2b,
  FLASH_CMD_WRSCUR = 0x2f,
  FLASH_CMD_ESRY = 0x70,
  FLASH_CMD_DSRY = 0x80,
  FLASH_CMD_CLSR = 0x30,
  FLASH_CMD_HPM = 0xa3,
  FLASH_CMD_WPSEL = 0x68,
  FLASH_CMD_SBLK = 0x36,
  FLASH_CMD_SBULK = 0x39,
  FLASH_CMD_RDBLOCK = 0x3c,
  FLASH_CMD_GBLK = 0x7e,
  FLASH_CMD_GBULK = 0x98
} flash_cmd_t;

typedef enum flash_status {
  FLASH_STATUS_WIP = 1 << 0,
  FLASH_STATUS_WEL = 1 << 1,
  FLASH_STATUS_SRWD = 1 << 7
} flash_status_t;

typedef enum flash_security {
  FLASH_SECURITY_CP = 1 << 4,
  FLASH_SECURITY_P_FAIL = 1 << 5,
  FLASH_SECURITY_E_FAIL = 1 << 6
} flash_security_t;

/* ************************************************************************** */

static const int FLASH_PAGE_WRITE_SIZE = 32; /** \todo Mulle flash: Make page write size more configurable. */

/* ************************************************************************** */

static struct {
  uint32_t active;
  flash_id_t id;
  uint32_t addr;
  const uint8_t *data;
  uint32_t size;
} scheduled_write;

/* ************************************************************************** */

static void   cmd_wrdi(const flash_id_t);
static void cmd_wren(const flash_id_t);
static uint8_t cmd_rdscur(const flash_id_t);
static uint8_t cmd_rdsr(const flash_id_t);
static uint32_t cmd_rdid(const flash_id_t);
static uint32_t cmd_pp(const flash_id_t, const flash_addr_t, const uint8_t *, const uint32_t);
static void cmd_se(const flash_id_t, const uint32_t);
static void cmd_ce(const flash_id_t);

/* ************************************************************************** */

static void
spi_write_addr(const flash_id_t id, const spi_transfer_flag_t flag, const flash_addr_t addr)
{
  uint8_t data_out[3];
  data_out[0] = (addr >> 16) & 0xff;
  data_out[1] = (addr >> 8) & 0xff;
  data_out[2] = addr & 0xff;
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, flag, &data_out[0], NULL, 3, 0);
}
/* ************************************************************************** */

/** Read ID */
static uint32_t
cmd_rdid(const flash_id_t id)
{
  static const uint8_t data_out = FLASH_CMD_RDID;
  union {
    uint8_t u8[4];
    uint32_t u32;
  } data_in = { .u32 = 0 };

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &data_out, &data_in.u8[0], 1, 3);

  return data_in.u32;
}
/* ************************************************************************** */

/** Read status register */
static uint8_t
cmd_rdsr(const flash_id_t id)
{
  static const uint8_t data_out = FLASH_CMD_RDSR;
  uint8_t data_in;

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &data_out, &data_in, 1, 1);

  return data_in;
}
/* ************************************************************************** */

static uint8_t
cmd_rdscur(const flash_id_t id)
{
  uint8_t data_in;
  static const uint8_t data_out = FLASH_CMD_RDSCUR;

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &data_out, &data_in, 1, 1);

  return data_in;
}
/* ************************************************************************** */

static void
cmd_wren(const flash_id_t id)
{
  static const uint8_t data_out = FLASH_CMD_WREN;

  while(!(cmd_rdsr(id) & FLASH_STATUS_WEL)) {
    spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &data_out, NULL, 1, 0);
  }
}
/* ************************************************************************** */

static void
cmd_wrdi(const flash_id_t id)
{
  static const uint8_t data_out = FLASH_CMD_WRDI;

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &data_out, NULL, 1, 0);
}
/* ************************************************************************** */

/* Page program */
static uint32_t
cmd_pp(const flash_id_t id, const flash_addr_t addr, const uint8_t *data, const uint32_t size)
{
  static const uint8_t cmd_out = FLASH_CMD_PP;

  if(size == 0) {
    return E_FLASH_OK;
  }

  /* Send command */
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_CONT, &cmd_out, NULL, 1, 0);

  /* Send address */
  spi_write_addr(id, SPI_TRANSFER_CONT, addr);

  /* Send data */
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, data, NULL, size, 0);

/*
   if (cmd_rdscur(id) & FLASH_SECURITY_P_FAIL) {
    return E_FLASH_WRITE_FAILED;
   }
 */

  return E_FLASH_OK;
}
/* ************************************************************************** */

/* Page program, 1's complement */
static uint32_t
cmd_ppi(const flash_id_t id, const flash_addr_t addr, const uint8_t *data, const uint32_t size)
{
  static const uint8_t cmd_out = FLASH_CMD_PP;
  uint8_t data_out;
  uint32_t i;

  if(size == 0) {
    return E_FLASH_OK;
  }

  /* Send command */
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_CONT, &cmd_out, NULL, 1, 0);

  /* Send address */
  spi_write_addr(id, SPI_TRANSFER_CONT, addr);

  /* Send data */
  /** \todo Smarter invert method in flash cmd_ppi */
  for(i = 0; i < size - 1; ++i) {
    data_out = ~(*(data++));
    spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_CONT, &data_out, NULL, 1, 0);
  }
  data_out = ~(*(data++));
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &data_out, NULL, 1, 0);

/*  if (cmd_rdscur(id) & FLASH_SECURITY_P_FAIL) { */
/*    return E_FLASH_WRITE_FAILED; */
/*  } */

  return E_FLASH_OK;
}
/* ************************************************************************** */

/* Sector erase */
static void
cmd_se(const flash_id_t id, const uint32_t sector)
{
  static const uint8_t cmd_out = FLASH_CMD_SE;
  uint32_t addr = sector * FLASH_SECTOR_SIZE;

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_CONT, &cmd_out, NULL, 1, 0);

  spi_write_addr(id, SPI_TRANSFER_DONE, addr);
}
/* ************************************************************************** */

/* Bulk erase (chip erase) */
static void
cmd_ce(const flash_id_t id)
{
  static const uint8_t cmd_out = FLASH_CMD_CE;

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &cmd_out, NULL, 1, 0);
}
/* ************************************************************************** */
/* Deep power down */
static void
cmd_dp(const flash_id_t id)
{
  static const uint8_t cmd_out = FLASH_CMD_DP;

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &cmd_out, NULL, 1, 0);
}
/* ************************************************************************** */

/* Release from deep power down */
static void
cmd_rdp(const flash_id_t id)
{
  static const uint8_t cmd_out = FLASH_CMD_RDP;

  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, &cmd_out, NULL, 1, 0);
}
/* ************************************************************************** */

static void
cmd_read(const flash_id_t id, const flash_addr_t addr, uint8_t *dest, const uint32_t size)
{
  static const uint8_t cmd_out = FLASH_CMD_READ;

  /* Send command */
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_CONT, &cmd_out, NULL, 1, 0);

  /* Send address */
  spi_write_addr(id, SPI_TRANSFER_CONT, addr);

  /* Read data bytes */
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, NULL, dest, 0, size);
}
/* ************************************************************************** */

static void
cmd_readi(const flash_id_t id, const flash_addr_t addr, uint8_t *dest, const uint32_t size)
{
  static const uint8_t cmd_out = FLASH_CMD_READ;
  uint32_t i;

  /* Send command */
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_CONT, &cmd_out, NULL, 1, 0);

  /* Send address */
  spi_write_addr(id, SPI_TRANSFER_CONT, addr);

  /* Read data bytes */
  spi_transfer_blocking(FLASH_SPI_NUM, FLASH_CTAS, id, SPI_TRANSFER_DONE, NULL, dest, 0, size);

  /* 1's complement */
  for(i = 0; i < size; ++i) {
    dest[i] = ~dest[i];
  }
}
/* ************************************************************************** */

static void
flash_busy_wait(const flash_id_t id)
{
  uint8_t status;
  spi_acquire_bus(FLASH_SPI_NUM);
  status = cmd_rdsr(id);
  spi_release_bus(FLASH_SPI_NUM);

  while(status & FLASH_STATUS_WIP) {
    /* XXX: Attempt to fix slow erase times on some chips by not flooding the device with status requests */
    udelay(10000);
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(id);
    spi_release_bus(FLASH_SPI_NUM);
  }
}
/* ************************************************************************** */

static void
flash_write_prepare(const flash_id_t id)
{
  flash_busy_wait(id);
  spi_acquire_bus(FLASH_SPI_NUM);
  cmd_wren(id);
  spi_release_bus(FLASH_SPI_NUM);
}
/* ************************************************************************** */

flash_error_t
flash_init(void)
{
  uint32_t status;
  uint32_t jedec_id;
  int i;

  /* Wait a while for memories to start */
  /* Data sheet for M25P16 says this should be around 10 ms max, but this may
   * also depend on how much decoupling is used for the power circuit. */
  /* TODO(henrik) Change this to more exact times. */
  for(i = 0; i < 200; ++i) {
    udelay(1000);
  }

  for(i = FLASH_ID0; i <= FLASH_ID0; ++i) {
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(i);
    spi_release_bus(FLASH_SPI_NUM);
    if(status) {
      DEBUG("Error: Status of flash 0 is non-zero (0x%02x).\n", status);
      return E_FLASH_INVALID_STATUS;
    }
    spi_acquire_bus(FLASH_SPI_NUM);
    jedec_id = cmd_rdid(i);
    spi_release_bus(FLASH_SPI_NUM);
    if(jedec_id != expected_jedec_id.u32) {
      DEBUG("Flash0: Invalid JEDEC-ID: 0x%08x\n", jedec_id);
      return E_FLASH_UNKNOWN;
    }
  }

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_erase_chip(const flash_id_t id, const flash_flags_t flags)
{
  DEBUG("Starting chip erase...");

  if((flags & FLASH_WAIT) == 0) {
    uint8_t status;
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(id);
    spi_release_bus(FLASH_SPI_NUM);

    if(status & FLASH_STATUS_WIP) {
      return E_FLASH_BUSY;
    }
  }

  flash_write_prepare(id);

  spi_acquire_bus(FLASH_SPI_NUM);
  cmd_ce(id);
  spi_release_bus(FLASH_SPI_NUM);

  if(flags & FLASH_FINISH) {
    flash_busy_wait(id);
/*    if (cmd_rdscur(id) & FLASH_SECURITY_E_FAIL) { */
/*      DEBUG("failed!\n"); */
/*    } */
    spi_acquire_bus(FLASH_SPI_NUM);
    cmd_wrdi(id);
    spi_release_bus(FLASH_SPI_NUM);
  }

  DEBUG("done.\n");

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_erase_sector(const flash_id_t id, const uint32_t sector, const flash_flags_t flags)
{
  if((flags & FLASH_WAIT) == 0) {
    uint8_t status;
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(id);
    spi_release_bus(FLASH_SPI_NUM);

    if(status & FLASH_STATUS_WIP) {
      return E_FLASH_BUSY;
    }
  }

  DEBUG("Starting sector erase...");

  flash_write_prepare(id);

  spi_acquire_bus(FLASH_SPI_NUM);
  cmd_se(id, sector);
  spi_release_bus(FLASH_SPI_NUM);

  if(flags & FLASH_FINISH) {
    flash_busy_wait(id);
/*    if (cmd_rdscur(id) & FLASH_SECURITY_E_FAIL) { */
/*      DEBUG("failed! (0x%02x)\n"); */
/*      return E_FLASH_ERASE_FAILED; */
/*    } */
    spi_acquire_bus(FLASH_SPI_NUM);
    cmd_wrdi(id);
    spi_release_bus(FLASH_SPI_NUM);
  }

  DEBUG("done.\n");

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_write_queue(const flash_id_t id, const flash_addr_t addr, const uint8_t *data, const uint32_t size, const flash_flags_t flags)
{
  if(scheduled_write.active) {
    DEBUG("We already have a scheduled write.\n");
    return E_FLASH_QUEUE_FULL;
  }

  if(size == 0) {
    return E_FLASH_OK;
  }

  scheduled_write.active = 1;
  scheduled_write.id = id;
  scheduled_write.addr = addr;
  scheduled_write.data = data;
  scheduled_write.size = size;

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_write_process(const flash_flags_t flags)
{
  uint32_t count;

  if(scheduled_write.active == 0) {
    return E_FLASH_OK;
  }

  if((flags & FLASH_WAIT) == 0) {
    uint8_t status;
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(scheduled_write.id);
    spi_release_bus(FLASH_SPI_NUM);

    if(status & FLASH_STATUS_WIP) {
      return E_FLASH_BUSY;
    }
  }

  flash_write_prepare(scheduled_write.id);

  /* NOTE: FLASH_PAGE_WRITE_SIZE controls how much we write each time... */
  if(FLASH_PAGE_WRITE_SIZE >= FLASH_PAGE_SIZE) {
    DEBUG("Sanity-check failed! We are trying to write too large chunks!\n");
  }
  count = FLASH_PAGE_WRITE_SIZE - (scheduled_write.addr % FLASH_PAGE_WRITE_SIZE);
  if(scheduled_write.size < count) {
    count = scheduled_write.size;
  }

  if(cmd_pp(scheduled_write.id, scheduled_write.addr, scheduled_write.data, count) != E_FLASH_OK) {
    DEBUG("flash_write_process(): Page-programming failed.\n");
    return E_FLASH_WRITE_FAILED;
  }

  scheduled_write.size -= count;
  scheduled_write.addr += count;
  scheduled_write.data += count;

  if(flags & FLASH_FINISH) {
    uint8_t scur;

    flash_busy_wait(scheduled_write.id);
    spi_acquire_bus(FLASH_SPI_NUM);
    scur = cmd_rdscur(scheduled_write.id);
    spi_release_bus(FLASH_SPI_NUM);

    if(scur & FLASH_SECURITY_P_FAIL) {
      DEBUG("flash_write_process(): Page-programming failed.\n");
      return E_FLASH_WRITE_FAILED;
    }
  }

  if(scheduled_write.size == 0) {
    scheduled_write.active = 0;
    scheduled_write.id = 0;
    scheduled_write.addr = 0;
    scheduled_write.data = 0;
    return E_FLASH_OK;
  }

  return E_FLASH_BUSY;
}
/* ************************************************************************** */

flash_error_t
flash_write(const flash_id_t id, const flash_addr_t addr, const uint8_t *data, const uint32_t size, const flash_flags_t flags)
{
  uint32_t count;
  uint32_t offset = 0;

  DEBUG("Starting flash write operation...");

  if((flags & FLASH_WAIT) == 0) {
    uint8_t status;
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(id);
    spi_release_bus(FLASH_SPI_NUM);

    if(status & FLASH_STATUS_WIP) {
      return E_FLASH_BUSY;
    }
  }

  if(size == 0) {
    return E_FLASH_OK;
  }

  count = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);
  if(size > count) {
    uint8_t status;
    flash_write_prepare(id);
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_pp(id, addr + offset, data + offset, count);
    spi_release_bus(FLASH_SPI_NUM);

    if(status != E_FLASH_OK) {
      DEBUG("failed!\n");
      return E_FLASH_WRITE_FAILED;
    }

    offset += count;
  }

  while(offset < size) {
    uint8_t status;
    count = size - offset;
    if(count > FLASH_PAGE_SIZE) {
      count = FLASH_PAGE_SIZE;
    }

    flash_write_prepare(id);
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_pp(id, addr + offset, data + offset, count);
    spi_release_bus(FLASH_SPI_NUM);
    if(status != E_FLASH_OK) {
      DEBUG("failed!\n");
      return E_FLASH_WRITE_FAILED;
    }

    offset += count;
  }

  if(flags & FLASH_FINISH) {
    flash_busy_wait(id);
  }

  DEBUG("done.\n");

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_writei(const flash_id_t id, const flash_addr_t addr, const uint8_t *data, const uint32_t size, const flash_flags_t flags)
{
  uint32_t count;
  uint32_t offset = 0;

  DEBUG("Starting flash write operation...");

  if((flags & FLASH_WAIT) == 0) {
    uint8_t status;
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(id);
    spi_release_bus(FLASH_SPI_NUM);

    if(status & FLASH_STATUS_WIP) {
      return E_FLASH_BUSY;
    }
  }

  if(size == 0) {
    return E_FLASH_OK;
  }

  count = FLASH_PAGE_SIZE - (addr % FLASH_PAGE_SIZE);
  if(size > count) {
    uint8_t status;
    flash_write_prepare(id);
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_ppi(id, addr + offset, data + offset, count);
    spi_release_bus(FLASH_SPI_NUM);

    if(status != E_FLASH_OK) {
      DEBUG("failed!\n");
      return E_FLASH_WRITE_FAILED;
    }

    offset += count;
  }

  while(offset < size) {
    uint8_t status;
    count = size - offset;
    if(count > FLASH_PAGE_SIZE) {
      count = FLASH_PAGE_SIZE;
    }

    flash_write_prepare(id);
    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_ppi(id, addr + offset, data + offset, count);
    spi_release_bus(FLASH_SPI_NUM);
    if(status != E_FLASH_OK) {
      DEBUG("failed!\n");
      return E_FLASH_WRITE_FAILED;
    }

    offset += count;
  }

  if(flags & FLASH_FINISH) {
    flash_busy_wait(id);
  }

  DEBUG("done.\n");

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_read(const flash_id_t id, const flash_addr_t addr, uint8_t *dest, const uint32_t size, const flash_flags_t flags)
{
  if(flags & FLASH_WAIT) {
    flash_busy_wait(id);
  } else {
    uint8_t status;

    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(id);
    spi_release_bus(FLASH_SPI_NUM);

    if(status & FLASH_STATUS_WIP) {
      return E_FLASH_BUSY;
    }
  }

  if(size == 0) {
    return E_FLASH_OK;
  }

  DEBUG("Starting flash read operation...");
  spi_acquire_bus(FLASH_SPI_NUM);
  cmd_read(id, addr, dest, size);
  spi_release_bus(FLASH_SPI_NUM);
  DEBUG("done.\n");

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_readi(const flash_id_t id, const flash_addr_t addr, uint8_t *dest, const uint32_t size, const flash_flags_t flags)
{
  if(flags & FLASH_WAIT) {
    flash_busy_wait(id);
  } else {
    uint8_t status;

    spi_acquire_bus(FLASH_SPI_NUM);
    status = cmd_rdsr(id);
    spi_release_bus(FLASH_SPI_NUM);

    if(status & FLASH_STATUS_WIP) {
      return E_FLASH_BUSY;
    }
  }

  if(size == 0) {
    return E_FLASH_OK;
  }

  DEBUG("Starting flash read operation...");
  spi_acquire_bus(FLASH_SPI_NUM);
  cmd_readi(id, addr, dest, size);
  spi_release_bus(FLASH_SPI_NUM);
  DEBUG("done.\n");

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_status(const flash_id_t id)
{
  if(cmd_rdsr(id) & FLASH_STATUS_WIP) {
    return E_FLASH_BUSY;
  } else if(scheduled_write.active) {
    return E_FLASH_BUSY;
  }

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_sleep(const flash_id_t id, const flash_flags_t flags)
{
  spi_acquire_bus(FLASH_SPI_NUM);

  cmd_dp(id);

  spi_release_bus(FLASH_SPI_NUM);

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_wakeup(const flash_id_t id, const flash_flags_t flags)
{
  spi_acquire_bus(FLASH_SPI_NUM);

  cmd_rdp(id);

  spi_release_bus(FLASH_SPI_NUM);

  return E_FLASH_OK;
}
/* ************************************************************************** */

flash_error_t
flash_dump(const flash_id_t id, void (*dump)(const uint8_t))
{
  flash_addr_t offset = 0;
  uint8_t buf[FLASH_PAGE_SIZE];

  while(offset < FLASH_SIZE) {
    int i;
    flash_error_t err;

    err = flash_read(id, offset, buf, sizeof(buf), FLASH_BLOCKING);
    if(err != E_FLASH_OK) {
      DEBUG("Reading from flash failed while dumping (0x%02x).\r\n", err);
      return E_FLASH_WRITE_FAILED;
    }

    for(i = 0; i < sizeof(buf); i++) {
      if(dump) {
        dump(buf[i]);
      }
    }

    offset += sizeof(buf);
  }

  return E_FLASH_OK;
}
