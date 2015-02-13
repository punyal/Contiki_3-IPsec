/*
 * Copyright (c) 2014-2015 Eistec AB.
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
 *         Network initialization for the Mulle platform
 * \author
 *         Henrik Mäkitaavola <henrik.makitaavola@gmail.com>
 *         Joakim Gebart <joakim.gebart@eistec.se
 */

#include <stdio.h>
#include <string.h>
#include "queuebuf.h"
#include "K60.h"

#include "contiki.h"
#include "rf230bb.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

#ifndef NODE_ID
#define NODE_ID 1
#warning Node id = 1
#else
#warning Using user defined node id
#endif

static union {
  uint64_t u64;
  uint32_t u32[sizeof(uint64_t) / sizeof(uint32_t)];
  uint16_t u16[sizeof(uint64_t) / sizeof(uint16_t)];
  uint8_t u8[sizeof(uint64_t) / sizeof(uint8_t)];
} id = { .u8 = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, NODE_ID } };

/** @brief Simple hash function used for generating link-local IPv6 and EUI64
 *         (64 bit) from CPUID (128 bit)
 *
 * @see http://www.cse.yorku.ca/~oz/hash.html */
static uint32_t
djb2_hash(const uint8_t *buf, size_t len)
{
    uint32_t hash = 5381;
    size_t i;

    for (i = 0; i < len; ++i)
        hash = ((hash << 5) + hash) + buf[i]; /* hash * 33 + c */

    return hash;
}

/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  linkaddr_t addr;
  unsigned int i;

  /* memset(&addr, 0x65, sizeof(linkaddr_t)); */
  memcpy(addr.u8, id.u8, sizeof(addr.u8));

  linkaddr_set_node_addr(&addr);
  PRINTF("Rime started with address ");
  PRINTF("%d", addr.u8[0]);
  for(i = 1; i < sizeof(addr.u8); i++) {
    PRINTF(".%d", addr.u8[i]);
  }
  PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
void
init_net(void)
{
#ifndef WITH_SLIP
  uint8_t i;
  id.u32[0] = djb2_hash((const uint8_t *)&(SIM->UIDH), 8); /* Use SIM_UIDH, SIM_UIDMH for first half */
  id.u32[1] = djb2_hash((const uint8_t *)&(SIM->UIDML), 8); /* Use SIM_UIDML, SIM_UIDL for second half */
  id.u8[0] |= 0x02; /* Set the Local/Universal bit to Local */
#else
  /* Use fixed address for border router. */
  id.u32[0] = 0x00000000;
  id.u32[1] = 0x00000000;
  id.u8[0] = 0x02;
  id.u8[7] = 0x01;
#endif
#if NETSTACK_CONF_WITH_IPV6
  set_rime_addr();
  NETSTACK_RADIO.init();
  {
    uint8_t longaddr[8];
    uint16_t shortaddr;

    shortaddr = (linkaddr_node_addr.u8[0] << 8) +
      linkaddr_node_addr.u8[1];
    memset(longaddr, 0, sizeof(longaddr));
    linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
    rf230_set_pan_addr(IEEE802154_CONF_PANID, shortaddr, longaddr);
  }
  rf230_set_channel(RF_CHANNEL);

  memcpy(&uip_lladdr.addr, id.u8, sizeof(uip_lladdr.addr));

  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  PRINTF("%s %s, channel check rate %d Hz, radio channel %d\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1 :
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);

  process_start(&tcpip_process, NULL);

  PRINTF("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      PRINTF("%04x:", lladdr->ipaddr.u8[i * 2] * 256 +
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    PRINTF("%04x\n", lladdr->ipaddr.u8[14] * 256 + lladdr->ipaddr.u8[15]);
  }

  if(!UIP_CONF_IPV6_RPL) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xfdfd, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    PRINTF("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      PRINTF("%04x:",
             ipaddr.u8[i * 2] * 256 + ipaddr.u8[i * 2 + 1]);
    }
    PRINTF("%04x\n",
           ipaddr.u8[7 * 2] * 256 + ipaddr.u8[7 * 2 + 1]);
  }

#else /* If no radio stack should be used only turn on radio and set it to sleep for minimal power consumption */
  rf230_init();
  rf230_driver.off();
#endif /* NETSTACK_CONF_WITH_IPV6 */
}
