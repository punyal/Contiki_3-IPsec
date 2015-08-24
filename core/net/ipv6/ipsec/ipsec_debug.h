/*
 * Copyright (c) 2015, Luleå University of Technology (LTU).
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         A set of debugging macros.
 *
 * \author Pablo Puñal Pereira <pablo.punal@ltu.se>
 */

#ifndef IPSEC_DEBUG_H
#define IPSEC_DEBUG_H

#include <stdio.h>
#include <stdint.h>

#define NONE    0
#define ERROR   1
#define INFO    2
#define TEST    4
#define FULL    8

#define DEFAULT ERROR

/* Set type of Debug Mode */
#define IPSEC_DEBUG_MODE TEST

#ifndef IPSEC_DEBUG_MODE
#define IPSEC_DEBUG_MODE DEFAULT
#endif

#if (IPSEC_DEBUG_MODE) & ERROR || (IPSEC_DEBUG_MODE) & INFO || (IPSEC_DEBUG_MODE) & TEST || (IPSEC_DEBUG_MODE) & FULL
#define IPSEC_ERR(...) \
  do{ \
    printf("[IPSEC]-ERR: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0)
#else
#define IPSEC_ERR(...)
#endif /* (IPSEC_DEBUG_MODE) & ERROR */

#if (IPSEC_DEBUG_MODE) & INFO || (IPSEC_DEBUG_MODE) & TEST || (IPSEC_DEBUG_MODE) & FULL
#define IPSEC_INF(...) \
  do{ \
    printf("[IPSEC]-INF: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0)
#else
#define IPSEC_INF(...)
#endif /* (IPSEC_DEBUG_MODE) & INFO */

#if (IPSEC_DEBUG_MODE) & TEST || (IPSEC_DEBUG_MODE) & FULL
#define IPSEC_TST(...) \
  do{ \
    printf("[IPSECv2]-TST: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0);
#define IPSEC_TST_MEM(str, ptr, len) \
  do {                    \
    printf(str  " (len %u):\n", len);          \
    ipsec_memprint(ptr, len);   \
  } while(0);
#define IPSEC_TST_UIP_IP(ip) \
  do {  \
    printf("vtc:%u tcflow:%u flow:%u len:%u-%u proto:%u ttl:%u\nFrom:\t", \
    ip->vtc, ip->tcflow, ip->flow, ip->len[0], ip->len[1], ip->proto, ip->ttl); \
    uip_debug_ipaddr_print(&ip->srcipaddr); \
    printf("\nTo:\t");\
    uip_debug_ipaddr_print(&ip->destipaddr); \
    printf("\n");\
    ipsec_memprint(ip, ip->len[1]);   \
  }while(0);
#else
#define IPSEC_TST(...)
#define IPSEC_TST_MEM(str, ptr, len)
#define IPSEC_TST_UIP_IP(ip)
#endif /* (IPSEC_DEBUG_MODE) & TEST || (IPSEC_DEBUG_MODE) & FULL */

#if (IPSEC_DEBUG_MODE) & FULL
#define IPSEC_DEB(...) \
  do{ \
    printf("[IPSEC]-DEB: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0)

#define IPSEC_DEB_COM(from,to) \
  do{ \
    printf("[IPSEC]-DEB: New packet from [");\
    uip_debug_ipaddr_print(from);\
    printf("] to [");\
    uip_debug_ipaddr_print(to);\
    printf("]\n");\
  }while(0)

#define IPSEC_DEB_IPSECADDR(addr) \
  do{ \
    printf("[IPSEC]-DEB: IPsec addr [");\
    uip_debug_ipaddr_print(addr->peer_addr);\
    printf("] proto: %u myPort %u peerPort %u\n", addr->nextlayer_proto, uip_ntohs(addr->my_port), uip_ntohs(addr->peer_port));\
  }while(0)

#define IPSEC_MEMPRINTF(str, ptr, len) \
  do {                    \
    printf(str  " (len %u):\n", len);          \
    ipsec_memprint(ptr, len);   \
  } while(0);

#define PRINTSPDENTRY(spd_entry)                              \
    do {                                                        \
      printf("Selector: ");                                     \
      /* PRINTADDRSET(&(spd_entry)->selector);   */                  \
      uint8_t str[3][8] = {                                     \
        { "PROTECT" },                                          \
        { "BYPASS" },                                           \
        { "DISCARD" }                                           \
      };                                                        \
      printf("Action: %s\n", str[(spd_entry)->proc_action]);    \
      printf("Offer at addr: %p\n", (spd_entry)->offer);        \
    } while(0)

#define PRINTSPDLOOKUPADDR(addr)																		\
  	do {																												\
  		printf("SPD lookup for traffic:\n");											\
  		PRINTADDR(addr);																					\
  	} while(0)

#define PRINTFOUNDSPDENTRY(spd_entry)                          \
  	do {                                                         \
    	printf("Found SPD entry:\n");                              \
    	PRINTSPDENTRY(spd_entry);                                  \
  	} while(0)                                                   \

#else
#define IPSEC_DEB(...)
#define IPSEC_DEB_COM(from,to)
#define IPSEC_DEB_IPSECADDR(addr)
#define IPSEC_MEMPRINTF(str, ptr, len)
#define PRINTFOUNDSPDENTRY
#define SDPLOOKUPADDR
#define PRINTSPDENTRY
#endif /* (IPSEC_DEBUG_MODE) & DEB */


void ipsec_memprint(uint8_t *ptr, const uint16_t len);

#endif /* IPSEC_DEBUG_H */
