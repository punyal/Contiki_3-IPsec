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

#ifndef IKE_DEBUG_H
#define IKE_DEBUG_H

#include <stdio.h>
#include <stdint.h>
#include "machine.h"

#define NONE    0
#define ERROR   1
#define INFO    2
#define TEST    4
#define FULL    8

#define DEFAULT ERROR

/* Set type of Debug Mode */
#define IKE_DEBUG_MODE TEST

#ifndef IKE_DEBUG_MODE
#define IKE_DEBUG_MODE DEFAULT
#endif

#if (IKE_DEBUG_MODE) & ERROR || (IKE_DEBUG_MODE) & INFO || (IKE_DEBUG_MODE) & TEST || (IKE_DEBUG_MODE) & FULL
#define IKE_ERR(...) \
  do{ \
    printf("[IKEv2]-ERR: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0)
#else
#define IKE_ERR(...)
#endif /* (IKE_DEBUG_MODE) & ERROR */

#if (IKE_DEBUG_MODE) & INFO || (IKE_DEBUG_MODE) & TEST || (IKE_DEBUG_MODE) & FULL
#define IKE_INF(...) \
  do{ \
    printf("[IKEv2]-INF: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0)
#else
#define IKE_INF(...)
#endif /* (IKE_DEBUG_MODE) & INFO */

#if (IKE_DEBUG_MODE) & TEST || (IKE_DEBUG_MODE) & FULL
#define IKE_TST(...) \
  do{ \
    printf("[IKEv2]-TST: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0);
#else
#define IKE_TST(...)
#endif /* (IKE_DEBUG_MODE) & TEST || (IKE_DEBUG_MODE) & FULL */

#if (IKE_DEBUG_MODE) & FULL
#define IKE_DEB(...) \
  do{ \
    printf("[IKEv2]-DEB: ");\
    printf(__VA_ARGS__);\
    printf("\n");\
  }while(0)
#define IKE_MEMPRINTF(str, ptr, len) \
  do {                    \
    printf(str  " (len %u):\n", len);          \
    ike_memprint(ptr, len);   \
  } while(0);
#else
#define IKE_DEB(...)
#define IKE_MEMPRINTF(str, ptr, len)
#endif /* (IKE_DEBUG_MODE) & DEB */


void ike_memprint(uint8_t *ptr, const uint16_t len);

void ike_check_session(ike_statem_session_t *session);
void ike_check_offer(spd_proposal_tuple_t *offer);

#endif /* IKE_DEBUG_H */
