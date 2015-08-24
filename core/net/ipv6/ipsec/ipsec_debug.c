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

 #include "ipsec_debug.h"

 void ipsec_memprint(uint8_t *ptr, const uint16_t len)
  {
    uint16_t r,s,t,i=0;
    for (r = 0; r < (len / 16) + 1 && i<len; ++r) { // Row
      printf("%p (%4u) ", (uint8_t *) ptr + r * 16, r * 16);
      for (s = 0; s < 4; ++s) { // Group
        for (t = 0; t < 4; ++t){
          if (i++<len) printf("%.2X ", ptr[r * 16 + s * 4 + t]);

        }
      }
      printf("\n");
    }
  }
