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

#include "ike_debug.h"
#include "net/ip/uip-debug.h"

void ike_memprint(uint8_t *ptr, const uint16_t len)
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

void ike_check_session(ike_statem_session_t *session) {
  uint8_t i;
  printf("Checking Session...------------------\n");

  //struct ike_statem_session *next;
  printf(" Next: %p\n", session->next);

  //uip_ip6addr_t peer;
  printf(" Peer: ");
  uip_debug_ipaddr_print(&session->peer);
  printf("\n");

  //uint16_t initiator_and_my_spi;
  printf(" My SPI:%u\n", uip_htonl((uint32_t)IKE_STATEM_MYSPI_GET_MYSPI(session)));
  printf(" Initiator:%u\n", uip_htonl(IKE_STATEM_IS_INITIATOR(session)));

  //uint32_t peer_spi_high, peer_spi_low;

  //uint8_t my_msg_id, peer_msg_id;
  printf(" My msg ID:%u\n", session->my_msg_id);
  printf(" Peer msg ID:%u\n", session->peer_msg_id);

  //struct ctimer retrans_timer;
  //printf(" Retransmission Time:%u\n", session->retrans_timer);

  //sa_ike_t sa;
  printf(" SA info: \n");
    //sa_encr_transform_type_t encr;    // ESP, IKE
    printf("  encr:  (%3u) - ", session->sa.encr);
      switch (session->sa.encr) {
        case SA_ENCR_RESERVED:
          printf("RESERVED\n");
          break;
        case SA_ENCR_3DES:
          printf("3DES\n");
          break;
        case SA_ENCR_NULL:
          printf("NULL\n");
          break;
        case SA_ENCR_AES_CBC:
          printf("AES CBC\n");
          break;
        case SA_ENCR_AES_CTR:
          printf("AES CTR\n");
          break;
        case SA_ENCR_UNASSIGNED:
          printf("UNASSIGNED\n");
          break;
        default:
          printf("UNKNONW\n");
          break;
      }
    //sa_prf_transform_type_t prf;      // IKE
    printf("  prf:   (%3u) - ", session->sa.prf);
      switch (session->sa.prf) {
        case SA_PRF_RESERVED:
          printf("RESERVED\n");
          break;
        case SA_PRF_HMAC_MD5:
          printf("HMAC MD5\n");
          break;
        case SA_PRF_HMAC_SHA1:
          printf("HMAC SHA1\n");
          break;
        case SA_PRF_AES128_CBC:
          printf("AES128 CBC\n");
          break;
        case SA_PRF_UNASSIGNED:
          printf("UNASSIGNED\n");
          break;
        default:
          printf("UNKNONW\n");
          break;
      }
    //sa_integ_transform_type_t integ;  // IKE, AH, ESP (optional)
    printf("  integ: (%3u) - ", session->sa.integ);
      switch (session->sa.integ) {
        case SA_INTEG_NONE:
          printf("NONE\n");
          break;
        case SA_INTEG_HMAC_MD5_95:
          printf("HMAC MD5 95\n");
          break;
        case SA_INTEG_HMAC_SHA1_96:
          printf("HMAC SHA1 96\n");
          break;
        case SA_INTEG_AES_XCBC_MAC_96:
          printf("AES XCBC MAC 96\n");
          break;
        case SA_INTEG_UNASSIGNED:
          printf("UNASSIGNED\n");
          break;
        default:
          printf("UNKNONW\n");
          break;
      }
    //sa_dh_transform_type_t dh;        // IKE, AH (optional), ESP (optional)
    printf("  dh:    (%3u) - ", session->sa.dh);
      switch (session->sa.dh) {
        case SA_DH_1024_MODP_GROUP:
          printf("1024 MODP GROUP\n");
          break;
        case SA_DH_2048_MODP_GROUP:
          printf("2048 MODP GROUP\n");
          break;
        case SA_DH_256_RND_ECP_GROUP:
          printf("256 RND ECP GROUP\n");
          break;
        case SA_DH_384_RND_ECP_GROUP:
          printf("384 RND ECP GROUP\n");
          break;
        case SA_DH_521_RND_ECP_GROUP:
          printf("521 RND ECP GROUP\n");
          break;
        case SA_DH_1024_MODP_GROUP_160_PRIME:
          printf("1024 MODP GROUP 160 PRIME\n");
          break;
        case SA_DH_2048_MODP_GROUP_224_PRIME:
          printf("2048 MODP GROUP 224 PRIME\n");
          break;
        case SA_DH_2048_MODP_GROUP_256_PRIME:
          printf("2048 MODP GROUP 256 PRIME\n");
          break;
        case SA_DH_192_RND_ECP_GROUP:
          printf("192 RND ECP GROUP\n");
          break;
        case SA_DH_224_RND_ECP_GROUP:
          printf("224 RND ECP GROUP\n");
          break;
        default:
          printf("UNKNONW\n");
          break;
      }
    //uint8_t sk_d[SA_PRF_MAX_OUTPUT_LEN];
    printf("  sk_d:  ");
    for(i=0;i<SA_PRF_MAX_OUTPUT_LEN;i++)
      printf("%3u ", session->sa.sk_d[i]);
    printf("\n");
    //uint8_t sk_ai[SA_INTEG_MAX_KEYMATLEN];
    printf("  sk_ai: ");
    for(i=0;i<SA_PRF_MAX_OUTPUT_LEN;i++)
      printf("%3u ", session->sa.sk_ai[i]);
    printf("\n");
    //uint8_t sk_ar[SA_INTEG_MAX_KEYMATLEN];
    printf("  sk_ar: ");
    for(i=0;i<SA_PRF_MAX_OUTPUT_LEN;i++)
      printf("%3u ", session->sa.sk_ar[i]);
    printf("\n");
    //uint8_t sk_ei[SA_ENCR_MAX_KEYMATLEN];
    printf("  sk_ei: ");
    for(i=0;i<SA_PRF_MAX_OUTPUT_LEN;i++)
      printf("%3u ", session->sa.sk_ei[i]);
    printf("\n");
    //uint8_t sk_er[SA_ENCR_MAX_KEYMATLEN];
    printf("  sk_er: ");
    for(i=0;i<SA_PRF_MAX_OUTPUT_LEN;i++)
      printf("%3u ", session->sa.sk_er[i]);
    printf("\n");
    //uint8_t encr_keylen; // Length of key _in bytes_
    printf("  encrytion key len: %u\n",session->sa.encr_keylen);

  //ike_statem_ephemeral_info_t *ephemeral_info;
    //spd_entry_t *spd_entry;
    //ipsec_addr_set_t my_ts_offer_addr_set;
    //uint32_t my_child_spi;
    //uint32_t peer_child_spi;
    //uint8_t sk_pi[SA_PRF_MAX_PREFERRED_KEYMATLEN];
    //uint8_t sk_pr[SA_PRF_MAX_PREFERRED_KEYMATLEN];
    //uint16_t my_nonce_seed;
    //uint8_t peernonce[IKE_PAYLOAD_PEERNONCE_LEN];
    //uint8_t peernonce_len;
    //uint8_t peer_first_msg[IKE_STATEM_FIRSTMSG_MAXLEN];
    //uint16_t peer_first_msg_len;
    //spd_proposal_tuple_t ike_proposal_reply[IKE_REPLY_MAX_PROPOSAL_TUPLES];
    //spd_proposal_tuple_t child_proposal_reply[IKE_REPLY_MAX_PROPOSAL_TUPLES];
    //NN_DIGIT my_prv_key[IKE_DH_SCALAR_BUF_LEN];

  //ike_payload_generic_hdr_t *cookie_payload;

  //uint16_t (*transition_fn)(struct ike_statem_session *);

  //uint8_t (*next_state_fn)(struct ike_statem_session *);


  printf("-------------------------------------\n\n");

}


void ike_check_offer(spd_proposal_tuple_t *offer){
  uint8_t i;
  printf("Checking Offer...\n");

  for(i=0; i<7; i++){
    printf(" #%u - ", i);
      switch (offer[i].type) {
        case SA_CTRL_TRANSFORM_TYPE_ENCR:
          printf("TRANSFORM ENCR");
          break;
        case SA_CTRL_TRANSFORM_TYPE_PRF:
          printf("TRANSFORM PRF");
          break;
        case SA_CTRL_TRANSFORM_TYPE_INTEG:
          printf("TRANSFORM INTEG");
          break;
        case SA_CTRL_TRANSFORM_TYPE_DH:
          printf("TRANSFORM DH");
          break;
        case SA_CTRL_TRANSFORM_TYPE_ESN:
          printf("TRANSFORM ESN");
          break;
        case SA_CTRL_NEW_PROPOSAL:
          printf("NEW PROPOSAL");
          break;
        case SA_CTRL_ATTRIBUTE_KEY_LEN:
          printf("ATTRIBUTE KEY LEN");
          break;
        case SA_CTRL_END_OF_OFFER:
          printf("END OF OFFER");
          break;
        default:
          printf("UNKNONW", offer[i].type);
          break;
      }
      printf("(%u) = %u\n", offer[i].type, offer[i].value);
  }

}
