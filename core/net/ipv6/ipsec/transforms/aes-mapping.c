/**
 * \file
 *         Mapping the AES software crypto functions
 *         to the generic aes_implem interface
 * \author
 *         Pablo Puñal Pereira <pablo.punal@ltu.se>
 */

#include "ipsec.h"
#include "aes-moo.h"

#if FREESCALE_MMCAU
#include "cau_api.h"

uint32_t key_sch[60];
const int nr = 10;

/*---------------------------------------------------------------------------*/
static void
init(const unsigned char *key)
{
  cau_aes_set_key( key, 128, key_sch);
}
/*---------------------------------------------------------------------------*/
static void
encrypt(unsigned char *buff)
{
  cau_aes_encrypt(buff, key_sch, nr, buff);
}
/*---------------------------------------------------------------------------*/
static void
decrypt(unsigned char *buff)
{
  cau_aes_decrypt(buff, key_sch, nr, buff);
}
/*---------------------------------------------------------------------------*/
struct aes_implem AES_Soft = {
    init,
    encrypt,
    decrypt,
};
/*---------------------------------------------------------------------------*/
#else
#include <aes.h>
aes_context context;

/*---------------------------------------------------------------------------*/
static void
init(const unsigned char *key)
{
  aes_set_key( key, 16, &context );
}
/*---------------------------------------------------------------------------*/
static void
encrypt(unsigned char *buff)
{
  aes_encrypt( buff, buff, &context);
}
/*---------------------------------------------------------------------------*/
static void
decrypt(unsigned char *buff)
{
  aes_decrypt( buff, buff, &context);
}
/*---------------------------------------------------------------------------*/
struct aes_implem AES_Soft = {
    init,
    encrypt,
    decrypt,
};
/*---------------------------------------------------------------------------*/
#endif
