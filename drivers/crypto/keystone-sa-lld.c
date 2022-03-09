/*
 * Keystone crypto accelerator driver
 *
 * Copyright (C) 2015, 2016 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors:	Sandeep Nair
 *		Vitaly Andrianov
 *
 * Contributors:Tinku Mannan
 *		Hao Zhang
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/types.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>

#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/sha.h>
#include <crypto/md5.h>

#include "keystone-sa.h"
#include "keystone-sa-hlp.h"

/* Byte offset for key in encryption security context */
#define SC_ENC_KEY_OFFSET (1 + 27 + 4)
/* Byte offset for Aux-1 in encryption security context */
#define SC_ENC_AUX1_OFFSET (1 + 27 + 4 + 32)

struct sa_eng_mci_tbl sa_mci_tbl;

/* Perform 16 byte swizzling */
void sa_swiz_128(u8 *in, u8 *out, u16 len)
{
	u8 data[16];
	int i, j;

	for (i = 0; i < len; i += 16) {
		memcpy(data, &in[i], 16);
		for (j = 0; j < 16; j++)
			out[i + j] = data[15 - j];
	}
}

/* Convert CRA name to internal algorithm ID */
void sa_conv_calg_to_salg(const char *cra_name, int *ealg_id, int *aalg_id)
{
	*ealg_id = SA_EALG_ID_NONE;
	*aalg_id = SA_AALG_ID_NONE;

	if (!strcmp(cra_name, "authenc(hmac(sha1),cbc(aes))")) {
		*ealg_id = SA_EALG_ID_AES_CBC;
		*aalg_id = SA_AALG_ID_HMAC_SHA1;
	} else if (!strcmp(cra_name, "authenc(hmac(sha1),ecb(cipher_null))")) {
		*ealg_id = SA_EALG_ID_NULL;
		*aalg_id = SA_AALG_ID_HMAC_SHA1;
	} else if (!strcmp(cra_name, "authenc(hmac(sha1),cbc(des3_ede))")) {
		*ealg_id = SA_EALG_ID_3DES_CBC;
		*aalg_id = SA_AALG_ID_HMAC_SHA1;
	} else if (!strcmp(cra_name, "authenc(xcbc(aes),cbc(aes))")) {
		*ealg_id = SA_EALG_ID_AES_CBC;
		*aalg_id = SA_AALG_ID_AES_XCBC;
	} else if (!strcmp(cra_name, "authenc(xcbc(aes),cbc(des3_ede))")) {
		*ealg_id = SA_EALG_ID_3DES_CBC;
		*aalg_id = SA_AALG_ID_AES_XCBC;
	} else if (!strcmp(cra_name, "cbc(aes)")) {
		*ealg_id = SA_EALG_ID_AES_CBC;
	} else if (!strcmp(cra_name, "cbc(des3_ede)")) {
		*ealg_id = SA_EALG_ID_3DES_CBC;
	} else if (!strcmp(cra_name, "hmac(sha1)")) {
		*aalg_id = SA_AALG_ID_HMAC_SHA1;
	} else if (!strcmp(cra_name, "xcbc(aes)")) {
		*aalg_id = SA_AALG_ID_AES_XCBC;
	}
}

/* Given an algorithm ID get the engine details */
void sa_get_engine_info(int alg_id, struct sa_eng_info *info)
{
	switch (alg_id) {
	case SA_EALG_ID_AES_CBC:
	case SA_EALG_ID_3DES_CBC:
	case SA_EALG_ID_DES_CBC:
		info->eng_id = SA_ENG_ID_EM1;
		info->sc_size = SA_CTX_ENC_TYPE1_SZ;
		break;

	case SA_EALG_ID_NULL:
		info->eng_id = SA_ENG_ID_NONE;
		info->sc_size = 0;
		break;

	case SA_AALG_ID_HMAC_SHA1:
	case SA_AALG_ID_HMAC_MD5:
		info->eng_id = SA_ENG_ID_AM1;
		info->sc_size = SA_CTX_AUTH_TYPE2_SZ;
		break;

	case SA_AALG_ID_AES_XCBC:
	case SA_AALG_ID_CMAC:
		info->eng_id = SA_ENG_ID_EM1;
		info->sc_size = SA_CTX_AUTH_TYPE1_SZ;
		break;

	default:
		pr_err("%s: unsupported algo\n", __func__);
		info->eng_id = SA_ENG_ID_NONE;
		info->sc_size = 0;
		break;
	}
}

/* Given an algorithm get the hash size */
int sa_get_hash_size(u16 aalg_id)
{
	int hash_size = 0;

	switch (aalg_id) {
	case SA_AALG_ID_MD5:
	case SA_AALG_ID_HMAC_MD5:
		hash_size = MD5_DIGEST_SIZE;
		break;

	case SA_AALG_ID_SHA1:
	case SA_AALG_ID_HMAC_SHA1:
		hash_size = SHA1_DIGEST_SIZE;
		break;

	case SA_AALG_ID_SHA2_224:
	case SA_AALG_ID_HMAC_SHA2_224:
		hash_size = SHA224_DIGEST_SIZE;
		break;

	case SA_AALG_ID_SHA2_256:
	case SA_AALG_ID_HMAC_SHA2_256:
		hash_size = SHA256_DIGEST_SIZE;
		break;

	case SA_AALG_ID_AES_XCBC:
	case SA_AALG_ID_CMAC:
		hash_size = AES_BLOCK_SIZE;
		break;

	default:
		pr_err("%s: unsupported hash\n", __func__);
		break;
	}

	return hash_size;
}

/* Initialize MD5 digest */
static inline void md5_init(u32 *hash)
{
	/* Load magic initialization constants */
	hash[0] = 0x67452301;
	hash[1] = 0xefcdab89;
	hash[2] = 0x98badcfe;
	hash[3] = 0x10325476;
}

/* Generate HMAC-MD5 intermediate Hash */
static void sa_hmac_md5_get_pad(const u8 *key, u16 key_sz, u32 *ipad, u32 *opad)
{
	u8 k_ipad[MD5_MESSAGE_BYTES];
	u8 k_opad[MD5_MESSAGE_BYTES];
	int i;

	for (i = 0; i < key_sz; i++) {
		k_ipad[i] = key[i] ^ 0x36;
		k_opad[i] = key[i] ^ 0x5c;
	}
	/* Instead of XOR with 0 */
	for (; i < SHA_MESSAGE_BYTES; i++) {
		k_ipad[i] = 0x36;
		k_opad[i] = 0x5c;
	}

	/* SHA-1 on k_ipad */
	md5_init(ipad);
	md5_transform(ipad, (u32 *)k_ipad);

	/* SHA-1 on k_opad */
	md5_init(opad);
	md5_transform(ipad, (u32 *)k_opad);
}

/* Generate HMAC-SHA1 intermediate Hash */
static
void sa_hmac_sha1_get_pad(const u8 *key, u16 key_sz, u32 *ipad, u32 *opad)
{
	u32 ws[SHA_WORKSPACE_WORDS];
	u8 k_ipad[SHA_MESSAGE_BYTES];
	u8 k_opad[SHA_MESSAGE_BYTES];
	int i;

	for (i = 0; i < key_sz; i++) {
		k_ipad[i] = key[i] ^ 0x36;
		k_opad[i] = key[i] ^ 0x5c;
	}
	/* Instead of XOR with 0 */
	for (; i < SHA_MESSAGE_BYTES; i++) {
		k_ipad[i] = 0x36;
		k_opad[i] = 0x5c;
	}

	/* SHA-1 on k_ipad */
	sha_init(ipad);
	sha_transform(ipad, k_ipad, ws);

	for (i = 0; i < SHA_DIGEST_WORDS; i++)
		ipad[i] = cpu_to_be32(ipad[i]);

	/* SHA-1 on k_opad */
	sha_init(opad);
	sha_transform(opad, k_opad, ws);

	for (i = 0; i < SHA_DIGEST_WORDS; i++)
		opad[i] = cpu_to_be32(opad[i]);
}

/* Derive the inverse key used in AES-CBC decryption operation */
static inline int sa_aes_inv_key(u8 *inv_key, const u8 *key, u16 key_sz)
{
	struct crypto_aes_ctx ctx;
	int key_pos;

	if (crypto_aes_expand_key(&ctx, key, key_sz)) {
		pr_err("%s: bad key len(%d)\n", __func__, key_sz);
		return -1;
	}

	/* Refer the implementation of crypto_aes_expand_key()
	 * to understand the below logic
	 */
	switch (key_sz) {
	case AES_KEYSIZE_128:
	case AES_KEYSIZE_192:
		key_pos = key_sz + 24;
		break;

	case AES_KEYSIZE_256:
		key_pos = key_sz + 24 - 4;
		break;

	default:
		pr_err("%s: bad key len(%d)\n", __func__, key_sz);
		return -1;
	}

	memcpy(inv_key, &ctx.key_enc[key_pos], key_sz);
	return 0;
}

/* Set Security context for the encryption engine */
int sa_set_sc_enc(u16 alg_id, const u8 *key, u16 key_sz,
		  u16 aad_len, u8 enc, u8 *sc_buf)
{
	u8 ghash[16]; /* AES block size */
	const u8 *mci = NULL;
	/* Convert the key size (16/24/32) to the key size index (0/1/2) */
	int key_idx = (key_sz >> 3) - 2;

	/* Set Encryption mode selector to crypto processing */
	sc_buf[0] = 0;

	/* Select the mode control instruction */
	switch (alg_id) {
	case SA_EALG_ID_AES_CBC:
		mci = (enc) ? sa_mci_tbl.aes_enc[SA_ENG_ALGO_CBC][key_idx] :
			sa_mci_tbl.aes_dec[SA_ENG_ALGO_CBC][key_idx];
		break;

	case SA_EALG_ID_CCM:
		mci = (enc) ? sa_mci_tbl.aes_enc[SA_ENG_ALGO_CCM][key_idx] :
			sa_mci_tbl.aes_dec[SA_ENG_ALGO_CCM][key_idx];
		break;

	case SA_EALG_ID_AES_F8:
		mci = sa_mci_tbl.aes_enc[SA_ENG_ALGO_F8][key_idx];
		break;

	case SA_EALG_ID_AES_CTR:
		mci = sa_mci_tbl.aes_enc[SA_ENG_ALGO_CTR][key_idx];
		break;

	case SA_EALG_ID_GCM:
		mci = (enc) ? sa_mci_tbl.aes_enc[SA_ENG_ALGO_GCM][key_idx] :
			sa_mci_tbl.aes_dec[SA_ENG_ALGO_GCM][key_idx];
		/* Set AAD length at byte offset 23 in Aux-1 */
		sc_buf[SC_ENC_AUX1_OFFSET + 23] = (aad_len << 3);
		/* fall through to GMAC */

	case SA_AALG_ID_GMAC:
		/* copy GCM Hash in Aux-1 */
		memcpy(&sc_buf[SC_ENC_AUX1_OFFSET], ghash, 16);
		break;

	case SA_AALG_ID_AES_XCBC:
	case SA_AALG_ID_CMAC:
		mci = sa_mci_tbl.aes_enc[SA_ENG_ALGO_CMAC][key_idx];
		break;

	case SA_AALG_ID_CBC_MAC:
		mci = sa_mci_tbl.aes_enc[SA_ENG_ALGO_CBCMAC][key_idx];
		break;

	case SA_EALG_ID_3DES_CBC:
		mci = (enc) ? sa_mci_tbl._3des_enc[SA_ENG_ALGO_CBC] :
			sa_mci_tbl._3des_dec[SA_ENG_ALGO_CBC];
		break;
	}

	/* Set the mode control instructions in security context */
	if (mci)
		memcpy(&sc_buf[1], mci, 27);

	/* For AES-CBC decryption get the inverse key */
	if ((alg_id == SA_EALG_ID_AES_CBC) && !enc) {
		if (sa_aes_inv_key(&sc_buf[SC_ENC_KEY_OFFSET], key, key_sz))
			return -1;
	}
	/* For AES-XCBC-MAC get the subkey */
	else if (alg_id == SA_AALG_ID_AES_XCBC) {
		if (sa_aes_xcbc_subkey(&sc_buf[SC_ENC_KEY_OFFSET], NULL,
				       NULL, key, key_sz))
			return -1;
	}
	/* For all other cases: key is used */
	else
		memcpy(&sc_buf[SC_ENC_KEY_OFFSET], key, key_sz);

	return 0;
}

/* Set Security context for the authentication engine */
void sa_set_sc_auth(u16 alg_id, const u8 *key, u16 key_sz, u8 *sc_buf)
{
	u32 ipad[8], opad[8];
	u8 mac_sz, keyed_mac = 0;

	/* Set Authentication mode selector to hash processing */
	sc_buf[0] = 0;

	/* Auth SW ctrl word: bit[6]=1 (upload computed hash to TLR section) */
	sc_buf[1] = 0x40;

	switch (alg_id) {
	case SA_AALG_ID_MD5:
		/* Auth SW ctrl word: bit[4]=1 (basic hash)
		 * bit[3:0]=1 (MD5 operation)*/
		sc_buf[1] |= (0x10 | 0x1);
		break;

	case SA_AALG_ID_SHA1:
		/* Auth SW ctrl word: bit[4]=1 (basic hash)
		 * bit[3:0]=2 (SHA1 operation)*/
		sc_buf[1] |= (0x10 | 0x2);
		break;

	case SA_AALG_ID_SHA2_224:
		/* Auth SW ctrl word: bit[4]=1 (basic hash)
		 * bit[3:0]=3 (SHA2-224 operation)*/
		sc_buf[1] |= (0x10 | 0x3);
		break;

	case SA_AALG_ID_SHA2_256:
		/* Auth SW ctrl word: bit[4]=1 (basic hash)
		 * bit[3:0]=4 (SHA2-256 operation)*/
		sc_buf[1] |= (0x10 | 0x4);
		break;

	case SA_AALG_ID_HMAC_MD5:
		/* Auth SW ctrl word: bit[4]=0 (HMAC)
		 * bit[3:0]=1 (MD5 operation)*/
		sc_buf[1] |= 0x1;
		keyed_mac = 1;
		mac_sz = MD5_DIGEST_SIZE;
		sa_hmac_md5_get_pad(key, key_sz, ipad, opad);
		break;

	case SA_AALG_ID_HMAC_SHA1:
		/* Auth SW ctrl word: bit[4]=0 (HMAC)
		 * bit[3:0]=2 (SHA1 operation)*/
		sc_buf[1] |= 0x2;
		keyed_mac = 1;
		mac_sz = SHA1_DIGEST_SIZE;
		sa_hmac_sha1_get_pad(key, key_sz, ipad, opad);
		break;

	case SA_AALG_ID_HMAC_SHA2_224:
		/* Auth SW ctrl word: bit[4]=0 (HMAC)
		 * bit[3:0]=3 (SHA2-224 operation)*/
		sc_buf[1] |= 0x3;
		keyed_mac = 1;
		mac_sz = SHA224_DIGEST_SIZE;
		break;

	case SA_AALG_ID_HMAC_SHA2_256:
		/* Auth SW ctrl word: bit[4]=0 (HMAC)
		 * bit[3:0]=4 (SHA2-256 operation)*/
		sc_buf[1] |= 0x4;
		keyed_mac = 1;
		mac_sz = SHA256_DIGEST_SIZE;
		break;
	}

	/* Copy the keys or ipad/opad */
	if (keyed_mac) {
		/* Copy ipad to AuthKey */
		memcpy(&sc_buf[32], ipad, mac_sz);
		/* Copy opad to Aux-1 */
		memcpy(&sc_buf[64], opad, mac_sz);
	}
}
