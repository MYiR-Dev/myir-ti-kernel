/*
 * Keystone crypto accelerator driver
 *
 * Copyright (C) 2015,2016 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors:	Sandeep Nair
 *		Vitaly Andrianov
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

#ifndef _KEYSTONE_SA_HLP_
#define _KEYSTONE_SA_HLP_

#include <linux/soc/ti/knav_dma.h>
#include <linux/soc/ti/knav_qmss.h>
#include <linux/interrupt.h>
#include <linux/hw_random.h>
#include <linux/skbuff.h>
#include <asm/aes_glue.h>

/* Enable the below macro for testing with run-time
 * self tests in the cryptographic algorithm manager
 * framework */
/* #define TEST */

/* For enabling debug prints */
/* #define DEBUG */

/* Algorithm constants */
#define MD5_BLOCK_SIZE    64
#define AES_XCBC_DIGEST_SIZE	16

/* Values for NULL algorithms */
#define NULL_KEY_SIZE		0
#define NULL_BLOCK_SIZE	1
#define NULL_DIGEST_SIZE	0
#define NULL_IV_SIZE		0

/* Number of 32 bit words in EPIB  */
#define SA_DMA_NUM_EPIB_WORDS	4

/* Number of 32 bit words in PS data  */
#define SA_DMA_NUM_PS_WORDS	16

/* Number of meta data elements passed in descriptor to SA */
#define SA_NUM_DMA_META_ELEMS	2

/* Maximum number of simultaeneous security contexts
 * supported by the driver */
#define SA_MAX_NUM_CTX	512

/* Encoding used to identify the typo of crypto operation
 * performed on the packet when the packet is returned
 * by SA
 */
#define SA_REQ_SUBTYPE_ENC	0x0001
#define SA_REQ_SUBTYPE_DEC	0x0002
#define SA_REQ_SUBTYPE_SHIFT	16
#define SA_REQ_SUBTYPE_MASK	0xffff

/* Maximum size of authentication tag
 * NOTE: update this macro as we start supporting
 * algorithms with bigger digest size
 */
#define SA_MAX_AUTH_TAG_SZ SHA1_DIGEST_SIZE

/* Memory map of the SA register set */
struct sa_mmr_regs {
	u32 PID;
	u32 RES01;
	u32 CMD_STATUS;
	u32 RES02;
	u32 PA_FLOWID;
	u32 CDMA_FLOWID;
	u32 PA_ENG_ID;
	u32 CDMA_ENG_ID;
	u8  RSVD0[224];
	u32 CTXCACH_CTRL;
	u32 CTXCACH_SC_PTR;
	u32 CTXCACH_SC_ID;
	u32 CTXCACH_MISSCNT;
};

/*
 * Register Overlay Structure for TRNG module
 */
struct sa_trng_regs {
	u32 TRNG_OUTPUT_L;
	u32 TRNG_OUTPUT_H;
	u32 TRNG_STATUS;
	u32 TRNG_INTMASK;
	u32 TRNG_INTACK;
	u32 TRNG_CONTROL;
	u32 TRNG_CONFIG;
	u32 RSVD0[228];
};

struct sa_regs {
	struct sa_mmr_regs mmr;
};

/* Driver statistics */
struct sa_drv_stats {
	/* Number of data pkts dropped while submitting to CP_ACE */
	atomic_t tx_dropped;
	/* Number of tear-down pkts dropped while submitting to CP_ACE */
	atomic_t sc_tear_dropped;
	/* Number of crypto requests sent to CP_ACE */
	atomic_t tx_pkts;
	/* Number of crypto request completions received from CP_ACE */
	atomic_t rx_pkts;
};

/* Crypto driver instance data */
struct keystone_crypto_data {
	struct platform_device	*pdev;
	struct clk		*clk;
	struct tasklet_struct	rx_task;
	struct tasklet_struct	tx_task;
	struct dma_pool		*sc_pool;
	struct kmem_cache	*dma_req_ctx_cache;
	struct sa_regs		*regs;
	struct sa_trng_regs	*trng_regs;

	void		*rx_chan;
	void		*rx_fdq[KNAV_DMA_FDQ_PER_CHAN];
	void		*rx_compl_q;
	void		*tx_chan;
	void		*tx_submit_q;
	void		*tx_compl_q;
	u32		tx_submit_qid;
	u32		tx_compl_qid;
	u32		rx_compl_qid;
	const char	*rx_chan_name;
	const char	*tx_chan_name;
	u32		tx_queue_depth;
	u32		rx_queue_depths[KNAV_DMA_FDQ_PER_CHAN];
	u32		rx_buffer_sizes[KNAV_DMA_FDQ_PER_CHAN];
	u32		rx_pool_size;
	u32		rx_pool_region_id;
	void		*rx_pool;
	u32		tx_pool_size;
	u32		tx_pool_region_id;
	void		*tx_pool;

	struct hwrng	rng;
	int		rng_initialized;

	spinlock_t	scid_lock; /* lock for SC-ID allocation */
	spinlock_t	trng_lock; /* reading random data from TRNG */

	struct kobject	stats_kobj;
	int		stats_fl;

	/* Security context data */
	u16		sc_id_start;
	u16		sc_id_end;
	u16		sc_id;

	/* Bitmap to keep track of Security context ID's */
	unsigned long	ctx_bm[DIV_ROUND_UP(SA_MAX_NUM_CTX,
				BITS_PER_LONG)];
	/* Driver stats */
	struct sa_drv_stats	stats;
	atomic_t	rx_dma_page_cnt; /* N buf from 2nd pool available */
	atomic_t	tx_dma_desc_cnt; /* Tx DMA desc-s available */
};

/* Packet structure used in Rx */
#define SA_SGLIST_SIZE	(MAX_SKB_FRAGS + SA_NUM_DMA_META_ELEMS)
struct sa_packet {
	struct scatterlist		 sg[SA_SGLIST_SIZE];
	int				 sg_ents;
	struct keystone_crypto_data	*priv;
	struct dma_chan			*chan;
	struct dma_async_tx_descriptor	*desc;
	dma_cookie_t			 cookie;
	u32				 epib[SA_DMA_NUM_EPIB_WORDS];
	u32				 psdata[SA_DMA_NUM_PS_WORDS];
	struct completion		 complete;
	void				*data;
};

/* Command label updation info */
struct sa_cmdl_param_info {
	u16	index;
	u16	offset;
	u16	size;
};

/* Maximum length of Auxiliary data in 32bit words */
#define SA_MAX_AUX_DATA_WORDS	8

struct sa_cmdl_upd_info {
	u16	flags;
	u16	submode;
	struct sa_cmdl_param_info	enc_size;
	struct sa_cmdl_param_info	enc_size2;
	struct sa_cmdl_param_info	enc_offset;
	struct sa_cmdl_param_info	enc_iv;
	struct sa_cmdl_param_info	enc_iv2;
	struct sa_cmdl_param_info	aad;
	struct sa_cmdl_param_info	payload;
	struct sa_cmdl_param_info	auth_size;
	struct sa_cmdl_param_info	auth_size2;
	struct sa_cmdl_param_info	auth_offset;
	struct sa_cmdl_param_info	auth_iv;
	struct sa_cmdl_param_info	aux_key_info;
	u32				aux_key[SA_MAX_AUX_DATA_WORDS];
};

enum sa_submode {
	SA_MODE_GEN = 0,
	SA_MODE_CCM,
	SA_MODE_GCM,
	SA_MODE_GMAC
};

/* TFM Context info */

/*
 * Number of 32bit words appended after the command label
 * in PSDATA to identify the crypto request context.
 * word-0: Request type
 * word-1: pointer to request
 */
#define SA_PSDATA_CTX_WORDS 4

/* Maximum size of Command label in 32 words */
#define SA_MAX_CMDL_WORDS (SA_DMA_NUM_PS_WORDS - SA_PSDATA_CTX_WORDS)

struct sa_ctx_info {
	u8		*sc;
	dma_addr_t	sc_phys;
	u16		sc_id;
	u16		cmdl_size;
	u32		cmdl[SA_MAX_CMDL_WORDS];
	struct sa_cmdl_upd_info cmdl_upd_info;
	/* Store Auxiliary data such as K2/K3 subkeys in AES-XCBC */
	u32		epib[SA_DMA_NUM_EPIB_WORDS];
	u32		rx_flow;
	u32		rx_compl_qid;
};

struct sa_tfm_ctx {
	struct keystone_crypto_data *dev_data;
	struct sa_ctx_info enc;
	struct sa_ctx_info dec;
	struct sa_ctx_info auth;
};

/* Tx DMA callback param */
struct sa_dma_req_ctx {
	struct keystone_crypto_data *dev_data;
	u32		cmdl[SA_MAX_CMDL_WORDS + SA_PSDATA_CTX_WORDS];
	unsigned	map_idx;
	struct sg_table sg_tbl;
	dma_cookie_t	cookie;
	struct dma_chan *tx_chan;
	bool		pkt;
};

/* Encryption algorithms */
enum sa_ealg_id {
	SA_EALG_ID_NONE = 0,        /* No encryption */
	SA_EALG_ID_NULL,            /* NULL encryption */
	SA_EALG_ID_AES_CTR,         /* AES Counter mode */
	SA_EALG_ID_AES_F8,          /* AES F8 mode */
	SA_EALG_ID_AES_CBC,         /* AES CBC mode */
	SA_EALG_ID_DES_CBC,         /* DES CBC mode */
	SA_EALG_ID_3DES_CBC,        /* 3DES CBC mode */
	SA_EALG_ID_CCM,             /* Counter with CBC-MAC mode */
	SA_EALG_ID_GCM,             /* Galois Counter mode */
	SA_EALG_ID_LAST
};

/* Authentication algorithms */
enum sa_aalg_id {
	SA_AALG_ID_NONE = 0,               /* No Authentication  */
	SA_AALG_ID_NULL = SA_EALG_ID_LAST, /* NULL Authentication  */
	SA_AALG_ID_MD5,                    /* MD5 mode */
	SA_AALG_ID_SHA1,                   /* SHA1 mode */
	SA_AALG_ID_SHA2_224,               /* 224-bit SHA2 mode */
	SA_AALG_ID_SHA2_256,               /* 256-bit SHA2 mode */
	SA_AALG_ID_HMAC_MD5,               /* HMAC with MD5 mode */
	SA_AALG_ID_HMAC_SHA1,              /* HMAC with SHA1 mode */
	SA_AALG_ID_HMAC_SHA2_224,          /* HMAC with 224-bit SHA2 mode */
	SA_AALG_ID_HMAC_SHA2_256,          /* HMAC with 256-bit SHA2 mode */
	SA_AALG_ID_GMAC,                   /* Galois Message
					      Authentication Code mode */
	SA_AALG_ID_CMAC,                   /* Cipher-based Message
					      Authentication Code mode */
	SA_AALG_ID_CBC_MAC,                /* Cipher Block Chaining */
	SA_AALG_ID_AES_XCBC                /* AES Extended
					      Cipher Block Chaining */
};

/* Mode control engine algorithms used to index the
 * mode control instruction tables
 */
enum sa_eng_algo_id {
	SA_ENG_ALGO_ECB = 0,
	SA_ENG_ALGO_CBC,
	SA_ENG_ALGO_CFB,
	SA_ENG_ALGO_OFB,
	SA_ENG_ALGO_CTR,
	SA_ENG_ALGO_F8,
	SA_ENG_ALGO_F8F9,
	SA_ENG_ALGO_GCM,
	SA_ENG_ALGO_GMAC,
	SA_ENG_ALGO_CCM,
	SA_ENG_ALGO_CMAC,
	SA_ENG_ALGO_CBCMAC,
	SA_NUM_ENG_ALGOS
};

/* 3DES only supports ECB, CBC, CFB and OFB. */
#define SA_3DES_FIRST_ALGO          SA_ENG_ALGO_ECB
#define SA_3DES_LAST_ALGO           SA_ENG_ALGO_OFB
#define SA_3DES_NUM_ALGOS           (SA_3DES_LAST_ALGO - SA_3DES_FIRST_ALGO + 1)

#define NKEY_SZ			3
#define MCI_SZ			27

struct sa_eng_info {
	u8	eng_id;
	u16	sc_size;
};

#define DMA_HAS_PSINFO		BIT(31)
#define DMA_HAS_EPIB		BIT(30)

void sa_register_algos(const struct device *dev);
void sa_unregister_algos(const struct device *dev);
void sa_tx_completion_process(struct keystone_crypto_data *dev_data);
void sa_rx_completion_process(struct keystone_crypto_data *dev_data);

void sa_set_sc_auth(u16 alg_id, const u8 *key, u16 key_sz, u8 *sc_buf);
int sa_set_sc_enc(u16 alg_id, const u8 *key, u16 key_sz,
		  u16 aad_len, u8 enc, u8 *sc_buf);

void sa_swiz_128(u8 *in, u8 *out, u16 len);
void sa_conv_calg_to_salg(const char *cra_name, int *ealg_id, int *aalg_id);
void sa_get_engine_info(int alg_id, struct sa_eng_info *info);
int sa_get_hash_size(u16 aalg_id);

/*
 * Derive sub-key k1, k2 and k3 used in the AES XCBC MAC mode
 * detailed in RFC 3566
 */
static inline int sa_aes_xcbc_subkey(u8 *sub_key1, u8 *sub_key2,
				     u8 *sub_key3, const u8 *key,
				     u16 key_sz)
{
	struct AES_KEY enc_key;

	if (private_AES_set_encrypt_key(key, (key_sz * 8), &enc_key) == -1) {
		pr_err("%s: failed to set enc key\n", __func__);
		return -1;
	}

	if (sub_key1) {
		memset(sub_key1, 0x01, AES_BLOCK_SIZE);
		AES_encrypt(sub_key1, sub_key1, &enc_key);
	}

	if (sub_key2) {
		memset(sub_key2, 0x02, AES_BLOCK_SIZE);
		AES_encrypt(sub_key2, sub_key2, &enc_key);
	}

	if (sub_key3) {
		memset(sub_key3, 0x03, AES_BLOCK_SIZE);
		AES_encrypt(sub_key3, sub_key3, &enc_key);
	}

	return 0;
}

struct sa_eng_mci_tbl {
	uint8_t aes_enc[SA_NUM_ENG_ALGOS][NKEY_SZ][MCI_SZ];
	uint8_t aes_dec[SA_NUM_ENG_ALGOS][NKEY_SZ][MCI_SZ];
	uint8_t _3des_enc[SA_3DES_NUM_ALGOS][MCI_SZ];
	uint8_t _3des_dec[SA_3DES_NUM_ALGOS][MCI_SZ];
};

extern struct sa_eng_mci_tbl sa_mci_tbl;

extern struct device *sa_ks2_dev;

#endif /* _KEYSTONE_SA_HLP_ */
