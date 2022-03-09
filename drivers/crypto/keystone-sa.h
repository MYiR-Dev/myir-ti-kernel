/*
 * Keystone crypto accelerator driver
 *
 * Copyright (C) 2013-2016 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Sandeep Nair <sandeep_n@ti.com>
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

/*
 *
 *     0                   1                   2                   3
 *     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ -----
 *   |                                                               |
 *   |   Software only section (not fetched by CP_ACE)               |
 *   |               (optional)                                      |
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+<-Must be
 *   |               SCCTL                                           |  64 byte
 *   |               (8 bytes)                                       |  aligned
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |                                                               |
 *   |   PHP module specific section (fetched by CP_ACE)             |
 *   |               (56 bytes)                                      |
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ -----
 *   |                                                               |
 *   |   Encryption module specific section (fetched by CP_ACE)      |
 *   |               (variable size)                                 |
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+<-64 byte
 *   |                                                               |  aligned
 *   |   Authentication module specific section (fetched by CP_ACE)  |
 *   |               (variable size)                                 |
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ -----
 *
 *              Figure: Security Context memory layout
 *
 *
 *     0                   1                   2                   3
 *     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ -----
 *   |O|Evict done   |  F/E control  |           SCID                |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |                 SCPTR (Security Context Pointer)              |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ -----
 *              O : Owner
 *              SCID & SCPTR are filled by hardware
 *              Figure: Security Context control word (SCCTL)
 *
 *
 *     0                   1                   2                   3
 *     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |D|  Pkt Type   |  Flow Index   |   Dest Queue ID               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |               SWINFO-0 (4 bytes)                              |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |               SWINFO-1 (4 bytes)                              |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   | PktID (16 bits)               |                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |                                                               |
 *   |                Protocol Specific Parameters                   |
 *   |                (Variable Size up to 116 bytes                 |
 *   ...                                                           ...
 *   |                                                               |
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *              D : Direction
 *              Figure: PHP engine Security Context Format
 *
 *
 *     0                   1                   2                   3
 *     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |M| R |nEngineID|                                               |
 *   +-+-+-+-+-+-+-+-+                                               +
 *   |                  encryption mode ctrl word                    |
 *   ...                                                           ...
 *   |                       (27 bytes)                              |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Reserved (4 bytes) (must initialize to 0)            |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Encryption Key value (32 bytes)                      |
 *   ...                                                           ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Encryption Aux-1 (32 bytes) (optional)               |
 *   ...                                                           ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Encryption Aux-2 (16 bytes) (optional)               |
 *   ...                                                           ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Encryption Aux-3 (16 bytes) (optional)               |
 *   ...                                                           ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Encryption Aux-4 (16 bytes)                          |
 *   ...                                                           ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Pre-crypto data store (15 bytes)                     |
 *   |                                                               |
 *   |                                               +-+-+-+-+-+-+-+-|
 *   |                                               |               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *              M : Encryption Mode selector (0=crypto processing, 1=NULL)
 *              R : Reserved
 *              Figure: Encryption engine Security Context Format
 *
 *
 *     0                   1                   2                   3
 *     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |M| R |nEngineID| Auth SW Ctrl  |                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+                               +
 *   |                  Reserved (6 bytes)                           |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Authentication length (8 bytes)                      |
 *   |          ( 0 = let h/w calculate the length )                 |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Reserved (12 bytes)                                  |
 *   ...                                                           ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          HW crtl word (4 bytes) must be set to 0 by SW        |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Authentication Key value (32 bytes)                  |
 *   ...        Master Key or pre-computed inner digest for HMAC   ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Authentication Aux-1 (32 bytes) (optional)           |
 *   ...        Pre-computed outer opad for HMAC                   ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Authentication Aux-2 (32 bytes) (optional)           |
 *   ...                                                           ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Pre-crypto data store (32 bytes)                     |
 *   ...        ( HW access only)                                   ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *   |          Pre-crypto data store (32 bytes)                     |
 *   ...        ( HW access only)                                   ...
 *   |                                                               |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ------
 *              M : Authentication Mode selector (0=hash processing, 1=NULL)
 *              R : Reserved
 *              Figure: Authentication engine Security Context Format
 *
 */

#ifndef _KEYSTONE_SA_H_
#define _KEYSTONE_SA_H_

/*
 * This type represents the various packet types to be processed
 * by the PHP engine in SA.
 * It is used to identify the corresponding PHP processing function.
 */
typedef u8 SA_CTX_PE_PKT_TYPE_T;
#define SA_CTX_PE_PKT_TYPE_3GPP_AIR    0    /* 3GPP Air Cipher */
#define SA_CTX_PE_PKT_TYPE_SRTP        1    /* SRTP */
#define SA_CTX_PE_PKT_TYPE_IPSEC_AH    2    /* IPSec Authentication Header */
#define SA_CTX_PE_PKT_TYPE_IPSEC_ESP   3    /* IPSec Encapsulating
					       Security Payload */
#define SA_CTX_PE_PKT_TYPE_NONE        4    /* Indicates that it is in
					       data mode, It may not be
					       used by PHP */

#define SA_CTX_ENC_TYPE1_SZ	64	/* Encryption SC with Key only */
#define SA_CTX_ENC_TYPE2_SZ	96	/* Encryption SC with Key and Aux1 */

#define SA_CTX_AUTH_TYPE1_SZ	64	/* Auth SC with Key only */
#define SA_CTX_AUTH_TYPE2_SZ	96	/* Auth SC with Key and Aux1 */

#define SA_CTX_PHP_PE_CTX_SZ	64	/* Size of security ctx for
					   PHP engine */

#define SA_CTX_MAX_SZ (64 + SA_CTX_ENC_TYPE2_SZ + SA_CTX_AUTH_TYPE2_SZ)

/*
 * Encoding of F/E control in SCCTL
 *  Bit 0-1: Fetch PHP Bytes
 *  Bit 2-3: Fetch Encryption/Air Ciphering Bytes
 *  Bit 4-5: Fetch Authentication Bytes or Encr pass 2
 *  Bit 6-7: Evict PHP Bytes
 *
 *  where   00 = 0 bytes
 *          01 = 64 bytes
 *          10 = 96 bytes
 *          11 = 128 bytes
 */
#define SA_CTX_DMA_SIZE_0       0
#define SA_CTX_DMA_SIZE_64      1
#define SA_CTX_DMA_SIZE_96      2
#define SA_CTX_DMA_SIZE_128     3

#define SA_CTX_SCCTL_MK_DMA_INFO(php_f, eng0_f, eng1_f, php_e) \
	((php_f) | \
	 ((eng0_f) << 2) | \
	 ((eng1_f) << 4) | \
	 ((php_e) << 6))

/*
 * Byte offset of the owner word in SCCTL
 * in the security context
 */
#define SA_CTX_SCCTL_OWNER_OFFSET 0

/*
 * Assumption: CTX size is multiple of 32
 */
#define SA_CTX_SIZE_TO_DMA_SIZE(ctx_sz)	\
	((ctx_sz) ? ((ctx_sz) / 32 - 1) : 0)

#define SA_CTX_ENC_KEY_OFFSET	32
#define SA_CTX_ENC_AUX1_OFFSET	64
#define SA_CTX_ENC_AUX2_OFFSET	96
#define SA_CTX_ENC_AUX3_OFFSET	112
#define SA_CTX_ENC_AUX4_OFFSET	128

/* Next Engine Select code in CP_ACE */
#define SA_ENG_ID_EM1	2	/*  Encryption/Decryption engine
				    with AES/DES core */
#define SA_ENG_ID_EM2	3	/*  Encryption/Decryption enginefor pass 2 */
#define SA_ENG_ID_AM1	4	/*  Authentication engine with
				    SHA1/MD5/SHA2 core */
#define SA_ENG_ID_AM2	5	/*  Authentication engine for pass 2 */
#define SA_ENG_ID_OUTPORT2 20	/*  Egress module 2  */
#define SA_ENG_ID_NONE  0xff

/*
 * Command Label Definitions
 */
#define SA_CMDL_OFFSET_NESC           0      /* Next Engine Select Code */
#define SA_CMDL_OFFSET_LABEL_LEN      1      /* Engine Command Label Length */
#define SA_CMDL_OFFSET_DATA_LEN       2      /* 16-bit Length of Data to be
						processed */
#define SA_CMDL_OFFSET_DATA_OFFSET    4      /* Stat Data Offset */
#define SA_CMDL_OFFSET_OPTION_CTRL1   5      /* Option Control Byte 1 */
#define SA_CMDL_OFFSET_OPTION_CTRL2   6      /* Option Control Byte 2 */
#define SA_CMDL_OFFSET_OPTION_CTRL3   7      /* Option Control Byte 3 */
#define SA_CMDL_OFFSET_OPTION_BYTE    8

#define SA_CMDL_HEADER_SIZE_BYTES          8

#define SA_CMDL_OPTION_BYTES_MAX_SIZE     72
#define SA_CMDL_MAX_SIZE_BYTES (SA_CMDL_HEADER_SIZE_BYTES + \
				SA_CMDL_OPTION_BYTES_MAX_SIZE)

/* SWINFO word-0 flags */
#define SA_SW_INFO_FLAG_EVICT	0x0001
#define SA_SW_INFO_FLAG_TEAR	0x0002
#define SA_SW_INFO_FLAG_NOPD	0x0004

/*
 * TRNG module definitions
 */

/* Offset to TRNG module in CP_ACE memory map */
#define SA_REG_MAP_TRNG_OFFSET	0x24000

/* TRNG enable control in CP_ACE */
#define SA_CMD_STATUS_REG_TRNG_ENABLE	BIT(3)

/* TRNG start control in TRNG module */
#define SA_TRNG_CONTROL_REG_TRNG_ENABLE	BIT(10)

/* Data ready indicator in STATUS register */
#define SA_TRNG_STATUS_REG_READY BIT(0)

/* Data ready clear control in INTACK register */
#define SA_TRNG_INTACK_REG_READY BIT(0)

/* Number of samples taken to gather entropy during startup.
 * If value is 0, the number of samples is 2^24 else
 * equals value times 2^8.
 */
#define SA_TRNG_DEF_STARTUP_CYCLES	0
#define SA_TRNG_CONTROL_REG_STARTUP_CYCLES_SHIFT 16

/* Minimum number of samples taken to regenerate entropy
 * If value is 0, the number of samples is 2^24 else
 * equals value times 2^6.
 */
#define SA_TRNG_DEF_MIN_REFILL_CYCLES	1
#define SA_TRNG_CONFIG_REG_MIN_REFILL_CYCLES_SHIFT 0

/* Maximum number of samples taken to regenerate entropy
 * If value is 0, the number of samples is 2^24 else
 * equals value times 2^8.
 */
#define SA_TRNG_DEF_MAX_REFILL_CYCLES	0
#define SA_TRNG_CONFIG_REG_MAX_REFILL_CYCLES_SHIFT 16

/* Number of CLK input cycles between samples */
#define SA_TRNG_DEF_CLK_DIV_CYCLES	0
#define SA_TRNG_CONFIG_REG_SAMPLE_DIV_SHIFT 8

#define SA_CMD_ENCSS_EN		0x00000001
#define SA_CMD_AUTHSS_EN	0x00000002
#define SA_CMD_AIRSS_EN		0x00000004
#define SA_CMD_TRNG_EN		0x00000008
#define SA_CMD_PKA_EN		0x00000010
#define SA_CMD_PHP1SS_EN	0x00000020
#define SA_CMD_PHP2SS_EN	0x00000040
#define SA_CMD_CTXCACH_EN	0x00000080
#define SA_CMD_SA1_IN_EN	0x00000100
#define SA_CMD_SA0_IN_EN	0x00000200
#define SA_CMD_SA1_OUT_EN	0x00000400
#define SA_CMD_SA0_OUT_EN	0x00000800

#endif
