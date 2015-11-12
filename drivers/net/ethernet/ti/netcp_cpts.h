/*
 * TI Keystone Common Platform Time Sync
 * Based on cpts.h by Richard Cochran <richardcochran@gmail.com>
 *
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors:	Wingman Kwok <w-kwok2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _TI_NETCP_CPTS_H_
#define _TI_NETCP_CPTS_H_

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clocksource.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/skbuff.h>
#include <linux/timecounter.h>

#define PTP_CLASS_VLAN_IPV4  0x50 /* event in a VLAN tagged IPV4 packet */
#define PTP_CLASS_VLAN_IPV6  0x60 /* event in a VLAN tagged IPV6 packet */

/* Fields commented with a version number vn.m means they are
 * available from version vn.m and up
 */
struct cpsw_cpts {
	u32 idver;                /* Identification and version */
	u32 control;              /* Time sync control */
	u32 rftclk_sel;           /* Ref Clock Select, v1.5 */
	u32 ts_push;              /* Time stamp event push */
	u32 ts_load_val;          /* Time stamp load value */
	u32 ts_load_en;           /* Time stamp load enable */
	u32 ts_comp_val;          /* Time stamp comparison value, v1.5 */
	u32 ts_comp_length;       /* Time stamp comp assert len, v1.5 */
	u32 intstat_raw;          /* Time sync interrupt status raw */
	u32 intstat_masked;       /* Time sync interrupt status masked */
	u32 int_enable;           /* Time sync interrupt enable */
	u32 res3;
	u32 event_pop;            /* Event interrupt pop */
	u32 event_low;            /* 32 Bit Event Time Stamp */
	u32 event_high;           /* Event Type Fields */
	u32 event_info2;          /* Domain number in Ptp message v1.5 */
	u32 event_info3;	  /* Upper 32 Bit Time Stamp, v1.6 */
	u32 ts_load_val_h;	  /* Upper 32 Bit Time stamp load, v1.6 */
	u32 ts_comp_val_h;	  /* Upper 32 Bit Time stamp comparison, v1.6 */
};

/* Bit definitions for the IDVER register */
#define TX_IDENT_SHIFT       (16)    /* TX Identification Value */
#define TX_IDENT_MASK        (0xffff)
#define RTL_VER_SHIFT        (11)    /* RTL Version Value */
#define RTL_VER_MASK         (0x1f)
#define MAJOR_VER_SHIFT      (8)     /* Major Version Value */
#define MAJOR_VER_MASK       (0x7)
#define MINOR_VER_SHIFT      (0)     /* Minor Version Value */
#define MINOR_VER_MASK       (0xff)

/* Bit definitions for the CONTROL register */
#define HW4_TS_PUSH_EN       BIT(11) /* Hardware push 4 enable */
#define HW3_TS_PUSH_EN       BIT(10) /* Hardware push 3 enable */
#define HW2_TS_PUSH_EN       BIT(9)  /* Hardware push 2 enable */
#define HW1_TS_PUSH_EN       BIT(8)  /* Hardware push 1 enable */
#define TS_COMP_POLARITY     BIT(2)  /* TS_COMP Polarity */
#define INT_TEST             BIT(1)  /* Interrupt Test */
#define CPTS_EN              BIT(0)  /* Time Sync Enable */

/* Definitions for the single bit resisters:
 * TS_PUSH TS_LOAD_EN  INTSTAT_RAW INTSTAT_MASKED INT_ENABLE EVENT_POP
 */
#define TS_PUSH             BIT(0)  /* Time stamp event push */
#define TS_LOAD_EN          BIT(0)  /* Time Stamp Load */
#define TS_PEND_RAW         BIT(0)  /* int read (before enable) */
#define TS_PEND             BIT(0)  /* masked interrupt read (after enable) */
#define TS_PEND_EN          BIT(0)  /* masked interrupt enable */
#define EVENT_POP           BIT(0)  /* writing discards one event */

/* Bit definitions for the EVENT_HIGH register */
#define PORT_NUMBER_SHIFT    (24)    /* Indicates Ethernet port or HW pin */
#define PORT_NUMBER_MASK     (0x1f)
#define EVENT_TYPE_SHIFT     (20)    /* Time sync event type */
#define EVENT_TYPE_MASK      (0xf)
#define MESSAGE_TYPE_SHIFT   (16)    /* PTP message type */
#define MESSAGE_TYPE_MASK    (0xf)
#define SEQUENCE_ID_SHIFT    (0)     /* PTP message sequence ID */
#define SEQUENCE_ID_MASK     (0xffff)

enum {
	CPTS_EV_PUSH, /* Time Stamp Push Event */
	CPTS_EV_ROLL, /* Time Stamp Rollover Event */
	CPTS_EV_HALF, /* Time Stamp Half Rollover Event */
	CPTS_EV_HW,   /* Hardware Time Stamp Push Event */
	CPTS_EV_RX,   /* Ethernet Receive Event */
	CPTS_EV_TX,   /* Ethernet Transmit Event */
	CPTS_EV_COMP, /* Time Stamp Compare Event */
};

#define CPTS_OVERFLOW_PERIOD	(HZ / 5)
#define CPTS_COMP_TMO		(CPTS_OVERFLOW_PERIOD * 2)
#define CPTS_TMO		2

#define CPTS_FIFO_DEPTH 16
#define CPTS_MAX_EVENTS 32

struct cpts_event {
	struct list_head list;
	unsigned long tmo;
	u32 low;
	u32 high;
	u32 info2;
	u32 info3;
};

struct cpts {
	struct device *dev;
	struct cpsw_cpts __iomem *reg;
	int tx_enable;
	int rx_enable;
#ifdef CONFIG_TI_KEYSTONE_NETCP_CPTS
	struct ptp_clock_info info;
	struct ptp_clock *clock;
	spinlock_t lock; /* protects time registers */
	u32 cc_mult; /* for the nominal frequency */
	struct cyclecounter cc;
	struct timecounter tc;
	struct delayed_work overflow_work;
	int phc_index;
	struct clk *refclk;
	struct list_head events;
	struct list_head pool;
	struct cpts_event pool_data[CPTS_MAX_EVENTS];
	int rftclk_sel;
	u32 rftclk_freq;
	int pps_enable;
	u32 pps_one_sec; /* counter val equivalent of 1 sec */
	u32 ts_comp_length;
	u32 ts_comp_polarity;
	u64 ts_comp_last;
	u32 cc_div;
	u64 cc_total;
	u64 tc_base;
	u64 max_cycles;
	u64 max_nsec;
	s32 ppb;
	bool ignore_adjfreq;
	u32 hw_ts_enable;
	struct sk_buff_head tx_ts_queue;
#endif
};

#ifdef CONFIG_TI_KEYSTONE_NETCP_CPTS
int netcp_cpts_rx_timestamp(struct cpts *cpts, struct sk_buff *skb);
int netcp_cpts_tx_timestamp(struct cpts *cpts, struct sk_buff *skb);
#else
static inline int netcp_cpts_rx_timestamp(struct cpts *cpts,
					  struct sk_buff *skb)
{
	return 0;
}

static inline int netcp_cpts_tx_timestamp(struct cpts *cpts,
					  struct sk_buff *skb)
{
	return 0;
}
#endif

int netcp_cpts_register(struct device *dev, struct cpts *cpts,
			u32 mult, u32 shift, u32 div);
void netcp_cpts_unregister(struct cpts *cpts);

#endif
