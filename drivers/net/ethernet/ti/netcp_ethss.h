/*
 * NetCP ethss header file
 *
 * Copyright (C) 2014 - 2016 Texas Instruments Incorporated
 * Authors:	Sandeep Nair <sandeep_n@ti.com>
 *		Sandeep Paulraj <s-paulraj@ti.com>
 *		Cyril Chemparathy <cyril@ti.com>
 *		Santosh Shilimkar <santosh.shilimkar@ti.com>
 *		Wingman Kwok <w-kwok2@ti.com>
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

#ifndef __NETCP_ETHSS_H__
#define __NETCP_ETHSS_H__

#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/phy/phy.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/ethtool.h>

#include "netcp_ale.h"
#include "netcp.h"
#include "netcp_cpts.h"

#define MAX_NUM_SERDES				2
#define MAX_SLAVES				8

struct gbe_ss_regs_ofs {
	u16	id_ver;
	u16	control;
	u16	rgmii_status; /* 2U */
};

struct gbe_switch_regs_ofs {
	u16	id_ver;
	u16	control;
	u16	soft_reset;
	u16	emcontrol;
	u16	stat_port_en;
	u16	ptype;
	u16	flow_control;
};

struct gbe_port_regs_ofs {
	u16	port_vlan;
	u16	tx_pri_map;
	u16	rx_pri_map;
	u16	sa_lo;
	u16	sa_hi;
	u16	ts_ctl;
	u16	ts_seq_ltype;
	u16	ts_vlan;
	u16	ts_ctl_ltype2;
	u16	ts_ctl2;
	u16	rx_maxlen;	/* 2U, NU */
};

struct gbe_host_port_regs_ofs {
	u16	port_vlan;
	u16	tx_pri_map;
	u16	rx_maxlen;
};

struct gbe_emac_regs_ofs {
	u16	mac_control;
	u16	soft_reset;
	u16	rx_maxlen;
};

#define GBE_MAX_HW_STAT_MODS			9

struct ts_ctl {
	int	uni;
	u8	dst_port_map;
	u8	maddr_map;
	u8	ts_mcast_type;
};

struct gbe_priv {
	struct device			*dev;
	struct netcp_device		*netcp_device;
	struct timer_list		timer;
	u32				num_slaves;
	u32				ale_entries;
	u32				ale_ports;
	bool				enable_ale;
	u8				max_num_slaves;
	u8				max_num_ports; /* max_num_slaves + 1 */
	u8				num_stats_mods;
	u8				num_serdeses;
	struct netcp_tx_pipe		tx_pipe;

	int				host_port;
	u32				rx_packet_max;
	u32				ss_version;
	u32				stats_en_mask;

	struct regmap			*ss_regmap;
	struct regmap			*pcsr_regmap;
	void __iomem                    *ss_regs;
	void __iomem			*switch_regs;
	void __iomem			*host_port_regs;
	void __iomem			*ale_reg;
	void __iomem			*sgmii_port_regs;
	void __iomem			*sgmii_port34_regs;
	void __iomem			*hw_stats_regs[GBE_MAX_HW_STAT_MODS];

	struct gbe_ss_regs_ofs		ss_regs_ofs;
	struct gbe_switch_regs_ofs	switch_regs_ofs;
	struct gbe_host_port_regs_ofs	host_port_regs_ofs;

	struct cpsw_ale			*ale;
	unsigned int			tx_queue_id;
	const char			*dma_chan_name;

	struct list_head		gbe_intf_head;
	struct list_head		secondary_slaves;
	struct net_device		*dummy_ndev;

	u64				*hw_stats;
	u32				*hw_stats_prev;
	const struct netcp_ethtool_stat *et_stats;
	int				num_et_stats;
	/*  Lock for updating the hwstats */
	spinlock_t			hw_stats_lock;
	struct phy			*serdes_phy[MAX_NUM_SERDES];

	struct kobject			port_ts_kobj[MAX_SLAVES];
	u32				cpts_rftclk_sel;
	u32				cpts_clock_mult;
	u32				cpts_clock_shift;
	u32				cpts_clock_div;
	int                             cpts_registered;
	struct cpts			cpts;
};

int gbe_create_cpts_sysfs(struct gbe_priv *gbe_dev);

#define for_each_intf(i, priv) \
	list_for_each_entry((i), &(priv)->gbe_intf_head, gbe_intf_list)

#endif /* __NETCP_ETHSS_H */
