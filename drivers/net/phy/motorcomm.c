/*
 * drivers/net/phy/motorcomm.c
 *
 * Driver for Motorcomm PHYs
 *
 * Author: Leilei Zhao <leilei.zhao@motorcomm.com>
 *
 * Copyright (c) 2019 Motorcomm, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support : Motorcomm Phys:
 *		Giga phys: yt8511, yt8521
 *		100/10 Phys : yt8512, yt8512b, yt8510
 *		Automotive 100Mb Phys : yt8010
 *		Automotive 100/10 hyper range Phys: yt8510
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/motorcomm_phy.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/version.h>
/*for wol, 20210604*/
#include <linux/netdevice.h>

#include "yt8614-phy.h"

/**** configuration section begin ***********/

/* if system depends on ethernet packet to restore from sleep, please define this macro to 1
 * otherwise, define it to 0.
 */
#define SYS_WAKEUP_BASED_ON_ETH_PKT 	0

/* to enable system WOL of phy, please define this macro to 1
 * otherwise, define it to 0.
 */
#define YTPHY_ENABLE_WOL 		0

/* some GMAC need clock input from PHY, for eg., 125M, please enable this macro
 * by degault, it is set to 0
 * NOTE: this macro will need macro SYS_WAKEUP_BASED_ON_ETH_PKT to set to 1
 */
#define GMAC_CLOCK_INPUT_NEEDED 0


#define YT8521_PHY_MODE_FIBER	1 //fiber mode only
#define YT8521_PHY_MODE_UTP		2 //utp mode only
#define YT8521_PHY_MODE_POLL	3 //fiber and utp, poll mode

/* please make choice according to system design
 * for Fiber only system, please define YT8521_PHY_MODE_CURR 1
 * for UTP only system, please define YT8521_PHY_MODE_CURR 2
 * for combo system, please define YT8521_PHY_MODE_CURR 3 
 */
#define YT8521_PHY_MODE_CURR	3

/**** configuration section end ***********/


/* no need to change below */

#if (YTPHY_ENABLE_WOL)
#undef SYS_WAKEUP_BASED_ON_ETH_PKT
#define SYS_WAKEUP_BASED_ON_ETH_PKT 	1
#endif

static int ytphy_read_ext(struct phy_device *phydev, u32 regnum)
{
	int ret;
	int val;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	val = phy_read(phydev, REG_DEBUG_DATA);

	return val;
}

static int ytphy_write_ext(struct phy_device *phydev, u32 regnum, u16 val)
{
	int ret;

	ret = phy_write(phydev, REG_DEBUG_ADDR_OFFSET, regnum);
	if (ret < 0)
		return ret;

	ret = phy_write(phydev, REG_DEBUG_DATA, val);

	return ret;
}

static int yt8511_config_init(struct phy_device *phydev)
{
	int ret;
	int val;

	ret = genphy_config_init(phydev);
	if(ret < 0){
		return ret;
	}

	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID) {
		
	    /* disable auto sleep */
	    val = ytphy_read_ext(phydev, 0x27);
	    val &= (~BIT(15));

	    ret = ytphy_write_ext(phydev, 0x27, val);

	    /* enable RXC clock when no wire plug */
	    val = ytphy_read_ext(phydev, 0xc);

	    /* ext reg 0xc b[7:4]
		Tx Delay time = 150ps * N - 250ps
	    */
	    val |= (0xf << 4);
	    val |= 0x1;
	    ret = ytphy_write_ext(phydev, 0xc, val);
//	    printk("yt8511_config_en_txdelay..phy txdelay, val=%#08x\n",val);
	}
	return ret;
}
static struct phy_driver ytphy_drvs[] = {
 {
		.phy_id		= PHY_ID_YT8511,
		.name		= "YT8511 Gigabit Ethernet",
		.phy_id_mask	= MOTORCOMM_PHY_ID_MASK,
		.features	= PHY_GBIT_FEATURES,
		.flags			= PHY_HAS_INTERRUPT,
		.config_aneg	= genphy_config_aneg,
		.config_init	= yt8511_config_init,
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	}
};

module_phy_driver(ytphy_drvs);

MODULE_DESCRIPTION("Motorcomm PHY driver");
MODULE_AUTHOR("Leilei Zhao");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused motorcomm_tbl[] = {
	{ PHY_ID_YT8010, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8510, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8511, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8512B, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8521, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8531S, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8531, MOTORCOMM_PHY_ID_MASK },
	{ PHY_ID_YT8618, MOTORCOMM_MPHY_ID_MASK },
	{ PHY_ID_YT8614, MOTORCOMM_MPHY_ID_MASK_8614 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, motorcomm_tbl);

