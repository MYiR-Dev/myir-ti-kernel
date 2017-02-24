/*
 * Driver for the Texas Instruments DP83848 PHY
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/phy.h>

#define DP83848C_PHY_ID			0x20005c90
#define DP83848I_PHY_ID			0x20005ca0
#define TLK10X_PHY_ID			0x2000a210
/**
* @def TLKPHY_REGCR_REG
*      Value reg for accessing extended registers
*/
#define TLKPHY_REGCR_REG  0x000D
/**
* @def TLKPHY_ADDR_REG
*      ADDR reg for accessing extended registers
*/
#define TLKPHY_ADDR_REG   0x000E
#define TLKPHY_PHYCR_REG                          (0x19)
#define TLKPHY_LEDCR_REG                          (0x18)
#define TLKPHY_PHYSCR_REG                         (0x11)
#define TLKPHY_PHYSTS_REG                         (0x10)


#define TLKPHY_CR1_REG                            (0x9)
#define TLKPHY_CR2_REG                            (0xA)
#define TLKPHY_CR3_REG                            (0xB)

/**
* @def EXT_REG_ADDRESS_ACCESS
*      Extended reg address access value
*/
#define EXT_REG_ADDRESS_ACCESS 0x001F
/**
* @def EXT_REG_DATA_NORMAL_ACCESS
*      Extended reg data access value
*/
#define EXT_REG_DATA_NORMAL_ACCESS 0x401F

#define TLK105_EXT_MLEDCR_REG                     (0x0025)
#define EXT_REG_DATA_NORMAL_ACCESS 0x401F

/**
* @def EXT_REG_ADDRESS_ACCESS
*      Extended reg address access value
*/
#define EXT_REG_ADDRESS_ACCESS 0x001F


/* Registers */
#define DP83848_MICR			0x11 /* MII Interrupt Control Register */
#define DP83848_MISR			0x12 /* MII Interrupt Status Register */

/* MICR Register Fields */
#define DP83848_MICR_INT_OE		BIT(0) /* Interrupt Output Enable */
#define DP83848_MICR_INTEN		BIT(1) /* Interrupt Enable */

/* MISR Register Fields */
#define DP83848_MISR_RHF_INT_EN		BIT(0) /* Receive Error Counter */
#define DP83848_MISR_FHF_INT_EN		BIT(1) /* False Carrier Counter */
#define DP83848_MISR_ANC_INT_EN		BIT(2) /* Auto-negotiation complete */
#define DP83848_MISR_DUP_INT_EN		BIT(3) /* Duplex Status */
#define DP83848_MISR_SPD_INT_EN		BIT(4) /* Speed status */
#define DP83848_MISR_LINK_INT_EN	BIT(5) /* Link status */
#define DP83848_MISR_ED_INT_EN		BIT(6) /* Energy detect */
#define DP83848_MISR_LQM_INT_EN		BIT(7) /* Link Quality Monitor */

#define DP83848_INT_EN_MASK		\
	(DP83848_MISR_ANC_INT_EN |	\
	 DP83848_MISR_DUP_INT_EN |	\
	 DP83848_MISR_SPD_INT_EN |	\
	 DP83848_MISR_LINK_INT_EN)

static int dp83848_ack_interrupt(struct phy_device *phydev)
{
	int err = phy_read(phydev, DP83848_MISR);

	return err < 0 ? err : 0;
}

static int dp83848_config_intr(struct phy_device *phydev)
{
	int control, ret;

	control = phy_read(phydev, DP83848_MICR);
	if (control < 0)
		return control;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		control |= DP83848_MICR_INT_OE;
		control |= DP83848_MICR_INTEN;

		ret = phy_write(phydev, DP83848_MISR, DP83848_INT_EN_MASK);
		if (ret < 0)
			return ret;
	} else {
		control &= ~DP83848_MICR_INTEN;
	}

	return phy_write(phydev, DP83848_MICR, control);
}

static int tlk105_config_init(struct phy_device *phydev)
{
        int ret;
	unsigned short phyregval = 0;

        ret = genphy_config_init(phydev);
        if (ret < 0)
                return ret;

        ret = phy_write(phydev, TLKPHY_REGCR_REG,
                        EXT_REG_ADDRESS_ACCESS);
        if (ret)
                return ret;
        ret = phy_write(phydev, TLKPHY_ADDR_REG,
                        TLK105_EXT_MLEDCR_REG);
        if (ret)
                return ret;
        ret = phy_write(phydev, TLKPHY_REGCR_REG,
                        EXT_REG_DATA_NORMAL_ACCESS);
        if (ret)
                return ret;
	phyregval = phy_read(phydev, TLKPHY_ADDR_REG);
	phyregval |= ((1<<10) | (5<<3)|0);
	phyregval &= ~(1<<9);

        ret = phy_write(phydev, TLKPHY_ADDR_REG,
                        phyregval);
        if (ret)
                return ret;

	phyregval = phy_read(phydev, TLKPHY_PHYCR_REG);
	phyregval &= ~(1<<5);
        ret = phy_write(phydev, TLKPHY_PHYCR_REG,
                        phyregval);
        if (ret)
                return ret;

        return 0;
}

static struct mdio_device_id __maybe_unused dp83848_tbl[] = {
	{ DP83848C_PHY_ID, 0xfffffff0 },
	{ DP83848I_PHY_ID, 0xfffffff0 },
	{ TLK10X_PHY_ID, 0xfffffff0 },
	{ }
};
MODULE_DEVICE_TABLE(mdio, dp83848_tbl);

#define DP83848_PHY_DRIVER(_id, _name)				\
	{							\
		.phy_id		= _id,				\
		.phy_id_mask	= 0xfffffff0,			\
		.name		= _name,			\
		.features	= PHY_BASIC_FEATURES,		\
		.flags		= PHY_HAS_INTERRUPT,		\
								\
		.soft_reset	= genphy_soft_reset,		\
		.config_init	= tlk105_config_init,		\
		.suspend	= genphy_suspend,		\
		.resume		= genphy_resume,		\
		.config_aneg	= genphy_config_aneg,		\
		.read_status	= genphy_read_status,		\
								\
		/* IRQ related */				\
		.ack_interrupt	= dp83848_ack_interrupt,	\
		.config_intr	= dp83848_config_intr,		\
								\
		.driver		= { .owner = THIS_MODULE, },	\
	}

static struct phy_driver dp83848_driver[] = {
	DP83848_PHY_DRIVER(DP83848C_PHY_ID, "TI DP83848C 10/100 Mbps PHY"),
	DP83848_PHY_DRIVER(DP83848I_PHY_ID, "TI DP83848I 10/100 Mbps PHY"),
	DP83848_PHY_DRIVER(TLK10X_PHY_ID, "TI TLK10X 10/100 Mbps PHY"),
};
module_phy_driver(dp83848_driver);

MODULE_DESCRIPTION("Texas Instruments DP83848 PHY driver");
MODULE_AUTHOR("Andrew F. Davis <afd@ti.com");
MODULE_LICENSE("GPL");
