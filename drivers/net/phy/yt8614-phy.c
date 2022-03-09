#include "yt8614-phy.h"

/*
 * customer should change it accordingly
 */
#define SWDRV_ERR_PRINT_VALUE_RET_T(a, b, c, logstr, arg...)  \
        do { \
			/*nothing*/; \
		}while(0)

#define YT8614_PHY_BASE_PHY_ADDR	8
#define MPHY_LPORT_ARRAY_COUNT		1 
phy_info_s g_phy_info[MPHY_LPORT_ARRAY_COUNT]; 

static int driver_link_mode_8614[YT8614_MAX_LPORT_ID + 1] = {0}; //0: no link; 1: utp; 32: fiber. traced that 1000m fiber uses 32.

phy_info_s * yt8614_app_get_phy_info(u32 lport);

/* 上电硬复位后，初始化配置 PHY 相关寄存器，如下:

   #切换到了 UTP 地址空间
   Write phy_base_addr+0 ext_reg0xa000: 0x0

   #配置 port0~3 的相关扩展寄存器，并配置 utp mii_reg0x0 作软复位
   Write phy_base_addr+0 ext_reg0x41: 0x33
   Write phy_base_addr+0 ext_reg0x42: 0x66
   Write phy_base_addr+0 ext_reg0x43: 0xaa
   Write phy_base_addr+0 ext_reg0x44: 0xd0d
   Write phy_base_addr+0 mii_reg0x0 : 0x9140

   Write phy_base_addr+1 ext_reg0x41: 0x33
   Write phy_base_addr+1 ext_reg0x42: 0x66
   Write phy_base_addr+1 ext_reg0x43: 0xaa
   Write phy_base_addr+1 ext_reg0x44: 0xd0d
   Write phy_base_addr+1 mii_reg0x0 : 0x9140
   
   Write phy_base_addr+2 ext_reg0x41: 0x33
   Write phy_base_addr+2 ext_reg0x42: 0x66
   Write phy_base_addr+2 ext_reg0x43: 0xaa
   Write phy_base_addr+2 ext_reg0x44: 0xd0d
   Write phy_base_addr+2 ext_reg0x57: 0x2929 #注意：此配置与其它口不同
   Write phy_base_addr+2 mii_reg0x0 : 0x9140

   Write phy_base_addr+3 ext_reg0x41: 0x33
   Write phy_base_addr+3 ext_reg0x42: 0x66
   Write phy_base_addr+3 ext_reg0x43: 0xaa
   Write phy_base_addr+3 ext_reg0x44: 0xd0d
   Write phy_base_addr+3 mii_reg0x0 : 0x9140 */

static s32 yt8614_mdio_read_reg(unsigned int bus_id, unsigned int phy_addr, unsigned int reg, u16* val)
{
    s32 ret = SYS_E_NONE;

    return ret;
}

static s32 yt8614_mdio_write_reg(unsigned int bus_id, unsigned int phy_addr, unsigned int reg, const u16 val)
{
    s32 ret = SYS_E_NONE;

    return ret;
}

static s32 yt8614_read_ext_reg(struct phy_info_str *info, phy_data_s *param)
{
    s32 ret = SYS_E_NONE;
    
	ret = yt8614_mdio_write_reg(info->bus_id, info->phy_addr, REG_MII_EXT_ADDR, (const u16)param->reg);
	SWDRV_ERR_PRINT_VALUE_RET_T(ret, 0, 0, 
                                "lport:%d bus:%d phy_addr:%d reg:%d",
                                info->lport,
                                info->bus_id,
                                info->phy_addr,
                                param->reg);
        
	ret = yt8614_mdio_read_reg(info->bus_id, info->phy_addr, REG_MII_EXT_DATA, (u16*)&(param->val));        
	SWDRV_ERR_PRINT_VALUE_RET_T(ret, 0, 0, 
                                "lport:%d bus:%d phy_addr:%d reg:%d",
                                info->lport,
                                info->bus_id,
                                info->phy_addr,
                                param->val);

    return ret;
}

static s32 yt8614_write_ext_reg(struct phy_info_str *info, phy_data_s *param)
{
	s32 ret = SYS_E_NONE;
	
	ret = yt8614_mdio_write_reg(info->bus_id, info->phy_addr, REG_MII_EXT_ADDR, (const u16)param->reg);
	SWDRV_ERR_PRINT_VALUE_RET_T(ret, 0, 0, 
								"lport:%d bus:%d phy_addr:%d reg:%d",
								info->lport,
								info->bus_id,
								info->phy_addr,
								param->reg);
		
	ret = yt8614_mdio_write_reg(info->bus_id, info->phy_addr, REG_MII_EXT_DATA, (const u16)param->val);		
	SWDRV_ERR_PRINT_VALUE_RET_T(ret, 0, 0, 
								"lport:%d bus:%d phy_addr:%d reg:%d",
								info->lport,
								info->bus_id,
								info->phy_addr,
								param->val);

	return ret;
}

static s32 yt8614_set_reg_space(struct phy_info_str *info, u8 smiType)
{   
    s32 ret = SYS_E_NONE;
    phy_data_s data;

    memset(&data, 0, sizeof(phy_data_s));
        
    if((smiType == YT8614_SMI_SEL_PHY) || (smiType == YT8614_SMI_SEL_SDS_QSGMII) || (smiType == YT8614_SMI_SEL_SDS_SGMII))
    {
        data.reg = YT8614_REG_COM_SMI_MUX;
        data.val = smiType;
        ret = yt8614_write_ext_reg(info, &data);
        SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d reg:0x%x",
                                    info->lport, data.reg);        
    }
    else
    {
        SWDRV_ERR_PRINT_VALUE_RET_T(0, 0, 0, "phy_yt8614_set_space: invalid smi type(%d)\n", smiType);
        return SYS_E_PARAM;
    }

    return SYS_E_NONE;
}

s32 yt8614_read_reg(struct phy_info_str *info, phy_data_s *param)
{   
    if(param->regType == YT8614_TYPE_COMMON)
    {
        return yt8614_read_ext_reg(info, param);
    }
    else if(param->regType == YT8614_TYPE_UTP_MII)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
        return yt8614_mdio_read_reg(info->bus_id, info->phy_addr, param->reg, (u16 *)&(param->val));
    }
    else if(param->regType == YT8614_TYPE_LDS_MII)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
        return yt8614_mdio_read_reg(info->bus_id, info->phy_addr, param->reg, (u16 *)&(param->val));
    }
    else if(param->regType == YT8614_TYPE_UTP_MMD)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
		/* tbd. */
        return SYS_E_PARAM;
    }
    else if(param->regType == YT8614_TYPE_UTP_EXT)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
        return yt8614_read_ext_reg(info, param);
    }
    else if(param->regType == YT8614_TYPE_SDS_QSGMII_MII)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_QSGMII);
        return yt8614_mdio_read_reg(info->bus_id, info->phy_addr, param->reg, (u16 *)&(param->val));
    }
    else if(param->regType == YT8614_TYPE_SDS_QSGMII_EXT)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_QSGMII);
        return yt8614_read_ext_reg(info, param);
    }
    else if(param->regType == YT8614_TYPE_SDS_SGMII_MII)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_SGMII);
        return yt8614_mdio_read_reg(info->bus_id, info->phy_addr, param->reg, (u16 *)&(param->val));
    }
    else if(param->regType == YT8614_TYPE_SDS_SGMII_EXT)
    {
        yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_SGMII);
        return yt8614_read_ext_reg(info, param);
    }
    else
    {
        SWDRV_ERR_PRINT_VALUE_RET_T(0, 0, 0, "yt8614_read_reg: invalid register type(%x)\n", param->regType);
        return SYS_E_PARAM;
    }

    return SYS_E_NONE;
}

s32 yt8614_write_reg(struct phy_info_str *info, phy_data_s *param)
{	
	if(param->regType == YT8614_TYPE_COMMON)
	{
		return yt8614_write_ext_reg(info, param);
	}
	else if(param->regType == YT8614_TYPE_UTP_MII)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
		return yt8614_mdio_write_reg(info->bus_id, info->phy_addr, param->reg, param->val);
	}
	else if(param->regType == YT8614_TYPE_LDS_MII)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
		return yt8614_mdio_write_reg(info->bus_id, info->phy_addr, param->reg, param->val);
	}
	else if(param->regType == YT8614_TYPE_UTP_MMD)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
		/* tbd. */
		return SYS_E_PARAM;
	}
	else if(param->regType == YT8614_TYPE_UTP_EXT)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_PHY);
		return yt8614_write_ext_reg(info, param);
	}
	else if(param->regType == YT8614_TYPE_SDS_QSGMII_MII)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_QSGMII);
		return yt8614_mdio_write_reg(info->bus_id, info->phy_addr, param->reg, param->val);
	}
	else if(param->regType == YT8614_TYPE_SDS_QSGMII_EXT)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_QSGMII);
		return yt8614_write_ext_reg(info, param);
	}
	else if(param->regType == YT8614_TYPE_SDS_SGMII_MII)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_SGMII);
		return yt8614_mdio_write_reg(info->bus_id, info->phy_addr, param->reg, param->val);
	}
	else if(param->regType == YT8614_TYPE_SDS_SGMII_EXT)
	{
		yt8614_set_reg_space(info, YT8614_SMI_SEL_SDS_SGMII);
		return yt8614_write_ext_reg(info, param);
	}
	else
	{
		SWDRV_ERR_PRINT_VALUE_RET_T(0, 0, 0, "yt8614_write_reg: invalid register type(%x)\n", param->regType);
		return SYS_E_PARAM;
	}

	return SYS_E_NONE;
}


/*
 * soft reset to phy
 * including:
 *	UTP port
 * 	sgmii interface
 *	qsgmii interface
 * 
 * input: lport, make no sense. reserved for future using.
 */
s32 yt8614_phy_soft_reset(u32 lport)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }
    
    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg     = REG_SDS_BMCR;
    data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d speed:%d", lport, speed);   
    
    data.val |= BMCR_RESET;
        
    ret = phy_info->write(phy_info, &data); 
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d speed:%d", lport, speed);

    return ret;
}

/* 
 * note, the lport is 0 based.
 * this function do basic initialization for each port
 */   
s32 yt8614_phy_init(u32 lport)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }
    
    ////////////////////////////////////////////////////
    /* Write phy_base_addr+0 ext_reg0x41: 0x33 */
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = 0x41;
    data.regType = YT8614_TYPE_UTP_EXT;
    data.val     = 0x33;
    
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);

    ////////////////////////////////////////////////////
    /* Write phy_base_addr+1 ext_reg0x42: 0x66 */
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = 0x42;
    data.regType = YT8614_TYPE_UTP_EXT;
    data.val     = 0x66;
    
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);

    ////////////////////////////////////////////////////
    /* Write phy_base_addr+3 ext_reg0x43: 0xaa */
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = 0x43;
    data.regType = YT8614_TYPE_UTP_EXT;
    data.val     = 0xaa;
    
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);

    ////////////////////////////////////////////////////
    /* Write phy_base_addr+3 ext_reg0x44: 0xd0d */
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = 0x44;
    data.regType = YT8614_TYPE_UTP_EXT;
    data.val     = 0xd0d;
    
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);

    ////////////////////////////////////////////////////
    /* Write phy_base_addr+2,5 ext_reg0x57: 0x2929 #there are difference with other ports */
    if((lport==2) || (lport==5))
    {
        memset(&data, 0, sizeof(phy_data_s));
        data.reg     = 0x57;
        data.regType = YT8614_TYPE_UTP_EXT;
        data.val     = 0x2929;
        
        ret = phy_info->write(phy_info, &data);    
        SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
    }

    ////////////////////////////////////////////////////
    /* Write phy_base_addr+3 mii_reg0x0 :bit15=1, soft reset phy */
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = 0x0;
    data.regType = YT8614_TYPE_UTP_MII;
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);    
    data.val |= BMCR_RESET;
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);

    ////////////////////////////////////////////////////
	/* set pole positive or negative if needed */
#if 0	
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = YT8614_REG_COM_SLED_CFG0;
    data.regType = YT8614_TYPE_COMMON;
    
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);    

	/* clesr or set the bit FIBER_SLED_CFG0_ACT_LOW accordingly */
    data.val &= ~FIBER_SLED_CFG0_ACT_LOW;
    data.val |= FIBER_SLED_CFG0_ACT_LOW;
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
#endif

    return SYS_E_NONE;
}

/*
 * this function set fiber/sgmii to POWERDOWN mode or not
 * 
 */
s32 yt8614_fiber_enable(u32 lport, BOOL enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = REG_SDS_BMCR;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;
    
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);    
    #if 0
    printf("phy_yt8614_set_enable_1(port%02d enable=%d): value=0x%x\n",
           lport, enable, data.val);
    #endif
    if(enable)
    {
        data.val &= ~FIBER_BMCR_PDOWN;
    }
    else
    {
        data.val |= FIBER_BMCR_PDOWN;
    }
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);    

    #if 0
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);    
    printf("phy_yt8614_set_enable_2(port%d enable=%d): value=0x%x\n",
           lport, enable, data.val);
    #endif
    
    return ret;
}

/*
 * this function set utp to POWERDOWN mode or not
 * 
 */
s32 yt8614_utp_enable(u32 lport, BOOL enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = REG_SDS_BMCR;
    data.regType = YT8614_TYPE_UTP_MII;
    
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);    

    if(enable)
    {
        data.val &= ~BMCR_PDOWN;
    }
    else
    {
        data.val |= BMCR_PDOWN;
    }
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);    

    return ret;
}

/*
 * for fiber mode, force to unidirection.
 */
s32 yt8614_fiber_unidirection_set(u32 lport, int speed, BOOL enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = REG_SDS_BMCR;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;
    
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);   
    if(ret)
    {
        return SYS_E_PARAM;
    }
	
    if(enable)
    {
        data.val &= ~FIBER_BMCR_ANENABLE;
        data.val |= FIBER_BMCR_DUPLEX_MODE;
        data.val |= FIBER_BMCR_EN_UNIDIR;
    }
    else
    {
        data.val |= FIBER_BMCR_ANENABLE;
        //not to clear this bit. data.val &= ~FIBER_BMCR_DUPLEX_MODE;
        data.val &= ~FIBER_BMCR_EN_UNIDIR;
    }
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);    
    if(ret)
    {
        return SYS_E_PARAM;
    }

	if(enable)
		ret = yt8614_fiber_speed_set(lport, speed); 

    return ret;
}

/*
 * Fiber auto sensing for sgmii interface
 */
s32 yt8614_fiber_autosensing_set(u32 lport, BOOL enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg     = YT8614_REG_SGMII_EXT_HIDE_AUTO_SEN;
    data.regType = YT8614_TYPE_SDS_SGMII_EXT;
    
    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);   

    if(enable)
    {
        data.val |= FIBER_AUTO_SEN_ENABLE;
    }
    else
    {
        data.val &= ~FIBER_AUTO_SEN_ENABLE;
    }
    
    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg     = REG_SDS_BMCR;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);   

    // soft reset
    // and clear power down bit automatically 
    data.val |= BMCR_RESET;
        
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable); 

    return SYS_E_NONE;
}

/*
 * set fiber speed, can be:
 * 		YT8614_COMBO_FIBER_1000M,
 * 		YT8614_COMBO_FIBER_100M,
 */
s32 yt8614_fiber_speed_set(u32 lport, int fiber_speed)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);
	unsigned int speed_bit;
	u16 cur_val;
	
    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = YT8614_REG_COM_HIDE_SPEED;
    data.regType = YT8614_TYPE_COMMON;
    
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d combo_speed read:%d", lport, combo_speed);    
    if(ret)
    {
        return ret;
    }

	speed_bit = 1 << (lport & 0x3) /* port 0-3 */;
	cur_val = data.val;
	
	switch(fiber_speed) {
	case YT8614_COMBO_FIBER_1000M:
		data.val |= speed_bit;
		break;
	case YT8614_COMBO_FIBER_100M:
		data.val &= ~speed_bit;
		break;
	default:
		return SYS_E_PARAM;;
	}

	if(cur_val != data.val)
	{
		ret = phy_info->write(phy_info, &data);	
		SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d combo_speed, write:%d", lport, combo_speed);    
		if(!ret)
		{
			return ret;
		}
	}
	ret = yt8614_fiber_autosensing_set(lport, FALSE);
   
    return ret;
}


/*
 * set autonigotiation for qsgmii interface
 */
s32 yt8614_qsgmii_autoneg_set(u32 lport, BOOL enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg     = REG_MII_BMCR;
    //data.regType = YT8614_TYPE_SDS_QSGMII_EXT;
    data.regType = YT8614_TYPE_SDS_QSGMII_MII;
    
    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);   

    if(enable)
    {
        data.val |= BMCR_ANENABLE;
    }
    else
    {
        data.val &= ~BMCR_ANENABLE;
    }
    
    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);

    return ret;
}

/*
 * set autonigotiation for sgmii phy mode
 * 
 */
s32 yt8614_sgmii_autoneg_set(u32 lport, BOOL enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg     = REG_MII_BMCR;
    //data.regType = YT8614_TYPE_SDS_QSGMII_EXT;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;
    
    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);   

    if(enable)
    {
        data.val |= BMCR_ANENABLE;
    }
    else
    {
        data.val &= ~BMCR_ANENABLE;
    }
    
    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d enable:%d", lport, enable);

    return ret;
}


s32 yt8614_qsgmii_sgmii_link_status_get(u32 lport, BOOL *enable, BOOL if_qsgmii)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_SDS_SPEC_STATUS;
    
    if(if_qsgmii)
    {
        data.regType = YT8614_TYPE_SDS_QSGMII_MII;
    }
    else
    {
        data.regType = YT8614_TYPE_SDS_SGMII_MII;
    }

    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport); 

    if((data.val & FIBER_SSR_LSTATUS) == 0)
    {
        *enable = FALSE;
    }
    else
    {
        *enable = TRUE;
    }

    return ret;
}

/* for combo mode, set media priority
 * input:
 *		fiber=1, fiber of priority;
 *		fiber=0, upt of priority;
 */
int yt8614_combo_media_priority_set (u32 lport, int fiber)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);
	u16 cur_val;

	if (!phy_info)
		return SYS_E_PARAM;

	memset(&data, 0, sizeof(phy_data_s));
	data.reg	 = YT8614_REG_COM_HIDE_SPEED;
	data.regType = YT8614_TYPE_COMMON;
	ret = phy_info->read(phy_info, &data);	  
	SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return SYS_E_PARAM;

	cur_val = data.val;
	if (fiber) {
		if (!(data.val & (YT8614_REG_COM_HIDE_SPEED_CMB_PRI))) {
			data.val |= YT8614_REG_COM_HIDE_SPEED_CMB_PRI;
		}
	}else {
		if (data.val & (YT8614_REG_COM_HIDE_SPEED_CMB_PRI)) {
			data.val &= ~(YT8614_REG_COM_HIDE_SPEED_CMB_PRI);
		}
	}

	if (cur_val != data.val)
	{
		ret = phy_info->write(phy_info, &data);	  
		SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
		if (!ret)
			return ret;
	}

	return ret;
}

/* for combo mode, set media priority
 * input:
 *		fiber=1, fiber of priority;
 *		fiber=0, upt of priority;
 */
int yt8614_combo_media_priority_get (u32 lport, int *fiber)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

	if (!phy_info)
		return SYS_E_PARAM;

	memset(&data, 0, sizeof(phy_data_s));
	data.reg	 = YT8614_REG_COM_HIDE_SPEED;
	data.regType = YT8614_TYPE_COMMON;
	ret = phy_info->read(phy_info, &data);	  
	SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return SYS_E_PARAM;

	*fiber = !!(data.val & (YT8614_REG_COM_HIDE_SPEED_CMB_PRI));

	return ret;
}

/*
 * set utp autonegotiation or force link mode.
 */
s32 yt8614_utp_autoneg_set(u32 lport, BOOL enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_BMCR;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autogeg read:%d", lport, enable); 

    if(enable)
    {
        data.val |= BMCR_ANENABLE;
    }
    else
    {
        data.val &= ~BMCR_ANENABLE;
    }
    
    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autogeg write:%d", lport, enable); 

    return ret;
}


/*
 * get utp autonegotiation or force link mode.
 */
s32 yt8614_utp_autoneg_get(u32 lport, BOOL *enable)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_BMCR;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autogeg read:%d", lport, *enable); 

    *enable = !!(data.val & BMCR_ANENABLE);

    return ret;
}

/*
 * input: cap_mask, bit definitions, pause capbility and 100/10 capbilitys follow the definition of mii reg4
 *		for 1000M capability, bit0=1000M half; bit1=1000M full, see mii reg9.
 */
s32 yt8614_utp_autoneg_ability_set(u32 lport, unsigned int cap_mask)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_ADVERTISE;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autoneg_ability read:%d", lport, cap_mask); 
    if(ret)
    {
        return SYS_E_PARAM;
    }


    if(ADVERTISE_10HALF & cap_mask)
    {
        data.val |= ADVERTISE_10HALF;
    }
    else
    {
        data.val &= ~ADVERTISE_10HALF;
    }

    if(ADVERTISE_10FULL & cap_mask)
    {
        data.val |= ADVERTISE_10FULL;
    }
    else
    {
        data.val &= ~ADVERTISE_10FULL;
    }

    if(ADVERTISE_100HALF & cap_mask)
    {
        data.val |= ADVERTISE_100HALF;
    }
    else
    {
        data.val &= ~ADVERTISE_100HALF;
    }

    if(ADVERTISE_100FULL & cap_mask)
    {
        data.val |= ADVERTISE_100FULL;
    }
    else
    {
        data.val &= ~ADVERTISE_100FULL;
    }

    if(ADVERTISE_PAUSE_CAP & cap_mask)
    {
        data.val |= ADVERTISE_PAUSE_CAP;
    }
    else
    {
        data.val &= ~ADVERTISE_PAUSE_CAP;
    }

    if(ADVERTISE_PAUSE_ASYM & cap_mask)
    {
        data.val |= ADVERTISE_PAUSE_ASYM;
    }
    else
    {
        data.val &= ~ADVERTISE_PAUSE_ASYM;
    }

    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autoneg_ability write:%d", lport, cap_mask); 

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_CTRL1000;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autoneg_ability read:%d", lport, cap_mask); 
    if(ret)
    {
        return SYS_E_PARAM;
    }


    if(0x1 & cap_mask)
    {
        data.val |= ADVERTISE_1000HALF;
    }
    else
    {
        data.val &= ~ADVERTISE_1000HALF;
    }

    if(0x2 & cap_mask)
    {
        data.val |= ADVERTISE_1000FULL;
    }
    else
    {
        data.val &= ~ADVERTISE_1000FULL;
    }

    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autoneg_ability write:%d", lport, cap_mask); 

    return yt8614_phy_soft_reset(lport);
}

/*
 * input: lport
 * output: 
 *	cap_mask, bit definitions:
 *		pause capbility and 100/10 capbilitys follow the definition of mii reg4.
 *		for 1000M capability, bit0=1000M half; bit1=1000M full, refer to mii reg9.[9:8].
 */
s32 yt8614_utp_autoneg_ability_get(u32 lport, unsigned int *cap_mask)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_ADVERTISE;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autoneg_ability read:%d", lport, cap_mask); 
    if(!ret)
    {
        return SYS_E_PARAM;
    }


    if(ADVERTISE_10HALF & data.val)
    {
        *cap_mask |= ADVERTISE_10HALF;
    }
    else
    {
        *cap_mask &= ~ADVERTISE_10HALF;
    }

    if(ADVERTISE_10FULL & data.val)
    {
        *cap_mask |= ADVERTISE_10FULL;
    }
    else
    {
        *cap_mask &= ~ADVERTISE_10FULL;
    }

    if(ADVERTISE_100HALF & data.val)
    {
        *cap_mask |= ADVERTISE_100HALF;
    }
    else
    {
        *cap_mask &= ~ADVERTISE_100HALF;
    }

    if(ADVERTISE_100FULL & data.val)
    {
        *cap_mask |= ADVERTISE_100FULL;
    }
    else
    {
        *cap_mask &= ~ADVERTISE_100FULL;
    }

    if(ADVERTISE_PAUSE_CAP & data.val)
    {
        *cap_mask |= ADVERTISE_PAUSE_CAP;
    }
    else
    {
        *cap_mask &= ~ADVERTISE_PAUSE_CAP;
    }

    if(ADVERTISE_PAUSE_ASYM & data.val)
    {
        *cap_mask |= ADVERTISE_PAUSE_ASYM;
    }
    else
    {
        *cap_mask &= ~ADVERTISE_PAUSE_ASYM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_CTRL1000;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d autoneg_ability read:%d", lport, cap_mask); 

    if(ADVERTISE_1000HALF & data.val )
    {
        *cap_mask |= 0x1;
    }
    else
    {
        *cap_mask &= ~0x1;
    }

    if(ADVERTISE_1000FULL & data.val )
    {
        *cap_mask |= 0x2;
    }
    else
    {
        *cap_mask &= ~0x2;
    }

    return ret;
}


s32 yt8614_utp_force_duplex_set(u32 lport, BOOL full)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_BMCR;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d duplex read:%d", lport, duplex); 

    if(full)
    {
        data.val |= BMCR_FULLDPLX;
    }
    else
    {
        data.val &= ~BMCR_FULLDPLX;
    }
    
    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d duplex write:%d", lport, duplex); 

    return ret;
}

s32 yt8614_utp_force_duplex_get(u32 lport, BOOL *full)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_BMCR;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d duplex read:%d", lport, duplex); 

    *full = !!(data.val &= BMCR_FULLDPLX);

    return ret;
}

/*
 * this function set speed configuration.
 */
s32 yt8614_utp_force_speed_set(u32 lport, unsigned int speed)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);
	unsigned int speed_bit6, speed_bit13;
	
    if(!phy_info)
    {
        return SYS_E_PARAM;
    }
	
	switch (speed) {
		case SPEED_1000M:
			speed_bit6 = BMCR_SPEED1000;
			speed_bit13 = 0;
			break;
		case SPEED_100M:
			speed_bit13 = BMCR_SPEED100;
			speed_bit6 = 0;
			break;
		case SPEED_10M:
			speed_bit13 = 0;
			speed_bit6 = 0;
			break;
		default:
			return SYS_E_PARAM;;
	}
	
    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_BMCR;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d speed read:%d", lport, speed); 

			
    if(speed_bit6)
    {
        data.val |= speed_bit6;
    }
    else
    {
        data.val &= ~speed_bit6;
    }

    if(speed_bit13)
    {
        data.val |= speed_bit13;
    }
    else
    {
        data.val &= ~speed_bit13;
    }
    
    ret = phy_info->write(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d speed write:%d", lport, speed); 

    return ret;
}

s32 yt8614_utp_force_speed_get(u32 lport, unsigned int *speed)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);
	unsigned int speed_bit6, speed_bit13;
	
    if(!phy_info)
    {
        return SYS_E_PARAM;
    }

	speed_bit6 = BMCR_SPEED1000;
	speed_bit13 = BMCR_SPEED100;

    ////////////////////////////////////////////////////
    memset(&data, 0, sizeof(phy_data_s));
    
    data.reg = REG_MII_BMCR;
	data.regType = YT8614_TYPE_UTP_MII;

    ret = phy_info->read(phy_info, &data);
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d speed read:%d", lport, speed); 

	if((data.val & speed_bit6) && !(data.val & speed_bit13)) {
		*speed = SPEED_1000M;
	}else if (!(data.val & speed_bit6) && (data.val & speed_bit13)) {
		*speed = SPEED_100M;
	}else if (!(data.val & speed_bit6) && !(data.val & speed_bit13)) {
		*speed = SPEED_10M;
	}else {
		*speed = SPEED_UNKNOWN;
		return SYS_E_PARAM;
	}

    return ret;
}


/*
 * this function get the state of autoneg for both UTP and fiber
 * input:
 * 	lport - phy port
 * 	speed: speed_1000M or speed_100M, for fiber only
 * output:
 * 	aneg: 1 or 0
 */
int yt8614_autoneg_done_get (u32 lport, int speed, int *aneg)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

	if (!phy_info)
		return SYS_E_PARAM;

	/*it should be used for 8614 fiber*/
	if((32 == driver_link_mode_8614[lport]) && (SPEED_100M == speed))
	{
		*aneg = TRUE;
		return SYS_E_NONE;
	}

    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = MII_BMSR;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);

	*aneg = !!(data.val & (BMSR_ANEGCOMPLETE));
	return ret;
}

/* this function used to double check the speed. for fiber, this is no 10M */
static int yt8614_adjust_status(u32 lport, int val, int is_utp, int* speed, int* duplex)
{
	int speed_mode;

	//printk ("8614 status adjust call in...\n");
	*speed = -1;
	*duplex = (val & BIT(YT8614_DUPLEX_BIT)) >> YT8614_DUPLEX_BIT;
	speed_mode = (val & YT8614_SPEED_MODE) >> YT8614_SPEED_MODE_BIT;
	switch (speed_mode) {
	case 0:
		if (is_utp)
			*speed = SPEED_10M;
		break;
	case 1:
		*speed = SPEED_100M;
		break;
	case 2:
		*speed = SPEED_1000M;
		break;
	case 3:
		break;
	default:
		break;
	}
	//printk (KERN_INFO "yzhang..8521 status adjust call out,regval=0x%04x,mode=%s,speed=%dm...\n", val,is_utp?"utp":"fiber", phydev->speed);

	return 0;
}

/*
 * input: lport - port id, 0 based.
 * output:
 * 		speed: SPEED_10M, SPEED_100M, SPEED_1000M or -1;
 *		duplex: 0 or 1, see reg 0x11, bit YT8614_DUPLEX_BIT.
 *		ret_link: 0 or 1, link down or up.
 *		media: only valid when ret_link=1, (YT8614_SMI_SEL_SDS_SGMII + 1) for fiber; (YT8614_SMI_SEL_PHY + 1) for utp. -1 for link down.
 */
int yt8614_media_status_get(u32 lport, int* speed, int* duplex, int* ret_link, int *media)
{
    s32 ret = SYS_E_NONE;
    phy_data_s data;
    phy_info_s *phy_info = yt8614_app_get_phy_info(lport);

	volatile int val, yt8614_fiber_latch_val, yt8614_fiber_curr_val;
	volatile int link;
	int link_utp = 0, link_fiber = 0;

	if (!phy_info)
		return SYS_E_PARAM;

#if (YT8614_PHY_MODE_CURR != YT8614_PHY_MODE_FIBER)

	/* switch to utp and reading regs  */
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = 0xa000;
    data.regType = YT8614_TYPE_UTP_EXT;
    data.val     = 0x0;
    
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return ret;

    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = REG_PHY_SPEC_STATUS;
    data.regType = YT8614_TYPE_UTP_MII;
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return ret;

	link = data.val & (BIT(YT8614_LINK_STATUS_BIT));
	if (link) {
		link_utp = 1;
		yt8614_adjust_status(lport, data.val, 1, speed, duplex);
	} else {
		link_utp = 0;
	}
#endif //(YT8614_PHY_MODE_CURR != YT8521_PHY_MODE_FIBER)

#if (YT8614_PHY_MODE_CURR != YT8614_PHY_MODE_UTP)
	/* reading Fiber/sgmii */
    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = 0xa000;
    data.regType = YT8614_TYPE_UTP_EXT;
    data.val     = 0x3;
    
    ret = phy_info->write(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return ret;

    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = REG_PHY_SPEC_STATUS;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return ret;

	//printk (KERN_INFO "yt8614 read fiber status=%04x\n", data.val,(unsigned long)phydev->attached_dev);

	/* for fiber, from 1000m to 100m, there is not link down from 0x11, and check reg 1 to identify such case */	
	val = data.val;

    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = MII_BMSR/*reg 0x1*/;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return ret;
	
	yt8614_fiber_latch_val = data.val;

    memset(&data, 0, sizeof(phy_data_s));
    data.reg     = MII_BMSR/*reg 0x1*/;
    data.regType = YT8614_TYPE_SDS_SGMII_MII;
    ret = phy_info->read(phy_info, &data);    
    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
	if (ret)
		return ret;

	yt8614_fiber_curr_val = data.val;

	link = val & (BIT(YT8614_LINK_STATUS_BIT));
	if((link) && (yt8614_fiber_latch_val != yt8614_fiber_curr_val))
	{
		link = 0;
		SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "yzhang..8614 fiber link down detect,latch=%04x,curr=%04x\n", yt8614_fiber_latch_val,yt8614_fiber_curr_val);
	}
	
	if (link) {
		link_fiber = 1;
		yt8614_adjust_status(lport, val, 0, speed, duplex);
		driver_link_mode_8614[lport] = 32; //fiber mode. used only for fiber 100M

	} else {
		link_fiber = 0;
	}
#endif //(YT8521_PHY_MODE_CURR != YT8521_PHY_MODE_UTP)

	if (link_utp || link_fiber) {
		memset(&data, 0, sizeof(phy_data_s));
		data.reg	 = YT8614_REG_COM_HIDE_SPEED;
		data.regType = YT8614_TYPE_COMMON;
		ret = phy_info->read(phy_info, &data);	  
		SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
		if (ret)
			return ret;

		if(data.val & (YT8614_REG_COM_HIDE_SPEED_CMB_PRI)) {
			/* case of fiber of priority */
			if(link_utp) *media = (YT8614_SMI_SEL_PHY + 1);
			if(link_fiber) *media = (YT8614_SMI_SEL_SDS_SGMII + 1);
		} else {
			/* case of utp of priority */
			if(link_fiber) *media = (YT8614_SMI_SEL_SDS_SGMII + 1);
			if(link_utp) *media = (YT8614_SMI_SEL_PHY + 1);
		}
		*ret_link = TRUE;
		
	} else {
		*ret_link = FALSE;
		*media = -1;
              *speed= -1;
              *duplex = -1;
		driver_link_mode_8614[lport] = 0;
	}

	if (link_utp) {
	    memset(&data, 0, sizeof(phy_data_s));
	    data.reg     = 0xa000;
	    data.regType = YT8614_TYPE_UTP_EXT;
	    data.val     = 0x0;
	    
	    ret = phy_info->write(phy_info, &data);    
	    SWDRV_ERR_PRINT_VALUE_RET_T(ret, SYS_E_NONE, ret, "lport:%d", lport);
		if (ret)
			return ret;
	}
	//printk (KERN_INFO "yzhang..8614 read status call out,link=%d,linkmode=%d\n", ret_link, link_mode_8614[lport] );

	return 0;
}


/*
 * customer should overwirte this function according to there system
 */
phy_info_s * yt8614_app_get_phy_info(u32 lport)
{
	g_phy_info[0].lport = lport;
	g_phy_info[0].phy_addr = (YT8614_PHY_BASE_PHY_ADDR + lport) & 0x1F;
    return &g_phy_info[0];
}

s32 yt8614_app_init(void)
{
    s32 ret = SYS_E_NONE;
    
    memset(g_phy_info, 0, sizeof(g_phy_info));
	g_phy_info[0].read	= yt8614_read_reg;
	g_phy_info[0].write = yt8614_write_reg;
	g_phy_info[0].lport = 0xFF; //invalid port
	g_phy_info[0].bus_id = 0;
	g_phy_info[0].phy_addr = YT8614_PHY_BASE_PHY_ADDR;
	
    return ret;
}

