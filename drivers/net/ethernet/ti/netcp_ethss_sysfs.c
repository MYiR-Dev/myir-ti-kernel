/*
 * Keystone GBE and XGBE sysfs driver code
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

#include "netcp_ethss.h"

#define to_gbe_dev(obj) container_of(obj, struct gbe_priv, kobj)
#define tx_pri_to_gbe_dev(obj) container_of(obj, struct gbe_priv, tx_pri_kobj)
#define pvlan_to_gbe_dev(obj) container_of(obj, struct gbe_priv, pvlan_kobj)
#define stats_to_gbe_dev(obj) container_of(obj, struct gbe_priv, stats_kobj)
#define gbe_sw_mod_info_field_val(r, i) \
	((r & BITMASK(i->bits, i->shift)) >> i->shift)

#define __GBE_SW_ATTR_FULL(_name, _mode, _show, _store, _info,	\
				_info_size, _ctxt)		\
	{ \
		.attr = {.name = __stringify(_name), .mode = _mode },	\
		.show	= _show,		\
		.store	= _store,		\
		.info	= _info,		\
		.info_size = _info_size,	\
		.context = (_ctxt),		\
	}

#define __GBE_SW_ATTR(_name, _mode, _show, _store, _info) \
		__GBE_SW_ATTR_FULL(_name, _mode, _show, _store, _info, \
					(ARRAY_SIZE(_info)), NULL)

#define __GBE_SW_CTXT_ATTR(_name, _mode, _show, _store, _info, _ctxt) \
		__GBE_SW_ATTR_FULL(_name, _mode, _show, _store, _info, \
					(ARRAY_SIZE(_info)), _ctxt)

#define BITS(x)			(BIT(x) - 1)
#define BITMASK(n, s)		(BITS(n) << (s))

enum gbe_sysfs_sw_entry {
	GBE_SYSFS_SW_CONTROL,
	GBE_SYSFS_SW_TX_PRIO,
	GBE_SYSFS_SW_VLAN,
	GBE_SYSFS_SW_STATS,
	GBE_SYSFS_SW_MAX
};

struct gbe_sw_mod_info {
	const char	*name;
	int		shift;
	int		bits;
};

struct gbe_sw_parse_result {
	int control;
	int port;
	u32 value;
};

struct gbe_attribute {
	struct attribute attr;
	ssize_t (*show)(struct gbe_priv *gbe_dev,
			struct gbe_attribute *attr, char *buf);
	ssize_t	(*store)(struct gbe_priv *gbe_dev,
			 struct gbe_attribute *attr, const char *, size_t);
	const struct gbe_sw_mod_info *info;
	ssize_t info_size;
	void *context;
};

#define to_gbe_attr(_attr) container_of(_attr, struct gbe_attribute, attr)

static struct gbe_slave *gbe_port_num_get_slave(struct gbe_priv *gbe_dev,
						int port)
{
	struct gbe_intf *gbe_intf;

	for_each_intf(gbe_intf, gbe_dev) {
		if (gbe_intf->slave->port_num == port)
			return gbe_intf->slave;
	}
	return NULL;
}

#ifdef CONFIG_TI_KEYSTONE_NETCP_CPTS
struct gbe_ts_attribute {
	struct attribute attr;
	ssize_t (*show)(struct gbe_priv *gbe_dev,
			struct gbe_ts_attribute *attr, char *buf, void *);
	ssize_t	(*store)(struct gbe_priv *gbe_dev,
			 struct gbe_ts_attribute *attr,
			 const char *, size_t, void *);
};

#define to_gbe_ts_attr(_attr) \
	container_of(_attr, struct gbe_ts_attribute, attr)

#define __GBE_TS_ATTR(_name, _mode, _show, _store)			\
	{								\
		.attr = {.name = __stringify(_name), .mode = _mode },	\
		.show	= _show,					\
		.store	= _store,					\
	}

#define pts_to_gbe_dev(obj) container_of(obj, struct gbe_priv, pts_kobj)

#define pts_n_to_gbe_dev(obj, n) \
	container_of(obj, struct gbe_priv, port_ts_kobj[n])

static ssize_t gbe_port_ts_uni_show(struct gbe_priv *gbe_dev,
				    struct gbe_ts_attribute *attr,
				    char *buf, void *context)
{
	struct gbe_slave *slave;
	int len, port;
	u32 reg;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(buf, PAGE_SIZE, "%lu\n",
		       ((reg & TS_UNI_EN) >> TS_UNI_EN_SHIFT));

	return len;
}

static ssize_t gbe_port_ts_uni_store(struct gbe_priv *gbe_dev,
				     struct gbe_ts_attribute *attr,
				     const char *buf, size_t count,
				     void *context)
{
	struct gbe_slave *slave;
	int port, val;
	u32 reg, mode;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	if (kstrtoint(buf, 0, &val) < 0)
		return -EINVAL;

	if (val)
		mode = TS_UNI_EN;
	else
		mode = (slave->ts_ctl.maddr_map << TS_CTL_MADDR_SHIFT);

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	reg &= ~(TS_UNI_EN | TS_CTL_MADDR_ALL);
	reg |= mode;
	writel(reg, GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));

	slave->ts_ctl.uni = (val ? 1 : 0);
	return count;
}

static struct gbe_ts_attribute gbe_pts_uni_attribute =
				__GBE_TS_ATTR(uni_en, S_IRUGO | S_IWUSR,
					      gbe_port_ts_uni_show,
					      gbe_port_ts_uni_store);

static ssize_t gbe_port_ts_maddr_show(struct gbe_priv *gbe_dev,
				      struct gbe_ts_attribute *attr,
				      char *buf, void *context)
{
	struct gbe_slave *slave;
	int len, port;
	u32 reg;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(buf, PAGE_SIZE, "%02x\n",
		       (reg >> TS_CTL_MADDR_SHIFT) & 0x1f);
	return len;
}

static ssize_t gbe_port_ts_maddr_store(struct gbe_priv *gbe_dev,
				       struct gbe_ts_attribute *attr,
				       const char *buf, size_t count,
				       void *context)
{
	struct gbe_slave *slave;
	int port;
	u32 reg;
	u8 val;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	if (kstrtou8(buf, 0, &val) < 0)
		return -EINVAL;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	reg &= ~TS_CTL_MADDR_ALL;
	reg |= ((val & 0x1f) << TS_CTL_MADDR_SHIFT);
	writel(reg, GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));

	slave->ts_ctl.maddr_map = val & 0x1f;
	return count;
}

static struct gbe_ts_attribute gbe_pts_maddr_attribute =
				__GBE_TS_ATTR(mcast_addr, S_IRUGO | S_IWUSR,
					      gbe_port_ts_maddr_show,
					      gbe_port_ts_maddr_store);

static ssize_t gbe_port_ts_dst_port_show(struct gbe_priv *gbe_dev,
					 struct gbe_ts_attribute *attr,
					 char *buf, void *context)
{
	struct gbe_slave *slave;
	int len, port;
	u32 reg;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(buf, PAGE_SIZE, "%01x\n",
		       (reg >> TS_CTL_DST_PORT_SHIFT) & 0x3);
	return len;
}

static ssize_t gbe_port_ts_dst_port_store(struct gbe_priv *gbe_dev,
					  struct gbe_ts_attribute *attr,
					  const char *buf, size_t count,
					  void *context)
{
	struct gbe_slave *slave;
	int port;
	u32 reg;
	u8 val;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	if (kstrtou8(buf, 0, &val) < 0)
		return -EINVAL;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	reg &= ~TS_CTL_DST_PORT;
	reg |= ((val & 0x3) << TS_CTL_DST_PORT_SHIFT);
	writel(reg, GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));

	slave->ts_ctl.dst_port_map = val & 0x3;
	return count;
}

static struct gbe_ts_attribute gbe_pts_dst_port_attribute =
				__GBE_TS_ATTR(dst_port, S_IRUGO | S_IWUSR,
					      gbe_port_ts_dst_port_show,
					      gbe_port_ts_dst_port_store);

static ssize_t gbe_port_ts_config_show(struct gbe_priv *gbe_dev,
				       struct gbe_ts_attribute *attr,
				       char *buf, void *context)
{
	struct gbe_slave *slave;
	int len, port, total_len = 0;
	u32 reg;
	char *p = buf;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(p, PAGE_SIZE, "%08x ", reg);
	p += len;
	total_len += len;

	reg = readl(GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2));
	len = snprintf(p, PAGE_SIZE, "%08x\n", reg);
	p += len;
	total_len += len;

	return total_len;
}

static ssize_t gbe_port_ts_config_store(struct gbe_priv *gbe_dev,
					struct gbe_ts_attribute *attr,
					const char *buf, size_t count,
					void *context)
{
	struct gbe_slave *slave;
	unsigned long reg, val;
	int len, port;
	char tmp_str[4];
	u8 reg_num = 0;
	u32 __iomem *p;

	port = (int)context;

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	len = strcspn(buf, " ");
	if (len > 1)
		return -ENOMEM;

	strncpy(tmp_str, buf, len);
	tmp_str[len] = '\0';
	if (kstrtou8(tmp_str, 0, &reg_num))
		return -EINVAL;

	buf += (len + 1);
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	switch (reg_num) {
	case 1:
		p = GBE_REG_ADDR(slave, port_regs, ts_ctl);
		break;
	case 2:
		p = GBE_REG_ADDR(slave, port_regs, ts_seq_ltype);
		break;
	case 3:
		p = GBE_REG_ADDR(slave, port_regs, ts_vlan);
		break;
	case 4:
		p = GBE_REG_ADDR(slave, port_regs, ts_ctl_ltype2);
		break;
	case 5:
		p = GBE_REG_ADDR(slave, port_regs, ts_ctl2);
		break;
	default:
		return -EINVAL;
	}

	reg = readl(p);
	if (reg != val)
		writel(val, p);

	return count;
}

static struct gbe_ts_attribute gbe_pts_config_attribute =
				__GBE_TS_ATTR(config, S_IRUGO | S_IWUSR,
					      gbe_port_ts_config_show,
					      gbe_port_ts_config_store);

static struct attribute *gbe_pts_n_default_attrs[] = {
	&gbe_pts_uni_attribute.attr,
	&gbe_pts_maddr_attribute.attr,
	&gbe_pts_dst_port_attribute.attr,
	&gbe_pts_config_attribute.attr,
	NULL
};

struct gbe_priv *gbe_port_ts_kobj_to_priv(struct kobject *kobj, int *port)
{
	char name[4];
	struct gbe_priv *gbe_dev;
	struct kobject *kobj_0;
	int i = 0;

	*port = -1;

	while (i < MAX_SLAVES) {
		snprintf(name, sizeof(name), "%d", i + 1);
		if (strncmp(name, kobject_name(kobj), strlen(name)) == 0)
			*port = i + 1;
		i++;
	}

	if (*port < 0)
		return NULL;

	kobj_0 = kobj - (*port - 1);
	gbe_dev = pts_n_to_gbe_dev(kobj_0, 0);
	return gbe_dev;
}

static ssize_t gbe_pts_n_attr_show(struct kobject *kobj,
				   struct attribute *attr, char *buf)
{
	struct gbe_ts_attribute *attribute = to_gbe_ts_attr(attr);
	struct gbe_priv *gbe_dev;
	int port = -1;

	if (!attribute->show)
		return -EIO;

	gbe_dev = gbe_port_ts_kobj_to_priv(kobj, &port);
	if (!gbe_dev)
		return -EIO;

	return attribute->show(gbe_dev, attribute, buf, (void *)port);
}

static ssize_t gbe_pts_n_attr_store(struct kobject *kobj,
				    struct attribute *attr,
				    const char *buf, size_t count)
{
	struct gbe_ts_attribute *attribute = to_gbe_ts_attr(attr);
	struct gbe_priv *gbe_dev;
	int port = -1;

	if (!attribute->store)
		return -EIO;

	gbe_dev = gbe_port_ts_kobj_to_priv(kobj, &port);
	if (!gbe_dev)
		return -EIO;

	return attribute->store(gbe_dev, attribute, buf, count, (void *)port);
}

static const struct sysfs_ops gbe_pts_n_sysfs_ops = {
	.show = gbe_pts_n_attr_show,
	.store = gbe_pts_n_attr_store,
};

static struct kobj_type gbe_pts_n_ktype = {
	.sysfs_ops = &gbe_pts_n_sysfs_ops,
	.default_attrs = gbe_pts_n_default_attrs,
};

static int gbe_create_cpts_sysfs(struct gbe_priv *gbe_dev)
{
	struct kobject *pts_kobj;
	char name[4];
	int i, ret;

	pts_kobj = kobject_create_and_add("port_ts",
					  kobject_get(&gbe_dev->dev->kobj));
	if (!pts_kobj) {
		dev_err(gbe_dev->dev,
			"failed to create sysfs port_ts entry\n");
		kobject_put(&gbe_dev->dev->kobj);
		return -ENOMEM;
	}

	for (i = 0; i < gbe_dev->num_slaves; i++) {
		snprintf(name, sizeof(name), "%d", i + 1);
		ret = kobject_init_and_add(&gbe_dev->port_ts_kobj[i],
					   &gbe_pts_n_ktype,
					   kobject_get(pts_kobj), name);

		if (ret) {
			dev_err(gbe_dev->dev,
				"failed to create sysfs port_ts/%s entry\n",
				name);
			kobject_put(&gbe_dev->port_ts_kobj[i]);
			kobject_put(pts_kobj);
			return ret;
		}
	}

	return 0;
}
#else
static int gbe_create_cpts_sysfs(struct gbe_priv *gbe_dev)
{
	return 0;
}
#endif /* CONFIG_TI_KEYSTONE_NETCP_CPTS */

static ssize_t gbe_sw_version_show(struct gbe_priv *gbe_dev,
				   struct gbe_attribute *attr, char *buf)
{
	u32 reg;

	reg = readl(GBE_REG_ADDR(gbe_dev, switch_regs, id_ver));

	return snprintf(buf, PAGE_SIZE,
		"\nGBE Switch Version %d.%d (%d) Identification value 0x%x\n",
		 GBE_MAJOR_VERSION(reg), GBE_MINOR_VERSION(reg),
		 GBE_RTL_VERSION(reg), GBE_IDENT(reg));
}

static struct gbe_attribute gbe_sw_version_attribute =
	      __ATTR(version, S_IRUGO, gbe_sw_version_show, NULL);

static const struct gbe_sw_mod_info gbe_sw_ver14_controls[] = {
	{
		.name		= "fifo_loopback",
		.shift		= 0,
		.bits		= 1,
	},
	{
		.name		= "vlan_aware",
		.shift		= 1,
		.bits		= 1,
	},
	{
		.name		= "p0_enable",
		.shift		= 2,
		.bits		= 1,
	},
	{
		.name		= "p0_pass_pri_tagged",
		.shift		= 3,
		.bits		= 1,
	},
	{
		.name		= "p1_pass_pri_tagged",
		.shift		= 4,
		.bits		= 1,
	},
	{
		.name		= "p2_pass_pri_tagged",
		.shift		= 5,
		.bits		= 1,
	},
	{
		.name		= "p3_pass_pri_tagged",
		.shift		= 7,
		.bits		= 1,
	},
	{
		.name		= "p4_pass_pri_tagged",
		.shift		= 8,
		.bits		= 1,
	},
};

static const struct gbe_sw_mod_info gbe_sw_xge_controls[] = {
	{
		.name		= "fifo_loopback",
		.shift		= 0,
		.bits		= 1,
	},
	{
		.name		= "vlan_aware",
		.shift		= 1,
		.bits		= 1,
	},
	{
		.name		= "p0_enable",
		.shift		= 2,
		.bits		= 1,
	},
	{
		.name		= "p0_pass_pri_tagged",
		.shift		= 3,
		.bits		= 1,
	},
	{
		.name		= "p1_pass_pri_tagged",
		.shift		= 4,
		.bits		= 1,
	},
	{
		.name		= "p2_pass_pri_tagged",
		.shift		= 5,
		.bits		= 1,
	},
	{
		.name		= "p0_tx_crc_type",
		.shift		= 12,
		.bits		= 1,
	},
};

static const struct gbe_sw_mod_info gbe_sw_nu_controls[] = {
	{
		.name		= "vlan_aware",
		.shift		= 1,
		.bits		= 1,
	},
	{
		.name		= "p0_enable",
		.shift		= 2,
		.bits		= 1,
	},
	{
		.name		= "p0_pass_pri_tagged",
		.shift		= 3,
		.bits		= 1,
	},
	{
		.name		= "p1_pass_pri_tagged",
		.shift		= 4,
		.bits		= 1,
	},
	{
		.name		= "p2_pass_pri_tagged",
		.shift		= 5,
		.bits		= 1,
	},
	{
		.name		= "p3_pass_pri_tagged",
		.shift		= 6,
		.bits		= 1,
	},
	{
		.name		= "p4_pass_pri_tagged",
		.shift		= 7,
		.bits		= 1,
	},
	{
		.name		= "p5_pass_pri_tagged",
		.shift		= 8,
		.bits		= 1,
	},
	{
		.name		= "p6_pass_pri_tagged",
		.shift		= 9,
		.bits		= 1,
	},
	{
		.name		= "p7_pass_pri_tagged",
		.shift		= 10,
		.bits		= 1,
	},
	{
		.name		= "p8_pass_pri_tagged",
		.shift		= 11,
		.bits		= 1,
	},
	{
		.name		= "p0_tx_crc_type",
		.shift		= 12,
		.bits		= 1,
	},
	{
		.name		= "p0_rx_pad",
		.shift		= 14,
		.bits		= 1,
	},
	{
		.name		= "p0_rx_pass_crc_err",
		.shift		= 15,
		.bits		= 1,
	},
};

static inline void
gbe_sw_info_set_reg_field(void __iomem *reg, const struct gbe_sw_mod_info *info,
			  int val)
{
	u32 rv;

	rv = readl(reg);
	rv = ((rv & ~BITMASK(info->bits, info->shift)) | (val << info->shift));
	writel(rv, reg);
}

static ssize_t
gbe_sw_attr_parse_set_command(struct gbe_priv *gbe_dev,
			      struct gbe_attribute *attr,
			      const char *buf, size_t count,
			      struct gbe_sw_parse_result *res)
{
	char ctrl_str[33], tmp_str[9];
	int port = -1, value, len, control;
	unsigned long end;
	const struct gbe_sw_mod_info *info = attr->info;

	len = strcspn(buf, ".=");
	if (len >= 32)
		return -ENOMEM;

	strncpy(ctrl_str, buf, len);
	ctrl_str[len] = '\0';
	buf += len;

	if (*buf == '.') {
		++buf;
		len = strcspn(buf, "=");
		if (len >= 8)
			return -ENOMEM;
		strncpy(tmp_str, buf, len);
		tmp_str[len] = '\0';
		if (kstrtoul(tmp_str, 0, &end))
			return -EINVAL;
		port = (int)end;
		buf += len;
	}

	if (*buf != '=')
		return -EINVAL;

	if (kstrtoul(buf + 1, 0, &end))
		return -EINVAL;

	value = (int)end;

	for (control = 0; control < attr->info_size; control++)
		if (strcmp(ctrl_str, info[control].name) == 0)
			break;

	if (control >= attr->info_size)
		return -ENOENT;

	res->control = control;
	res->port = port;
	res->value = value;

	dev_info(gbe_dev->dev, "parsed command %s.%d=%d\n",
		 attr->info[control].name, port, value);

	return 0;
}

static ssize_t
gbe_sw_attr_info_show(const struct gbe_sw_mod_info *info, int info_size,
		      u32 reg_val, char *buf)
{
	int i, len = 0;

	for (i = 0; i < info_size; i++, info++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
			"%s=%d\n", info->name,
			(int)gbe_sw_mod_info_field_val(reg_val, info));
	}

	return len;
}

static ssize_t gbe_sw_control_show(struct gbe_priv *gbe_dev,
				   struct gbe_attribute *attr, char *buf)
{
	u32 reg_val = readl(GBE_REG_ADDR(gbe_dev, switch_regs, control));

	return gbe_sw_attr_info_show(attr->info, attr->info_size, reg_val, buf);
}

static ssize_t
gbe_sw_control_store(struct gbe_priv *gbe_dev, struct gbe_attribute *attr,
		     const char *buf, size_t count)
{
	const struct gbe_sw_mod_info *info;
	struct gbe_sw_parse_result res;
	int ret;

	ret = gbe_sw_attr_parse_set_command(gbe_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	info = &attr->info[res.control];

	gbe_sw_info_set_reg_field(GBE_REG_ADDR(gbe_dev, switch_regs, control),
				  info, res.value);
	return count;
}

static struct gbe_attribute gbe_sw_ver14_control_attribute =
	      __GBE_SW_ATTR(control, S_IRUGO | S_IWUSR, gbe_sw_control_show,
			    gbe_sw_control_store, gbe_sw_ver14_controls);

static struct gbe_attribute gbe_sw_xge_control_attribute =
	      __GBE_SW_ATTR(control, S_IRUGO | S_IWUSR, gbe_sw_control_show,
			    gbe_sw_control_store, gbe_sw_xge_controls);

static struct gbe_attribute gbe_sw_nu_control_attribute =
	      __GBE_SW_ATTR(control, S_IRUGO | S_IWUSR, gbe_sw_control_show,
			    gbe_sw_control_store, gbe_sw_nu_controls);

static const struct gbe_sw_mod_info gbe_sw_ver14_ptypes[] = {
	{
		.name		= "escalate_pri_load_val",
		.shift		= 0,
		.bits		= 5,
	},
	{
		.name		= "port0_pri_type_escalate",
		.shift		= 8,
		.bits		= 1,
	},
	{
		.name		= "port1_pri_type_escalate",
		.shift		= 9,
		.bits		= 1,
	},
	{
		.name		= "port2_pri_type_escalate",
		.shift		= 10,
		.bits		= 1,
	},
	{
		.name		= "port3_pri_type_escalate",
		.shift		= 11,
		.bits		= 1,
	},
	{
		.name		= "port4_pri_type_escalate",
		.shift		= 12,
		.bits		= 1,
	},
};

static const struct gbe_sw_mod_info gbe_sw_xge_ptypes[] = {
	{
		.name		= "escalate_pri_load_val",
		.shift		= 0,
		.bits		= 5,
	},
	{
		.name		= "port0_pri_type_escalate",
		.shift		= 8,
		.bits		= 1,
	},
	{
		.name		= "port1_pri_type_escalate",
		.shift		= 9,
		.bits		= 1,
	},
	{
		.name		= "port2_pri_type_escalate",
		.shift		= 10,
		.bits		= 1,
	},
};

static const struct gbe_sw_mod_info gbe_sw_nu_ptypes[] = {
	{
		.name		= "escalate_pri_load_val",
		.shift		= 0,
		.bits		= 5,
	},
	{
		.name		= "port0_pri_type_escalate",
		.shift		= 8,
		.bits		= 1,
	},
	{
		.name		= "port1_pri_type_escalate",
		.shift		= 9,
		.bits		= 1,
	},
	{
		.name		= "port2_pri_type_escalate",
		.shift		= 10,
		.bits		= 1,
	},
	{
		.name		= "port3_pri_type_escalate",
		.shift		= 11,
		.bits		= 1,
	},
	{
		.name		= "port4_pri_type_escalate",
		.shift		= 12,
		.bits		= 1,
	},
	{
		.name		= "port5_pri_type_escalate",
		.shift		= 13,
		.bits		= 1,
	},
	{
		.name		= "port6_pri_type_escalate",
		.shift		= 14,
		.bits		= 1,
	},
	{
		.name		= "port7_pri_type_escalate",
		.shift		= 15,
		.bits		= 1,
	},
	{
		.name		= "port8_pri_type_escalate",
		.shift		= 16,
		.bits		= 1,
	},
};

static ssize_t gbe_sw_pri_type_show(struct gbe_priv *gbe_dev,
				    struct gbe_attribute *attr, char *buf)
{
	u32 reg_val = readl(GBE_REG_ADDR(gbe_dev, switch_regs, ptype));

	return gbe_sw_attr_info_show(attr->info, attr->info_size, reg_val, buf);
}

static ssize_t gbe_sw_pri_type_store(struct gbe_priv *gbe_dev,
				     struct gbe_attribute *attr,
				     const char *buf, size_t count)
{
	const struct gbe_sw_mod_info *info;
	struct gbe_sw_parse_result res;
	int ret;

	ret = gbe_sw_attr_parse_set_command(gbe_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	info = &attr->info[res.control];

	gbe_sw_info_set_reg_field(GBE_REG_ADDR(gbe_dev, switch_regs, ptype),
				  info, res.value);
	return count;
}

static struct gbe_attribute gbe_sw_ver14_pri_type_attribute =
			__GBE_SW_ATTR(priority_type, S_IRUGO | S_IWUSR,
				      gbe_sw_pri_type_show,
				      gbe_sw_pri_type_store,
				      gbe_sw_ver14_ptypes);

static struct gbe_attribute gbe_sw_xge_pri_type_attribute =
			__GBE_SW_ATTR(priority_type, S_IRUGO | S_IWUSR,
				      gbe_sw_pri_type_show,
				      gbe_sw_pri_type_store,
				      gbe_sw_xge_ptypes);

static struct gbe_attribute gbe_sw_nu_pri_type_attribute =
			__GBE_SW_ATTR(priority_type, S_IRUGO | S_IWUSR,
				      gbe_sw_pri_type_show,
				      gbe_sw_pri_type_store,
				      gbe_sw_nu_ptypes);

static const struct gbe_sw_mod_info gbe_sw_ver14_flow_controls[] = {
	{
		.name		= "port0_flow_control_en",
		.shift		= 0,
		.bits		= 1,
	},
	{
		.name		= "port1_flow_control_en",
		.shift		= 1,
		.bits		= 1,
	},
	{
		.name		= "port2_flow_control_en",
		.shift		= 2,
		.bits		= 1,
	},
	{
		.name		= "port3_flow_control_en",
		.shift		= 3,
		.bits		= 1,
	},
	{
		.name		= "port4_flow_control_en",
		.shift		= 4,
		.bits		= 1,
	},
};

static const struct gbe_sw_mod_info gbe_sw_xge_flow_controls[] = {
	{
		.name		= "port0_flow_control_en",
		.shift		= 0,
		.bits		= 1,
	},
	{
		.name		= "port1_flow_control_en",
		.shift		= 1,
		.bits		= 1,
	},
	{
		.name		= "port2_flow_control_en",
		.shift		= 2,
		.bits		= 1,
	},
};

static ssize_t gbe_sw_flow_control_show(struct gbe_priv *gbe_dev,
					struct gbe_attribute *attr, char *buf)
{
	u32 reg_val = readl(GBE_REG_ADDR(gbe_dev, switch_regs, flow_control));

	return gbe_sw_attr_info_show(attr->info, attr->info_size, reg_val, buf);
}

static ssize_t gbe_sw_flow_control_store(struct gbe_priv *gbe_dev,
					 struct gbe_attribute *attr,
					 const char *buf, size_t count)
{
	const struct gbe_sw_mod_info *info;
	struct gbe_sw_parse_result res;
	int ret;

	ret = gbe_sw_attr_parse_set_command(gbe_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	info = &attr->info[res.control];

	gbe_sw_info_set_reg_field(GBE_REG_ADDR(gbe_dev, switch_regs,
					       flow_control),
					       info, res.value);
	return count;
}

static struct gbe_attribute gbe_sw_ver14_flow_control_attribute =
			__GBE_SW_ATTR(flow_control, S_IRUGO | S_IWUSR,
				      gbe_sw_flow_control_show,
				      gbe_sw_flow_control_store,
				      gbe_sw_ver14_flow_controls);

static struct gbe_attribute gbe_sw_xge_flow_control_attribute =
			__GBE_SW_ATTR(flow_control, S_IRUGO | S_IWUSR,
				      gbe_sw_flow_control_show,
				      gbe_sw_flow_control_store,
				      gbe_sw_xge_flow_controls);

static struct attribute *gbe_sw_ver14_default_attrs[] = {
	&gbe_sw_version_attribute.attr,
	&gbe_sw_ver14_control_attribute.attr,
	&gbe_sw_ver14_pri_type_attribute.attr,
	&gbe_sw_ver14_flow_control_attribute.attr,
	NULL
};

static struct attribute *gbe_sw_xge_default_attrs[] = {
	&gbe_sw_version_attribute.attr,
	&gbe_sw_xge_control_attribute.attr,
	&gbe_sw_xge_pri_type_attribute.attr,
	&gbe_sw_xge_flow_control_attribute.attr,
	NULL
};

static struct attribute *gbe_sw_nu_default_attrs[] = {
	&gbe_sw_version_attribute.attr,
	&gbe_sw_nu_control_attribute.attr,
	&gbe_sw_nu_pri_type_attribute.attr,
	NULL
};

static const struct gbe_sw_mod_info gbe_sw_port_tx_pri_maps[] = {
	{
		.name		= "port_tx_pri_0",
		.shift		= 0,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_1",
		.shift		= 4,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_2",
		.shift		= 8,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_3",
		.shift		= 12,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_4",
		.shift		= 16,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_5",
		.shift		= 20,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_6",
		.shift		= 24,
		.bits		= 3,
	},
	{
		.name		= "port_tx_pri_7",
		.shift		= 28,
		.bits		= 3,
	},
};

static ssize_t gbe_sw_port_tx_pri_map_show(struct gbe_priv *gbe_dev,
					   struct gbe_attribute *attr,
					   char *buf)
{
	int len = 0, total_len = 0, port;
	struct gbe_slave *slave;
	u32 reg_val;

	port = (int)(attr->context);

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	reg_val = readl(GBE_REG_ADDR(slave, port_regs, tx_pri_map));
	len = gbe_sw_attr_info_show(attr->info, attr->info_size, reg_val, buf);

	return (total_len += len);
}

static ssize_t gbe_sw_port_tx_pri_map_store(struct gbe_priv *gbe_dev,
					    struct gbe_attribute *attr,
					    const char *buf, size_t count)
{
	const struct gbe_sw_mod_info *info;
	struct gbe_sw_parse_result res;
	void __iomem *reg = NULL;
	struct gbe_slave *slave;
	int ret, port;

	port = (int)(attr->context);

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	ret = gbe_sw_attr_parse_set_command(gbe_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	info = &attr->info[res.control];

	reg = GBE_REG_ADDR(slave, port_regs, tx_pri_map);
	if (!reg)
		return  -ENOENT;

	gbe_sw_info_set_reg_field(reg, info, res.value);

	return count;
}

static struct gbe_attribute gbe_sw_tx_pri_0_attribute =
			__GBE_SW_CTXT_ATTR(0, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)0);

static struct gbe_attribute gbe_sw_tx_pri_1_attribute =
			__GBE_SW_CTXT_ATTR(1, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)1);

static struct gbe_attribute gbe_sw_tx_pri_2_attribute =
			__GBE_SW_CTXT_ATTR(2, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)2);

static struct gbe_attribute gbe_sw_tx_pri_3_attribute =
			__GBE_SW_CTXT_ATTR(3, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)3);

static struct gbe_attribute gbe_sw_tx_pri_4_attribute =
			__GBE_SW_CTXT_ATTR(4, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)4);

static struct gbe_attribute gbe_sw_tx_pri_5_attribute =
			__GBE_SW_CTXT_ATTR(5, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)5);

static struct gbe_attribute gbe_sw_tx_pri_6_attribute =
			__GBE_SW_CTXT_ATTR(6, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)6);

static struct gbe_attribute gbe_sw_tx_pri_7_attribute =
			__GBE_SW_CTXT_ATTR(7, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)7);

static struct gbe_attribute gbe_sw_tx_pri_8_attribute =
			__GBE_SW_CTXT_ATTR(8, S_IRUGO | S_IWUSR,
					   gbe_sw_port_tx_pri_map_show,
					   gbe_sw_port_tx_pri_map_store,
					   gbe_sw_port_tx_pri_maps, (void *)8);

static struct attribute *gbe_sw_ver14_tx_pri_default_attrs[] = {
	&gbe_sw_tx_pri_1_attribute.attr,
	&gbe_sw_tx_pri_2_attribute.attr,
	&gbe_sw_tx_pri_3_attribute.attr,
	&gbe_sw_tx_pri_4_attribute.attr,
	NULL
};

static struct attribute *gbe_sw_xge_tx_pri_default_attrs[] = {
	&gbe_sw_tx_pri_0_attribute.attr,
	&gbe_sw_tx_pri_1_attribute.attr,
	&gbe_sw_tx_pri_2_attribute.attr,
	NULL
};

static struct attribute *gbe_sw_nu_tx_pri_default_attrs[] = {
	&gbe_sw_tx_pri_1_attribute.attr,
	&gbe_sw_tx_pri_2_attribute.attr,
	&gbe_sw_tx_pri_3_attribute.attr,
	&gbe_sw_tx_pri_4_attribute.attr,
	&gbe_sw_tx_pri_5_attribute.attr,
	&gbe_sw_tx_pri_6_attribute.attr,
	&gbe_sw_tx_pri_7_attribute.attr,
	&gbe_sw_tx_pri_8_attribute.attr,
	NULL
};

static ssize_t gbe_sw_tx_pri_attr_show(struct kobject *kobj,
				       struct attribute *attr, char *buf)
{
	struct gbe_attribute *attribute = to_gbe_attr(attr);
	struct gbe_priv *gbe_dev = tx_pri_to_gbe_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(gbe_dev, attribute, buf);
}

static ssize_t gbe_sw_tx_pri_attr_store(struct kobject *kobj,
					struct attribute *attr,
					const char *buf, size_t count)
{
	struct gbe_attribute *attribute = to_gbe_attr(attr);
	struct gbe_priv *gbe_dev = tx_pri_to_gbe_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(gbe_dev, attribute, buf, count);
}

static const struct sysfs_ops gbe_sw_tx_pri_sysfs_ops = {
	.show = gbe_sw_tx_pri_attr_show,
	.store = gbe_sw_tx_pri_attr_store,
};

static struct kobj_type gbe_sw_ver14_tx_pri_ktype = {
	.sysfs_ops = &gbe_sw_tx_pri_sysfs_ops,
	.default_attrs = gbe_sw_ver14_tx_pri_default_attrs,
};

static struct kobj_type gbe_sw_xge_tx_pri_ktype = {
	.sysfs_ops = &gbe_sw_tx_pri_sysfs_ops,
	.default_attrs = gbe_sw_xge_tx_pri_default_attrs,
};

static struct kobj_type gbe_sw_nu_tx_pri_ktype = {
	.sysfs_ops = &gbe_sw_tx_pri_sysfs_ops,
	.default_attrs = gbe_sw_nu_tx_pri_default_attrs,
};

static const struct gbe_sw_mod_info gbe_sw_port_vlans[] = {
	{
		.name		= "port_vlan_id",
		.shift		= 0,
		.bits		= 12,
	},
	{
		.name		= "port_cfi",
		.shift		= 12,
		.bits		= 1,
	},
	{
		.name		= "port_vlan_pri",
		.shift		= 13,
		.bits		= 3,
	},
};

static ssize_t gbe_sw_port_vlan_show(struct gbe_priv *gbe_dev,
				     struct gbe_attribute *attr,
				     char *buf)
{
	int len = 0, total_len = 0, port;
	struct gbe_slave *slave;
	u32 reg_val;

	port = (int)(attr->context);

	if (port == gbe_dev->host_port) {
		/* Host port */
		reg_val = readl(GBE_REG_ADDR(gbe_dev, host_port_regs,
					     port_vlan));
		len = gbe_sw_attr_info_show(attr->info, attr->info_size,
					    reg_val, buf);
		return len;
	}

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	reg_val = readl(GBE_REG_ADDR(slave, port_regs, port_vlan));
	len = gbe_sw_attr_info_show(attr->info, attr->info_size, reg_val, buf);

	return (total_len += len);
}

static ssize_t gbe_sw_port_vlan_store(struct gbe_priv *gbe_dev,
				      struct gbe_attribute *attr,
				      const char *buf, size_t count)
{
	const struct gbe_sw_mod_info *info;
	struct gbe_sw_parse_result res;
	struct gbe_slave *slave;
	void __iomem *reg = NULL;
	int ret, port;

	port = (int)(attr->context);

	ret = gbe_sw_attr_parse_set_command(gbe_dev, attr, buf, count, &res);
	if (ret)
		return ret;

	info = &attr->info[res.control];

	/* Host port */
	if (port == gbe_dev->host_port) {
		reg = GBE_REG_ADDR(gbe_dev, host_port_regs, port_vlan);
		goto set;
	}

	slave = gbe_port_num_get_slave(gbe_dev, port);
	if (!slave)
		return 0;

	/* Slave port */
	reg = GBE_REG_ADDR(slave, port_regs, port_vlan);
	if (!reg)
		return  -ENOENT;

set:
	gbe_sw_info_set_reg_field(reg, info, res.value);

	return count;
}

static struct gbe_attribute gbe_sw_pvlan_0_attribute =
	__GBE_SW_CTXT_ATTR(0, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)0);

static struct gbe_attribute gbe_sw_pvlan_1_attribute =
	__GBE_SW_CTXT_ATTR(1, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)1);

static struct gbe_attribute gbe_sw_pvlan_2_attribute =
	__GBE_SW_CTXT_ATTR(2, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)2);

static struct gbe_attribute gbe_sw_pvlan_3_attribute =
	__GBE_SW_CTXT_ATTR(3, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)3);

static struct gbe_attribute gbe_sw_pvlan_4_attribute =
	__GBE_SW_CTXT_ATTR(4, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)4);

static struct gbe_attribute gbe_sw_pvlan_5_attribute =
	__GBE_SW_CTXT_ATTR(5, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)5);

static struct gbe_attribute gbe_sw_pvlan_6_attribute =
	__GBE_SW_CTXT_ATTR(6, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)6);

static struct gbe_attribute gbe_sw_pvlan_7_attribute =
	__GBE_SW_CTXT_ATTR(7, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)7);

static struct gbe_attribute gbe_sw_pvlan_8_attribute =
	__GBE_SW_CTXT_ATTR(8, S_IRUGO | S_IWUSR,
			   gbe_sw_port_vlan_show,
			   gbe_sw_port_vlan_store,
			   gbe_sw_port_vlans, (void *)8);

static struct attribute *gbe_sw_ver14_pvlan_default_attrs[] = {
	&gbe_sw_pvlan_0_attribute.attr,
	&gbe_sw_pvlan_1_attribute.attr,
	&gbe_sw_pvlan_2_attribute.attr,
	&gbe_sw_pvlan_3_attribute.attr,
	&gbe_sw_pvlan_4_attribute.attr,
	NULL
};

static struct attribute *gbe_sw_xge_pvlan_default_attrs[] = {
	&gbe_sw_pvlan_0_attribute.attr,
	&gbe_sw_pvlan_1_attribute.attr,
	&gbe_sw_pvlan_2_attribute.attr,
	NULL
};

static struct attribute *gbe_sw_nu_pvlan_default_attrs[] = {
	&gbe_sw_pvlan_0_attribute.attr,
	&gbe_sw_pvlan_1_attribute.attr,
	&gbe_sw_pvlan_2_attribute.attr,
	&gbe_sw_pvlan_3_attribute.attr,
	&gbe_sw_pvlan_4_attribute.attr,
	&gbe_sw_pvlan_5_attribute.attr,
	&gbe_sw_pvlan_6_attribute.attr,
	&gbe_sw_pvlan_7_attribute.attr,
	&gbe_sw_pvlan_8_attribute.attr,
	NULL
};

static ssize_t gbe_sw_pvlan_attr_show(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	struct gbe_attribute *attribute = to_gbe_attr(attr);
	struct gbe_priv *gbe_dev = pvlan_to_gbe_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(gbe_dev, attribute, buf);
}

static ssize_t gbe_sw_pvlan_attr_store(struct kobject *kobj,
				       struct attribute *attr,
				       const char *buf, size_t count)
{
	struct gbe_attribute *attribute = to_gbe_attr(attr);
	struct gbe_priv *gbe_dev = pvlan_to_gbe_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(gbe_dev, attribute, buf, count);
}

static const struct sysfs_ops gbe_sw_pvlan_sysfs_ops = {
	.show = gbe_sw_pvlan_attr_show,
	.store = gbe_sw_pvlan_attr_store,
};

static struct kobj_type gbe_sw_ver14_pvlan_ktype = {
	.sysfs_ops = &gbe_sw_pvlan_sysfs_ops,
	.default_attrs = gbe_sw_ver14_pvlan_default_attrs,
};

static struct kobj_type gbe_sw_xge_pvlan_ktype = {
	.sysfs_ops = &gbe_sw_pvlan_sysfs_ops,
	.default_attrs = gbe_sw_xge_pvlan_default_attrs,
};

static struct kobj_type gbe_sw_nu_pvlan_ktype = {
	.sysfs_ops = &gbe_sw_pvlan_sysfs_ops,
	.default_attrs = gbe_sw_nu_pvlan_default_attrs,
};

static ssize_t gbe_sw_stats_attr_store(struct kobject *kobj,
				       struct attribute *attr,
				       const char *buf, size_t count)
{
	struct gbe_attribute *attribute = to_gbe_attr(attr);
	struct gbe_priv *gbe_dev = stats_to_gbe_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(gbe_dev, attribute, buf, count);
}

static const struct sysfs_ops gbe_sw_stats_sysfs_ops = {
	.store = gbe_sw_stats_attr_store,
};

static ssize_t gbe_sw_stats_mod_store(struct gbe_priv *gbe_dev,
				      struct gbe_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long end;
	int stat_mod;

	if (kstrtoul(buf, 0, &end) != 0 || (end != 0))
		return -EINVAL;

	stat_mod = (int)(attr->context);
	spin_lock_bh(&gbe_dev->hw_stats_lock);
	if (gbe_dev->ss_version == GBE_SS_VERSION_14)
		gbe_reset_mod_stats_ver14(gbe_dev, stat_mod);
	else
		gbe_reset_mod_stats(gbe_dev, stat_mod);
	spin_unlock_bh(&gbe_dev->hw_stats_lock);
	return count;
}

static struct gbe_attribute gbe_sw_stats_a_attribute =
			__GBE_SW_ATTR_FULL(A, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0, (void *)GBE_STATSA_MODULE);

static struct gbe_attribute gbe_sw_stats_b_attribute =
			__GBE_SW_ATTR_FULL(B, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0, (void *)GBE_STATSB_MODULE);

static struct gbe_attribute gbe_sw_stats_c_attribute =
			__GBE_SW_ATTR_FULL(C, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0, (void *)GBE_STATSC_MODULE);

static struct gbe_attribute gbe_sw_stats_d_attribute =
			__GBE_SW_ATTR_FULL(D, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0, (void *)GBE_STATSD_MODULE);

static struct attribute *gbe_sw_ver14_stats_default_attrs[] = {
	&gbe_sw_stats_a_attribute.attr,
	&gbe_sw_stats_b_attribute.attr,
	&gbe_sw_stats_c_attribute.attr,
	&gbe_sw_stats_d_attribute.attr,
	NULL
};

static struct kobj_type gbe_sw_ver14_stats_ktype = {
	.sysfs_ops = &gbe_sw_stats_sysfs_ops,
	.default_attrs = gbe_sw_ver14_stats_default_attrs,
};

static struct gbe_attribute gbe_sw_xge_stats_0_attribute =
			__GBE_SW_ATTR_FULL(0, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0, (void *)XGBE_STATS0_MODULE);

static struct gbe_attribute gbe_sw_xge_stats_1_attribute =
			__GBE_SW_ATTR_FULL(1, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0, (void *)XGBE_STATS1_MODULE);

static struct gbe_attribute gbe_sw_xge_stats_2_attribute =
			__GBE_SW_ATTR_FULL(2, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0, (void *)XGBE_STATS2_MODULE);

static struct attribute *gbe_sw_xge_stats_default_attrs[] = {
	&gbe_sw_xge_stats_0_attribute.attr,
	&gbe_sw_xge_stats_1_attribute.attr,
	&gbe_sw_xge_stats_2_attribute.attr,
	NULL
};

static struct kobj_type gbe_sw_xge_stats_ktype = {
	.sysfs_ops = &gbe_sw_stats_sysfs_ops,
	.default_attrs = gbe_sw_xge_stats_default_attrs,
};

static struct gbe_attribute gbe_sw_stats_0_attribute =
			__GBE_SW_ATTR_FULL(0, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS0_MODULE);

static struct gbe_attribute gbe_sw_stats_1_attribute =
			__GBE_SW_ATTR_FULL(1, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS1_MODULE);

static struct gbe_attribute gbe_sw_stats_2_attribute =
			__GBE_SW_ATTR_FULL(2, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS2_MODULE);

static struct gbe_attribute gbe_sw_stats_3_attribute =
			__GBE_SW_ATTR_FULL(3, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS3_MODULE);

static struct gbe_attribute gbe_sw_stats_4_attribute =
			__GBE_SW_ATTR_FULL(4, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS4_MODULE);

static struct gbe_attribute gbe_sw_stats_5_attribute =
			__GBE_SW_ATTR_FULL(5, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS5_MODULE);

static struct gbe_attribute gbe_sw_stats_6_attribute =
			__GBE_SW_ATTR_FULL(6, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS6_MODULE);

static struct gbe_attribute gbe_sw_stats_7_attribute =
			__GBE_SW_ATTR_FULL(7, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS7_MODULE);

static struct gbe_attribute gbe_sw_stats_8_attribute =
			__GBE_SW_ATTR_FULL(8, S_IWUSR, NULL,
					   gbe_sw_stats_mod_store,
					   NULL, 0,
					   (void *)GBENU_STATS8_MODULE);

static struct attribute *gbe_sw_nu_stats_default_attrs[] = {
	&gbe_sw_stats_0_attribute.attr,
	&gbe_sw_stats_1_attribute.attr,
	&gbe_sw_stats_2_attribute.attr,
	&gbe_sw_stats_3_attribute.attr,
	&gbe_sw_stats_4_attribute.attr,
	&gbe_sw_stats_5_attribute.attr,
	&gbe_sw_stats_6_attribute.attr,
	&gbe_sw_stats_7_attribute.attr,
	&gbe_sw_stats_8_attribute.attr,
	NULL
};

static struct kobj_type gbe_sw_nu_stats_ktype = {
	.sysfs_ops = &gbe_sw_stats_sysfs_ops,
	.default_attrs = gbe_sw_nu_stats_default_attrs,
};

static ssize_t gbe_sw_attr_show(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	struct gbe_attribute *attribute = to_gbe_attr(attr);
	struct gbe_priv *gbe_dev = to_gbe_dev(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(gbe_dev, attribute, buf);
}

static ssize_t gbe_sw_attr_store(struct kobject *kobj,
				 struct attribute *attr, const char *buf,
				 size_t count)
{
	struct gbe_attribute *attribute = to_gbe_attr(attr);
	struct gbe_priv *gbe_dev = to_gbe_dev(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(gbe_dev, attribute, buf, count);
}

static const struct sysfs_ops gbe_sw_sysfs_ops = {
	.show = gbe_sw_attr_show,
	.store = gbe_sw_attr_store,
};

static struct kobj_type gbe_sw_ver14_ktype = {
	.sysfs_ops = &gbe_sw_sysfs_ops,
	.default_attrs = gbe_sw_ver14_default_attrs,
};

static struct kobj_type gbe_sw_xge_ktype = {
	.sysfs_ops = &gbe_sw_sysfs_ops,
	.default_attrs = gbe_sw_xge_default_attrs,
};

static struct kobj_type gbe_sw_nu_ktype = {
	.sysfs_ops = &gbe_sw_sysfs_ops,
	.default_attrs = gbe_sw_nu_default_attrs,
};

/* for ver14 switch */
static struct kobj_type *gbe_sw_ver14_kobjs[GBE_SYSFS_SW_MAX] = {
	&gbe_sw_ver14_ktype,
	&gbe_sw_ver14_tx_pri_ktype,
	&gbe_sw_ver14_pvlan_ktype,
	&gbe_sw_ver14_stats_ktype,
};

/* for xge switch */
static struct kobj_type *gbe_sw_xge_kobjs[GBE_SYSFS_SW_MAX] = {
	&gbe_sw_xge_ktype,
	&gbe_sw_xge_tx_pri_ktype,
	&gbe_sw_xge_pvlan_ktype,
	&gbe_sw_xge_stats_ktype,
};

/* for NU switch */
static struct kobj_type *gbe_sw_nu_kobjs[GBE_SYSFS_SW_MAX] = {
	&gbe_sw_nu_ktype,
	&gbe_sw_nu_tx_pri_ktype,
	&gbe_sw_nu_pvlan_ktype,
	&gbe_sw_nu_stats_ktype,
};

int gbe_create_sysfs_entries(struct gbe_priv *gbe_dev)
{
	struct device *dev = gbe_dev->dev;
	static struct kobj_type **kobjs;
	int ret;

	switch (gbe_dev->ss_version) {
	case XGBE_SS_VERSION_10:
		kobjs = &gbe_sw_xge_kobjs[0];
		break;
	case GBE_SS_VERSION_14:
		kobjs = &gbe_sw_ver14_kobjs[0];
		break;
	default:
		/* we don't support 2U switch */
		if (!IS_SS_ID_NU(gbe_dev))
			return 0;
		kobjs = &gbe_sw_nu_kobjs[0];
	}

	ret = kobject_init_and_add(&gbe_dev->kobj, kobjs[GBE_SYSFS_SW_CONTROL],
				   kobject_get(&dev->kobj), "gbe_sw");

	if (ret) {
		dev_err(dev, "failed to create gbe sw sysfs entry\n");
		kobject_put(&gbe_dev->kobj);
		kobject_put(&dev->kobj);
		return ret;
	}

	ret = kobject_init_and_add(&gbe_dev->tx_pri_kobj,
				   kobjs[GBE_SYSFS_SW_TX_PRIO],
				   kobject_get(&gbe_dev->kobj),
				   "port_tx_pri_map");

	if (ret) {
		dev_err(dev, "failed to create sysfs port_tx_pri_map entry\n");
		kobject_put(&gbe_dev->tx_pri_kobj);
		kobject_put(&gbe_dev->kobj);
		return ret;
	}

	ret = kobject_init_and_add(&gbe_dev->pvlan_kobj,
				   kobjs[GBE_SYSFS_SW_VLAN],
				   kobject_get(&gbe_dev->kobj), "port_vlan");

	if (ret) {
		dev_err(dev, "failed to create sysfs port_vlan entry\n");
		kobject_put(&gbe_dev->pvlan_kobj);
		kobject_put(&gbe_dev->kobj);
		return ret;
	}

	ret = kobject_init_and_add(&gbe_dev->stats_kobj,
				   kobjs[GBE_SYSFS_SW_STATS],
				   kobject_get(&gbe_dev->kobj), "stats");

	if (ret) {
		dev_err(dev, "failed to create sysfs stats entry\n");
		kobject_put(&gbe_dev->stats_kobj);
		kobject_put(&gbe_dev->kobj);
		return ret;
	}

	ret = gbe_create_cpts_sysfs(gbe_dev);

	return ret;
}
