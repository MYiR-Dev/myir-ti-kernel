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

struct gbe_slave *gbe_port_num_get_slave(struct gbe_priv *gbe_dev, int port)
{
	struct gbe_intf *gbe_intf;

	for_each_intf(gbe_intf, gbe_dev) {
		if (gbe_intf->slave->port_num == port)
			return gbe_intf->slave;
	}
	return NULL;
}

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

int gbe_create_cpts_sysfs(struct gbe_priv *gbe_dev)
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
int gbe_create_cpts_sysfs(struct gbe_priv *gbe_dev)
{
	return 0;
}
#endif /* CONFIG_TI_KEYSTONE_NETCP_CPTS */
