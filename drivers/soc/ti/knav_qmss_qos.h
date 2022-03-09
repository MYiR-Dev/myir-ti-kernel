/*
 * Keystone Navigator QMSS QoS driver internal header
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com
 * Author:	WingMan Kwok <w-kwok2@ti.com>
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

#ifndef __KNAV_QMSS_QOS_H__
#define __KNAV_QMSS_QOS_H__

#include "knav_qmss.h"

#ifdef CONFIG_KEYSTONE_NAVIGATOR_QMSS_QOS
int knav_init_qos_range(struct knav_device *kdev,
			struct device_node *node,
			struct knav_range_info *range);
#else
static int knav_init_qos_range(struct knav_device *kdev,
			       struct device_node *node,
			       struct knav_range_info *range)
{
	return 0;
}
#endif

#endif /* __KNAV_QMSS_QOS_H__ */
