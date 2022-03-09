/*
 * Copyright (C) 2015 Texas Instruments Incorporated
 * Authors: Reece Pollack <reece@theptrgroup.com>
 *	    WingMan Kwok <w-kwok2@ti.com>
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

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/soc/ti/knav_qmss.h>
#include <linux/soc/ti/knav_dma.h>

#include "netcp.h"

#define QOS_MODULE_NAME		"netcp-qos"
#define	QOS_TXHOOK_ORDER	20

struct qos_subqueue {
	struct netcp_tx_pipe	 tx_pipe;
	struct kobject		 kobj;
};

struct qos_priv {
	struct netcp_device		*netcp_device;
	struct device			*dev;
	const char			*dma_chan_name;
	struct kobject			 kobj;
};

struct qos_intf {
	struct qos_priv			*qos_priv;
	struct net_device		*ndev;
	struct device			*dev;
	struct kobject			 kobj;
	int				 max_subqueues;
	struct qos_subqueue		 subqueues[1];
	/* NB: tx-subqueues are allocated dynamically */
};

/* Sysfs stuff related to QoS Subqueues
 */
#define kobj_to_subqueue(kobj) container_of(kobj, struct qos_subqueue, kobj)

static ssize_t qos_txqueue_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct qos_subqueue *subq = kobj_to_subqueue(kobj);

	return snprintf(buf, PAGE_SIZE, "%u\n", subq->tx_pipe.dma_queue_id);
}

struct kobj_attribute qos_txqueue_attr =
	__ATTR(tx_queue, S_IRUGO, qos_txqueue_show, NULL);

static ssize_t qos_txchan_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	struct qos_subqueue *subq = kobj_to_subqueue(kobj);

	return snprintf(buf, PAGE_SIZE, "%s\n", subq->tx_pipe.dma_chan_name);
}

struct kobj_attribute qos_txchan_attr =
	__ATTR(tx_channel, S_IRUGO, qos_txchan_show, NULL);

static struct attribute *qos_subqueue_attrs[] = {
		&qos_txqueue_attr.attr,
		&qos_txchan_attr.attr,
		NULL
};

static struct kobj_type qos_subqueue_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
	.default_attrs = qos_subqueue_attrs,
};

/* Sysfs stuff related to QoS Interfaces */
static struct kobj_type qos_intf_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
};

static struct kobj_type qos_inst_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
};

static int qos_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct qos_intf *qos_intf = data;
	struct sk_buff *skb = p_info->skb;
	u16 queue_mapping = skb_get_queue_mapping(skb);

	dev_dbg(qos_intf->dev, "priority: %u, queue_mapping: %04x\n",
		skb->priority, queue_mapping);

	if ((queue_mapping < qos_intf->max_subqueues) &&
	    qos_intf->subqueues[queue_mapping].tx_pipe.dma_queue)
		p_info->tx_pipe = &qos_intf->subqueues[queue_mapping].tx_pipe;
	else {
		dev_warn(qos_intf->dev,
			 "queue mapping %hu invalid, QoS bypassed\n",
			 queue_mapping);
	}

	return 0;
}

static int qos_close(void *intf_priv, struct net_device *ndev)
{
	struct qos_intf *qos_intf = intf_priv;
	struct netcp_intf *netcp_intf = netdev_priv(ndev);
	int i;

	netcp_unregister_txhook(netcp_intf, QOS_TXHOOK_ORDER, qos_tx_hook,
				qos_intf);

	for (i = 0; i < qos_intf->max_subqueues; ++i) {
		struct qos_subqueue *subq = &qos_intf->subqueues[i];

		if (subq->tx_pipe.dma_chan_name)
			netcp_txpipe_close(&subq->tx_pipe);
	}

	return 0;
}

static int qos_open(void *intf_priv, struct net_device *ndev)
{
	struct qos_intf *qos_intf = intf_priv;
	struct netcp_intf *netcp_intf = netdev_priv(ndev);
	int ret;
	int i;

	/* Open the QoS input queues */
	for (i = 0; i < qos_intf->max_subqueues; ++i) {
		struct qos_subqueue *subq = &qos_intf->subqueues[i];

		if (subq->tx_pipe.dma_chan_name) {
			ret = netcp_txpipe_open(&subq->tx_pipe);
			if (ret)
				goto fail;
		}
	}

	netcp_register_txhook(netcp_intf, QOS_TXHOOK_ORDER,
			      qos_tx_hook, qos_intf);
	return 0;

fail:
	qos_close(intf_priv, ndev);
	return ret;
}

static int init_tx_subqueue(struct qos_intf *qos_intf, int index,
			    unsigned int tx_queue_id)
{
	struct netcp_intf *netcp = netdev_priv(qos_intf->ndev);
	struct qos_subqueue *subq = &qos_intf->subqueues[index];
	u8 name[32];
	int ret;

	netcp_txpipe_init(&subq->tx_pipe, netcp->netcp_device,
			  qos_intf->qos_priv->dma_chan_name, tx_queue_id);

	/* Create the per-subqueue entry and attributes */
	snprintf(name, sizeof(name), "subqueue-%d", index);
	ret = kobject_init_and_add(&subq->kobj, &qos_subqueue_ktype,
				   kobject_get(&qos_intf->kobj), name);
	if (ret) {
		dev_err(qos_intf->dev,
			"failed to create %s/%s/%s sysfs entry\n",
			qos_intf->qos_priv->kobj.name, qos_intf->kobj.name,
			name);
		kobject_put(&subq->kobj);
		kobject_put(&qos_intf->kobj);
		return ret;
	}

	return 0;
}

static int qos_attach(void *inst_priv, struct net_device *ndev,
		      struct device_node *node, void **intf_priv)
{
	struct netcp_intf *netcp = netdev_priv(ndev);
	struct qos_priv *qos_priv = inst_priv;
	struct qos_intf *qos_intf;
	int temp[NETCP_MAX_SUBQUEUES], size, ret, tx_queues, i;

	tx_queues = netcp->ndev->num_tx_queues;
	size = sizeof(struct qos_intf) +
			((tx_queues - 1) * sizeof(struct qos_subqueue));
	qos_intf = devm_kzalloc(qos_priv->dev, size, GFP_KERNEL);
	if (!qos_intf) {
		dev_err(qos_priv->dev,
			"qos interface memory allocation failed\n");
		return -ENOMEM;
	}
	qos_intf->max_subqueues = tx_queues;

	qos_intf->qos_priv = qos_priv;
	qos_intf->ndev = ndev;
	qos_intf->dev = qos_priv->dev;
	*intf_priv = qos_intf;

	/* Create the per-interface sysfs entry */
	ret = kobject_init_and_add(&qos_intf->kobj, &qos_intf_ktype,
				   kobject_get(&qos_intf->qos_priv->kobj),
				   node->name);
	if (ret) {
		dev_err(qos_intf->dev, "failed to create %s/%s sysfs entry\n",
			qos_intf->qos_priv->kobj.name, node->name);
		kobject_put(&qos_intf->kobj);
		kobject_put(&qos_intf->qos_priv->kobj);
		goto exit;
	}

	if (!of_get_property(node, "tx-queues", &tx_queues)) {
		dev_err(qos_priv->dev, "missing \"tx-queues\" parameter\n");
		return -ENODEV;
	}

	/* makes sure queues defined not exceeding intf can support */
	tx_queues = min(tx_queues, qos_intf->max_subqueues);

	ret = of_property_read_u32_array(node, "tx-queues", temp, tx_queues);
	if (ret < 0) {
		dev_err(qos_priv->dev, "error in \"tx-queues\" parameter\n");
		tx_queues = 1;
	}

	for (i = 0; i < tx_queues; i++) {
		if (temp[i] < 0) {
			dev_dbg(qos_intf->dev,
				"no tx-queue defined for mapping %d\n", i);
			continue;
		}

		dev_dbg(qos_intf->dev, "mapping %d as %d\n", temp[i], i);

		ret = init_tx_subqueue(qos_intf, i, temp[i]);
		if (ret) {
			dev_err(qos_intf->dev,
				"failed to initialize subqueue %d\n", temp[i]);
			goto exit;
		}
	}

	return 0;
exit:
	devm_kfree(qos_priv->dev, qos_intf);
	return ret;
}

static int qos_release(void *intf_priv)
{
	struct qos_intf *qos_intf = intf_priv;

	devm_kfree(qos_intf->dev, qos_intf);
	return 0;
}

static int qos_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct qos_priv *qos_priv = inst_priv;

	devm_kfree(qos_priv->dev, qos_priv);
	return 0;
}

static int qos_probe(struct netcp_device *netcp_device, struct device *dev,
		     struct device_node *node, void **inst_priv)
{
	struct qos_priv *qos_priv;
	int ret = 0;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	qos_priv = devm_kzalloc(dev, sizeof(struct qos_priv), GFP_KERNEL);
	if (!qos_priv)
		return -ENOMEM;

	*inst_priv = qos_priv;

	qos_priv->netcp_device = netcp_device;
	qos_priv->dev = dev;

	ret = of_property_read_string(node, "tx-channel",
				      &qos_priv->dma_chan_name);
	if (ret < 0) {
		dev_err(dev, "missing \"tx-channel\" parameter\n");
		ret = -ENODEV;
		goto exit;
	}

	/* Create the per-instance sysfs entry */
	ret = kobject_init_and_add(&qos_priv->kobj, &qos_inst_ktype,
				   kobject_get(&dev->kobj), node->name);
	if (ret) {
		dev_err(dev, "failed to create %s sysfs entry\n",
			node->name);
		kobject_put(&qos_priv->kobj);
		kobject_put(&dev->kobj);
		goto exit;
	}

	return 0;

exit:
	qos_remove(netcp_device, qos_priv);
	*inst_priv = NULL;
	return ret;
}

static struct netcp_module qos_module = {
	.name		= QOS_MODULE_NAME,
	.owner		= THIS_MODULE,
	.probe		= qos_probe,
	.open		= qos_open,
	.close		= qos_close,
	.remove		= qos_remove,
	.attach		= qos_attach,
	.release	= qos_release,
};

static int __init keystone_qos_init(void)
{
	return netcp_register_module(&qos_module);
}
module_init(keystone_qos_init);

static void __exit keystone_qos_exit(void)
{
	netcp_unregister_module(&qos_module);
}
module_exit(keystone_qos_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Reece Pollack <reece@theptrgroup.com>");
MODULE_DESCRIPTION("Quality of Service driver for Keystone devices");
