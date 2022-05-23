// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include "slate_mobvoi_rpc_rpmsg.h"

static struct slate_mobvoi_rpc_rpmsg_dev *pdev;

int slate_mobvoi_rpc_rpmsg_tx_msg(void  *msg, size_t len)
{
	int ret = 0;

	if (pdev == NULL || !pdev->chnl_state)
		pr_err("%s pmsg_device is null, channel is closed\n",__func__);

	pdev->message = msg;
	pdev->message_length = len;
	if (pdev->message) {
		ret = rpmsg_send(pdev->channel,
			pdev->message, pdev->message_length);
		if (ret)
			pr_err("%s rpmsg_send failed: %d\n",__func__, ret);

	}
	return ret;
}
EXPORT_SYMBOL(slate_mobvoi_rpc_rpmsg_tx_msg);

static int slate_mobvoi_rpc_rpmsg_probe(struct rpmsg_device  *rpdev)
{
	int ret = 0;
	void *msg = NULL;
	pr_err("slate_mobvoi_rpc_rpmsg_probe start\n");
	pdev = devm_kzalloc(&rpdev->dev, sizeof(*pdev), GFP_KERNEL);
	if (!pdev)
		return -ENOMEM;

	pdev->channel = rpdev->ept;
	pdev->dev = &rpdev->dev;
	if (pdev->channel == NULL)
		return -ENOMEM;

	pdev->chnl_state = true;
	dev_set_drvdata(&rpdev->dev, pdev);


	/* send a callback to slate-rsb driver*/
	slate_mobvoi_rpc_notify_glink_channel_state(true);
	if (pdev->message == NULL)
		ret = slate_mobvoi_rpc_rpmsg_tx_msg(msg, 0);

	pr_err("slate_mobvoi_rpc_rpmsg_probe end\n");
	return 0;
}

static void slate_mobvoi_rpc_rpmsg_remove(struct rpmsg_device *rpdev)
{
	pdev->chnl_state = false;
	pdev->message = NULL;
	dev_dbg(&rpdev->dev, "rpmsg client driver is removed\n");
	slate_mobvoi_rpc_notify_glink_channel_state(false);
	dev_set_drvdata(&rpdev->dev, NULL);
}

static int slate_mobvoi_rpc_rpmsg_cb(struct rpmsg_device *rpdev,
				void *data, int len, void *priv, u32 src)
{
	struct slate_mobvoi_rpc_rpmsg_dev *dev =
			dev_get_drvdata(&rpdev->dev);

	if (!dev)
		return -ENODEV;
	slate_mobvoi_rpc_rx_msg(data, len);
	return 0;
}

static const struct rpmsg_device_id rpmsg_driver_slatemobrpc_id_table[] = {
	{ "slate-mobvoi-rpc" },
	{},
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_slatemobrpc_id_table);

static const struct of_device_id rpmsg_driver_slatemobrpc_of_match[] = {
	{ .compatible = "qcom,mobvoi-rpc-rpmsg" },
	{},
};

static struct rpmsg_driver rpmsg_slate_mobvoi_rpc_client = {
	.id_table = rpmsg_driver_slatemobrpc_id_table,
	.probe = slate_mobvoi_rpc_rpmsg_probe,
	.callback = slate_mobvoi_rpc_rpmsg_cb,
	.remove = slate_mobvoi_rpc_rpmsg_remove,
	.drv = {
		.name = "qcom,slate_mobvoi_rpc_rpmsg",
		.of_match_table = rpmsg_driver_slatemobrpc_of_match,
	},
};
module_rpmsg_driver(rpmsg_slate_mobvoi_rpc_client);

MODULE_DESCRIPTION("Interface Driver for SLATE-MOBVOI and RPMSG");
MODULE_LICENSE("GPL v2");
