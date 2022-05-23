/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#ifndef SLATE_MOBVOI_RPC_RPMSG_H
#define SLATE_MOBVOI_RPC_RPMSG_H

#include <linux/rpmsg.h>
#include "slate_mobvoi_rpc.h"

#define TIMEOUT_MS 5000

struct slate_mobvoi_rpc_rpmsg_dev {
	struct rpmsg_endpoint *channel;
	struct device *dev;
	bool chnl_state;
	void *message;
	size_t message_length;
};


int slate_mobvoi_rpc_rpmsg_tx_msg(void  *msg, size_t len);

#endif
