/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */
#ifndef SLATE_MOBVOI_RPC_H
#define SLATE_MOBVOI_RPC_H

#include <linux/soc/qcom/slatecom_intf.h>


void slate_mobvoi_rpc_notify_glink_channel_state(bool state);
void slate_mobvoi_rpc_rx_msg(void *data, int len);

/*
 * Message header type - generic header structure
 */
struct msg_header_t {
	uint32_t opcode;
	uint32_t payload_size;
};


#endif /* SLATE_MOBVOI_RPC_H */

