/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#ifndef SLATE_MOBVOI_RPC_INTF_H
#define SLATE_MOBVOI_RPC_INTF_H

#include <linux/notifier.h>

/**
 * Opcodes to be received on slate-control channel.
 */
enum WMSlateCtrlChnlOpcode {
	/*
	 * Command to slate to enter TWM mode
	 */
	GMI_MGR_TEST1 = 1,

	GMI_MGR_TEST2 = 2,
	GMI_SLATE_MOBVOI_RPC_PANNEL_POWER_STATE = 3,
	GMI_SLATE_MOBVOI_RPC_SET_ASSERT = 4,
};

int slate_mobvoi_rpc_tx_msg_ext(void  *msg, size_t len);



#endif /* SLATE_MOBVOI_RPC_INTF_H */
