// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#define pr_fmt(msg) "slate_mobvoi_rpc:" msg

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include "linux/slatecom_interface.h"
#include "slate_mobvoi_rpc.h"
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/soc/qcom/slate_mobvoi_rpc_intf.h>


#include "peripheral-loader.h"
#include "../../misc/qseecom_kernel.h"
#include "slate_mobvoi_rpc_rpmsg.h"

#define TIMEOUT_MS 2000
#define TIMEOUT_MS_GLINK_OPEN 10000

#define MOBVOI_SLATE_RPC_IOCTL_CMD_PASSTHROUGH 0x80551001
typedef struct
{
    u32          opcode;
    u32          payload_size;
} wear_header_t;
typedef struct
{
    wear_header_t   header;
	unsigned char buff[128];
} wear_slate_mobvoi_rpc_packet_t;


#define SLATE_MOBVOI_RPC "slate_mobvoi_rpc"

#define MAX_APP_NAME_SIZE 100

/*pil_slate_intf.h*/
#define RESULT_SUCCESS 0
#define RESULT_FAILURE -1

#define BUF_SIZE 10

static void ssr_register(void);

wear_slate_mobvoi_rpc_packet_t rpc_data={0};
wear_slate_mobvoi_rpc_packet_t *p_rpc_data=&rpc_data;

enum slate_mobvoi_rpc_state {
	SLATE_MOBVOI_RPC_STATE_UNKNOWN,
	SLATE_MOBVOI_RPC_STATE_INIT,
	SLATE_MOBVOI_RPC_STATE_GLINK_OPEN,
	SLATE_MOBVOI_RPC_STATE_SLATE_SSR
};

struct slatemobrpc_priv {
	void *pil_h;
	void *slate_subsys_handle;
	int app_status;
	unsigned long attrs;
	u32 cmd_status;
	struct device *platform_dev;
	bool slate_mobvoi_rpc_rpmsg;
	bool slate_resp_cmplt;
	void *lhndl;
	wait_queue_head_t link_state_wait;
	char rx_buf[20];
	struct work_struct slate_up_work;
	struct work_struct slate_down_work;
	struct mutex glink_mutex;
	struct mutex slate_mobvoi_rpc_state_mutex;
	enum slate_mobvoi_rpc_state slate_mobvoi_rpc_current_state;
	struct workqueue_struct *slatemobrpc_wq;
	struct wakeup_source *slatemobrpc_ws;
};

static struct slatemobrpc_priv *dev;

static void *slate_mobvoi_rpc_drv;

struct slate_event {
	enum slate_event_type e_type;
};



static  DEFINE_MUTEX(slate_char_mutex);
static  struct cdev              slate_cdev;
static  struct class             *slate_class;
struct  device                   *mob_dev_ret;
static  dev_t                    slate_dev;


void slate_mobvoi_rpc_notify_glink_channel_state(bool state)
{
	struct slatemobrpc_priv *dev =
		container_of(slate_mobvoi_rpc_drv, struct slatemobrpc_priv, lhndl);

	pr_debug("%s: slate_ctrl channel state: %d\n", __func__, state);
	dev->slate_mobvoi_rpc_rpmsg = state;
}
EXPORT_SYMBOL(slate_mobvoi_rpc_notify_glink_channel_state);

void slate_mobvoi_rpc_rx_msg(void *data, int len)
{
	struct slatemobrpc_priv *dev =
		container_of(slate_mobvoi_rpc_drv, struct slatemobrpc_priv, lhndl);

	dev->slate_resp_cmplt = true;
	wake_up(&dev->link_state_wait);
	memcpy(dev->rx_buf, data, len);
}
EXPORT_SYMBOL(slate_mobvoi_rpc_rx_msg);

static int slate_mobvoi_rpc_tx_msg(struct slatemobrpc_priv *dev, void  *msg, size_t len)
{
	int rc = 0;
	uint8_t resp = 0;

	mutex_lock(&dev->glink_mutex);
	__pm_stay_awake(dev->slatemobrpc_ws);
	pr_info("rpc tx msg,wakeup hold");
	
	if (!dev->slate_mobvoi_rpc_rpmsg) {
		pr_err("slatemobrpc-rpmsg is not probed yet, waiting for it to be probed\n");
		rc = -1;
		goto err_ret;
	}
	rc = slate_mobvoi_rpc_rpmsg_tx_msg(msg, len);

	/* wait for sending command to SLATE */
	rc = wait_event_timeout(dev->link_state_wait,
			(rc == 0), msecs_to_jiffies(TIMEOUT_MS));
	if (rc == 0) {
		pr_err("failed to send command to SLATE %d\n", rc);
		rc = -1;
		goto err_ret;
	}

	/* wait for getting response from SLATE */
	rc = wait_event_timeout(dev->link_state_wait,
			dev->slate_resp_cmplt,
				 msecs_to_jiffies(TIMEOUT_MS));
	if (rc == 0) {
		pr_err("failed to get SLATE response %d\n", rc);
		rc = -1;
		goto err_ret;
	}
	dev->slate_resp_cmplt = false;
	/* check SLATE response */
	resp = *(uint8_t *)dev->rx_buf;
	if (resp == 0x01) {
		pr_err("Bad SLATE response\n");
		rc = -EINVAL;
		goto err_ret;
	}
	rc = 0;

err_ret:
	__pm_relax(dev->slatemobrpc_ws);
	pr_info("rpc tx msg,wakeup release");
	mutex_unlock(&dev->glink_mutex);
	return rc;
}
extern int slatecom_is_spi_active_ext(void);

int slate_mobvoi_rpc_tx_msg_ext(void  *msg, size_t len)
{
    int ret=0;
	if(!slatecom_is_spi_active_ext())
	{
		pr_err("spi is inactive,slate_mobvoi_rpc_tx_msg_ext warning\n");
		//return -1;
	}

	struct slatemobrpc_priv *dev =
        container_of(slate_mobvoi_rpc_drv, struct slatemobrpc_priv, lhndl);

	if(!dev)
	{
		return -1;
	}	
    ret=slate_mobvoi_rpc_tx_msg(dev,msg,len);
	return ret;
}
EXPORT_SYMBOL(slate_mobvoi_rpc_tx_msg_ext);


void slate_restart_subsys(void)
{

	wear_header_t req_header;
	char  tx_buf[32];
	int i=0;
	unsigned int tx_buf_size;
	unsigned char data=1;
	tx_buf_size = sizeof(req_header) + 4;
	if(tx_buf_size>32)
	{
		return;
	}
	req_header.opcode = GMI_SLATE_MOBVOI_RPC_SET_ASSERT;//0x4;
	req_header.payload_size = sizeof(data);

	memcpy(tx_buf, &req_header, sizeof(req_header));
	memcpy(tx_buf+sizeof(req_header), &data, sizeof(data));
	for(i=0; i< req_header.payload_size + sizeof(req_header);i++)
	{
		pr_err("slate_restart_subsys tx_buf[%d]=%d\n",i, tx_buf[i]);
	}

	printk("slate_restart_subsys \n");

	slate_mobvoi_rpc_tx_msg_ext(tx_buf,req_header.payload_size + sizeof(req_header));

}
EXPORT_SYMBOL(slate_restart_subsys);

static int slate_mobvoi_rpc_char_open(struct inode *inode, struct file *file)
{
	printk("slate_mobvoi_rpc_char_open\n");
	return 0;
}



static long slate_mobvoi_rpc_ioctl(struct file *filp,
		unsigned int ui_slatecom_cmd, unsigned long arg)
{
	int ret;
	int rs=-1;
	struct slatemobrpc_priv *dev = container_of(slate_mobvoi_rpc_drv,
					struct slatemobrpc_priv,
					lhndl);
	if (filp == NULL)
		return -EINVAL;
	pr_info("slate_mobvoi_rpc_ioctl cmd=0x%x,arg=%lu\n",ui_slatecom_cmd,arg);
	switch (ui_slatecom_cmd) {
		case MOBVOI_SLATE_RPC_IOCTL_CMD_PASSTHROUGH:
		{
			copy_from_user((unsigned char*)p_rpc_data,(unsigned char*)arg,sizeof(wear_slate_mobvoi_rpc_packet_t));
			pr_info("p_rpc_data: opcode=%d,payload_size=%d\n",p_rpc_data->header.opcode,p_rpc_data->header.payload_size);
			if(p_rpc_data->header.payload_size>128){
				return -1;
			}
			rs=slate_mobvoi_rpc_tx_msg_ext(p_rpc_data,p_rpc_data->header.payload_size+sizeof(wear_header_t));
			pr_info("slate_mobvoi_rpc_ioctl rs=%d\n",rs);
			ret=rs;
			break;
		}
		default:
			ret = -ENOIOCTLCMD;
			break;
	}
	return ret;
}

static ssize_t slate_mobvoi_rpc_char_write(struct file *f, const char __user *buf,
				size_t count, loff_t *off)
{
	unsigned char qcli_cmnd;

	struct slatemobrpc_priv *dev = container_of(slate_mobvoi_rpc_drv,
					struct slatemobrpc_priv,
					lhndl);

	if (copy_from_user(&qcli_cmnd, buf, sizeof(unsigned char)))
		return -EFAULT;

	pr_info("%s: command arg = %c\n", __func__, qcli_cmnd);

	switch (qcli_cmnd) {

	default:
		pr_err("MSM QCLI Invalid Option\n");
		break;
	}

	*off += count;
	return count;
}

static int slate_mobvoi_rpc_char_close(struct inode *inode, struct file *file)
{
	printk("slate_mobvoi_rpc_char_close\n");
	return 0;
}

static void slate_mobvoi_rpc_slateup_work(struct work_struct *work)
{
	int ret = 0;
	struct slatemobrpc_priv *dev =
			container_of(work, struct slatemobrpc_priv, slate_up_work);

	mutex_lock(&dev->slate_mobvoi_rpc_state_mutex);
	if (!dev->slate_mobvoi_rpc_rpmsg)
		pr_err("slate_mobvoi_rpc_rpmsg is not probed yet\n");
	ret = wait_event_timeout(dev->link_state_wait,
				dev->slate_mobvoi_rpc_rpmsg, msecs_to_jiffies(TIMEOUT_MS_GLINK_OPEN));
	if (ret == 0) {
		pr_err("channel connection time out %d\n", ret);
		goto glink_err;
	}
	dev->slate_mobvoi_rpc_current_state = SLATE_MOBVOI_RPC_STATE_GLINK_OPEN;
	goto unlock;

glink_err:
	dev->slate_mobvoi_rpc_current_state = SLATE_MOBVOI_RPC_STATE_INIT;
unlock:
	mutex_unlock(&dev->slate_mobvoi_rpc_state_mutex);
}


static void slate_mobvoi_rpc_slatedown_work(struct work_struct *work)
{
	struct slatemobrpc_priv *dev = container_of(work, struct slatemobrpc_priv,
								slate_down_work);

	mutex_lock(&dev->slate_mobvoi_rpc_state_mutex);

	pr_info("Slatemobrpc current state is : %d\n", dev->slate_mobvoi_rpc_current_state);

	dev->slate_mobvoi_rpc_current_state = SLATE_MOBVOI_RPC_STATE_SLATE_SSR;

	mutex_unlock(&dev->slate_mobvoi_rpc_state_mutex);
}

static int slate_mobvoi_rpc_rpmsg_init(struct slatemobrpc_priv *dev)
{
	slate_mobvoi_rpc_drv = &dev->lhndl;
	mutex_init(&dev->glink_mutex);
	mutex_init(&dev->slate_mobvoi_rpc_state_mutex);

	dev->slatemobrpc_wq =
		create_singlethread_workqueue("slatemobrpc-work-queue");
	if (!dev->slatemobrpc_wq) {
		pr_err("Failed to init Slatemobrpc work-queue\n");
		return -ENOMEM;
	}

	init_waitqueue_head(&dev->link_state_wait);

	/* set default slatecom state */
	dev->slate_mobvoi_rpc_current_state = SLATE_MOBVOI_RPC_STATE_INIT;

	/* Init all works */
	INIT_WORK(&dev->slate_up_work, slate_mobvoi_rpc_slateup_work);
	INIT_WORK(&dev->slate_down_work, slate_mobvoi_rpc_slatedown_work);

	return 0;
}

static int slate_mobvoi_rpc_probe(struct platform_device *pdev)
{
	struct device_node *node;
	int rc = 0;
	node = pdev->dev.of_node;

	dev = kzalloc(sizeof(struct slatemobrpc_priv), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	/* Add wake lock for PM suspend */
	dev->slatemobrpc_ws = wakeup_source_register(&pdev->dev, "mob_rpc_wake_lock");
	dev->slate_mobvoi_rpc_current_state = SLATE_MOBVOI_RPC_STATE_UNKNOWN;
	rc = slate_mobvoi_rpc_rpmsg_init(dev);
	if (rc)
		return -ENODEV;
	dev->platform_dev = &pdev->dev;
	pr_info("%s success\n", __func__);

	ssr_register();
	return 0;
}

static const struct of_device_id slate_mobvoi_rpc_of_match[] = {
	{ .compatible = "qcom,slate-mobvoirpc", },
	{ }
};
MODULE_DEVICE_TABLE(of, slate_mobvoi_rpc_of_match);

static struct platform_driver slate_mobvoi_rpc_driver = {
	.probe  = slate_mobvoi_rpc_probe,
	.driver = {
		.name = "slate-mobvoirpc",
		.of_match_table = slate_mobvoi_rpc_of_match,
	},
};

static const struct file_operations fops = {
	.owner          = THIS_MODULE,
	.open           = slate_mobvoi_rpc_char_open,
	.write          = slate_mobvoi_rpc_char_write,
	.release        = slate_mobvoi_rpc_char_close,
	.unlocked_ioctl = slate_mobvoi_rpc_ioctl,
};

static int ssr_slate_mobvoi_rpc_cb(struct notifier_block *this,
		unsigned long opcode, void *data)
{
	struct slatemobrpc_priv *dev = container_of(slate_mobvoi_rpc_drv,
				struct slatemobrpc_priv, lhndl);

	switch (opcode) {
	case SUBSYS_BEFORE_SHUTDOWN:
		queue_work(dev->slatemobrpc_wq, &dev->slate_down_work);
		break;
	case SUBSYS_AFTER_POWERUP:
		if (dev->slate_mobvoi_rpc_current_state == SLATE_MOBVOI_RPC_STATE_INIT ||
			dev->slate_mobvoi_rpc_current_state == SLATE_MOBVOI_RPC_STATE_SLATE_SSR)
			queue_work(dev->slatemobrpc_wq, &dev->slate_up_work);
		break;
	}
	return NOTIFY_DONE;
}


static struct notifier_block ssr_slate_nb = {
	.notifier_call = ssr_slate_mobvoi_rpc_cb,
	.priority = 0,
};



/**
 * ssr_register checks that domain id should be in range and register
 * SSR framework for value at domain id.
 */
static void ssr_register(void)
{
	struct notifier_block *nb;

	nb = &ssr_slate_nb;
	dev->slate_subsys_handle =
			subsys_notif_register_notifier("slatefw", nb);

	if (!dev->slate_subsys_handle) {
		dev->slate_subsys_handle = NULL;
	
	}
	
}


static ssize_t show_slate_asset(struct device *dev, struct device_attribute
			       *devattr, char *buf)
{
	int ret = 0;

	return ret;
}


static ssize_t set_slate_asset(struct device *dev, struct device_attribute
			 *devattr, const char *buf, size_t count)
{
	int rc,i;
	int data;
	wear_header_t req_header;
	char  *tx_buf;
	unsigned char  tn_enable = 0;
	unsigned int tx_buf_size;

	struct slatemobrpc_priv *pdev = container_of(slate_mobvoi_rpc_drv,
					struct slatemobrpc_priv,lhndl);

	tx_buf_size = sizeof(req_header) + 4;

	tx_buf = kzalloc(tx_buf_size, GFP_KERNEL);
	if (!tx_buf) {
		return -ENOMEM;
	}

	rc = kstrtoint(buf, 10, &data);
	if (rc) {
		pr_err("kstrtoint failed, rc = %d\n", rc);
		return -EINVAL;
	}
    tn_enable = data ? 1 : 0;

	req_header.opcode = GMI_SLATE_MOBVOI_RPC_SET_ASSERT;//0x4;
	req_header.payload_size = sizeof(tn_enable);

	memcpy(tx_buf, &req_header, sizeof(req_header));
	memcpy(tx_buf+sizeof(req_header), &tn_enable, sizeof(tn_enable));
	for(i=0; i< req_header.payload_size + sizeof(req_header);i++)
		pr_err("tx_buf[%d]=%d\n",i, tx_buf[i]);

	slate_mobvoi_rpc_tx_msg_ext(tx_buf,req_header.payload_size + sizeof(req_header));
	return count;
}

static DEVICE_ATTR(slate_asset, 0664, show_slate_asset,set_slate_asset);


static ssize_t show_tn_enable(struct device *dev, struct device_attribute
			       *devattr, char *buf)
{
	int ret = 0;

	return ret;
}


static ssize_t store_tn_enable(struct device *dev, struct device_attribute
			 *devattr, const char *buf, size_t count)
{
	int rc,i;
	int data;
	wear_header_t req_header;
	char  *tx_buf;
	unsigned char  tn_enable = 0;
	unsigned int tx_buf_size;

	struct slatemobrpc_priv *pdev = container_of(slate_mobvoi_rpc_drv,
					struct slatemobrpc_priv,lhndl);

	tx_buf_size = sizeof(req_header) + 4;

	tx_buf = kzalloc(tx_buf_size, GFP_KERNEL);
	if (!tx_buf) {
		return -ENOMEM;
	}

	rc = kstrtoint(buf, 10, &data);
	if (rc) {
		pr_err("kstrtoint failed, rc = %d\n", rc);
		return -EINVAL;
	}
    tn_enable = data ? 1 : 0;

	req_header.opcode = GMI_SLATE_MOBVOI_RPC_PANNEL_POWER_STATE;//0x3;
	req_header.payload_size = sizeof(tn_enable);

	memcpy(tx_buf, &req_header, sizeof(req_header));
	memcpy(tx_buf+sizeof(req_header), &tn_enable, sizeof(tn_enable));
	for(i=0; i< req_header.payload_size + sizeof(req_header);i++)
		pr_err("tx_buf[%d]=%d\n",i, tx_buf[i]);

	slate_mobvoi_rpc_tx_msg_ext(tx_buf,req_header.payload_size + sizeof(req_header));
	return count;
}

static DEVICE_ATTR(tn_enable, 0664, show_tn_enable,store_tn_enable);

static struct attribute *mobrpc_attributes[] = {
	&dev_attr_tn_enable.attr,
	&dev_attr_slate_asset.attr,
	NULL,
};

static struct attribute_group mobrpc_attr_group = {
	.attrs = mobrpc_attributes,
};
static int __init init_slate_mobvoi_rpc_dev(void)
{
	int ret;

	ret = alloc_chrdev_region(&slate_dev, 0, 1, SLATE_MOBVOI_RPC);
	if (ret  < 0) {
		pr_err("failed with error %d\n", ret);
		return ret;
	}
	cdev_init(&slate_cdev, &fops);

	ret = cdev_add(&slate_cdev, slate_dev, 1);
	if (ret < 0) {
		unregister_chrdev_region(slate_dev, 1);
		pr_err("device registration failed\n");
		return ret;
	}
	slate_class = class_create(THIS_MODULE, SLATE_MOBVOI_RPC);
	if (IS_ERR_OR_NULL(slate_class)) {
		cdev_del(&slate_cdev);
		unregister_chrdev_region(slate_dev, 1);
		pr_err("class creation failed\n");
		return PTR_ERR(slate_class);
	}

	mob_dev_ret = device_create(slate_class, NULL, slate_dev, NULL, SLATE_MOBVOI_RPC);
	if (IS_ERR_OR_NULL(mob_dev_ret)) {
		class_destroy(slate_class);
		cdev_del(&slate_cdev);
		unregister_chrdev_region(slate_dev, 1);
		pr_err("device create failed\n");
		return PTR_ERR(mob_dev_ret);
	}
	ret = sysfs_create_group(&mob_dev_ret->kobj, &mobrpc_attr_group);
	if (unlikely(ret)) {
		pr_err("failed to create slate mobvoi rpc  sysfs group\n");
	}

	if (platform_driver_register(&slate_mobvoi_rpc_driver))
		pr_err("%s: failed to register slate_mobvoi_rpc_driver register\n", __func__);

	return 0;
}

static void __exit exit_slate_mobvoi_rpc_dev(void)
{
	device_destroy(slate_class, slate_dev);
	class_destroy(slate_class);
	cdev_del(&slate_cdev);
	unregister_chrdev_region(slate_dev, 1);
	platform_driver_unregister(&slate_mobvoi_rpc_driver);
}

module_init(init_slate_mobvoi_rpc_dev);
module_exit(exit_slate_mobvoi_rpc_dev);
MODULE_LICENSE("GPL v2");
