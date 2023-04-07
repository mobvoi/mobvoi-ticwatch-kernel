#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include <linux/soc/qcom/slate_events_bridge_intf.h>
#include <asm/unistd.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/poll.h>

#define pr_fmt(msg) "sfb:" msg

#define SLATE_SLATE_FSTREAM_BRIDGE "slate_fstream_bridge"
#define SINGLE_PAYLOAD_SIZE   4096
#define SINGLE_HEAD_SIZE   16
#define SFB_TX_MEM_MALLOC_SIZE   (SINGLE_PAYLOAD_SIZE + SINGLE_HEAD_SIZE)
#define SFB_RX_MEM_MALLOC_SIZE   (SINGLE_PAYLOAD_SIZE + SINGLE_HEAD_SIZE)

static  struct cdev              sfb_cdev;
static  struct class             *sfb_class;
static  dev_t                    sfb_dev;
static atomic_t seb_fstream_rx_size;

struct sfb_prv {
	char *tx_kbuf;
	char *rx_kbuf;
	struct seb_notif_info		*seb_handle;
	struct notifier_block		seb_nb;
	bool	is_seb_up;

	wait_queue_head_t seb_rx_wq;
	bool	seb_rx_complete;

	int seb_rx_size;
	int status;
};

static struct sfb_prv sfb;

void enable_poll(void)
{
	pr_debug("%s\n", __func__);
	sfb.seb_rx_complete = true;
	wake_up_interruptible(&sfb.seb_rx_wq);
}

static int sfb_open(struct inode *inode, struct file *file)
{
	sfb.seb_rx_complete = false;
	atomic_set(&seb_fstream_rx_size, 0);

	pr_info("sfb_open\n");
	return 0;
}

int tx_paylod(char *buf, size_t size)
{
	char * pack_head;
	uint16_t cycle, rmd;
	int ret = 0;
	pr_debug("%s size:%d\n", __func__, size);

	ret = seb_send_event_to_slate(sfb.seb_handle, GMI_SLATE_EVENT_FSTREAM, buf, size);
	if (ret < 0) {
		pr_err("send failed\n");
		return -ECOMM;
	}

	return ret;
}

static ssize_t sfb_write(struct file *filp, const char __user *buf, size_t size, loff_t *lofp)
{
	char * psvm = sfb.tx_kbuf;
	int ret = 0;

	sfb.seb_rx_complete = false;

	pr_debug("sfb_write size:%u\n", size);

	if (size > SFB_TX_MEM_MALLOC_SIZE)
			return -EFBIG;

	if (raw_copy_from_user(psvm, buf, size))
		return -EFAULT;

	ret = tx_paylod(psvm, size);
	if(ret < 0)
		return ret;

	return size;
}

static int sfb_close(struct inode *inode, struct file *filp)
{
	pr_info("sfb_close\n");
	return 0;
}

static ssize_t sfb_read(struct file *, char __user * buf, size_t size, loff_t *lofp)
{

	int ret = 0;
	wait_event_interruptible(sfb.seb_rx_wq, sfb.seb_rx_complete);

	if(size < sfb.seb_rx_size)
		return 0;

	if (copy_to_user(buf, sfb.rx_kbuf, sfb.seb_rx_size))
		ret = -EFAULT;

	pr_debug("sfb_read ret:%d size:%d\n", ret, sfb.seb_rx_size);

	sfb.seb_rx_complete = false;

	return sfb.seb_rx_size;
}

static __poll_t sfb_poll(struct file *filp, struct poll_table_struct *wait)
{
	__poll_t mask = 0;

	poll_wait(filp, &sfb.seb_rx_wq, wait);
	if(sfb.seb_rx_complete)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations sfb_fops = {
	.owner          = THIS_MODULE,
	.open           = sfb_open,
	.write          = sfb_write,
	.read          	= sfb_read,
	.release        = sfb_close,
	.poll 			= sfb_poll,
};


void set_seb_fstream_rx_size(int len)
{
	atomic_set(&seb_fstream_rx_size, len);
}

static int seb_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	int len = 0;
    uint32_t p[4];

	pr_debug("%s: event:%d\n", __func__, event);

	if (event == GLINK_CHANNEL_STATE_UP) {
		sfb.is_seb_up = true;
		return 0;
	} else if (event != GMI_SLATE_EVENT_FSTREAM) {
		pr_info("SEB event is not for GMI_SLATE_EVENT_FSTREAM\n");
		return 0;
	}
	len = atomic_read(&seb_fstream_rx_size);
	atomic_set(&seb_fstream_rx_size, 0);

	if(len < SINGLE_HEAD_SIZE)
		return 0;

	memcpy(&p, data, SINGLE_HEAD_SIZE);
	pr_debug("opcode: %u addr:%u size:%u crc:%x", p[0], p[1], p[2], p[3]);
	pr_debug("%s: size:%d\n", __func__, len);

	memcpy(sfb.rx_kbuf, data, len);
	sfb.seb_rx_size = len;
	enable_poll();

	return 0;
}

static int __init slate_fstream_bridge_init(void)
{
    int ret;
	struct seb_notif_info *seb_handle;

	sfb.seb_nb.notifier_call = seb_notifier_cb;

	atomic_set(&seb_fstream_rx_size, 0);

	seb_handle = seb_register_for_slate_event(GMI_SLATE_EVENT_FSTREAM, &sfb.seb_nb);
	if (IS_ERR_OR_NULL(seb_handle)) {
		ret = seb_handle ? PTR_ERR(seb_handle) : -EINVAL;
		pr_err("Failed to register with Slate event bridge, rc=%d\n", ret);
		return ret;
	}

	sfb.seb_handle = seb_handle;
	sfb.is_seb_up = false;

	sfb.tx_kbuf = (char *)vmalloc(SFB_TX_MEM_MALLOC_SIZE);
	sfb.rx_kbuf = (char *)vmalloc(SFB_RX_MEM_MALLOC_SIZE);
	pr_info("entry slate_fstream_bridge_init\n");

	// init_waitqueue_head(&sfb.seb_wq);
	init_waitqueue_head(&sfb.seb_rx_wq);

	ret = alloc_chrdev_region(&sfb_dev, 0, 1, SLATE_SLATE_FSTREAM_BRIDGE);
	if (IS_ERR(ret)) {
		pr_err("failed with error %d\n", ret);
		return ret;
	}

	cdev_init(&sfb_cdev, &sfb_fops);

	ret = cdev_add(&sfb_cdev, sfb_dev, 1);
	if (IS_ERR(ret)) {
		unregister_chrdev_region(sfb_dev, 1);
		pr_err("device registration failed\n");
		return ret;
	}

	sfb_class = class_create(THIS_MODULE, SLATE_SLATE_FSTREAM_BRIDGE);
	if (IS_ERR_OR_NULL(sfb_class)) {
		cdev_del(&sfb_cdev);
		unregister_chrdev_region(sfb_dev, 1);
		pr_err("class creation failed\n");
		return PTR_ERR(sfb_class);
	}


	ret = device_create(sfb_class, NULL, sfb_dev, NULL, SLATE_SLATE_FSTREAM_BRIDGE);
	if (IS_ERR_OR_NULL(ret)) {
		class_destroy(sfb_class);
		cdev_del(&sfb_cdev);
		unregister_chrdev_region(sfb_dev, 1);
		pr_err("device create failed\n");
		return PTR_ERR(ret);
	}

    return 0;
}

static void __exit slate_fstream_bridge_exit (void)
{
	int ret;

	ret = seb_unregister_for_slate_event(sfb.seb_handle, &sfb.seb_nb);

	device_destroy(sfb_class, sfb_dev);
	class_destroy(sfb_class);
	cdev_del(&sfb_cdev);
	unregister_chrdev_region(sfb_dev, 1);

	pr_info("entry slate_fstream_bridge_exit\n");
	if (sfb.tx_kbuf != NULL)
		vfree(sfb.tx_kbuf);

	if (sfb.rx_kbuf != NULL)
		vfree(sfb.rx_kbuf);
}

module_init(slate_fstream_bridge_init);
module_exit(slate_fstream_bridge_exit);
MODULE_LICENSE("GPL v2");
