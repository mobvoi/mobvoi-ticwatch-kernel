#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/regulator/machine.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/power_supply.h>

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <stdbool.h>
#include <linux/notifier.h>
#include <linux/fb.h>

extern int atmel_mxt_wakeup(bool enabled);
extern int zinitix_wakeup(bool enabled);

struct tp_node_ts_info {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct class *tp_class;
	struct device *tp_ts_dev;
	char phys[32];

	bool enable_wakeup;
};
struct tp_node_ts_info *info;

static ssize_t show_enable_wakeup(struct device *dev, struct device_attribute
			       *devattr, char *buf)
{
	//struct tp_node_ts_info *info = dev_get_drvdata(dev);
	int count;

	//count = sprintf(buf, "%d\n", info->enable_touch_wake);
	count = snprintf(buf, 4, "%d\n", info->enable_wakeup);

	return count;
}


static ssize_t store_enable_wakeup(struct device *dev, struct device_attribute
			 *devattr, const char *buf, size_t count)
{
	int ret;
	int i;
	//struct tp_node_ts_info *info = dev_get_drvdata(dev);

	if (kstrtoint(buf, 0, &ret))
		return -EINVAL;

	if (ret){
		info->enable_wakeup = true;
		printk("info->enable_touch_wake = %d\n", info->enable_wakeup);

#ifdef CONFIG_TOUCHSCREEN_ZINITIX
	i = zinitix_wakeup(true);
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL
	i = atmel_mxt_wakeup(true);
#endif

	}
	else {
		info->enable_wakeup = false;
		printk("info->enable_touch_wake = %d\n", info->enable_wakeup);

#ifdef CONFIG_TOUCHSCREEN_ZINITIX
	i = zinitix_wakeup(false);
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL
	i = atmel_mxt_wakeup(false);
#endif

	}
	return count;
}

static DEVICE_ATTR(enable_wakeup, 0664, show_enable_wakeup,
		   store_enable_wakeup);

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_enable_wakeup.attr,
	NULL,
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes,
};

static int tp_node_create_device_class(struct tp_node_ts_info *info)
{
	int ret;

	info->tp_class = class_create(THIS_MODULE, "tsp");
        info->enable_wakeup = true;
	info->tp_ts_dev = device_create(info->tp_class, NULL, 0, info, "tsp");
	if (unlikely(!info->tp_ts_dev)) {
		pr_err("Failed to create factory dev\n");
		ret = -ENODEV;
		goto err_class;
	}

	ret = sysfs_create_group(&info->tp_ts_dev->kobj, &touchscreen_attr_group);
	if (unlikely(ret)) {
		pr_err("Failed to create touchscreen sysfs group\n");
		goto err_device;
	}


	return ret;

err_device:
	device_destroy(info->tp_class,0);
err_class:
	class_destroy(info->tp_class);

	return ret;
}

#if 0
static int tp_node_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	//struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tp_node_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;

	//struct device_node *np = client->dev.of_node;

	pr_info("probe----\n");
	info = kzalloc(sizeof(struct tp_node_ts_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		//goto err_mem_alloc;
	}
	i2c_set_clientdata(client, info);
	info->client = client;

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_alloc2;
	}

	snprintf(info->phys, sizeof(info->phys), "%s/input0",
		 dev_name(&client->dev));
	input_dev->name = "tp_node";
	//input_dev->id.bustype = BUS_I2C;
	input_dev->phys = info->phys;
	input_dev->dev.parent = &client->dev;

	info->input_dev = input_dev;

	ret = input_register_device(info->input_dev);
	if (ret) {
		pr_err("unable to register %s input device\n",
			info->input_dev->name);
		input_free_device(info->input_dev);

		return ret;
	}

	ret = tp_node_create_device_class(info);
	if (ret) {
		pr_err("Failed to init tp_node_create_device_class device\n");

		goto err_alloc1;
	}

	return ret;

err_alloc1:
	input_free_device(info->input_dev);
err_alloc2:
	kfree(info);

	return ret;
}

static int tp_node_remove(struct i2c_client *client)
{
	struct tp_node_ts_info *info = i2c_get_clientdata(client);
	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	kfree(info);
	return 0;
}

static const struct i2c_device_id tp_node_id[] = {
	{ "tps-node", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tp_node_id);

#if 0
static struct of_device_id tp_node_match_table[] = {
	{ .compatible = "atmel,tp-node",},
	{ },
};
#endif
#define tp_node_match_table NULL

static struct i2c_driver tp_node_driver = {
	.driver = {
		.name	= "tps-node",
		.owner	= THIS_MODULE,
		.of_match_table = tp_node_match_table,
	},
	.probe		= tp_node_probe,
	.remove		= tp_node_remove,
	.id_table	= tp_node_id,
};
#endif
static int __init tp_node_init(void)
{
	int ret;
	pr_info("atmel_zinitix_node: start mxt module.\n");

	info = kzalloc(sizeof(struct tp_node_ts_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
	}
	ret = tp_node_create_device_class(info);
	if(ret)
		kfree(info);

	return ret;
	//return i2c_add_driver(&tp_node_driver);
}

static void __exit tp_node_exit(void)
{
	//i2c_del_driver(&tp_node_driver);
	device_destroy(info->tp_class,0);
	class_destroy(info->tp_class);
	kfree(info);
}

late_initcall(tp_node_init);
module_exit(tp_node_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("tps wake up node");
MODULE_LICENSE("GPL");
