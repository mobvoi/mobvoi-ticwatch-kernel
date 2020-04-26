// SPDX-License-Identifier: GPL-2.0+
/*
 * gpio driver for the vibrator.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>

#define LOG_TAG "[GPIO_VIBRA]"

struct gpio_vibra_pwm {
	struct device *dev;
	struct timed_output_dev to_dev;
	struct hrtimer timer;
	struct mutex lock;
	struct work_struct vibrator_work;
	struct wake_lock wklock;
	int gpio_enable;
	int gpio_trigger;
	bool on;
};


static void vibrator_start(struct gpio_vibra_pwm *gpio_vibra)
{
	gpio_vibra->on = true;
	gpio_direction_output(gpio_vibra->gpio_enable, 1);
	gpio_direction_output(gpio_vibra->gpio_trigger, 1);

}

static void vibrator_stop(struct gpio_vibra_pwm *gpio_vibra)
{
	gpio_vibra->on = false;
	gpio_direction_output(gpio_vibra->gpio_enable, 0);
	gpio_direction_output(gpio_vibra->gpio_trigger, 0);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct gpio_vibra_pwm *gpio_vibra =
			container_of(timer, struct gpio_vibra_pwm, timer);

	schedule_work(&gpio_vibra->vibrator_work);

	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct gpio_vibra_pwm *gpio_vibra =
			container_of(work, struct gpio_vibra_pwm,
					vibrator_work);

	mutex_lock(&gpio_vibra->lock);

	if (!gpio_vibra->on)
		vibrator_start(gpio_vibra);
	else {
		vibrator_stop(gpio_vibra);
		wake_unlock(&gpio_vibra->wklock);
	}

	mutex_unlock(&gpio_vibra->lock);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct gpio_vibra_pwm *gpio_vibra =
			container_of(dev, struct gpio_vibra_pwm, to_dev);

	if (hrtimer_active(&gpio_vibra->timer)) {
		ktime_t r = hrtimer_get_remaining(&gpio_vibra->timer);

		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct gpio_vibra_pwm *gpio_vibra =
			container_of(dev, struct gpio_vibra_pwm, to_dev);

	hrtimer_cancel(&gpio_vibra->timer);
	cancel_work_sync(&gpio_vibra->vibrator_work);

	mutex_lock(&gpio_vibra->lock);

	if (value < 0) {
		pr_err(LOG_TAG"Error enable value: %d.\n", value);
		return;
	}
	vibrator_stop(gpio_vibra);

	if (value > 0) {
		wake_lock(&gpio_vibra->wklock);
		vibrator_start(gpio_vibra);
		pr_info(LOG_TAG"vibrator_enable hr timer start.\n");
		hrtimer_start(&gpio_vibra->timer,
				ns_to_ktime((u64)value * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
	}
	mutex_unlock(&gpio_vibra->lock);
}

#ifdef CONFIG_OF
static int gpio_vibra_pwm_parse_dt(struct gpio_vibra_pwm *data)
{
	struct device_node *dt = data->dev->of_node;
	int ret;

	if (!dt)
		return -ENODEV;

	ret = data->gpio_enable =
		of_get_named_gpio(dt, "gpio_vibra,en_gpio", 0);
	if (ret < 0) {
		pr_err(LOG_TAG"missing gpio_vibra,en_gpio in device tree\n");
		data->gpio_enable = 0;
	}

	ret = data->gpio_trigger =
		of_get_named_gpio(dt, "gpio_vibra,trigger_gpio", 0);
	if (ret < 0) {
		pr_err(LOG_TAG"missing gpio_vibra,trigger_gpio in device tree\n");
		data->gpio_trigger = 0;
	}
	return 0;
}
#endif

static int gpio_vibra_pwm_probe(struct platform_device *pdev)
{
	struct gpio_vibra_pwm *gpio_vibra;
	int err = 0;

	gpio_vibra = devm_kzalloc(&pdev->dev, sizeof(*gpio_vibra), GFP_KERNEL);
	if (!gpio_vibra)
		return -ENOMEM;

	gpio_vibra->dev = &pdev->dev;

#ifdef CONFIG_OF
	gpio_vibra_pwm_parse_dt(gpio_vibra);
#endif
	if (gpio_vibra->gpio_enable) {
		err = gpio_request(
			gpio_vibra->gpio_enable,
			"gpio_vibra Enable");
		if (err < 0) {
			pr_err(LOG_TAG"%s: GPIO request Enable error\n",
				__func__);
			goto exit_gpio_request_failed;
		}
		gpio_direction_output(gpio_vibra->gpio_enable, 0);
	}

	if (gpio_vibra->gpio_trigger) {
		err = gpio_request(
			gpio_vibra->gpio_trigger,
			"gpio_vibra Trigger");
		if (err < 0) {
			pr_err(LOG_TAG"%s: GPIO request Trigger error\n",
				__func__);
			goto exit_gpio_request_failed;
		}
		gpio_direction_output(gpio_vibra->gpio_trigger, 0);
	}
	platform_set_drvdata(pdev, gpio_vibra);

	gpio_vibra->to_dev.name = "vibrator";
	gpio_vibra->to_dev.get_time = vibrator_get_time;
	gpio_vibra->to_dev.enable = vibrator_enable;
	if (timed_output_dev_register(&(gpio_vibra->to_dev)) < 0) {
		pr_err(LOG_TAG"fail to create timed output dev\n");
		goto exit_timed_output_dev_failed;
	}

	hrtimer_init(&gpio_vibra->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gpio_vibra->timer.function = vibrator_timer_func;
	INIT_WORK(&gpio_vibra->vibrator_work, vibrator_work_routine);

	wake_lock_init(&gpio_vibra->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&gpio_vibra->lock);

	return 0;

exit_timed_output_dev_failed:
exit_gpio_request_failed:
	if (gpio_vibra->gpio_trigger)
		gpio_free(gpio_vibra->gpio_trigger);

	if (gpio_vibra->gpio_enable)
		gpio_free(gpio_vibra->gpio_enable);

	return err;
}

static __maybe_unused int gpio_vibra_pwm_suspend(struct device *dev)
{
	struct gpio_vibra_pwm *gpio_vibra = dev_get_drvdata(dev);

	hrtimer_cancel(&gpio_vibra->timer);
	cancel_work_sync(&gpio_vibra->vibrator_work);

	mutex_lock(&gpio_vibra->lock);

	vibrator_stop(gpio_vibra);
	wake_unlock(&gpio_vibra->wklock);
	mutex_unlock(&gpio_vibra->lock);
	return 0;
}

static __maybe_unused int gpio_vibra_pwm_resume(struct device *dev)
{
	struct gpio_vibra_pwm *gpio_vibra = dev_get_drvdata(dev);

	mutex_lock(&gpio_vibra->lock);
	mutex_unlock(&gpio_vibra->lock);
	return 0;
}

static int gpio_vibra_pwm_remove(struct platform_device *pdev)
{
	struct gpio_vibra_pwm *gpio_vibra = platform_get_drvdata(pdev);

	if (gpio_vibra->gpio_trigger)
		gpio_free(gpio_vibra->gpio_trigger);

	if (gpio_vibra->gpio_enable)
		gpio_free(gpio_vibra->gpio_enable);

	devm_kfree(gpio_vibra->dev, gpio_vibra);
	return 0;
}

static const struct of_device_id gpio_vibra_pwm_of_match[] = {
	{ .compatible = "qcom,gpio-pwm-vibrator" },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_vibra_pwm_of_match);

static const struct dev_pm_ops gpio_vibra_pwm_pm_ops = {
	.suspend = gpio_vibra_pwm_suspend,
	.resume  = gpio_vibra_pwm_resume,
};

static struct platform_driver gpio_vibra_pwm_driver = {
	.driver = {
		.name = "gpio-msm-vibrator",
		.of_match_table = gpio_vibra_pwm_of_match,
		.pm = &gpio_vibra_pwm_pm_ops,
	},
	.probe = gpio_vibra_pwm_probe,
	.remove = gpio_vibra_pwm_remove,
};
module_platform_driver(gpio_vibra_pwm_driver);

MODULE_AUTHOR("Fanming Kong <fmkong@mobovi.com>");
MODULE_DESCRIPTION("GPIO driver for the vibrator.");
MODULE_LICENSE("GPL");