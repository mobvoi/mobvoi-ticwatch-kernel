#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <soc/qcom/clock-local2.h>

struct mdss_blk_pwm_dev {
	struct clk *pclk;
	struct rcg_clk *gp_rcg_clk;
	struct pinctrl *pwm_pinctrl;
	struct pinctrl_state *pinctrl_state_default;

	u32 clk_base;
	u32 d_offset;
	u32 rcgr_offset;
	u32 cbcr_offset;

	u32 default_duty;
	u32 default_freq;
};

static struct mdss_blk_pwm_dev *g_pwm_dev;

#ifdef CONFIG_OF

static int blk_pwm_dev_parse_dt(struct device *dev)
{
	struct device_node *dt = dev->of_node;
	struct mdss_blk_pwm_dev *pwm_dev;
	int ret;

	if (!dt)
		return -ENODEV;

	pwm_dev = dev_get_drvdata(dev);

	pr_info("mdss-blk-pwm: parse dt\n");

	ret = of_property_read_u32(dt, "qcom,pwm-clk-base-offset",
		&pwm_dev->clk_base);
	if (ret < 0) {
		pr_err("mdss-blk-pwm: missing qcom,pwm-clk-base-offset in device tree\n");
		return -EFAULT;
	}

	ret = of_property_read_u32(dt, "qcom,pwm-clk-d-offset",
		&pwm_dev->d_offset);
	if (ret < 0) {
		pr_err("mdss-blk-pwm: missing qcom,pwm-clk-d-offset in device tree\n");
		return -EFAULT;
	}

	ret = of_property_read_u32(dt, "qcom,pwm-clk-rcgr-offset",
		&pwm_dev->rcgr_offset);
	if (ret < 0) {
		pr_err("mdss-blk-pwm: missing qcom,pwm-clk-rcgr-offset in device tree\n");
		return -EFAULT;
	}

	ret = of_property_read_u32(dt, "qcom,pwm-clk-cbcr-offset",
		&pwm_dev->cbcr_offset);
	if (ret < 0) {
		pr_err("mdss-blk-pwm: missing qcom,pwm-clk-cbcr-offset in device tree\n");
		return -EFAULT;
	}

	ret = of_property_read_u32(dt, "qcom,pwm-clk-default-freq",
		&pwm_dev->default_freq);
	if (ret < 0) {
		pr_err("mdss-blk-pwm: missing qcom,pwm-clk-default-freq in device tree\n");
		return -EFAULT;
	}

	ret = of_property_read_u32(dt, "qcom,pwm-clk-default-duty",
		&pwm_dev->default_duty);
	if (ret < 0) {
		pr_err("mdss-blk-pwm: missing qcom,pwm-clk-default-duty in device tree\n");
		return -EFAULT;
	}

	pr_info("mdss-blk-pwm: parse dt finished.\n");

	return 0;
}
#endif

void mdss_set_gpio_pwm(int level)
{
	struct mdss_blk_pwm_dev *pwm_dev;
	void *addr;

	if (!g_pwm_dev)
		return;

	pwm_dev = g_pwm_dev;

	if(level == 0){
		mb();
		/* The register Manual, representative of the clk register is enabled,
		 write 1 represents enable, write 0 indicates disable.
		 When the level value of 0 clk off.*/
		addr = *pwm_dev->gp_rcg_clk->base + pwm_dev->clk_base + pwm_dev->cbcr_offset;
		writel_relaxed(0x0, addr);

	}else{
		/* 128 is the value of N, if just output clock, please remove this line. */
		addr = *pwm_dev->gp_rcg_clk->base + pwm_dev->clk_base + pwm_dev->d_offset;
		writel_relaxed(((~(128 * level / 50)) & 0x0ff), addr);
		/* level here is the corresponding level, where only 1 to 100,
		so when we pass the value to be converted to 0-255 re-use.
		Also note that negated sign in front, this is a Qualcomm scheduled to die,
		we write the backlight level corresponding value is negated. */
		mb();
		addr = *pwm_dev->gp_rcg_clk->base + pwm_dev->clk_base + pwm_dev->rcgr_offset;
		writel_relaxed(0x3, addr); /* RCGR */
		mb();
		addr = *pwm_dev->gp_rcg_clk->base + pwm_dev->clk_base + pwm_dev->cbcr_offset;
		writel_relaxed(0x1, addr); /* CBCR */
	}
}

static int mdss_dsi_bl_probe(struct platform_device *pdev)
{
	int ret;
	struct mdss_blk_pwm_dev *pwm_dev;

	pr_info("mdss-blk-pwm: probe\n");
	if (!pdev || !pdev->dev.of_node) {
		pr_err("mdss-blk-pwm: %s: pdev not found for DSI controller\n",
			__func__);
		return -ENODEV;
	}

	pwm_dev = devm_kzalloc(&pdev->dev, sizeof(*pwm_dev), GFP_KERNEL);
	if (!pwm_dev)
		return -ENOMEM;

	g_pwm_dev = pwm_dev;
	dev_set_drvdata(&pdev->dev, pwm_dev);
	ret = blk_pwm_dev_parse_dt(&pdev->dev);
	if (ret) {
		pr_err("mdss-blk-pwm: failed to parse device tree.\n");
		goto failed_parse_dt;
	}

	pwm_dev->pclk = devm_clk_get(&pdev->dev, "gpio-pwm-clk"); /* get clock */
	if (!pwm_dev->pclk) {
		pr_err("mdss-blk-pwm: failed to get clk.");
		ret = -EFAULT;
		goto failed_parse_dt;
	}

	ret = clk_set_rate(pwm_dev->pclk, pwm_dev->default_freq); /* set the default freq */
	if (ret) {
		pr_err("mdss-blk-pwm: clk set rate %d fail, ret = %d\n",
			pwm_dev->default_freq, ret);
		goto failed_parse_dt;
	}

	ret = clk_prepare_enable(pwm_dev->pclk); /* enable clk */
	if (ret) {
		pr_err("mdss-blk-pwm: %s: clk_prepare error!!!\n", __func__);
		goto failed_parse_dt;
	}

	pr_info("mdss-blk-pwm: %s: clk_prepare success!\n", __func__);

	pwm_dev->gp_rcg_clk = to_rcg_clk(pwm_dev->pclk);

	/* Get pinctrl if target uses pinctrl */
	pwm_dev->pwm_pinctrl = devm_pinctrl_get(&(pdev->dev));
	if (IS_ERR_OR_NULL(pwm_dev->pwm_pinctrl)) {
		ret = PTR_ERR(pwm_dev->pwm_pinctrl);
		pr_err("mdss-blk-pwm: Target does not use pinctrl %d\n", ret);
		goto failed_parse_dt;
	}

	pwm_dev->pinctrl_state_default = pinctrl_lookup_state(pwm_dev->pwm_pinctrl, "default");
	if (IS_ERR_OR_NULL(pwm_dev->pinctrl_state_default)) {
		ret = PTR_ERR(pwm_dev->pinctrl_state_default);
		pr_err("mdss-blk-pwm: Can not lookup %s pinstate %d\n",
			   "default", ret);
		goto failed_parse_dt;
	}
	ret = pinctrl_select_state(pwm_dev->pwm_pinctrl, pwm_dev->pinctrl_state_default);

	mdss_set_gpio_pwm(pwm_dev->default_duty); /* set default level */

	pr_info("mdss-blk-pwm: probe successfull\n");
	return 0;

failed_parse_dt:
	if (pwm_dev) {
		devm_kfree(&pdev->dev, pwm_dev);
		pwm_dev = NULL;
	}
	g_pwm_dev = NULL;
	return ret;
}

static const struct of_device_id mdss_dsi_bl_dt_match[] = {
	{.compatible = "gpio-pwm-bkl"},
	{}
};
MODULE_DEVICE_TABLE(of, mdss_dsi_bl_dt_match);

static struct platform_driver mdss_dsi_bl_driver = {
	.probe = mdss_dsi_bl_probe,
	.shutdown = NULL,
	.driver = {
		.name = "gpio-pwm-bkl",
		.of_match_table = mdss_dsi_bl_dt_match,
	},
};

static int mdss_dsi_bl_register_driver(void)
{
	return platform_driver_register(&mdss_dsi_bl_driver);
}

static int __init mdss_dsi_bl_driver_init(void)
{
	int ret;

	pr_info("mdss-blk-pwm: init\n");
	ret = mdss_dsi_bl_register_driver();
	if (ret) {
		pr_err("mdss-blk-pwm: mdss_dsi_bl_register_driver() failed!\n");
		return ret;
	}

	return ret;
}
fs_initcall(mdss_dsi_bl_driver_init);
