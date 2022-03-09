/*
 * TI ADC IIO driver
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>

#include <linux/iio/adc/ti_am437x_adc1.h>

static const struct regmap_config ti_adc_regmap_config = {
	.name = "ti_adc1",
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
};

static unsigned int tiadc_readl(struct ti_adc1_dev *adc, unsigned int reg)
{
	return readl(adc->adc_base + reg);
}

static void tiadc_writel(struct ti_adc1_dev *adc, unsigned int reg,
					unsigned int val)
{
	writel(val, adc->adc_base + reg);
}

static void am437x_adc_se_update(struct ti_adc1_dev *adc, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&adc->reg_lock, flags);
	if (!adc->adc_pending)
		tiadc_writel(adc, REG_SE, adc->reg_se_cache | val);
	spin_unlock_irqrestore(&adc->reg_lock, flags);
}

static void am437x_adc_se_set_cont(struct ti_adc1_dev *adc, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&adc->reg_lock, flags);
	adc->reg_se_cache |= val;
	spin_unlock_irqrestore(&adc->reg_lock, flags);
	am437x_adc_se_update(adc, 0);
}

static void am437x_adc_se_set_once(struct ti_adc1_dev *adc, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&adc->reg_lock, flags);
	adc->adc_pending = true;
	tiadc_writel(adc, REG_SE, adc->reg_se_cache | val);
	spin_unlock_irqrestore(&adc->reg_lock, flags);
}

static void am437x_adc_se_done(struct ti_adc1_dev *adc)
{
	unsigned long flags;

	spin_lock_irqsave(&adc->reg_lock, flags);
	adc->adc_pending = false;
	spin_unlock_irqrestore(&adc->reg_lock, flags);
}

static void am437x_adc_se_clr(struct ti_adc1_dev *adc, u32 val)
{
	unsigned long flags;

	spin_lock_irqsave(&adc->reg_lock, flags);
	adc->reg_se_cache &= ~val;
	spin_unlock_irqrestore(&adc->reg_lock, flags);
	am437x_adc_se_update(adc, 0);
}

static u32 get_adc_step_mask(struct ti_adc1_dev *adc_dev)
{
	u32 step_en;

	step_en = ((1 << adc_dev->channels) - 1);
	step_en <<= TOTAL_STEPS - adc_dev->channels + 1;
	return step_en;
}

static u32 get_adc_chan_step_mask(struct ti_adc1_dev *adc_dev,
		struct iio_chan_spec const *chan)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adc_dev->channel_step); i++) {
		if (chan->channel == adc_dev->channel_line[i]) {
			u32 step;

			step = adc_dev->channel_step[i];
			/* +1 for the charger */
			return 1 << (step + 1);
		}
	}
	WARN_ON(1);
	return 0;
}

static void tiadc_step_config(struct ti_adc1_dev *adc_dev)
{
	unsigned int stepconfig;
	int i, steps;

	/*
	 * There are 16 configurable steps and 8 analog input
	 * lines available
	 *
	 * Steps backwards i.e. from 16 towards 0 are used by ADC
	 * depending on number of input lines needed.
	 * Channel would represent which analog input
	 * needs to be given to ADC to digitalize data.
	 */

	steps = TOTAL_STEPS - adc_dev->channels;
	stepconfig = STEPCONFIG_AVG_16 | STEPCONFIG_FIFO1 | STEPCHARGE_RFP(0x3);
	stepconfig &= ~STEPCONFIG_DIFFCNTRL;

	for (i = 0; i < adc_dev->channels; i++) {
		int chan;

		chan = adc_dev->channel_line[i];
		tiadc_writel(adc_dev, REG_STEPCONFIG(steps),
				stepconfig | STEPCONFIG_INP(chan));
		tiadc_writel(adc_dev, REG_STEPDELAY(steps),
				STEPCONFIG_OPENDLY);
		adc_dev->channel_step[i] = steps;
		steps++;
	}
}

static const char * const chan_name_ain[] = {
	"AIN0",
	"AIN1",
	"AIN2",
	"AIN3",
	"AIN4",
	"AIN5",
	"AIN6",
	"AIN7",
};

static int tiadc_channel_init(struct iio_dev *indio_dev, int channels)
{
	struct ti_adc1_dev *adc_dev = iio_priv(indio_dev);
	struct iio_chan_spec *chan_array;
	struct iio_chan_spec *chan;
	int i;

	indio_dev->num_channels = channels;
	chan_array = kcalloc(channels,
			sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (chan_array == NULL)
		return -ENOMEM;

	chan = chan_array;
	for (i = 0; i < channels; i++, chan++) {

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = adc_dev->channel_line[i];
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		chan->datasheet_name = chan_name_ain[chan->channel];
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 32;
	}

	indio_dev->channels = chan_array;

	return 0;
}

static void tiadc_channels_remove(struct iio_dev *indio_dev)
{
	kfree(indio_dev->channels);
}

static int tiadc_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	struct ti_adc1_dev *adc_dev = iio_priv(indio_dev);
	int i, map_val;
	unsigned int fifo1count, read, stepid;
	u32 step = UINT_MAX;
	bool found = false;
	u32 step_en;
	unsigned long timeout = jiffies + usecs_to_jiffies
				(IDLE_TIMEOUT * adc_dev->channels);

	step_en = get_adc_chan_step_mask(adc_dev, chan);
	if (!step_en)
		return -EINVAL;

	am437x_adc_se_set_once(adc_dev, step_en);

	/* Wait for Fifo threshold interrupt */
	while (!(tiadc_readl(adc_dev, REG_RAWIRQSTATUS) & IRQENB_FIFO1THRES)) {
		if (time_after(jiffies, timeout)) {
			am437x_adc_se_done(adc_dev);
			return -EAGAIN;
		}
	}
	map_val = chan->channel + TOTAL_CHANNELS;

	/*
	 * When the sub-system is first enabled,
	 * the sequencer will always start with the
	 * lowest step (1) and continue until step (16).
	 * For ex: If we have enabled 4 ADC channels and
	 * currently use only 1 out of them, the
	 * sequencer still configures all the 4 steps,
	 * leading to 3 unwanted data.
	 * Hence we need to flush out this data.
	 */

	for (i = 0; i < ARRAY_SIZE(adc_dev->channel_step); i++) {
		if (chan->channel == adc_dev->channel_line[i]) {
			step = adc_dev->channel_step[i];
			break;
		}
	}
	if (WARN_ON_ONCE(step == UINT_MAX)) {
		am437x_adc_se_done(adc_dev);
		return -EINVAL;
	}

	fifo1count = tiadc_readl(adc_dev, REG_FIFO1CNT);
	for (i = 0; i < fifo1count; i++) {
		read = tiadc_readl(adc_dev, REG_FIFO1);
		stepid = read & FIFOREAD_CHNLID_MASK;
		stepid = stepid >> 0x10;

		if (stepid == map_val) {
			read = (read & FIFOREAD_DATA_MASK) >> 1;
			found = true;
			*val = read;
		}
	}

	tiadc_writel(adc_dev, REG_IRQSTATUS, IRQENB_FIFO1THRES);
	am437x_adc_se_done(adc_dev);

	if (found == false)
		return -EBUSY;

	return IIO_VAL_INT;
}

static const struct iio_info tiadc_info = {
	.read_raw = &tiadc_read_raw,
	.driver_module = THIS_MODULE,
};

static int tiadc_probe(struct platform_device *pdev)
{
	struct iio_dev		*indio_dev;
	struct ti_adc1_dev	*ti_adc1;
	struct device_node	*node = pdev->dev.of_node;
	struct property		*prop;
	struct resource     *res;
	struct clk          *clk;
	const __be32		*cur;
	int			err;
	u32			val;
	int			channels = 0;
	int         clk_value, clock_rate;

	if (!node) {
		dev_err(&pdev->dev, "Could not find valid DT data.\n");
		return -EINVAL;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev,
					  sizeof(struct ti_adc1_dev));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "failed to allocate iio device\n");
		return -ENOMEM;
	}
	ti_adc1 = iio_priv(indio_dev);
	ti_adc1->dev = &pdev->dev;

	of_property_for_each_u32(node, "ti,adc-channels", prop, cur, val) {
		ti_adc1->channel_line[channels] = val;
		channels++;
	}
	ti_adc1->channels = channels;

	/* Get memory resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no memory resource defined.\n");
		return -EINVAL;
	}

	err = platform_get_irq(pdev, 0);
	if (err < 0) {
		dev_err(&pdev->dev, "no irq ID is specified.\n");
		return err;
	} else {
		ti_adc1->irq = err;
	}

	/* Request memory region */
	res = devm_request_mem_region(&pdev->dev,
								  res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "failed to reserve registers.\n");
		return -EBUSY;
	}

	/* IO memory remap */
	ti_adc1->adc_base = devm_ioremap(&pdev->dev,
									 res->start, resource_size(res));
	if (!ti_adc1->adc_base) {
		dev_err(&pdev->dev, "failed to map register.\n");
		return -ENOMEM;
	}

	/* Initialize mmio */
	ti_adc1->regmap_adc = devm_regmap_init_mmio(&pdev->dev,
											   ti_adc1->adc_base,
											   &ti_adc_regmap_config);
	if (IS_ERR(ti_adc1->regmap_adc)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		err = PTR_ERR(ti_adc1->regmap_adc);
		return err;
	}

	/* Initialize spin lock */
	spin_lock_init(&ti_adc1->reg_lock);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* Clock setting */
	clk = clk_get(&pdev->dev, "l3s_gclk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get adc fck\n");
		err = PTR_ERR(clk);
		goto err_disable_clk;
	}

	clock_rate = clk_get_rate(clk);
	clk_put(clk);
	clk_value = clock_rate / ADC_CLK;
	/* ADC1_CLKDIV needs to be configured to the value minus 1 */
	clk_value = clk_value - 1;
	tiadc_writel(ti_adc1, REG_CLKDIV, clk_value);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &tiadc_info;

	tiadc_step_config(ti_adc1);
	tiadc_writel(ti_adc1, REG_FIFO1THR, 1 - 1);

	/*
	 * Set the control register for enable adc1, power down
	 * and bypass preamplifier, and store step id in the FIFO
	 */
	val = tiadc_readl(ti_adc1, REG_CTRL);
	val |= (CNTRLREG_ADC1ENB | CNTRLREG_STEPIDTAG |
			CNTRLREG_PREAMP_PD | CNTRLREG_PREAMP_BYPASS);
	tiadc_writel(ti_adc1, REG_CTRL, val);

	val = tiadc_readl(ti_adc1, REG_IDLECONFIG);
	tiadc_writel(ti_adc1, REG_IDLECONFIG, val | STEPCHARGE_RFP(0x3));

	err = tiadc_channel_init(indio_dev, ti_adc1->channels);
	if (err < 0)
		return err;

	err = iio_device_register(indio_dev);
	if (err)
		goto err_free_channels;

	platform_set_drvdata(pdev, indio_dev);

	return 0;

 err_free_channels:
	tiadc_channels_remove(indio_dev);
 err_disable_clk:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return err;
}

static int tiadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct ti_adc1_dev *adc_dev = iio_priv(indio_dev);
	u32 step_en;

	iio_device_unregister(indio_dev);
	tiadc_channels_remove(indio_dev);

	step_en = get_adc_step_mask(adc_dev);
	am437x_adc_se_clr(adc_dev, step_en);

	return 0;
}

#ifdef CONFIG_PM
static int tiadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ti_adc1_dev *adc_dev = iio_priv(indio_dev);
	unsigned int idle;

	if (!device_may_wakeup(adc_dev->dev)) {
		idle = tiadc_readl(adc_dev, REG_CTRL);
		idle &= ~(CNTRLREG_TSCSSENB);
		tiadc_writel(adc_dev, REG_CTRL, (idle |
				CNTRLREG_POWERDOWN));
	}

	return 0;
}

static int tiadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ti_adc1_dev *adc_dev = iio_priv(indio_dev);
	unsigned int restore;

	/* Make sure ADC is powered up */
	restore = tiadc_readl(adc_dev, REG_CTRL);
	restore &= ~(CNTRLREG_POWERDOWN);
	tiadc_writel(adc_dev, REG_CTRL, restore);

	tiadc_step_config(adc_dev);

	return 0;
}

static const struct dev_pm_ops tiadc_pm_ops = {
	.suspend = tiadc_suspend,
	.resume = tiadc_resume,
};
#define TIADC_PM_OPS (&tiadc_pm_ops)
#else
#define TIADC_PM_OPS NULL
#endif

static const struct of_device_id ti_adc_dt_ids[] = {
	{ .compatible = "ti,am437x-adc1", },
	{ }
};
MODULE_DEVICE_TABLE(of, ti_adc_dt_ids);

static struct platform_driver tiadc_driver = {
	.driver = {
		.name   = "TI-am437x-adc",
		.owner	= THIS_MODULE,
		.pm	= TIADC_PM_OPS,
		.of_match_table = of_match_ptr(ti_adc_dt_ids),
	},
	.probe	= tiadc_probe,
	.remove	= tiadc_remove,
};
module_platform_driver(tiadc_driver);

MODULE_DESCRIPTION("TI ADC controller driver");
MODULE_AUTHOR("MYiR <support@myirtech.com>");
MODULE_LICENSE("GPL");
