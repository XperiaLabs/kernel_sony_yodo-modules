// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADS1015 - Texas Instruments Analog-to-Digital Converter
 *
 * Copyright (c) 2016, Intel Corporation.
 * 
 * IIO driver for ADS1015 ADC 7-bit I2C slave address:
 *	* 0x48 - ADDR connected to Ground
 *	* 0x49 - ADDR connected to Vdd
 *	* 0x4A - ADDR connected to SDA
 *	* 0x4B - ADDR connected to SCL
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/kthread.h>

#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define	GET_BIT(x, bit)	((x & (1 << bit)) >> bit)	/* 获取第bit位 */

#define ADS1015_DRV_NAME "aks-ads1015"

#define TEST_KTHREAD 0 
#define ENABLE_IRQ_PIN	0
#define ENABLE_VDD_CONFIG 1
#define ADS1015_CHANNELS 8

#define ADS1015_CONV_REG	0x00
#define ADS1015_CFG_REG		0x01
#define ADS1015_LO_THRESH_REG	0x02
#define ADS1015_HI_THRESH_REG	0x03

#define ADS1015_CFG_COMP_QUE_SHIFT	0
#define ADS1015_CFG_COMP_LAT_SHIFT	2
#define ADS1015_CFG_COMP_POL_SHIFT	3
#define ADS1015_CFG_COMP_MODE_SHIFT	4
#define ADS1015_CFG_DR_SHIFT	5
#define ADS1015_CFG_MOD_SHIFT	8
#define ADS1015_CFG_PGA_SHIFT	9
#define ADS1015_CFG_MUX_SHIFT	12

//#define ADS1015_CFG_COMP_QUE_MASK	(unsigned int)(~ GENMASK(1, 0))
#define ADS1015_CFG_COMP_QUE_MASK	GENMASK(1, 0)
#define ADS1015_CFG_COMP_LAT_MASK	BIT(2)
#define ADS1015_CFG_COMP_POL_MASK	BIT(3)
#define ADS1015_CFG_COMP_MODE_MASK	BIT(4)
#define ADS1015_CFG_DR_MASK	GENMASK(7, 5)
#define ADS1015_CFG_MOD_MASK	BIT(8)
#define ADS1015_CFG_PGA_MASK	GENMASK(11, 9)
#define ADS1015_CFG_MUX_MASK	GENMASK(14, 12)

/* Comparator queue and disable field */
#define ADS1015_CFG_COMP_DISABLE	3
#define ADS1015_CFG_COMP_ASSERT_AFTER_ONE_CONV	0
#define ADS1015_CFG_COMP_ASSERT_AFTER_TWO_CONV	1
#define ADS1015_CFG_COMP_ASSERT_AFTER_THREE_CONV	2
#define ADS1015_CFG_COMP_ASSERT_MODE ADS1015_CFG_COMP_ASSERT_AFTER_THREE_CONV

/* Comparator polarity field */
#define ADS1015_CFG_COMP_POL_LOW	0
#define ADS1015_CFG_COMP_POL_HIGH	1

/* Comparator mode field */
#define ADS1015_CFG_COMP_MODE_TRAD	0
#define ADS1015_CFG_COMP_MODE_WINDOW	1

/* device operating modes */
#define ADS1015_CONTINUOUS	0
#define ADS1015_SINGLESHOT	1

#define ADS1015_SLEEP_DELAY_MS		2000
#define ADS1015_DEFAULT_PGA		2
#define ADS1015_DEFAULT_DATA_RATE	4
#define ADS1015_DEFAULT_CHAN		0

#define VDD_LOAD_uA	(100000)
#define VDD_LOAD_uV	(3000000)
#define VDD_MIN_uV	(3000000)
#define VDD_MAX_uV	(3000000)

int last_conv_val;


enum chip_ids {
	ADSXXXX = 0,
	ADS1015,
	ADS1115,
};

enum ads1015_channels {
	ADS1015_AIN0_AIN1 = 0,
	ADS1015_AIN0_AIN3,
	ADS1015_AIN1_AIN3,
	ADS1015_AIN2_AIN3,
	ADS1015_AIN0,
	ADS1015_AIN1,
	ADS1015_AIN2,
	ADS1015_AIN3,
	ADS1015_TIMESTAMP,
};

static const unsigned int ads1015_data_rate[] = {
	128, 250, 490, 920, 1600, 2400, 3300, 3300
};

static const unsigned int ads1115_data_rate[] = {
	8, 16, 32, 64, 128, 250, 475, 860
};

/*
 * Translation from PGA bits to full-scale positive and negative input voltage
 * range in mV
 */
static int ads1015_fullscale_range[] = {
	6144, 4096, 2048, 1024, 512, 256, 256, 256
};

/*
 * Translation from COMP_QUE field value to the number of successive readings
 * exceed the threshold values before an interrupt is generated
 */
static const int ads1015_comp_queue[] = { 1, 2, 4 };

static const struct iio_event_spec ads1015_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) |
				BIT(IIO_EV_INFO_PERIOD),
	},
};

#define ADS1015_V_CHAN(_chan, _addr) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 12,					\
		.storagebits = 16,				\
		.shift = 4,					\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan,				\
}

#define ADS1015_V_DIFF_CHAN(_chan, _chan2, _addr) {		\
	.type = IIO_VOLTAGE,					\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 12,					\
		.storagebits = 16,				\
		.shift = 4,					\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}

#define ADS1115_V_CHAN(_chan, _addr) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan,				\
}

#define ADS1115_V_DIFF_CHAN(_chan, _chan2, _addr) {		\
	.type = IIO_VOLTAGE,					\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}

struct ads1015_channel_data {
	bool enabled;
	unsigned int pga;
	unsigned int data_rate;
};

struct ads1015_thresh_data {
	unsigned int comp_queue;
	int high_thresh;
	int low_thresh;
};

struct ads1015_data {
	struct regmap *regmap;
	/*
	 * Protects ADC ops, e.g: concurrent sysfs/buffered
	 * data reads, configuration updates
	 */
	struct mutex lock;
	struct ads1015_channel_data channel_data[ADS1015_CHANNELS];

	unsigned int event_channel;
	unsigned int comp_mode;
	struct ads1015_thresh_data thresh_data[ADS1015_CHANNELS];

	unsigned int *data_rate;
	/*
	 * Set to true when the ADC is switched to the continuous-conversion
	 * mode and exits from a power-down state.  This flag is used to avoid
	 * getting the stale result from the conversion register.
	 */
	bool conv_invalid;

	struct device *dev;
	struct regulator *vdd_reg;
	struct iio_trigger *trigger;
	int irq;
};

struct device *global_dev;


static bool ads1015_event_channel_enabled(struct ads1015_data *data)
{
	return data->event_channel != ADS1015_CHANNELS;
}

static void ads1015_event_channel_enable(struct ads1015_data *data, int chan,
					 int comp_mode)
{
	WARN_ON(ads1015_event_channel_enabled(data));
	data->event_channel = chan;
	data->comp_mode = comp_mode;
}

static void ads1015_event_channel_disable(struct ads1015_data *data, int chan)
{
	data->event_channel = ADS1015_CHANNELS;
}

static bool ads1015_is_writeable_reg(struct device *dev, unsigned int reg)
{
	//dev_err(global_dev, "%s(%d)...\n",__FUNCTION__, __LINE__);

	switch (reg) {
	case ADS1015_CFG_REG:
	case ADS1015_LO_THRESH_REG:
	case ADS1015_HI_THRESH_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config ads1015_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = ADS1015_HI_THRESH_REG,
	.writeable_reg = ads1015_is_writeable_reg,
};

static const struct iio_chan_spec ads1015_channels[] = {
	ADS1015_V_DIFF_CHAN(0, 1, ADS1015_AIN0_AIN1),
	ADS1015_V_DIFF_CHAN(0, 3, ADS1015_AIN0_AIN3),
	ADS1015_V_DIFF_CHAN(1, 3, ADS1015_AIN1_AIN3),
	ADS1015_V_DIFF_CHAN(2, 3, ADS1015_AIN2_AIN3),
	ADS1015_V_CHAN(0, ADS1015_AIN0),
	ADS1015_V_CHAN(1, ADS1015_AIN1),
	ADS1015_V_CHAN(2, ADS1015_AIN2),
	ADS1015_V_CHAN(3, ADS1015_AIN3),
	IIO_CHAN_SOFT_TIMESTAMP(ADS1015_TIMESTAMP),
};

static const struct iio_chan_spec ads1115_channels[] = {
	ADS1115_V_DIFF_CHAN(0, 1, ADS1015_AIN0_AIN1),
	ADS1115_V_DIFF_CHAN(0, 3, ADS1015_AIN0_AIN3),
	ADS1115_V_DIFF_CHAN(1, 3, ADS1015_AIN1_AIN3),
	ADS1115_V_DIFF_CHAN(2, 3, ADS1015_AIN2_AIN3),
	ADS1115_V_CHAN(0, ADS1015_AIN0),
	ADS1115_V_CHAN(1, ADS1015_AIN1),
	ADS1115_V_CHAN(2, ADS1015_AIN2),
	ADS1115_V_CHAN(3, ADS1015_AIN3),
	IIO_CHAN_SOFT_TIMESTAMP(ADS1015_TIMESTAMP),
};

#ifdef CONFIG_PM
static int ads1015_set_power_state(struct ads1015_data *data, bool on)
{
	int ret;
	struct device *dev = regmap_get_device(data->regmap);

	if (on) {
		//dev_err(global_dev, "%s(%d) power on...\n",__FUNCTION__, __LINE__);
		ret = pm_runtime_resume_and_get(dev);
	} else {
		//dev_err(global_dev, "%s(%d) power down...\n",__FUNCTION__, __LINE__);
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_autosuspend(dev);
	}

	return ret < 0 ? ret : 0;
}

#else /* !CONFIG_PM */

static int ads1015_set_power_state(struct ads1015_data *data, bool on)
{
	return 0;
}

#endif /* !CONFIG_PM */

static
int ads1015_get_adc_result(struct ads1015_data *data, int chan, int *val)
{
	int ret, pga, dr;
		//, dr_old, conv_time;
	unsigned int old, mask, cfg;
	if (chan < 0 || chan >= ADS1015_CHANNELS) {
		return -EINVAL;
	}

	ret = regmap_read(data->regmap, ADS1015_CFG_REG, &old);
	if (ret) {
		return ret;
	}

	pga = data->channel_data[chan].pga;
	dr = data->channel_data[chan].data_rate;
	mask = ADS1015_CFG_MUX_MASK | ADS1015_CFG_PGA_MASK | ADS1015_CFG_DR_MASK;
	cfg = chan << ADS1015_CFG_MUX_SHIFT | pga << ADS1015_CFG_PGA_SHIFT | dr << ADS1015_CFG_DR_SHIFT;
#if 0
	if (ads1015_event_channel_enabled(data)) {
		dev_err(global_dev, "%s(%d)\n",__FUNCTION__, __LINE__);
		mask |= ADS1015_CFG_COMP_QUE_MASK | ADS1015_CFG_COMP_MODE_MASK;
		cfg |= data->thresh_data[chan].comp_queue << ADS1015_CFG_COMP_QUE_SHIFT | data->comp_mode << ADS1015_CFG_COMP_MODE_SHIFT;
	}
#endif
	cfg = (old & ~mask) | (cfg & mask);
	if (old != cfg) {
		ret = regmap_write(data->regmap, ADS1015_CFG_REG, cfg);
		if (ret) {
			return ret;
		}
		data->conv_invalid = true;
	}

	if (data->conv_invalid) {
		#if 0
		dr_old = (old & ADS1015_CFG_DR_MASK) >> ADS1015_CFG_DR_SHIFT;
		conv_time = DIV_ROUND_UP(USEC_PER_SEC, data->data_rate[dr_old]);
		conv_time += DIV_ROUND_UP(USEC_PER_SEC, data->data_rate[dr]);
		conv_time += conv_time / 10; /* 10% internal clock inaccuracy */
		dev_err(global_dev, "%s(%d) conv_time = %d\n",__FUNCTION__, __LINE__, conv_time);
		#endif
		usleep_range(1375, 1376);
		//msleep(10);
		data->conv_invalid = false;
	}

	return regmap_read(data->regmap, ADS1015_CONV_REG, val);
}

#if 0
irqreturn_t sensor_trigger_handler(int irq, void *p)
{
	u16 buf[8];
	int i = 0;

	/* read data for each active channel */
	for_each_set_bit(bit, active_scan_mask, masklength)
		buf[i++] = sensor_get_data(bit)

	iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);

	iio_trigger_notify_done(trigger);
	return IRQ_HANDLED;
}
#endif

static irqreturn_t ads1015_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads1015_data *data = iio_priv(indio_dev);
	
#if 1
	int ret, res, bit, i=0;
	struct {
		s16 chans[3];
		s64 timestamp __aligned(8);
	} scan;

	//dev_err(global_dev, "%s(%d) indio_dev->active_scan_mask=0x%X, indio_dev->masklength=%d\n",__FUNCTION__, __LINE__, indio_dev->active_scan_mask, indio_dev->masklength);
	mutex_lock(&data->lock);
	for_each_set_bit(bit, indio_dev->active_scan_mask, indio_dev->masklength) {
		ret = ads1015_get_adc_result(data, bit, &res);
		if (ret < 0) {
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			mutex_unlock(&data->lock);
			goto err;
		}
		scan.chans[i++] = res;
	}
	mutex_unlock(&data->lock);
	
	//dev_err(global_dev, "%s(%d) Triggered channels=%d, data[0]=0x%X, data[1]=0x%X, data[2]=0x%X\n",__FUNCTION__, __LINE__, i, scan.chans[0], scan.chans[1], scan.chans[2]);
	iio_push_to_buffers_with_timestamp(indio_dev, &scan, iio_get_time_ns(indio_dev));
#else
	/* Ensure natural alignment of timestamp */
	struct {
		s16 chan;
		s64 timestamp __aligned(8);
	} scan;

	int chan, ret, res;
	memset(&scan, 0, sizeof(scan));

	mutex_lock(&data->lock);
	chan = find_first_bit(indio_dev->active_scan_mask, indio_dev->masklength);
	
	ret = ads1015_get_adc_result(data, chan, &res);
	if (ret < 0) {
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		mutex_unlock(&data->lock);
		goto err;
	}

	dev_err(global_dev, "[%d]Triggered chanel = %d, 0x%X \n", __LINE__, chan, res);

	scan.chan = res;
	mutex_unlock(&data->lock);

	iio_push_to_buffers_with_timestamp(indio_dev, &scan,
					   iio_get_time_ns(indio_dev));
#endif
err:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ads1015_set_scale(struct ads1015_data *data,
			     struct iio_chan_spec const *chan,
			     int scale, int uscale)
{
	int i;
	int fullscale = div_s64((scale * 1000000LL + uscale) << (chan->scan_type.realbits - 1), 1000000);

	for (i = 0; i < ARRAY_SIZE(ads1015_fullscale_range); i++) {
		if (ads1015_fullscale_range[i] == fullscale) {
			data->channel_data[chan->address].pga = i;
			return 0;
		}
	}
	dev_err(global_dev, "%s(%d) error...\n",__FUNCTION__, __LINE__);
	return -EINVAL;
}

static int ads1015_set_data_rate(struct ads1015_data *data, int chan, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1015_data_rate); i++) {
		if (data->data_rate[i] == rate) {
			data->channel_data[chan].data_rate = i;
			return 0;
		}
	}
	dev_err(global_dev, "%s(%d) error...\n",__FUNCTION__, __LINE__);
	return -EINVAL;
}

static int ads1015_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	int ret, idx;
	struct ads1015_data *data = iio_priv(indio_dev);
	mutex_lock(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		#if 1
		int shift = chan->scan_type.shift;
		ret = ads1015_get_adc_result(data, chan->address, val);
		//ret = regmap_read(data->regmap, ADS1015_CONV_REG, val);
		if (ret < 0) {
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		}
		*val = sign_extend32(*val >> shift, 15 - shift);
		ret = IIO_VAL_INT;
		#else
		int shift = chan->scan_type.shift;

		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret) {
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			break;
		}

		if (ads1015_event_channel_enabled(data) &&
				data->event_channel != chan->address) {
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			ret = -EBUSY;
			goto release_direct;
		}

		ret = ads1015_set_power_state(data, true);
		if (ret < 0) {
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			goto release_direct;
		}

		ret = ads1015_get_adc_result(data, chan->address, val);
		if (ret < 0) {
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			ads1015_set_power_state(data, false);
			goto release_direct;
		}

		*val = sign_extend32(*val >> shift, 15 - shift);
		//dev_err(global_dev, "sign_extend32= %d\n", *val);

		ret = ads1015_set_power_state(data, false);
		if (ret < 0) {
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			goto release_direct;
		}

		ret = IIO_VAL_INT;
release_direct:
		iio_device_release_direct_mode(indio_dev);
		#endif
		break;
	}
	case IIO_CHAN_INFO_SCALE:
		idx = data->channel_data[chan->address].pga;
		*val = ads1015_fullscale_range[idx];
		*val2 = chan->scan_type.realbits - 1;
		ret = IIO_VAL_FRACTIONAL_LOG2;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		idx = data->channel_data[chan->address].data_rate;
		*val = data->data_rate[idx];
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		break;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads1015_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		ret = ads1015_set_scale(data, chan, val, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ads1015_set_data_rate(data, chan->address, val);
		break;
	default:
		ret = -EINVAL;
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		break;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_read_event(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct ads1015_data *data = iio_priv(indio_dev);
	int ret;
	unsigned int comp_queue;
	int period;
	int dr;

	mutex_lock(&data->lock);
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = (dir == IIO_EV_DIR_RISING) ?
			data->thresh_data[chan->address].high_thresh :
			data->thresh_data[chan->address].low_thresh;
		ret = IIO_VAL_INT;
		break;
	case IIO_EV_INFO_PERIOD:
		dr = data->channel_data[chan->address].data_rate;
		comp_queue = data->thresh_data[chan->address].comp_queue;
		period = ads1015_comp_queue[comp_queue] *
			USEC_PER_SEC / data->data_rate[dr];

		*val = period / USEC_PER_SEC;
		*val2 = period % USEC_PER_SEC;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
		dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);
		break;
	}

	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_write_event(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct ads1015_data *data = iio_priv(indio_dev);
	int realbits = chan->scan_type.realbits;
	int ret = 0;
	long long period;
	int i;
	int dr;

	mutex_lock(&data->lock);
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (val >= 1 << (realbits - 1) || val < -1 << (realbits - 1)) {
			ret = -EINVAL;
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			break;
		}
		if (dir == IIO_EV_DIR_RISING)
			data->thresh_data[chan->address].high_thresh = val;
		else
			data->thresh_data[chan->address].low_thresh = val;
		break;
	case IIO_EV_INFO_PERIOD:
		dr = data->channel_data[chan->address].data_rate;
		period = val * USEC_PER_SEC + val2;

		for (i = 0; i < ARRAY_SIZE(ads1015_comp_queue) - 1; i++) {
			if (period <= ads1015_comp_queue[i] *
					USEC_PER_SEC / data->data_rate[dr])
				break;
		}
		data->thresh_data[chan->address].comp_queue = i;
		break;
	default:
		ret = -EINVAL;
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		break;
	}

	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_read_event_config(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct ads1015_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);
	if (data->event_channel == chan->address) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			ret = 1;
			break;
		case IIO_EV_DIR_EITHER:
			ret = (data->comp_mode == ADS1015_CFG_COMP_MODE_WINDOW);
			break;
		default:
			ret = -EINVAL;
			dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
			break;
		}
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_enable_event_config(struct ads1015_data *data,
	const struct iio_chan_spec *chan, int comp_mode)
{
	int low_thresh = data->thresh_data[chan->address].low_thresh;
	int high_thresh = data->thresh_data[chan->address].high_thresh;
	int ret;
	unsigned int val;
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	if (ads1015_event_channel_enabled(data)) {
		if (data->event_channel != chan->address ||
			(data->comp_mode == ADS1015_CFG_COMP_MODE_TRAD &&
				comp_mode == ADS1015_CFG_COMP_MODE_WINDOW)) {
			dev_err(global_dev, "%s(%d) EBUSY.\n",__FUNCTION__, __LINE__);
			return -EBUSY;
		}

		return 0;
	}

	if (comp_mode == ADS1015_CFG_COMP_MODE_TRAD) {
		low_thresh = max(-1 << (chan->scan_type.realbits - 1),
				high_thresh - 1);
	}
	ret = regmap_write(data->regmap, ADS1015_LO_THRESH_REG,
			low_thresh << chan->scan_type.shift);
	if (ret) {
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = regmap_write(data->regmap, ADS1015_HI_THRESH_REG,
			high_thresh << chan->scan_type.shift);
	if (ret) {
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}

	ret = ads1015_set_power_state(data, true);
	if (ret < 0) {
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}

	ads1015_event_channel_enable(data, chan->address, comp_mode);

	ret = ads1015_get_adc_result(data, chan->address, &val);
	if (ret) {
		ads1015_event_channel_disable(data, chan->address);
		ads1015_set_power_state(data, false);
	}

	return ret;
}

static int ads1015_disable_event_config(struct ads1015_data *data,
	const struct iio_chan_spec *chan, int comp_mode)
{
	int ret;
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);
	if (!ads1015_event_channel_enabled(data))
		return 0;

	if (data->event_channel != chan->address)
		return 0;

	if (data->comp_mode == ADS1015_CFG_COMP_MODE_TRAD &&
			comp_mode == ADS1015_CFG_COMP_MODE_WINDOW)
		return 0;

	ret = regmap_update_bits(data->regmap, ADS1015_CFG_REG,
				ADS1015_CFG_COMP_QUE_MASK,
				ADS1015_CFG_COMP_ASSERT_MODE <<
					ADS1015_CFG_COMP_QUE_SHIFT);
	if (ret) {
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}

	ads1015_event_channel_disable(data, chan->address);

	return ads1015_set_power_state(data, false);
}

static int ads1015_write_event_config(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	struct ads1015_data *data = iio_priv(indio_dev);
	int ret;
	int comp_mode = (dir == IIO_EV_DIR_EITHER) ?
		ADS1015_CFG_COMP_MODE_WINDOW : ADS1015_CFG_COMP_MODE_TRAD;

	mutex_lock(&data->lock);
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	/* Prevent from enabling both buffer and event at a time */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret) {
		mutex_unlock(&data->lock);
		dev_err(global_dev, "%s(%d) error: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}

	if (state)
		ret = ads1015_enable_event_config(data, chan, comp_mode);
	else
		ret = ads1015_disable_event_config(data, chan, comp_mode);

	iio_device_release_direct_mode(indio_dev);
	mutex_unlock(&data->lock);

	return ret;
}
#if 0
static int ads1015_set_trigger_state(struct iio_trigger *trig, bool state)
{
	const struct iio_dev         *indio_dev = dev_get_drvdata(trig->dev.parent);
	const struct ads1015_data    *priv = iio_priv(indio_dev);
	int                           err;

	if (!state) {
		/*
		 * Switch trigger off : in case of failure, interrupt is left
		 * disabled in order to prevent handler from accessing released
		 * resources.
		 */
		unsigned int val;

		/*
		 * As device is working in continuous mode, handlers may be
		 * accessing resources we are currently freeing...
		 * Prevent this by disabling interrupt handlers and ensure
		 * the device will generate no more interrupts unless explicitly
		 * required to, i.e. by restoring back to default one shot mode.
		 */
		//disable_irq(priv->irq);

		/*
		 * Disable continuous sampling mode to restore settings for
		 * one shot / direct sampling operations.
		 */
		err = ads1015_set_conv_mode(priv, ADS1015_SINGLESHOT);
		if (err) {
			dev_err(global_dev, "Can not switch to single shot mode");
			return err;
		}

		/*
		 * Re-enable interrupts only if we can guarantee the device will
		 * generate no more interrupts to prevent handlers from
		 * accessing released resources.
		 */
		//enable_irq(priv->irq);

		dev_err(global_dev, "continuous mode stopped");
	} else {
		err = ads1015_set_conv_mode(priv, ADS1015_CONTINUOUS);

		if (err) {
			dev_err(global_dev, "Can not switch to continious mode");
			return err;
		}

	}

	return 0;
}


static const struct iio_trigger_ops ads1015_trigger_ops = {
	.set_trigger_state = ads1015_set_trigger_state,
};

static int ads1015_init_managed_trigger(struct device          * parent,
					struct iio_dev         *indio_dev,
					struct ads1015_data   *private)
{
	struct iio_trigger *trigger;
	int ret;

	dev_err(parent, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	trigger = devm_iio_trigger_alloc(parent, "%s-dev%d",
					 indio_dev->name,
					 iio_device_id(indio_dev));
	if (!trigger) {
		dev_err(global_dev, "failed to devm_iio_trigger_alloc");
		return -ENOMEM;
	}

	/* Basic setup. */
	//trigger->ops = &ads1015_trigger_ops;

	private->trigger = trigger;

	/* Register to triggers space. */
	ret = devm_iio_trigger_register(parent, trigger);
	if (ret) {
		dev_err(parent, "failed to register hardware trigger (%d)", ret);
	}

	return ret;
}
#endif

#if ENABLE_IRQ_PIN
static irqreturn_t ads1015_event_handler(int irq, void *priv)
{
	struct iio_dev *indio_dev = priv;
	struct ads1015_data *data = iio_priv(indio_dev);
	int val;
	int ret;
	//dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	/* Clear the latched ALERT/RDY pin */
	ret = regmap_read(data->regmap, ADS1015_CONV_REG, &val);
	
	if (ret) {
		dev_err(global_dev, "%s(%d) IRQ_HANDLED\n",__FUNCTION__, __LINE__);
		return IRQ_HANDLED;
	}

#if 0
	else {
		if(last_conv_val!=val) {
			dev_err(global_dev, "%s(%d) ADS1015_CONV_REG = 0x%X\n",__FUNCTION__, __LINE__, val);
			last_conv_val = val;
		}
	}

	iio_trigger_poll_chained(data->trigger);
#endif

	if (ads1015_event_channel_enabled(data)) {
		enum iio_event_direction dir;
		u64 code;
		dev_err(global_dev, "%s(%d) pushing event ...\n",__FUNCTION__, __LINE__);
		dev_err(global_dev, "%s(%d) event_channel=%d, val=0x%X\n",__FUNCTION__, __LINE__,data->event_channel, val);
		dir = data->comp_mode == ADS1015_CFG_COMP_MODE_TRAD ?
					IIO_EV_DIR_RISING : IIO_EV_DIR_EITHER;
		code = IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, data->event_channel,
					IIO_EV_TYPE_THRESH, dir);
		iio_push_event(indio_dev, code, iio_get_time_ns(indio_dev));
	}

	return IRQ_HANDLED;
}
#endif
static int ads1015_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ads1015_data *data = iio_priv(indio_dev);
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	/* Prevent from enabling both buffer and event at a time */
	if (ads1015_event_channel_enabled(data)) {
		dev_err(global_dev, "%s(%d) EBUSY...\n",__FUNCTION__, __LINE__);
		return -EBUSY;
	} else {
		dev_err(global_dev, "%s(%d) Channel(%d) enabled...\n",__FUNCTION__, __LINE__, data->event_channel);
	}

	return ads1015_set_power_state(iio_priv(indio_dev), true);
}

static int ads1015_buffer_postdisable(struct iio_dev *indio_dev)
{
	
	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);
	return ads1015_set_power_state(iio_priv(indio_dev), false);
}

static const struct iio_buffer_setup_ops ads1015_buffer_setup_ops = {
	.preenable	= ads1015_buffer_preenable,
	.postdisable	= ads1015_buffer_postdisable,
	/*
	.validate_scan_mask = &iio_validate_scan_mask_onehot,
	*/
};

static IIO_CONST_ATTR_NAMED(ads1015_scale_available, scale_available,
	"3 2 1 0.5 0.25 0.125");
static IIO_CONST_ATTR_NAMED(ads1115_scale_available, scale_available,
	"0.1875 0.125 0.0625 0.03125 0.015625 0.007813");

static IIO_CONST_ATTR_NAMED(ads1015_sampling_frequency_available,
	sampling_frequency_available, "128 250 490 920 1600 2400 3300");
static IIO_CONST_ATTR_NAMED(ads1115_sampling_frequency_available,
	sampling_frequency_available, "8 16 32 64 128 250 475 860");

static struct attribute *ads1015_attributes[] = {
	&iio_const_attr_ads1015_scale_available.dev_attr.attr,
	&iio_const_attr_ads1015_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads1015_attribute_group = {
	.attrs = ads1015_attributes,
};

static struct attribute *ads1115_attributes[] = {
	&iio_const_attr_ads1115_scale_available.dev_attr.attr,
	&iio_const_attr_ads1115_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads1115_attribute_group = {
	.attrs = ads1115_attributes,
};

static const struct iio_info ads1015_info = {
	.read_raw	= ads1015_read_raw,
	.write_raw	= ads1015_write_raw,
	.read_event_value = ads1015_read_event,
	.write_event_value = ads1015_write_event,
	.read_event_config = ads1015_read_event_config,
	.write_event_config = ads1015_write_event_config,
	.attrs          = &ads1015_attribute_group,
};

static const struct iio_info ads1115_info = {
	.read_raw	= ads1015_read_raw,
	.write_raw	= ads1015_write_raw,
	.read_event_value = ads1015_read_event,
	.write_event_value = ads1015_write_event,
	.read_event_config = ads1015_read_event_config,
	.write_event_config = ads1015_write_event_config,
	.attrs          = &ads1115_attribute_group,
};

static int ads1015_client_get_channels_config(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads1015_data *data = iio_priv(indio_dev);
	struct device *dev = &client->dev;
	struct fwnode_handle *node;
	int i = -1;

	device_for_each_child_node(dev, node) {
		u32 pval;
		unsigned int channel;
		unsigned int pga = ADS1015_DEFAULT_PGA;
		unsigned int data_rate = ADS1015_DEFAULT_DATA_RATE;

		if (fwnode_property_read_u32(node, "reg", &pval)) {
			dev_err(dev, "invalid reg on %pfw\n", node);
			continue;
		}

		channel = pval;
		if (channel >= ADS1015_CHANNELS) {
			dev_err(dev, "invalid channel index %d on %pfw\n",
				channel, node);
			continue;
		}

		if (!fwnode_property_read_u32(node, "ti,gain", &pval)) {
			pga = pval;
			if (pga > 6) {
				dev_err(dev, "invalid gain on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
		}

		if (!fwnode_property_read_u32(node, "ti,datarate", &pval)) {
			data_rate = pval;
			if (data_rate > 7) {
				dev_err(dev, "invalid data_rate on %pfw\n", node);
				fwnode_handle_put(node);
				return -EINVAL;
			}
		}

		data->channel_data[channel].pga = pga;
		data->channel_data[channel].data_rate = data_rate;
		i++;

		//dev_err(dev, "%s(%d)->[%d] Channel(%d) pga=%d, data_rate=%d\n", __FUNCTION__, __LINE__, i, channel, pga, data_rate);
	}

	return i < 0 ? -EINVAL : 0;
}

static void ads1015_get_channels_config(struct i2c_client *client)
{
	unsigned int k;
	int ret;

	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads1015_data *data = iio_priv(indio_dev);

	ret = ads1015_client_get_channels_config(client);

	if (!ret) {
		dev_err(data->dev, "%s(%d) get channel configs success!\n",__FUNCTION__, __LINE__);
		return;
	} else {
		dev_err(data->dev, "%s(%d) error, code: %d, Now fallback on default configuration\n",__FUNCTION__, __LINE__, ret);
	}

	/* fallback on default configuration */
	for (k = 0; k < ADS1015_CHANNELS; ++k) {
		data->channel_data[k].pga = ADS1015_DEFAULT_PGA;
		data->channel_data[k].data_rate = ADS1015_DEFAULT_DATA_RATE;
	}
}

#if ENABLE_VDD_CONFIG
static int ads1015_config_vdd(struct ads1015_data *data)

{
	int ret = 0;
	//struct iio_dev *indio_dev = i2c_get_clientdata(client);
	//struct ads1015_data *data = iio_priv(indio_dev);
	//struct device *dev = data->dev;
	dev_err(data->dev, "%s(%d) start!\n",__FUNCTION__, __LINE__);

	data->vdd_reg = devm_regulator_get(data->dev, "ti,vdd");
	if (IS_ERR(data->vdd_reg)) {
		ret = PTR_ERR(data->vdd_reg);
		dev_err(data->dev, "couldn't get vdd_reg regulator, ret:%d\n", ret);
		data->vdd_reg = NULL;
		return ret;
	} else {
		dev_err(data->dev, "%s(%d) devm_regulator_get success!\n",__FUNCTION__, __LINE__);
	}

	dev_err(data->dev, "%s(%d) regulator voltage min=%d, max=%d\n",__FUNCTION__, __LINE__, VDD_MIN_uV, VDD_MAX_uV);
	
	ret = regulator_set_voltage(data->vdd_reg, VDD_MIN_uV, VDD_MAX_uV);
	if (ret) {
		dev_err(data->dev, "%s(%d) regulator_set_voltage faile: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}  else {
		dev_err(data->dev, "%s(%d) regulator_set_voltage success\n",__FUNCTION__, __LINE__, ret);
	}
	ret = regulator_set_load(data->vdd_reg, VDD_LOAD_uA); 
	if (ret) {
		dev_err(data->dev, "%s(%d) regulator_set_load faile: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}  else {
		dev_err(data->dev, "%s(%d) regulator_set_load success\n",__FUNCTION__, __LINE__, ret);
	}
	ret = regulator_enable(data->vdd_reg);
	if (ret < 0) {
		dev_err(data->dev, "vdd_reg regulator failed, ret:%d\n", ret);
		regulator_set_voltage(data->vdd_reg, 0, VDD_LOAD_uV);
		regulator_set_load(data->vdd_reg, 0);
		return -EINVAL;
	} else {
		dev_err(data->dev, "%s(%d) regulator_enable success!\n",__FUNCTION__, __LINE__);
	}

	dev_err(data->dev, "%s(%d) success!\n",__FUNCTION__, __LINE__);
	return 0;
}
#endif

static int ads1015_set_conv_mode(struct ads1015_data *data, int mode)
{
	return regmap_update_bits(data->regmap, ADS1015_CFG_REG,
				  ADS1015_CFG_MOD_MASK,
				  mode << ADS1015_CFG_MOD_SHIFT);
}

#if TEST_KTHREAD

int _task_thread(void *data)
{
	struct ads1015_data *pdata = data;
	int val1, val2 = 0;
	while (1) {
		if (kthread_should_stop()) {
			break;
		}
		while(true) {
			set_current_state(TASK_INTERRUPTIBLE);
			regmap_read(pdata->regmap, ADS1015_CFG_REG, &val1);
			regmap_read(pdata->regmap, ADS1015_CONV_REG, &val2);
			dev_err(global_dev, "%s(%d) ADS1015_CFG_REG=0x%X, ADS1015_CONV_REG=0x%X\n",__FUNCTION__, __LINE__, val1, val2);
			msleep(1);
		}
	}
	return 0;
}

struct task_struct *th;

static int thread_init(void *data)
{

	dev_err(global_dev, "%s(%d) --------------- \n",__FUNCTION__, __LINE__);
	th = kthread_create(_task_thread, data, "_task_thread");

	if (th) {
		wake_up_process(th);
	} else {
		dev_err(global_dev, "%s(%d) --------------- \n",__FUNCTION__, __LINE__);
	}

	return 0;
}
#endif

static int ads1015_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ads1015_data *data;
	int ret;
	enum chip_ids chip;
	int i;
	unsigned int temp;
	int val = 0;

	dev_err(&client->dev, "%s(%d) i2c name=%s, addr=%p start!\n",__FUNCTION__, __LINE__, client->name, client->addr);

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev) {
		dev_err(&client->dev, "%s(%d) devm_iio_device_alloc failed!\n",__FUNCTION__, __LINE__);
		return -ENOMEM;
	} else {
		dev_err(&client->dev, "%s(%d) devm_iio_device_alloc success!\n",__FUNCTION__, __LINE__);
	}

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);

	data->dev = &client->dev;

	mutex_init(&data->lock);

	indio_dev->name = ADS1015_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	chip = (enum chip_ids)device_get_match_data(&client->dev);
	if (chip == ADSXXXX)
		chip = id->driver_data;
	switch (chip) {
	case ADS1015:
		dev_err(&client->dev, "%s(%d) chip_id ADS1015\n",__FUNCTION__, __LINE__);
		indio_dev->channels = ads1015_channels;
		indio_dev->num_channels = ARRAY_SIZE(ads1015_channels);
		indio_dev->info = &ads1015_info;
		data->data_rate = (unsigned int *) &ads1015_data_rate;
		break;
	case ADS1115:
		dev_err(&client->dev, "%s(%d) chip_id ADS1115\n",__FUNCTION__, __LINE__);
		indio_dev->channels = ads1115_channels;
		indio_dev->num_channels = ARRAY_SIZE(ads1115_channels);
		indio_dev->info = &ads1115_info;
		data->data_rate = (unsigned int *) &ads1115_data_rate;
		break;
	default:
		dev_err(&client->dev, "Unknown chip %d\n", chip);
		return -EINVAL;
	}

	data->event_channel = ADS1015_CHANNELS;
	/*
	 * Set default lower and upper threshold to min and max value
	 * respectively.
	 */
	for (i = 0; i < ADS1015_CHANNELS; i++) {
		int realbits = indio_dev->channels[i].scan_type.realbits;

		data->thresh_data[i].low_thresh = -1 << (realbits - 1);
		data->thresh_data[i].high_thresh = (1 << (realbits - 1)) - 1;

		//dev_err(&client->dev, "%s(%d) thresh[%d] low: %d / high: %d\n",__FUNCTION__, __LINE__, i, data->thresh_data[i].low_thresh,data->thresh_data[i].high_thresh);
	}

#if ENABLE_VDD_CONFIG
	//dev_err(&client->dev, "%s(%d)  %d, %d\n",__FUNCTION__, __LINE__, client->dev.power.runtime_error, client->dev.power.disable_depth);

	ret = ads1015_config_vdd(data);
	if (ret < 0) {
		dev_err(&client->dev, "%s(%d) error when config vdd, code: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}
#endif

	/* we need to keep this ABI the same as used by hwmon ADS1015 driver */
	ads1015_get_channels_config(client);

	data->regmap = devm_regmap_init_i2c(client, &ads1015_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "Failed to allocate register map\n");
		return PTR_ERR(data->regmap);
	} else {
		dev_err(&client->dev, "%s(%d)  init i2c regmap success!\n",__FUNCTION__, __LINE__);
		regmap_read(data->regmap, ADS1015_CFG_REG, &val);
		dev_err(&client->dev, "%s(%d) CFG_REG = 0x%X \n", __FUNCTION__, __LINE__, val);
		regmap_read(data->regmap, ADS1015_LO_THRESH_REG, &val);
		dev_err(&client->dev, "%s(%d) LO_THRESH_REG = 0x%X \n", __FUNCTION__, __LINE__, val);
		regmap_read(data->regmap, ADS1015_HI_THRESH_REG, &val);
		dev_err(&client->dev, "%s(%d) HI_THRESH_REG = 0x%X \n", __FUNCTION__, __LINE__, val);
	}

	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev, NULL,
					      ads1015_trigger_handler,
					      &ads1015_buffer_setup_ops);
	if (ret < 0) {
		dev_err(&client->dev, "%s(%d) iio triggered buffer setup failed, error: \n",__FUNCTION__, __LINE__, ret);
		return ret;
	} else {
		dev_err(&client->dev, "%s(%d) iio triggered buffer setup success\n",__FUNCTION__, __LINE__);
	}
	//ads1015_init_managed_trigger(&client->dev, indio_dev, data);

#if ENABLE_IRQ_PIN
	ret = regmap_write(data->regmap, ADS1015_LO_THRESH_REG, 0x4000);
	ret = regmap_write(data->regmap, ADS1015_HI_THRESH_REG, 0x800F);
	data->irq = 0;

	if (client->irq) {
		unsigned long irq_trig =
			irqd_get_trigger_type(irq_get_irq_data(client->irq));
		unsigned int cfg_comp_mask = ADS1015_CFG_COMP_QUE_MASK |
			ADS1015_CFG_COMP_LAT_MASK | ADS1015_CFG_COMP_POL_MASK;
		unsigned int cfg_comp =
			ADS1015_CFG_COMP_ASSERT_MODE << ADS1015_CFG_COMP_QUE_SHIFT | 0 << ADS1015_CFG_COMP_LAT_SHIFT;
		dev_err(&client->dev, "%s(%d) start config client, init_irq=%d, irq=%d, CFG_COMP_QUE_MASK=0x%X \n",__FUNCTION__, __LINE__, client->init_irq, client->irq, ADS1015_CFG_COMP_QUE_MASK);
		data->irq = client->irq;

		switch (irq_trig) {
		case IRQF_TRIGGER_LOW:
			dev_err(&client->dev, "%s(%d) i2c IRQF_TRIGGER_LOW\n",__FUNCTION__, __LINE__);
			cfg_comp |= ADS1015_CFG_COMP_POL_LOW << ADS1015_CFG_COMP_POL_SHIFT;
			break;
		case IRQF_TRIGGER_HIGH:
			dev_err(&client->dev, "%s(%d) i2c IRQF_TRIGGER_HIGH\n",__FUNCTION__, __LINE__);
			cfg_comp |= ADS1015_CFG_COMP_POL_HIGH << ADS1015_CFG_COMP_POL_SHIFT;
			break;
		default:
			dev_err(&client->dev, "%s(%d) i2c IRQF_TRIGGER error: invalide irq trigger type %d\n",__FUNCTION__, __LINE__, irq_trig);
			return -EINVAL;
		}

		//cfg_comp |= 6 << ADS1015_CFG_MUX_SHIFT | 1 << ADS1015_CFG_PGA_SHIFT | 6 << ADS1015_CFG_DR_SHIFT;
		dev_err(&client->dev, "%s(%d) regmap_update_bits: ADS1015_CFG_REG: mask=0x%X, cfg=0x%X \n",__FUNCTION__, __LINE__, cfg_comp_mask, cfg_comp);

		ret = regmap_update_bits(data->regmap, ADS1015_CFG_REG, cfg_comp_mask, cfg_comp);
		if (ret) {
			dev_err(&client->dev, "%s(%d) regmap_update_bits failed, code: %d\n",__FUNCTION__, __LINE__,ret);
			return ret;
 		} else {
			regmap_read(data->regmap, ADS1015_CFG_REG, &val);
			dev_err(&client->dev, "%s(%d) get reg result, ADS1015_CFG_REG value=0x%X\n",__FUNCTION__, __LINE__, val);
			regmap_read(data->regmap, ADS1015_LO_THRESH_REG, &val);
			dev_err(&client->dev, "%s(%d) get reg result, ADS1015_LO_THRESH_REG=0x%X\n",__FUNCTION__, __LINE__, val);
			regmap_read(data->regmap, ADS1015_HI_THRESH_REG, &val);
			dev_err(&client->dev, "%s(%d) get reg result, ADS1015_HI_THRESH_REG=0x%X\n",__FUNCTION__, __LINE__, val);
		}

		/* Request handler to be scheduled into threaded interrupt context. */
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL, ads1015_event_handler, irq_trig | IRQF_ONESHOT, client->name, indio_dev);
		if (ret) {
			dev_err(&client->dev, "%s(%d) devm_request_threaded_irq failed, code: %d\n",__FUNCTION__, __LINE__,ret);
			return ret;
		} else {
			dev_err(&client->dev, "%s(%d) devm_request_threaded_irq success\n",__FUNCTION__, __LINE__);
		}

	} else {
		dev_err(&client->dev, "%s(%d) IRQ not applied, Ignore it\n",__FUNCTION__, __LINE__);
	}
#endif
	ret = ads1015_set_conv_mode(data, ADS1015_CONTINUOUS);
	if (ret) {
		dev_err(&client->dev, "%s(%d) ads1015_set_conv_mode failed, code: %d\n",__FUNCTION__, __LINE__,ret);
		return ret;
	} else {
		dev_err(&client->dev, "%s(%d) ads1015_set_conv_mode %d\n",__FUNCTION__, __LINE__, ADS1015_CONTINUOUS);
	}

	ret = regmap_read(data->regmap, ADS1015_CFG_REG, &temp);
	if(ret) {
		dev_err(&client->dev, "%s(%d) read cfg reg failed, error %d\n",__FUNCTION__, __LINE__, ret);
	} else {
		dev_err(&client->dev, "%s(%d) cfg reg value = 0x%X\n",__FUNCTION__, __LINE__, temp);
	}

	data->conv_invalid = true;

	ret = pm_runtime_set_active(&client->dev);
	if (ret) {
		dev_err(&client->dev, "%s(%d) pm_runtime_set_active failed, code: %d\n",__FUNCTION__, __LINE__,ret);
		return ret;
	} else {
		dev_err(&client->dev, "%s(%d) pm_runtime_set_active OK \n",__FUNCTION__, __LINE__);
	}

#if 0
	pm_runtime_set_autosuspend_delay(&client->dev, ADS1015_SLEEP_DELAY_MS);
	dev_err(&client->dev, "%s(%d) pm_runtime_set_autosuspend_delay to %dms!\n",__FUNCTION__, __LINE__,ADS1015_SLEEP_DELAY_MS);

	pm_runtime_use_autosuspend(&client->dev);
	dev_err(&client->dev, "%s(%d) pm_runtime_use_autosuspend \n",__FUNCTION__, __LINE__);
#endif

	pm_runtime_enable(&client->dev);
	dev_err(&client->dev, "%s(%d) pm_runtime_enable done\n",__FUNCTION__, __LINE__);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "%s(%d) iio_device_register failed, code: %d\n",__FUNCTION__, __LINE__,ret);
		return ret;
	} else {
		dev_err(&client->dev, "%s(%d) indio_dev:->active_scan_mask=%ld, scan_timestamp=%d, scan_bytes=0x%X, currentmode=%d\n",
			__FUNCTION__, __LINE__, 
			indio_dev->trig,
			indio_dev->active_scan_mask,
			indio_dev->scan_timestamp,
			indio_dev->scan_bytes,
			indio_dev->currentmode);
		dev_err(&client->dev, "%s(%d)indio_dev->trig=%p\n",__FUNCTION__, __LINE__, indio_dev->trig);
	}

	dev_err(&client->dev, "%s(%d) success........!\n",__FUNCTION__, __LINE__);

#if TEST_KTHREAD
	thread_init(data);
#endif

	global_dev = &client->dev;

	return 0;
}

static int ads1015_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads1015_data *data = iio_priv(indio_dev);

	dev_err(global_dev, "%s(%d) ...\n",__FUNCTION__, __LINE__);

	iio_device_unregister(indio_dev);
#if 1
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
#endif
	
#if 0
	/* power down single shot mode */
	return ads1015_set_conv_mode(data, ADS1015_SINGLESHOT);
#else
	ads1015_set_conv_mode(data, ADS1015_SINGLESHOT);
	return regulator_disable(data->vdd_reg);
#endif
}

#ifdef CONFIG_PM
static int ads1015_runtime_suspend(struct device *dev)
{
#if 1
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads1015_data *data = iio_priv(indio_dev);
	//dev_err(global_dev, "%s(%d) suspend...\n",__FUNCTION__, __LINE__);

	return ads1015_set_conv_mode(data, ADS1015_SINGLESHOT);
#else
	dev_err(global_dev, "%s(%d) but do nothing...\n",__FUNCTION__, __LINE__);
	return 0;
#endif
}

static int ads1015_runtime_resume(struct device *dev)
{
#if 1
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads1015_data *data = iio_priv(indio_dev);
	int ret;

	//dev_err(global_dev, "%s(%d) resume...\n",__FUNCTION__, __LINE__);
	ret = ads1015_set_conv_mode(data, ADS1015_CONTINUOUS);
	if (!ret) {
		data->conv_invalid = true;
	}

	return ret;
#else
	dev_err(global_dev, "%s(%d) but do nothing...\n",__FUNCTION__, __LINE__);
	return 0;

#endif
}
#endif

static const struct dev_pm_ops ads1015_pm_ops = {
	SET_RUNTIME_PM_OPS(ads1015_runtime_suspend,
			   ads1015_runtime_resume, NULL)
};

static const struct i2c_device_id ads1015_id[] = {
	{"ads1015", ADS1015},
	{"ads1115", ADS1115},
	{}
};
MODULE_DEVICE_TABLE(i2c, ads1015_id);

static const struct of_device_id ads1015_of_match[] = {
	{
		.compatible = "aks,ti-ads1015",
		.data = (void *)ADS1015
	},
	{
		.compatible = "ti,ads1115",
		.data = (void *)ADS1115
	},
	{}
};
MODULE_DEVICE_TABLE(of, ads1015_of_match);

static struct i2c_driver ads1015_driver = {
	.driver = {
		.name = ADS1015_DRV_NAME,
		.of_match_table = ads1015_of_match,
		.pm = &ads1015_pm_ops,
	},
	.probe		= ads1015_probe,
	.remove		= ads1015_remove,
	.id_table	= ads1015_id,
};

module_i2c_driver(ads1015_driver);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com>");
MODULE_DESCRIPTION("Texas Instruments ADS1015 ADC driver");
MODULE_LICENSE("GPL v2");
