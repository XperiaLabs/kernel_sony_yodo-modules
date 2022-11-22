// SPDX-License-Identifier: GPL-2.0
/*
 * Input driver for Aksys.co.kr joysticks connected over ADC.
 * Copyright (c) 2022 Daniel <daniel@aksys.co.kr>
 */
#include <linux/ctype.h>
#include <linux/input.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/kernel.h>

#include <asm/unaligned.h>

#define ADC_JSK_POLL_INTERVAL	10
#define ADC_JSK_POLL_MIN	8
#define ADC_JSK_POLL_MAX	16

#define ADC_JOYSTICK_POLL 1
#define ADC_JOYSTICK_IIO_BUFFER_TRIGGER 0

#define FIXED_RANGE_MIN 0
#define FIXED_RANGE_MAX 512
#define FIXE_CENTER	(256)

#define FIXED_CENTER_SHIFT (8) //2^8 = FIXED_RANGE_MAX/2
#define ENLARGE_SHIFT (10)

struct adc_joystick_cali_info {
    s32 center;
    s32 minimal;
    s32 maximal;
    s32 ratio_negative;
    s32 ratio_pos;
};

struct adc_joystick_axis {
    u32 code;
    s32 range[2];
    s32 fuzz;
    s32 flat;
    struct adc_joystick_cali_info cali_info;
};

struct adc_joystick_cali_val {
    s32 x;
    s32 y;
    s32 t;
};

struct adc_joystick {
    struct input_dev *input;
    struct iio_cb_buffer *buffer;
    struct adc_joystick_axis *axes;
    struct iio_channel *chans;
    int num_chans;
    bool polled;
    struct adc_joystick_cali_val cali_val;
    bool need_calibrate;
};
struct device *g_dev;

static void adc_joystick_poll(struct input_dev *input)
{
    struct adc_joystick *joy = input_get_drvdata(input);
    s32 i, val, offset_val;

    for (i = 0; i < joy->num_chans; i++) {
        iio_read_channel_raw(&joy->chans[i], &val);
        //dev_err(g_dev, "channel[%d]=%d\n",joy->chans[i].channel->address, val);
        //offset_val = val;
#if 1
        switch(joy->axes[i].code) {
        case ABS_X:
        case ABS_Z:
        case ABS_Y:
        case ABS_RZ:
            val = (val < joy->axes[i].cali_info.minimal) ? joy->axes[i].cali_info.minimal : val;
            val = (val > joy->axes[i].cali_info.maximal) ? joy->axes[i].cali_info.maximal : val;
            //dev_err(g_dev, "channel[%d] val = %d\n",joy->chans[i].channel->address, val);
            if(val <= joy->axes[i].cali_info.center) {
                offset_val = ((val - joy->axes[i].cali_info.minimal) << ENLARGE_SHIFT) / joy->axes[i].cali_info.ratio_negative;
            } else {
                offset_val = FIXE_CENTER + ((val - joy->axes[i].cali_info.center) << ENLARGE_SHIFT) / joy->axes[i].cali_info.ratio_pos;
            }
            break;
        case ABS_BRAKE:
        case ABS_GAS:
            offset_val = (val > joy->axes[i].cali_info.maximal) ? joy->axes[i].cali_info.maximal : val;
            break;
        default:
            break;
        }
        //dev_err(g_dev, "channel[%d]=%d, offset=%d\n",joy->chans[i].channel->address, val, offset_val);
        input_report_abs(input, joy->axes[i].code, offset_val);
#else
        input_report_abs(input, joy->axes[i].code, val);
#endif
    }
    input_sync(input);
}

static int adc_joystick_inital_params(struct adc_joystick *joy)
{
    int ret = 0;
    ret = iio_read_channel_raw(&joy->chans[0], &(joy->cali_val.x)); //X axis init position
    if (ret != IIO_VAL_INT) {
        dev_err(g_dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, ret);
        return ret;
    }
    ret = iio_read_channel_raw(&joy->chans[1], &(joy->cali_val.y)); //Y axis init position
    if (ret != IIO_VAL_INT) {
        dev_err(g_dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, ret);
        return ret;
    }
    ret = iio_read_channel_raw(&joy->chans[2], &(joy->cali_val.t)); //Trigger init position
    if (ret != IIO_VAL_INT) {
        dev_err(g_dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, ret);
        return ret;
    }

    dev_err(g_dev, "Origin x=%d, y=%d, t=%d\n",joy->cali_val.x, joy->cali_val.y, joy->cali_val.t);

    return 0;
}

static int adc_joystick_handle(const void *data, void *private)
{
    struct adc_joystick *joy = private;
    enum iio_endian endianness;
    int bytes, msb, val, idx, i;

    //const u16 *data_u16;
    bool sign;

    typedef struct {
        s16 chans[3];
        s64 timestamp __aligned(8);
    } Scan;

    Scan *scan = (Scan*)data;
    bytes = joy->chans[0].channel->scan_type.storagebits >> 3;

    for (i = 0; i < joy->num_chans; ++i) {
        idx = joy->chans[i].channel->scan_index;
        endianness = joy->chans[i].channel->scan_type.endianness;
        msb = joy->chans[i].channel->scan_type.realbits - 1;
        sign = tolower(joy->chans[i].channel->scan_type.sign) == 's';
        val = scan->chans[i];
        val >>= joy->chans[i].channel->scan_type.shift;
        if (sign)
            val = sign_extend32(val, msb);
        else
            val &= GENMASK(msb, 0);
        input_report_abs(joy->input, joy->axes[i].code, val);
    }
    input_sync(joy->input);
    return 0;
}

static int adc_joystick_open(struct input_dev *dev)
{
    struct adc_joystick *joy = input_get_drvdata(dev);
    struct device *devp = &dev->dev;
    int ret = 0;
    struct iio_dev *indio_dev;
    dev_err(devp, "%s(%d) opening \n", __FUNCTION__, __LINE__);

    if (joy->polled) {
        ret = input_setup_polling(joy->input, adc_joystick_poll);
        if(ret < 0) {
            dev_err(devp, "%s(%d) input_setup_polling error: %d\n",__FUNCTION__, __LINE__, ret);
            return ret;
        }
        input_set_poll_interval(joy->input, ADC_JSK_POLL_INTERVAL);
        if(ret < 0) {
            dev_err(devp, "%s(%d) input_set_poll_interval error: %d\n",__FUNCTION__, __LINE__, ret);
            return ret;
        }
        input_set_min_poll_interval(joy->input, ADC_JSK_POLL_MIN);
        if(ret < 0) {
            dev_err(devp, "%s(%d) input_set_min_poll_interval error: %d\n",__FUNCTION__, __LINE__, ret);
            return ret;
        }
        input_set_max_poll_interval(joy->input, ADC_JSK_POLL_MAX);
        if(ret < 0) {
            dev_err(devp, "%s(%d) input_set_max_poll_interval error: %d\n",__FUNCTION__, __LINE__, ret);
            return ret;
        }
    } else {
        dev_err(devp, "%s(%d) IIO start \n", __FUNCTION__, __LINE__);

        if (joy->buffer == NULL) {
            dev_err(devp, "%s(%d) cb_buffer == NULL\n", __FUNCTION__, __LINE__);
            return -22;
        }
        indio_dev = iio_channel_cb_get_iio_dev(joy->buffer);

        dev_err(devp, "%s(%d) joy->buferr=%p, iio_dev=%p\n", __FUNCTION__, __LINE__, joy->buffer, indio_dev);
        dev_err(devp, "%s(%d) indio_dev:->active_scan_mask=%ld, scan_timestamp=%d, scan_bytes=0x%X, currentmode=%d\n",
                __FUNCTION__, __LINE__,
                indio_dev->trig,
                indio_dev->active_scan_mask,
                indio_dev->scan_timestamp,
                indio_dev->scan_bytes,
                indio_dev->currentmode);

        ret = iio_channel_start_all_cb(joy->buffer);
        if (ret) {
            dev_err(devp, "%s(%d) Unable to start callback buffer-> %d\n", __FUNCTION__, __LINE__, ret);
            return ret;
        } else {
            dev_err(devp, "%s(%d) iio_channel_start_all_cb OK!\n", __FUNCTION__, __LINE__);
        }

    }
    dev_err(devp, "%s(%d) dev %s open success!\n",__FUNCTION__, __LINE__, joy->input->name);
    return 0;
}

static void adc_joystick_close(struct input_dev *dev)
{
    //struct adc_joystick *joy = input_get_drvdata(dev);
    struct device *devp = &dev->dev;
    dev_err(devp, "%s(%d)\n", __FUNCTION__, __LINE__);
#if 0
    iio_channel_stop_all_cb(joy->buffer);
#endif
}

static void adc_joystick_cleanup(void *data)
{
    iio_channel_release_all_cb(data);
}

static int adc_joystick_set_axes(struct device *dev, struct adc_joystick *joy)
{
    struct adc_joystick_axis *axes;
    struct fwnode_handle *child;
    int num_axes, error, i;
    s32 origin_range[2];

    num_axes = device_get_child_node_count(dev);
    if (!num_axes) {
        dev_err(dev, "Unable to find child nodes\n");
        return -EINVAL;
    } else {
        dev_err(dev, "%s(%d) find child nodes %d!\n",__FUNCTION__, __LINE__, num_axes);
    }

    if (num_axes != joy->num_chans) {
        dev_err(dev, "Got %d child nodes for %d channels\n",
                num_axes, joy->num_chans);
        return -EINVAL;
    }

    axes = devm_kmalloc_array(dev, num_axes, sizeof(*axes), GFP_KERNEL);
    if (!axes) {
        return -ENOMEM;
    }

    device_for_each_child_node(dev, child) {
        error = fwnode_property_read_u32(child, "reg", &i);
        if (error) {
            dev_err(dev, "reg invalid or missing\n");
            goto err_fwnode_put;
        }

        if (i >= num_axes) {
            error = -EINVAL;
            dev_err(dev, "No matching axis for reg %d\n", i);
            goto err_fwnode_put;
        }

        error = fwnode_property_read_u32(child, "linux,code",
                                         &axes[i].code);
        if (error) {
            dev_err(dev, "linux,code invalid or missing\n");
            goto err_fwnode_put;
        }

        if(!joy->need_calibrate) {
            error = fwnode_property_read_u32_array(child, "abs-range", axes[i].range, 2);
            if (error) {
                dev_err(dev, "abs-range invalid or missing\n");
                goto err_fwnode_put;
            }
        } else {
            dev_err(g_dev, "Staring calibrate the axes center: Origin x=%d, y=%d, t=%d\n",joy->cali_val.x, joy->cali_val.y, joy->cali_val.t);
            error = fwnode_property_read_u32_array(child, "abs-range", origin_range, 2);
            dev_err(dev, "%s(%d) linux,code = %d, original-range= %d ~ %d!\n",__FUNCTION__, __LINE__, axes[i].code, origin_range[0], origin_range[1]);

            if (error) {
                dev_err(dev, "abs-range invalid or missing\n");
                goto err_fwnode_put;
            }

            axes[i].cali_info.maximal = max(origin_range[0], origin_range[1]);
            axes[i].cali_info.minimal = min(origin_range[0], origin_range[1]);

            //DO NOT do the float value calculate in Kernel
            //axes[i].cali_info.ratio_negative = ((axes[i].cali_info.center - axes[i].cali_info.minimal)/FIXE_CENTER);
            //axes[i].cali_info.ratio_pos = ((axes[i].cali_info.maximal - axes[i].cali_info.center)/FIXE_CENTER);

            switch(axes[i].code) {
            case ABS_X:
            case ABS_Z:
                axes[i].cali_info.center = joy->cali_val.x;
                axes[i].range[0] = FIXED_RANGE_MIN;
                axes[i].range[1] = FIXED_RANGE_MAX;
                axes[i].cali_info.ratio_negative = (((axes[i].cali_info.center - axes[i].cali_info.minimal) << ENLARGE_SHIFT) >> FIXED_CENTER_SHIFT);
                axes[i].cali_info.ratio_pos = (((axes[i].cali_info.maximal - axes[i].cali_info.center) << ENLARGE_SHIFT) >> FIXED_CENTER_SHIFT);
                break;

            case ABS_Y:
            case ABS_RZ:
                axes[i].cali_info.center = joy->cali_val.y;
                axes[i].range[0] = FIXED_RANGE_MIN;
                axes[i].range[1] = FIXED_RANGE_MAX;
                axes[i].cali_info.ratio_negative = (((axes[i].cali_info.center - axes[i].cali_info.minimal) << ENLARGE_SHIFT) >> FIXED_CENTER_SHIFT);
                axes[i].cali_info.ratio_pos = (((axes[i].cali_info.maximal - axes[i].cali_info.center) << ENLARGE_SHIFT) >> FIXED_CENTER_SHIFT);
                break;

            case ABS_BRAKE:
            case ABS_GAS:
                if(origin_range[0] > origin_range[1]) {
                    axes[i].range[1] = joy->cali_val.t;
                    axes[i].range[0] = origin_range[0];
                } else if(origin_range[0] < origin_range[1]) {
                    axes[i].range[0] = joy->cali_val.t;
                    axes[i].range[1] = origin_range[1];
                } else {
                    dev_err(dev, "%s(%d) code<%X> error range from %d to %d !\n",__FUNCTION__, __LINE__, axes[i].code, axes[i].range[0], axes[i].range[1]);
                }
                break;
            }

            dev_err(dev, "%s(%d)-----code<%X> ratio_negative=%d, ratio_pos=%d!\n",__FUNCTION__, __LINE__, \
                    axes[i].code, axes[i].cali_info.ratio_negative, axes[i].cali_info.ratio_pos);

            dev_err(dev, "%s(%d)-----code<%X> axes[i].cali_info.maximal=%d, axes[i].cali_info.minimal=%d!\n",__FUNCTION__, __LINE__, \
                    axes[i].code, axes[i].cali_info.maximal, axes[i].cali_info.minimal);
        }


        fwnode_property_read_u32(child, "abs-fuzz", &axes[i].fuzz);
        fwnode_property_read_u32(child, "abs-flat", &axes[i].flat);
        dev_err(dev, "%s(%d) linux,code=<%d>, abs-range= %d ~ %d, abs-fuzz= %d, abs-flat=%d!\n",__FUNCTION__, __LINE__, \
                axes[i].code, axes[i].range[0], axes[i].range[1], axes[i].fuzz, axes[i].flat);

        input_set_abs_params(joy->input, axes[i].code,
                             axes[i].range[0], axes[i].range[1],
                             axes[i].fuzz, axes[i].flat);
        input_set_capability(joy->input, EV_ABS, axes[i].code);
    }

    input_set_capability(joy->input, EV_KEY, BTN_JOYSTICK); //Mark as joystick device in Android Event Hub.

    joy->axes = axes;

    return 0;

err_fwnode_put:
    fwnode_handle_put(child);
    return error;
}

static int adc_joystick_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct adc_joystick *joy;
    struct input_dev *input;
    struct iio_cb_buffer *cb_buffer;
    int error, bits, i;

    g_dev = dev;

    joy = devm_kzalloc(dev, sizeof(*joy), GFP_KERNEL);
    if (!joy) {
        dev_err(dev, "%s(%d) devm_kzalloc failed!\n",__FUNCTION__, __LINE__);
        return -ENOMEM;
    } else {
        dev_err(dev, "%s(%d) devm_kzalloc OK!\n",__FUNCTION__, __LINE__);
    }

    joy->chans = devm_iio_channel_get_all(dev);
    if (IS_ERR(joy->chans)) {
        error = PTR_ERR(joy->chans);
        if (error != -EPROBE_DEFER)
            dev_err(dev, "Unable to get IIO channels");
        return error;
    }

    /* Count how many channels we got. NULL terminated. */
    for (i = 0; joy->chans[i].indio_dev; i++) {
        bits = joy->chans[i].channel->scan_type.storagebits;
        if (!bits || bits > 16) {
            dev_err(dev, "Unsupported channel storage size\n");
            return -EINVAL;
        } else {
            //dev_err(dev, "%s(%d) channel storage size: %d bits\n",__FUNCTION__, __LINE__, bits);
        }
        if (bits != joy->chans[0].channel->scan_type.storagebits) {
            dev_err(dev, "Channels must have equal storage size\n");
            return -EINVAL;
        }
    }
    joy->num_chans = i;

    dev_err(dev, "%s(%d) joy->num_chans = %d\n",__FUNCTION__, __LINE__, joy->num_chans);

    input = devm_input_allocate_device(dev);
    if (!input) {
        dev_err(dev, "Unable to allocate input device\n");
        return -ENOMEM;
    } else {
        dev_err(dev, "%s(%d) devm_input_allocate_device OK. input name is: %s\n",__FUNCTION__, __LINE__, pdev->name);
    }

    joy->input = input;
    input->name = pdev->name;

    input->phys = "JOYSTICK";
    dev_err(dev, "%s(%d) input->phys is: %s\n",__FUNCTION__, __LINE__, input->phys);
    input->id.bustype = BUS_HOST;
    input->id.vendor = 0x0001;
    input->id.product = 0x0002;
    input->id.version = 0x0001;

#if 0
#if ADC_JOYSTICK_IIO_BUFFER_TRIGGER
    input->open = adc_joystick_open;
    input->close = adc_joystick_close;
#else
    joy->polled = true;
    if (joy->polled) {
        ret = input_setup_polling(input, adc_joystick_poll);
        if(ret < 0) {
            dev_err(dev, "%s(%d) input_setup_polling error: %d\n",__FUNCTION__, __LINE__, ret);
        }
        input_set_poll_interval(input, ADC_JSK_POLL_INTERVAL);
        if(ret < 0) {
            dev_err(dev, "%s(%d) input_setup_polling error: %d\n",__FUNCTION__, __LINE__, ret);
        }
        input_set_min_poll_interval(input, ADC_JSK_POLL_MIN);
        if(ret < 0) {
            dev_err(dev, "%s(%d) input_setup_polling error: %d\n",__FUNCTION__, __LINE__, ret);
        }
        input_set_max_poll_interval(input, ADC_JSK_POLL_MAX);
        if(ret < 0) {
            dev_err(dev, "%s(%d) input_setup_polling error: %d\n",__FUNCTION__, __LINE__, ret);
        }
    } else {
        input->open = adc_joystick_open;
        input->close = adc_joystick_close;
    }
#endif

#else
    input->open = adc_joystick_open;
    input->close = adc_joystick_close;
    joy->polled = true;
#endif
    dev_err(dev, "%s(%d) trigger mode?= %d\n",__FUNCTION__, __LINE__, ADC_JOYSTICK_IIO_BUFFER_TRIGGER);

    error = adc_joystick_inital_params(joy);
    if(error) {
        joy->need_calibrate = false;
    } else {
        joy->need_calibrate = true;
    }

    dev_err(dev, "%s(%d) should calibrate? -> %d\n",__FUNCTION__, __LINE__, joy->need_calibrate);
    error = adc_joystick_set_axes(dev, joy);
    if (error) {
        return error;
    } else {
        dev_err(dev, "%s(%d) adc_joystick_set_axes OK\n",__FUNCTION__, __LINE__);
    }

    input_set_drvdata(input, joy);

    error = input_register_device(input);
    if (error) {
        dev_err(dev, "Unable to register input device\n");
        return error;
    } else {
        dev_err(dev, "%s(%d) input_register_device input->name=%s, input->phys=%s, input->uniq=%s\n",__FUNCTION__, __LINE__, input->name, input->phys, input->uniq);
    }
    if (!joy->polled) {
        joy->buffer = iio_channel_get_all_cb(dev, adc_joystick_handle, joy);
        if (IS_ERR(joy->buffer)) {
            error = PTR_ERR(joy->buffer);
            dev_err(dev, "Unable to allocate callback buffer, code = %d\n", error);
            return error;
        } else {
            dev_err(dev, "%s(%d) iio_channel_get_all_cb OK\n",__FUNCTION__, __LINE__);
        }

        cb_buffer = joy->buffer;

        dev_err(dev, "%s(%d) joy->buferr=%p \n",__FUNCTION__, __LINE__, joy->buffer);

        error = devm_add_action_or_reset(dev, adc_joystick_cleanup, joy->buffer);
        if (error)  {
            dev_err(dev, "Unable to add action\n");
            return error;
        } else {
            dev_err(dev, "%s(%d) devm_add_action_or_reset OK\n",__FUNCTION__, __LINE__);
        }


        dev_err(dev, "%s(%d) joy->buferr=%p\n", __FUNCTION__, __LINE__, joy->buffer);
    }
#if 0
    ret = iio_channel_start_all_cb(joy->buffer);
    if (ret) {
        dev_err(dev, "%s(%d) Unable to start callback buffer-> %d\n", __FUNCTION__, __LINE__, ret);
    } else {
        dev_err(dev, "%s(%d) iio_channel_start_all_cb OK!\n", __FUNCTION__, __LINE__);
    }
#endif
    return 0;
}

static const struct of_device_id adc_joystick_of_match[] = {
    { .compatible = "aks-adc-joystick", },
    { }
};
MODULE_DEVICE_TABLE(of, adc_joystick_of_match);

static struct platform_driver adc_joystick_driver = {
    .driver = {
        .name = "aks-adc-joystick",
        .of_match_table = adc_joystick_of_match,
    },
    .probe = adc_joystick_probe,
};
module_platform_driver(adc_joystick_driver);

MODULE_DESCRIPTION("Input driver for Aksys.co.kr joysticks connected over ADC.");
MODULE_AUTHOR("Daniel <daniel@aksys.co.kr>");
MODULE_LICENSE("GPL");
