// SPDX-License-Identifier: GPL-2.0
/*
 * Input driver for Aksys.co.kr all in one input device driver
 * Copyright (c) 2022 Daniel <daniel@aksys.co.kr>
 */
#include <linux/ctype.h>
#include <linux/math.h>
#include <linux/input.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/kernel.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <asm/unaligned.h>

//GPIO inlcudes below
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <dt-bindings/input/gpio-keys.h>

//cdev
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#define TEST_UPDATE_ABS_PARAMS 0

#if 0
#include <linux/kthread.h>
#include <linux/delay.h>
#endif

#if 0
struct adc_joystick {
	struct device *dev;
    struct input_dev *input;
    struct adc_joystick_axis *axes;
    struct iio_channel *chans;
    int num_chans;
    bool polled;
    struct adc_joystick_cali_val cali_val;
    bool need_calibrate;
	int max_touch_num;
	struct mutex mutex;
};

struct aks_joystick {
	struct device *pdev;
    struct input_dev *input;
    struct adc_joystick_axis *axes;
    struct iio_channel *chans;
    int num_chans;
    bool polled;
    struct adc_joystick_cali_val cali_val;
    bool need_calibrate;
	int max_touch_num;
	struct mutex mutex;
};

#endif

#define AKS_GAMEPAD_ANALOG_POLL (true)

#define AKS_GAMEPAD_ANALOG_POLL_INTERVAL (10)
//#define AKS_GAMEPAD_ANALOG_POLL_INTERVAL_MIN	(AKS_GAMEPAD_ANALOG_POLL_INTERVAL-2)
//#define AKS_GAMEPAD_ANALOG_POLL_INTERVAL_MAX	(AKS_GAMEPAD_ANALOG_POLL_INTERVAL+5)

#define  AKS_GAMEPAD_FIXED_CENTER_SHIFT (8) //2^8 = FIXED_RANGE_MAX/2
#define  AKS_GAMEPAD_ENLARGE_SHIFT (10)

#define AKS_GAMEPAD_MT_MAX_CONTACTS  (10)
#define AKS_GAMEPAD_MT_MAX_HEIGHT	 (4096)
#define AKS_GAMEPAD_MT_MAX_WIDTH	 (4096)

#define AKS_GAMEPAD_WORK_MODE_TOUCH 1
#define AKS_GAMEPAD_WORK_MODE_DIRECT 2

#define AKS_GAMEPAD_MT_ID_JS_L 	1
#define AKS_GAMEPAD_MT_ID_JS_R 	2
#define AKS_GAMEPAD_MT_ID_TRIG_L	3
#define AKS_GAMEPAD_MT_ID_TRIG_R	4

#define AKS_IOC_MAGIC  'k'

#define AKS_IOC_INIT    _IO(AKS_IOC_MAGIC, 1)
#define AKS_IOC_GETDATA _IOR(AKS_IOC_MAGIC, 2, int)
#define AKS_IOC_SETDATA _IOW(AKS_IOC_MAGIC, 3, int)
#define AKS_IOC_SETMODE _IOW(AKS_IOC_MAGIC, 4, int)
#define AKS_IOC_GETMODE _IOR(AKS_IOC_MAGIC, 5, int)

#define AKS_IOC_MAXNR 5

static int CURRENT_WORK_MODE=AKS_GAMEPAD_WORK_MODE_DIRECT;

struct aks_gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct gpio_desc *gpiod;

	unsigned short *code;

	struct hrtimer release_timer;
	unsigned int release_delay;	/* in msecs, for IRQ-only buttons */

	struct delayed_work work;
	struct hrtimer debounce_timer;
	unsigned int software_debounce;	/* in msecs, for GPIO-driven buttons */

	unsigned int irq;
	unsigned int wakeup_trigger_type;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
	bool suspended;
	bool debounce_use_hrtimer;
};

struct aks_gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned short *keymap;
	struct aks_gpio_button_data data[];
};

struct aks_analog_cali_data {
    s32 physic_zero_pos;
	s32 logic_zero_pos;
    s32 rang_min;
    s32 rang_max;
	s32 rang_min_calied;
	s32 rang_max_calied;
    s32 ratio_neg;
    s32 ratio_pos;
};

struct aks_analog_key_axis {
	u32 code;
	s32 range[2];
	s32 fuzz;
	s32 flat;
    struct aks_analog_cali_data cali;
};

struct aks_analog_key_zero_coordinate {
	int x;
	int y;
	int z;
	int rz;
};

struct aks_analog_key_data {
	struct input_dev *input;
	struct iio_cb_buffer *buffer;
	struct aks_analog_key_axis *axes;
	struct iio_channel *chans;
	struct aks_analog_key_zero_coordinate z_coord;
	int num_chans;
	bool polled;
	struct mutex amutex;
};

struct aks_multitouch_data {
	struct aks_input_device *aks_dev;
	struct input_dev *mt_input;
	int max_touch_num;	
};

int tx=0, ty=0;

#if 0
struct aks_cdev_data {
	int aks_cdev_major = 251;
	int aks_cdev_minor=0;
	dev_t aks_cdev_devno;
	struct class *aks_cdev_cls;
	struct device *aks_cdev_device;
	char* aks_cdev_name = "aks_input";
}
#endif

struct aks_input_device {
	struct device *dev;
	struct input_dev *input;
	struct aks_gpio_button_data *btn_data;
	struct aks_gpio_keys_drvdata *btn_drv_data;
	struct aks_analog_key_data *analog_data;
	struct aks_multitouch_data *mt_data;
	struct input_dev *mt_input;
	int work_mode;
};

struct aks_input_device *g_aks_dev;


#if 0
static int aks_gamepad_get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}


static const unsigned long *aks_gamepad_get_bm_events_by_type(struct input_dev *dev, int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? dev->keybit : dev->swbit;
}
#endif

static __inline int touched(struct aks_analog_key_data *adata, int x, int y) {
	if((abs(x - adata->z_coord.x) < 50) && (abs(y - adata->z_coord.y) < 50)) {
		return 0;
	} else {
		return 1;
	}
}

static __inline int moving(int old_x, int x) {
	return abs(old_x - x) > 5;
}

static inline bool str_starts_with(const char *a, const char *b)
{
	if(strncmp(a, b, strlen(b)) == 0) 
		return 1;
	return 0;
}

static void aks_gamepad_gpio_keys_quiesce_key(void *data)
{
	struct aks_gpio_button_data *bdata = data;

	if (!bdata->gpiod)
		hrtimer_cancel(&bdata->release_timer);
	if (bdata->debounce_use_hrtimer)
		hrtimer_cancel(&bdata->debounce_timer);
	else
		cancel_delayed_work_sync(&bdata->work);
}


static void aks_gamepad_gpio_keys_gpio_report_event(struct aks_gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state;

	state = bdata->debounce_use_hrtimer ?
			gpiod_get_value(bdata->gpiod) :
			gpiod_get_value_cansleep(bdata->gpiod);
	if (state < 0) {
		dev_err(input->dev.parent, "failed to get gpio state: %d\n", state);
		return;
	}

	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		dev_err(input->dev.parent, "Code=%d, state=%d\n", *bdata->code, state);

		if(state) {
			switch(button->code) {
				case BTN_DPAD_UP:
					input_report_abs(input, ABS_HAT0Y, 99);
					input_sync(input);
					break;
				case BTN_DPAD_DOWN:
					input_report_abs(input, ABS_HAT0Y, 1);
					input_sync(input);
					break;
				case BTN_DPAD_LEFT:
					input_report_abs(input, ABS_HAT0X, 1);
					input_sync(input);
					break;
				case BTN_DPAD_RIGHT:
					input_report_abs(input, ABS_HAT0X, 99);
					input_sync(input);
					break;
				default:
					input_event(input, type, *bdata->code, state);
					break;
			}
		}else {
		switch(button->code) {
				case BTN_DPAD_UP:
					input_report_abs(input, ABS_HAT0Y, 50);
					input_sync(input);
					break;
				case BTN_DPAD_DOWN:
					input_report_abs(input, ABS_HAT0Y, 50);
					input_sync(input);
					break;
				case BTN_DPAD_LEFT:
					input_report_abs(input, ABS_HAT0X, 50);
					input_sync(input);
					break;
				case BTN_DPAD_RIGHT:
					input_report_abs(input, ABS_HAT0X, 50);
					input_sync(input);
					break;
				default:
					input_event(input, type, *bdata->code, state);
					break;
			}

		}
	}
}

static void aks_gamepad_gpio_keys_debounce_event(struct aks_gpio_button_data *bdata)
{
	aks_gamepad_gpio_keys_gpio_report_event(bdata);
	input_sync(bdata->input);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static void aks_gamepad_gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct aks_gpio_button_data *bdata =
		container_of(work, struct aks_gpio_button_data, work.work);

	aks_gamepad_gpio_keys_debounce_event(bdata);
}

static enum hrtimer_restart aks_gamepad_gpio_keys_debounce_timer(struct hrtimer *t)
{
	struct aks_gpio_button_data *bdata =
		container_of(t, struct aks_gpio_button_data, debounce_timer);

	aks_gamepad_gpio_keys_debounce_event(bdata);

	return HRTIMER_NORESTART;
}

static irqreturn_t aks_gamepad_gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct aks_gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup) {
		const struct gpio_keys_button *button = bdata->button;

		pm_stay_awake(bdata->input->dev.parent);
		if (bdata->suspended  &&
		    (button->type == 0 || button->type == EV_KEY)) {
			/*
			 * Simulate wakeup key press in case the key has
			 * already released by the time we got interrupt
			 * handler to run.
			 */
			input_report_key(bdata->input, button->code, 1);
		}
	}

	if (bdata->debounce_use_hrtimer) {
		hrtimer_start(&bdata->debounce_timer,
			      ms_to_ktime(bdata->software_debounce),
			      HRTIMER_MODE_REL);
	} else {
		mod_delayed_work(system_wq,
				 &bdata->work,
				 msecs_to_jiffies(bdata->software_debounce));
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart aks_gamepad_gpio_keys_irq_timer(struct hrtimer *t)
{
	struct aks_gpio_button_data *bdata = container_of(t,
						      struct aks_gpio_button_data,
						      release_timer);
	struct input_dev *input = bdata->input;

	if (bdata->key_pressed) {
		input_event(input, EV_KEY, *bdata->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}

	return HRTIMER_NORESTART;
}

static irqreturn_t aks_gamepad_gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct aks_gpio_button_data *bdata = dev_id;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

		input_event(input, EV_KEY, *bdata->code, 1);
		input_sync(input);

		if (!bdata->release_delay) {
			input_event(input, EV_KEY, *bdata->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->release_delay)
		hrtimer_start(&bdata->release_timer,
			      ms_to_ktime(bdata->release_delay),
			      HRTIMER_MODE_REL_HARD);
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}


static int aks_gamepad_gpio_keys_setup_key(struct device *dev,
				struct input_dev *input,
				struct aks_gpio_keys_drvdata *ddata,
				const struct gpio_keys_button *button,
				int idx,
				struct fwnode_handle *child)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct aks_gpio_button_data *bdata = &ddata->data[idx];
	irq_handler_t isr;
	unsigned long irqflags;
	int irq;
	int error;
	unsigned flags;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);
	//dev_err(dev, "aks_gamepad_gpio_keys_setup_key, %s\n", desc);

	if (child) {
		bdata->gpiod = devm_fwnode_gpiod_get(dev, child, NULL, GPIOD_IN, desc);
		//bdata->gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL, child, GPIOD_IN, desc);
		if (IS_ERR(bdata->gpiod)) {
			error = PTR_ERR(bdata->gpiod);
			if (error == -ENOENT) {
				/*
				 * GPIO is optional, we may be dealing with
				 * purely interrupt-driven setup.
				 */
				bdata->gpiod = NULL;
			} else {
				if (error != -EPROBE_DEFER)
					dev_err(dev, "failed to get gpio: %d\n", error);
				return error;
			}
		}
	} else if (gpio_is_valid(button->gpio)) {
		/*
		 * Legacy GPIO number, so request the GPIO here and
		 * convert it to descriptor.
		 */
		 
		dev_err(dev, "button->gpio = %d\n", button->gpio);
		flags = GPIOF_IN;

		if (button->active_low)
			flags |= GPIOF_ACTIVE_LOW;

		error = devm_gpio_request_one(dev, button->gpio, flags, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		bdata->gpiod = gpio_to_desc(button->gpio);
		if (!bdata->gpiod)
			return -EINVAL;
	}

	if (bdata->gpiod) {
		bool active_low = gpiod_is_active_low(bdata->gpiod);
		//dev_err(g_aks_dev->dev, "%s------(%d) active low: %d\n",__FUNCTION__, __LINE__, active_low);
		if (button->debounce_interval) {
			error = gpiod_set_debounce(bdata->gpiod, button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->software_debounce = button->debounce_interval;

			/*
			 * If reading the GPIO won't sleep, we can use a
			 * hrtimer instead of a standard timer for the software
			 * debounce, to reduce the latency as much as possible.
			 */
			bdata->debounce_use_hrtimer = !gpiod_cansleep(bdata->gpiod);
		}

		if (button->irq) {
			bdata->irq = button->irq;
		} else {
			irq = gpiod_to_irq(bdata->gpiod);
			if (irq < 0) {
				error = irq;
				dev_err(dev,
					"Unable to get irq number for GPIO %d, error %d\n",
					button->gpio, error);
				return error;
			}
			bdata->irq = irq;
		}

		INIT_DELAYED_WORK(&bdata->work, aks_gamepad_gpio_keys_gpio_work_func);

		hrtimer_init(&bdata->debounce_timer,
			     CLOCK_REALTIME, HRTIMER_MODE_REL);
		bdata->debounce_timer.function = aks_gamepad_gpio_keys_debounce_timer;

		isr = aks_gamepad_gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

		switch (button->wakeup_event_action) {
		case EV_ACT_ASSERTED:
			bdata->wakeup_trigger_type = active_low ?
				IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING;
			break;
		case EV_ACT_DEASSERTED:
			bdata->wakeup_trigger_type = active_low ?
				IRQ_TYPE_EDGE_RISING : IRQ_TYPE_EDGE_FALLING;
			break;
		case EV_ACT_ANY:
		default:
			/*
			 * For other cases, we are OK letting suspend/resume
			 * not reconfigure the trigger type.
			 */
			break;
		}
	} else {
		if (!button->irq) {
			dev_err(dev, "Found button without gpio or irq\n");
			return -EINVAL;
		}

		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->release_delay = button->debounce_interval;
		hrtimer_init(&bdata->release_timer,
			     CLOCK_REALTIME, HRTIMER_MODE_REL_HARD);
		bdata->release_timer.function = aks_gamepad_gpio_keys_irq_timer;

		isr = aks_gamepad_gpio_keys_irq_isr;
		irqflags = 0;

		/*
		 * For IRQ buttons, there is no interrupt for release.
		 * So we don't need to reconfigure the trigger type for wakeup.
		 */
	}

	bdata->code = &ddata->keymap[idx];
	*bdata->code = button->code;
	input_set_capability(input, button->type ?: EV_KEY, *bdata->code);

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	error = devm_add_action(dev, aks_gamepad_gpio_keys_quiesce_key, bdata);
	if (error) {
		dev_err(dev, "failed to register quiesce action, error: %d\n",
			error);
		return error;
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = devm_request_any_context_irq(dev, bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static inline bool valid_gpio_key_node(const char* node_name) {
	if(str_starts_with(node_name, "btn") || str_starts_with(node_name, "key")) {
		return true;
	}
	return false;
}

static struct gpio_keys_platform_data *
aks_gamepad_gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	struct fwnode_handle *child;
	const char* node_name;
	int nbuttons = 0;
	int idx= 0;

	device_for_each_child_node(dev, child) {
		fwnode_property_read_string(child, "name", &node_name);
		
		//dev_err(dev, "node_name = %s\n", node_name);
		if(valid_gpio_key_node(node_name)) {
			nbuttons++;
		}
	}

	if (nbuttons == 0) {
		return ERR_PTR(-ENODEV);
		dev_err(dev, "got sub node count NULL\n");
	}

	//dev_err(dev, "got sub node count = %d\n", nbuttons);

	pdata = devm_kzalloc(dev, sizeof(*pdata) + nbuttons * sizeof(*button), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	button = (struct gpio_keys_button *)(pdata + 1);

	pdata->buttons = button;
	pdata->nbuttons = nbuttons;

	pdata->rep = device_property_read_bool(dev, "autorepeat");

	device_property_read_string(dev, "label", &pdata->name);

	//dev_err(dev, "--label=%s, rep=%d\n", pdata->name, pdata->rep);
	device_for_each_child_node(dev, child) {
		fwnode_property_read_string(child, "name", &node_name);
		if(!valid_gpio_key_node(node_name)) {
			//dev_err(dev, "Ignore node: %s\n", node_name);
			continue;
		}

		if (is_of_node(child)) {
			button->irq = irq_of_parse_and_map(to_of_node(child), 0);
		}

		if (fwnode_property_read_u32(child, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

		fwnode_property_read_string(child, "label", &button->desc);

		if (fwnode_property_read_u32(child, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = 
			fwnode_property_read_bool(child, "wakeup-source") ||
			/* legacy name */
			fwnode_property_read_bool(child, "gpio-key,wakeup");

		fwnode_property_read_u32(child, "wakeup-event-action", &button->wakeup_event_action);

		button->can_disable = fwnode_property_read_bool(child, "linux,can-disable");

		if (fwnode_property_read_u32(child, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;
		//dev_err(dev, "--[%d] irq=%d, keycode=%d, label=%s, type=%d\n", idx, button->irq, button->code, button->desc, button->type);
		button++;
		idx++;
	}
	return pdata;
}

static int aks_gamepad_config_gpio_keys(struct aks_input_device* aks_dev) {
	struct gpio_keys_platform_data *pdata = NULL;
	struct device* dev = aks_dev->dev;

	struct fwnode_handle *child = NULL;
	struct aks_gpio_keys_drvdata *ddata;
	struct input_dev *input = aks_dev->input;
	const char* node_name;
	int i, error;
	int wakeup = 0;

	pdata = aks_gamepad_gpio_keys_get_devtree_pdata(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	ddata = devm_kzalloc(dev, struct_size(ddata, data, pdata->nbuttons), GFP_KERNEL); 

	if (!ddata) {
		dev_err(dev, "failed to allocate button drive data\n");
		return -ENOMEM;
	}

	ddata->keymap = devm_kcalloc(dev, pdata->nbuttons, sizeof(ddata->keymap[0]), GFP_KERNEL);
	if (!ddata->keymap)
		return -ENOMEM;

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	input->keycode = ddata->keymap;
	input->keycodesize = sizeof(ddata->keymap[0]);
	input->keycodemax = pdata->nbuttons;
	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	dev_err(g_aks_dev->dev, "%s------(%d)\n",__FUNCTION__, __LINE__);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];

		child = device_get_next_child_node(dev, child);
		if (!child) {
			dev_err(dev, "missing child device node for entry %d\n", i);
			return -EINVAL;
		}

		fwnode_property_read_string(child, "name", &node_name);
		if(!valid_gpio_key_node(node_name)) {
			dev_err(dev, "Ignore node: %s\n", node_name);
			continue;
		}

		error = aks_gamepad_gpio_keys_setup_key(dev, input, ddata, button, i, child);
		if (error) {
			fwnode_handle_put(child);
			dev_err(g_aks_dev->dev, "error: %d for setup %s, but continue...\n", error, node_name);
			//success = false;
			//return error;
		}

		if (button->wakeup)
			wakeup = 1;
	}

	fwnode_handle_put(child);


	device_init_wakeup(dev, wakeup);
	aks_dev->btn_drv_data = ddata;

	return 0;
}

static void ask_gamepad_gpio_keys_report_state(struct aks_gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct aks_gpio_button_data *bdata = &ddata->data[i];
		if (bdata->gpiod)
			aks_gamepad_gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}


static int ask_gamepad_gpio_keys_open(struct aks_input_device *aks_dev)
{
	struct aks_gpio_keys_drvdata *ddata = aks_dev->btn_drv_data;
	struct input_dev *input = aks_dev->input;
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	
	int error;
	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error) {
			dev_err(g_aks_dev->dev, "---> %s(%d) error: %d\n",__FUNCTION__, __LINE__, error);
			return error;
		}
	}

	/* Report current state of buttons that are connected to GPIOs */
	ask_gamepad_gpio_keys_report_state(ddata);

	return 0;
}

static void ask_gamepad_gpio_keys_close(struct aks_input_device *aks_dev)
{
	struct input_dev *input = aks_dev->input;
	struct aks_gpio_keys_drvdata *ddata = aks_dev->btn_drv_data;
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}


static int aks_gamepad_analog_keys_handle(const void *data, void *private)
{
    struct aks_analog_key_data *joy = private;
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


static void aks_gamepad_analog_keys_cleanup(void *data)
{
    iio_channel_release_all_cb(data);
}


static void aks_gamepad_analog_keys_poll(struct input_dev *input)
{
	//s32 i=0, val=0;//, offset_val;
	//struct adc_joystick *joy = input_get_drvdata(input);
	//iio_read_channel_raw(&joy->chans[i], &val);
#if 1
    struct aks_input_device *joy = input_get_drvdata(input);
	struct aks_analog_key_axis *axis;
    s32 i, val, offset_val;
	int id;
	int _x, _y;
	bool process = false;

	mutex_lock(&joy->analog_data->amutex);
    for (i = 0; i < joy->analog_data->num_chans; i++) {
		axis = &joy->analog_data->axes[i];
        iio_read_channel_raw(&joy->analog_data->chans[i], &val);
        //dev_err(g_aks_dev->dev, "code=%d, channel[%d]=%d\n",joy->analog_data->axes[i].code, joy->analog_data->chans[i].channel->address, val);

		if(joy->work_mode == AKS_GAMEPAD_WORK_MODE_DIRECT) {

			#if 1
	        switch(axis->code) {
		        case ABS_X:
		        case ABS_Z:
		        case ABS_Y:
		        case ABS_RZ:					
					val = (val < axis->cali.rang_min_calied) ? axis->cali.rang_min_calied : val;
					val = (val > axis->cali.rang_max_calied) ? axis->cali.rang_max_calied : val;
					offset_val = val;
		            #if 0
		            if(val < axis->cali.zero_pos) {
						offset_val = (axis->cali.zero_pos - (((axis->cali.zero_pos - val) * axis->cali.ratio_neg) >> AKS_GAMEPAD_ENLARGE_SHIFT));
					} else if (val > axis->cali.zero_pos) {
		                offset_val = (axis->cali.zero_pos + (((val - axis->cali.zero_pos) * axis->cali.ratio_pos) >> AKS_GAMEPAD_ENLARGE_SHIFT));
		            }
					#else 
		            if(val >= axis->cali.physic_zero_pos) {
						offset_val = axis->cali.logic_zero_pos + ((axis->cali.ratio_pos * (val - axis->cali.physic_zero_pos)) >> AKS_GAMEPAD_ENLARGE_SHIFT);
					} else {
		                offset_val = axis->cali.logic_zero_pos - ((axis->cali.ratio_neg * (axis->cali.physic_zero_pos - val)) >> AKS_GAMEPAD_ENLARGE_SHIFT);
		            }
					#endif
		            break;
		        case ABS_BRAKE:
		        case ABS_GAS:
					val = (val < axis->cali.physic_zero_pos) ? axis->cali.physic_zero_pos : val;
					offset_val = axis->cali.logic_zero_pos + ((axis->cali.ratio_pos * (val - axis->cali.physic_zero_pos)) >> AKS_GAMEPAD_ENLARGE_SHIFT);
		            break;
		        default:
		            break;
	        }
			#endif
			//if(axis->code == ABS_RZ)
	        //dev_err(g_aks_dev->dev, "code=%d, val=%d, offset_val=%d\n", axis->code, val, offset_val);
	        input_report_abs(input, axis->code, offset_val);
		}else if (joy->work_mode == AKS_GAMEPAD_WORK_MODE_TOUCH) {
			offset_val = val;
			if(axis->code == ABS_X || axis->code == ABS_Y) {
		        switch(axis->code) {
			        case ABS_X:
						id = AKS_GAMEPAD_MT_ID_JS_L;
						if(moving(tx, offset_val)) {
							process = true;
						}
						tx = offset_val;
						break;
					case ABS_Y:
						id = AKS_GAMEPAD_MT_ID_JS_L;
						if(moving(ty, offset_val)) {
							process = true;
						}
						ty = offset_val;
						break;
		        }

				if(process) {
					if(touched(joy->analog_data, tx, ty)) {
						_x = tx;
						_y = ty;
						swap(_x, _y);
						input_mt_slot(joy->mt_input, id);
						input_mt_report_slot_state(joy->mt_input, MT_TOOL_FINGER, true);
						input_report_abs(joy->mt_input, ABS_MT_POSITION_X, _x);
						input_report_abs(joy->mt_input, ABS_MT_POSITION_Y, _y);
						input_report_abs(joy->mt_input, ABS_MT_TOUCH_MAJOR, 0x6000);
						input_report_key(joy->mt_input, BTN_TOUCH, 1);
					} else {
						input_mt_slot(joy->mt_input, id);
						input_mt_report_slot_state(joy->mt_input, MT_TOOL_FINGER, false);
						input_report_key(joy->mt_input, BTN_TOUCH, 0);
					}
				}
			}
		}
    }
	if(joy->work_mode == AKS_GAMEPAD_WORK_MODE_DIRECT) {
		input_sync(input);
	} else if (joy->work_mode == AKS_GAMEPAD_WORK_MODE_TOUCH) {
		input_sync(joy->mt_input);
	}

	mutex_unlock(&joy->analog_data->amutex);
#endif
}

static void aks_gamepad_analog_init_zero_coordinate(struct aks_analog_key_data *adata, int code, int val) {
	switch(code) {
		case ABS_X:
			adata->z_coord.x = val;
			break;
		case ABS_Y:
			adata->z_coord.y = val;
			break;
		case ABS_Z:
			adata->z_coord.z = val;
			break;
		case ABS_RZ:
			adata->z_coord.rz = val;
			break;
		default:
			break;
	}
}

static int aks_gamepad_analog_init_cali_data(struct aks_analog_key_data *adata)
{
    int ret=0, i, val;

	if (!adata) {
		dev_err(g_aks_dev->dev, "%s(%d) error: adata NULL!\n",__FUNCTION__, __LINE__);
		return -EINVAL;
	}

    for (i = 0; i < adata->num_chans; i++) {
        ret = iio_read_channel_raw(&adata->chans[i], &val);
	    if (ret != IIO_VAL_INT) {
        	dev_err(g_aks_dev->dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, ret);
        	return ret;
    	} else {
			adata->axes[i].cali.physic_zero_pos = val;
			aks_gamepad_analog_init_zero_coordinate(adata, adata->axes[i].code, val);
        	//dev_err(g_aks_dev->dev, "code=%d, channel[%d], zero=%d\n", adata->axes[i].code, adata->chans[i].channel->address, adata->axes[i].cali.zero_pos);
		}
    }

    return 0;
}

static void aks_gamepad_analog_calibrate(struct aks_analog_key_data *adata)
{
	struct aks_analog_key_axis* axis;
	int i=0; 
	int diff_neg, diff_pos;

	for(i=0; i < adata->num_chans; i++) {
		axis = &adata->axes[i];

		if(axis->code == ABS_X || axis->code == ABS_Y || axis->code == ABS_Z|| axis->code == ABS_RZ) {
			#if 1
			axis->cali.rang_min_calied = axis->cali.rang_min;
			axis->cali.rang_max_calied = axis->cali.rang_max;

			//The logic center is in the middle of range
			axis->cali.logic_zero_pos = (axis->cali.rang_min_calied + axis->cali.rang_max_calied) >> 1;
			diff_pos = abs(axis->cali.rang_max - axis->cali.physic_zero_pos);
			diff_neg = abs(axis->cali.physic_zero_pos - axis->cali.rang_min);

			axis->cali.ratio_pos = ((((axis->cali.rang_max_calied - axis->cali.rang_min_calied) >> 1) << AKS_GAMEPAD_ENLARGE_SHIFT) / diff_pos);
			axis->cali.ratio_neg = ((((axis->cali.rang_max_calied - axis->cali.rang_min_calied) >> 1) << AKS_GAMEPAD_ENLARGE_SHIFT) / diff_neg);

			#else
		
			diff_neg = abs(axis->cali.zero_pos - axis->cali.rang_min);
			diff_pos = abs(axis->cali.rang_max - axis->cali.zero_pos);
			if(diff_neg > diff_pos) {
				axis->cali.rang_max_calied = axis->cali.zero_pos + diff_neg;
				axis->cali.rang_min_calied = axis->cali.rang_min;
				axis->cali.ratio_pos = ((diff_neg << AKS_GAMEPAD_ENLARGE_SHIFT) /diff_pos);
				axis->cali.ratio_neg = 1 << AKS_GAMEPAD_ENLARGE_SHIFT;
			} else if (diff_neg < diff_pos) {
				axis->cali.rang_max_calied = axis->cali.rang_max;
				axis->cali.rang_min_calied = axis->cali.zero_pos - diff_pos;
				axis->cali.ratio_pos = 1 << AKS_GAMEPAD_ENLARGE_SHIFT;
				axis->cali.ratio_neg = ((diff_pos << AKS_GAMEPAD_ENLARGE_SHIFT) / diff_neg);
			} else {
				axis->cali.rang_max_calied = axis->cali.rang_max;
				axis->cali.rang_min_calied = axis->cali.rang_min;
				axis->cali.ratio_neg = axis->cali.ratio_neg = 1 << AKS_GAMEPAD_ENLARGE_SHIFT;
			}
			#endif
			#if 0
			diff_pos = abs(axis->cali.rang_max - axis->cali.physic_zero_pos);
			diff_neg = abs(axis->cali.physic_zero_pos - axis->cali.rang_min);
			if(diff_neg > diff_pos) {
				axis->cali.rang_max_calied = axis->cali.rang_max;
				axis->cali.rang_min_calied = axis->cali.physic_zero_pos - diff_pos;
			} else if (diff_neg < diff_pos) {
				axis->cali.rang_max_calied = axis->cali.physic_zero_pos + diff_neg;
				axis->cali.rang_min_calied = axis->cali.rang_min;
			} else {
				axis->cali.rang_max_calied = axis->cali.rang_max;
				axis->cali.rang_min_calied = axis->cali.rang_min;
			}
			#endif
		} else if(axis->code == ABS_BRAKE || axis->code == ABS_GAS) {
			#if 0
			axis->cali.rang_min_calied = axis->cali.physic_zero_pos;
			axis->cali.rang_max_calied = axis->cali.rang_max;
			axis->cali.ratio_neg = axis->cali.ratio_neg = 1;
			#else
			diff_pos = axis->cali.rang_max - axis->cali.physic_zero_pos;
			axis->cali.ratio_pos = ((500 << AKS_GAMEPAD_ENLARGE_SHIFT) / diff_pos);
			axis->cali.rang_min_calied = 1;
			axis->cali.rang_max_calied = 501;
			#endif
		}
		dev_err(g_aks_dev->dev, "code=%d, min=%d, max=%d, cali_min=%d, cali_max=%d, ratio_pos=%d, ratio_neg=%d, pc=%d, lc=%d\n", \
			axis->code, axis->cali.rang_min, axis->cali.rang_max, \
			axis->cali.rang_min_calied, axis->cali.rang_max_calied, \
			axis->cali.ratio_pos, axis->cali.ratio_neg, \
			axis->cali.physic_zero_pos, axis->cali.logic_zero_pos);
	}
}

static int aks_gamepad_analog_update_paras(struct aks_analog_key_data *adata) {
	
	int i=0;
	if(!adata) {
		return -EINVAL;
	}

	//update range
	for(i=0; i < adata->num_chans; i++) {
		if(adata->axes[i].code == ABS_X || adata->axes[i].code == ABS_RZ) {
			input_set_abs_params(adata->input, adata->axes[i].code, \
				adata->axes[i].cali.rang_max_calied, adata->axes[i].cali.rang_min_calied, \
				adata->axes[i].fuzz, adata->axes[i].flat);

		} else {
			input_set_abs_params(adata->input, adata->axes[i].code, \
				adata->axes[i].cali.rang_min_calied, adata->axes[i].cali.rang_max_calied, \
				adata->axes[i].fuzz, adata->axes[i].flat);
		}

		dev_err(g_aks_dev->dev, "code=%d, cali_min=%d, cali_max=%d\n", \
			adata->axes[i].code, adata->axes[i].cali.rang_min_calied, adata->axes[i].cali.rang_max_calied);
	}
	return 0;
}

static int aks_gamepad_analog_setup_extra(struct aks_analog_key_data *adata) {
	if(!adata) {
		return -EINVAL;
	}

	//simulate DPAD to analog data, Only -1, 0, and 1
	input_set_abs_params(adata->input, ABS_HAT0X,1, 99, 3, 3);
	input_set_abs_params(adata->input, ABS_HAT0Y,1, 99, 3, 3);
	return 0;
}


static int aks_gamepad_analog_keys_set_axes(struct device *dev, struct aks_analog_key_data *joy)
{
    struct aks_analog_key_axis *axes;
    struct fwnode_handle *child;

    int num_axes=0, error, i;
	const char* node_name;

	bool forbid = false;

	device_for_each_child_node(dev, child) {
		fwnode_property_read_string(child, "name", &node_name);
		if(str_starts_with(node_name, "axis")) {
			num_axes++;
		}
	}

	dev_err(dev, "Got %d child nodes for %d channels\n",num_axes, joy->num_chans);

    if (!num_axes) {
        dev_err(dev, "Unable to find child nodes\n");
        return -EINVAL;
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
        error = fwnode_property_read_u32(child, "index", &i);
        if (error) {
			continue; // ignore the gpio buttons
            //goto err_fwnode_put;
        }

        if (i >= num_axes) {
            error = -EINVAL;
            dev_err(dev, "No matching axis for reg %d\n", i);
            goto err_fwnode_put;
        }

        error = fwnode_property_read_u32(child, "linux,code", &axes[i].code);
        if (error) {
            dev_err(dev, "linux,code invalid or missing\n");
            goto err_fwnode_put;
        }

        error = fwnode_property_read_u32_array(child, "abs-range", axes[i].range, 2);

        if (error) {
            dev_err(dev, "abs-range invalid or missing\n");
            goto err_fwnode_put;
        }

        axes[i].cali.rang_min = min(axes[i].range[0], axes[i].range[1]);
        axes[i].cali.rang_max = max(axes[i].range[0], axes[i].range[1]);

        fwnode_property_read_u32(child, "abs-fuzz", &axes[i].fuzz);
        fwnode_property_read_u32(child, "abs-flat", &axes[i].flat);
        dev_err(dev, "%s(%d) linux,code=<%d>, abs-range= %d ~ %d, abs-fuzz= %d, abs-flat=%d!\n",\
			__FUNCTION__, __LINE__, \
            axes[i].code, axes[i].range[0], axes[i].range[1], axes[i].fuzz, axes[i].flat);

        input_set_abs_params(joy->input, axes[i].code, axes[i].range[0], axes[i].range[1], axes[i].fuzz, axes[i].flat);
        input_set_capability(joy->input, EV_ABS, axes[i].code);
    }

    input_set_capability(joy->input, EV_KEY, BTN_JOYSTICK); //Mark as joystick device in Android Event Hub.

	joy->axes = axes;

	error = aks_gamepad_analog_init_cali_data(joy);
	if(error) {
		return -EINVAL;
	}
	if(!forbid) {
		aks_gamepad_analog_calibrate(joy);
		aks_gamepad_analog_update_paras(joy);
	}
	aks_gamepad_analog_setup_extra(joy);
    return 0;

err_fwnode_put:
    fwnode_handle_put(child);
    return error;
}


#if 0

int update_task_thread(void *data)
{
	struct aks_input_device *joy = data;
	int i=0;

	struct aks_analog_key_data *adata = joy->analog_data;

	msleep(8300);

	for(i=0; i<adata->num_chans; i++) {
		input_set_abs_params(joy->input, adata->axes[i].code, adata->axes[i].range[0]-100, adata->axes[i].range[1]+500, adata->axes[i].fuzz, adata->axes[i].flat);
	}

	return 0;
}

struct task_struct *th;

static int update_thread_init(void *data)
{

	dev_err(g_aks_dev->dev, "%s(%d) --------------- \n",__FUNCTION__, __LINE__);
	th = kthread_create(update_task_thread, data, "update_task_thread");

	if (th) {
		wake_up_process(th);
	} else {
		dev_err(g_aks_dev->dev, "%s(%d) --------------- \n",__FUNCTION__, __LINE__);
	}

	return 0;
}
#endif

static int aks_gamepad_config_analog_keys(struct aks_input_device* aks_dev) {
	struct aks_analog_key_data *adata;
	struct device* dev = aks_dev->dev;
	int error;
	int bits;
	int i;
	//unsigned int poll_interval;

	adata = devm_kzalloc(dev, sizeof(*adata), GFP_KERNEL);
	if (!adata) {
		dev_err(dev, "failed to allocate anolog data\n");
		return -ENOMEM;
	}

	adata->input = aks_dev->input;
	aks_dev->analog_data = adata;

	mutex_init(&adata->amutex);

	adata->polled = AKS_GAMEPAD_ANALOG_POLL;

	adata->chans = devm_iio_channel_get_all(dev);
	if (IS_ERR(adata->chans)) {
		error = PTR_ERR(adata->chans);
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Unable to get IIO channels");
		return error;
	}

	for (i = 0; adata->chans[i].indio_dev; i++) {
		if (adata->polled)
			continue;
		bits = adata->chans[i].channel->scan_type.storagebits;
		if (!bits || bits > 16) {
			dev_err(dev, "Unsupported channel storage size\n");
			return -EINVAL;
		}
		if (bits != adata->chans[0].channel->scan_type.storagebits) {
			dev_err(dev, "Channels must have equal storage size\n");
			return -EINVAL;
		}
	}
	adata->num_chans = i;

	error = aks_gamepad_analog_keys_set_axes(dev, adata);
	if (error)
		return error;

	if (adata->polled) {
		input_setup_polling(aks_dev->input, aks_gamepad_analog_keys_poll);
		input_set_poll_interval(aks_dev->input, AKS_GAMEPAD_ANALOG_POLL_INTERVAL);
	} else {
		adata->buffer = iio_channel_get_all_cb(dev, aks_gamepad_analog_keys_handle, adata);
		if (IS_ERR(adata->buffer)) {
			dev_err(dev, "Unable to allocate callback buffer\n");
			return PTR_ERR(adata->buffer);
		}

		error = devm_add_action_or_reset(dev, aks_gamepad_analog_keys_cleanup, adata->buffer);
		if (error)  {
			dev_err(dev, "Unable to add action\n");
			return error;
		}
	}

	return 0;
}


static int aks_gamepad_configure_mt_dev(struct aks_input_device *aks_dev)
{
	struct aks_multitouch_data *mdata;
	struct device* dev = aks_dev->dev;
	int error = 0;
	mdata = devm_kzalloc(dev, sizeof(*mdata), GFP_KERNEL);
	if (!mdata) {
		dev_err(dev, "failed to allocate moutch touch data\n");
		return -ENOMEM;
	}

	//mdata->input = aks_dev->input;
	aks_dev->mt_data = mdata;

	mdata->max_touch_num = AKS_GAMEPAD_MT_MAX_CONTACTS;


	mdata->mt_input = devm_input_allocate_device(dev);
	if (!mdata->mt_input) {
		dev_err(dev, "Failed to allocate moutch touch input device.");
		return -ENOMEM;
	}

	mdata->mt_input->name = "AKS TouchScreen";
	mdata->mt_input->phys = "input/input1";
	mdata->mt_input->id.bustype = BUS_HOST;
	mdata->mt_input->id.vendor = 0x2212;
	mdata->mt_input->id.product = 0x0011;
	mdata->mt_input->id.version = 0x0016;

	dev_err(dev, "%s------(%d)\n",__FUNCTION__, __LINE__);

	set_bit(EV_SYN, mdata->mt_input->evbit);
	set_bit(EV_KEY, mdata->mt_input->evbit);
	set_bit(EV_ABS, mdata->mt_input->evbit);

	set_bit(BTN_TOUCH, mdata->mt_input->keybit);
	set_bit(BTN_TOOL_FINGER, mdata->mt_input->keybit);
	set_bit(INPUT_PROP_DIRECT, mdata->mt_input->propbit);
	/* set input parameters */
	input_set_abs_params(mdata->mt_input, ABS_MT_POSITION_X, 0, AKS_GAMEPAD_MT_MAX_WIDTH - 1, 0, 0);
	input_set_abs_params(mdata->mt_input, ABS_MT_POSITION_Y, 0, AKS_GAMEPAD_MT_MAX_HEIGHT - 1, 0, 0);
	input_set_abs_params(mdata->mt_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_mt_init_slots(mdata->mt_input, AKS_GAMEPAD_MT_MAX_CONTACTS, INPUT_MT_DIRECT);

	aks_dev->mt_input = mdata->mt_input;
	mdata->aks_dev = aks_dev;
	error = input_register_device(aks_dev->mt_input);
	if (error) {
		dev_err(dev, "Failed to register moutch touch input device: %d", error);
		return error;
	}

	return 0;
}


static int aks_gamepad_input_open(struct input_dev *input)
{
	struct aks_input_device *aks_dev = input_get_drvdata(input);
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);

	ask_gamepad_gpio_keys_open(aks_dev);

	return 0;
}

static void aks_gamepad_input_close(struct input_dev *input)
{
	struct aks_input_device *aks_dev = input_get_drvdata(input);

	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);
	ask_gamepad_gpio_keys_close(aks_dev);
}

static int aks_gamepad_config_input_dev(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int error;

	struct aks_input_device *aks_dev = platform_get_drvdata(pdev);

	if(!aks_dev) {
		dev_err(dev, "No aksys dev platform data\n");
		return -EFAULT;
	}
	
	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	
	input_set_drvdata(input, aks_dev);

	input->name = "AKS Gamepad";
	input->phys = "aks/input0";
	input->dev.parent = dev;
	input->open = aks_gamepad_input_open;
	input->close = aks_gamepad_input_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x2212;
	input->id.product = 0x0010;
	input->id.version = 0x0016;

	aks_dev->input = input;

	error = aks_gamepad_config_gpio_keys(aks_dev);
	if (error) {
		dev_err(dev, "Unable to config gpio keys error: %d\n", error);
		return error;
	}

	error = aks_gamepad_config_analog_keys(aks_dev);
	if (error) {
		dev_err(dev, "Unable to config analog keys error: %d\n", error);
		return error;
	}

	error = aks_gamepad_configure_mt_dev(aks_dev);
	if (error) {
		dev_err(dev, "Unable to config multi touch error: %d\n", error);
		return error;
	}

	error = input_register_device(input);
	//update_thread_init(aks_dev);

	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n", error);
		return error;
	}

	return 0;

}
//endif

//static dev_t aks_file_dev_no;

#if 1
struct aks_cdev_data
{
  char *data;
  unsigned long size;
};

#define AKS_CDEV_MAJOR 190

#define AKS_CDEV_NR_DEVS 2
#define AKS_CDEV_SIZE 4096

static int aks_cdev_major = AKS_CDEV_MAJOR;
 
module_param(aks_cdev_major, int, S_IRUGO);
 
struct aks_cdev_data *aks_cdev_data_p;
struct cdev aks_cdev;

struct class *aks_cdev_cls;
struct device *aks_cdev_device;
char* aks_cdev_name = "aks_input";
dev_t aks_cdev_no;

#endif

int aks_input_cdev_open(struct inode *inode, struct file *filp)
{
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);
	return 0;
}

int aks_input_cdev_release(struct inode *inode, struct file *filp)
{
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);
	return 0;
}

long aks_input_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int ioarg = 0;

    if (_IOC_TYPE(cmd) != AKS_IOC_MAGIC) 
        return -EINVAL;
    if (_IOC_NR(cmd) > AKS_IOC_MAXNR) 
        return -EINVAL;
#if 0
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
    if (err) 
        return -EFAULT;
#endif

    switch(cmd) {

      case AKS_IOC_INIT:
		  dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);
        break;

      case AKS_IOC_GETDATA:
        ioarg = 1101;
        ret = __put_user(ioarg, (int *)arg);
        break;

      case AKS_IOC_SETDATA:
        ret = __get_user(ioarg, (int *)arg);
        dev_err(g_aks_dev->dev,"<--- In Kernel MEMDEV_IOCSETDATA ioarg = %d --->\n\n",ioarg);
        break;

      case AKS_IOC_GETMODE:
        ioarg = g_aks_dev->work_mode;
        ret = __put_user(ioarg, (int *)arg);
        break;

      case AKS_IOC_SETMODE:
        ret = __get_user(ioarg, (int *)arg);
		if(ioarg == AKS_GAMEPAD_WORK_MODE_DIRECT || ioarg == AKS_GAMEPAD_WORK_MODE_TOUCH) {
			dev_err(g_aks_dev->dev, "---> %s(%d) change work mode to -> %d\n",__FUNCTION__, __LINE__, ioarg);
			g_aks_dev->work_mode = ioarg;
		} else {
			dev_err(g_aks_dev->dev, "---> %s(%d) Invalid work mode -> %d\n",__FUNCTION__, __LINE__, ioarg);
		}
        break;

      default:  
        return -EINVAL;
    }
    return ret;

}


static const struct file_operations aks_input_cdev_fops =
{
  .owner = THIS_MODULE,
  .open = aks_input_cdev_open,
  .release = aks_input_cdev_release,
  .unlocked_ioctl = aks_input_cdev_ioctl,
};

static int aks_gamepad_cdev_init_ioctl() {
	#if 0

	int aks_cdev_major = 251;
	int aks_cdev_minor=0;
	dev_t aks_cdev_devno;
	struct class *aks_cdev_cls;
	struct device *aks_cdev_device;
	char* aks_cdev_name = "aks_input";
	int ret = 0;

	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);

 
	aks_cdev_devno = MKDEV(aks_cdev_major,aks_cdev_minor);
	ret = register_chrdev(aks_cdev_major, aks_cdev_name ,&aks_cdev_fops);
	if(ret) {
		dev_err(g_aks_dev->dev, "---> %s(%d), Unable to register char device: %d\n",__FUNCTION__, __LINE__, ret);
		return ret;
	}
 
	aks_cdev_cls = class_create(THIS_MODULE, "aks_cdev_class");
	if(IS_ERR(aks_cdev_cls)) {
		
		dev_err(g_aks_dev->dev, "---> %s(%d), Unable to create class\n",__FUNCTION__, __LINE__);
		unregister_chrdev(aks_cdev_major, aks_cdev_name);
		return -EBUSY;
	}
	aks_cdev_device = device_create(aks_cdev_cls, NULL, aks_cdev_devno, NULL, aks_cdev_name);//mknod /dev/hello
	if(IS_ERR(aks_cdev_device)) {
		
		dev_err(g_aks_dev->dev, "---> %s(%d), Unable to create device\n",__FUNCTION__, __LINE__);
		class_destroy(aks_cdev_cls);
		unregister_chrdev(aks_cdev_major, aks_cdev_name);
		return -EBUSY;
	}
	return 0;

	
	#else
	int result;
	int i;

	aks_cdev_no = MKDEV(aks_cdev_major, 0);
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);

	if (aks_cdev_major) {
		result = register_chrdev_region(aks_cdev_no, 2, "aks_cdev");
	} else {
		result = alloc_chrdev_region(&aks_cdev_no, 0, 2, "aks_cdev");
		aks_cdev_major = MAJOR(aks_cdev_no);
	}

	if (result < 0) {
		dev_err(g_aks_dev->dev, "---> %s(%d): error: %d\n",__FUNCTION__, __LINE__,result);
		return result;
	}

	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);

	cdev_init(&aks_cdev, &aks_input_cdev_fops);
	aks_cdev.owner = THIS_MODULE;
	aks_cdev.ops = &aks_input_cdev_fops;

	cdev_add(&aks_cdev, MKDEV(aks_cdev_major, 0), AKS_CDEV_NR_DEVS);

	aks_cdev_data_p = kmalloc(AKS_CDEV_NR_DEVS * sizeof(struct aks_cdev_data), GFP_KERNEL);
	if (!aks_cdev_data_p) {
		dev_err(g_aks_dev->dev, "---> %s(%d) error\n",__FUNCTION__, __LINE__);
		result =  - ENOMEM;
		goto fail_malloc;
	}

	memset(aks_cdev_data_p, 0, sizeof(struct aks_cdev_data));

	for (i=0; i < AKS_CDEV_NR_DEVS; i++) 
	{
		aks_cdev_data_p[i].size = AKS_CDEV_SIZE;
		aks_cdev_data_p[i].data = kmalloc(AKS_CDEV_SIZE, GFP_KERNEL);
		memset(aks_cdev_data_p[i].data, 0, AKS_CDEV_SIZE);
	}
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);


	aks_cdev_cls = class_create(THIS_MODULE, aks_cdev_name);
	if(IS_ERR(aks_cdev_cls)) {
		dev_err(g_aks_dev->dev, "---> %s(%d), Unable to create class\n",__FUNCTION__, __LINE__);
		result = -EBUSY;
		goto fail_malloc;
	}
	aks_cdev_device = device_create(aks_cdev_cls, NULL, aks_cdev_no, NULL, aks_cdev_name);//mknod /dev/hello
	if(IS_ERR(aks_cdev_device)) {
		dev_err(g_aks_dev->dev, "---> %s(%d), Unable to create device\n",__FUNCTION__, __LINE__);
		class_destroy(aks_cdev_cls);
		result = -EBUSY;
		goto fail_malloc;
	}

	return 0;

	fail_malloc: 
	unregister_chrdev_region(aks_cdev_no, 1);

	return result;
	#endif
}

static int aks_gamepad_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

    g_aks_dev = devm_kzalloc(dev, sizeof(*g_aks_dev), GFP_KERNEL);
    if (!g_aks_dev) {
        dev_err(dev, "%s(%d) failed allocate aks input dev!\n",__FUNCTION__, __LINE__);
        return -ENOMEM;
    } else {
		dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);
	}

	g_aks_dev->dev = dev;
	g_aks_dev->work_mode = CURRENT_WORK_MODE;
	dev_set_drvdata(dev, g_aks_dev);
	platform_set_drvdata(pdev, g_aks_dev);

	aks_gamepad_config_input_dev(pdev);

	aks_gamepad_cdev_init_ioctl();
	
    return 0;
}

static int aks_gamepad_remove(struct platform_device *pdev)
{
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);

	device_destroy(aks_cdev_cls, aks_cdev_no);
	class_destroy(aks_cdev_cls);	

	cdev_del(&aks_cdev);
 	kfree(aks_cdev_data_p);
  	unregister_chrdev_region(MKDEV(aks_cdev_major, 0), 2); /*释放设备号*/

    return 0;
}


static const struct of_device_id adc_joystick_of_match[] = {
    { .compatible = "aks,joystick_mt", },
    { }
};
MODULE_DEVICE_TABLE(of, adc_joystick_of_match);

static struct platform_driver adc_joystick_driver = {
    .driver = {
        .name = "aks-adc-joystick",
        .of_match_table = adc_joystick_of_match,
    },
    .probe = aks_gamepad_probe,
    .remove = aks_gamepad_remove,
};
module_platform_driver(adc_joystick_driver);

MODULE_DESCRIPTION("Input driver for Aksys.co.kr All in one device.");
MODULE_AUTHOR("Daniel <daniel@aksys.co.kr>");
MODULE_LICENSE("GPL");
