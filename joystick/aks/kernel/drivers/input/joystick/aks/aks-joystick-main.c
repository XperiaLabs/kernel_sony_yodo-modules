
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Gamepad, Touchpad, Mouse mode for AKSYS hyperion project
 *  Copyright (c) AKS 2022
 */

/* #define DEBUG */

#include <linux/input.h>
#include <linux/slab.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>

#include "hid-ids.h"
#include "aks-joystick-controller.h"

#define TOUCH_SCREEN 4 // (0x09, 0x04)
#define GAMEPAD 5 // (0x09, 0x05)

#define AKSYS_QRD_USB 0x1000
#define AKSYS_ANDROID_BT 0x0016

#define DEVICE_NAME "AKSYS QRD"

#define GAMEPAD_MODE 1
#define TOUCHPAD_MODE 2
#define MOUSE_MODE 3

#ifdef CONFIG_AKSYS_QRD_FF

struct aksys_dev {

	struct input_dev *game_input_dev;
	struct input_dev *touch_input_dev;
	struct input_dev *mouse_input_dev;
	struct hid_device *hdev;

	//u8 left;
	//u8 right;

};

static const struct {int x; int y;} aksys_hat_mapping[] = {
    {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1},
	{0, 0}
};

static const struct {int x; int y;} aksys_xy_mapping[] = {
    {100, 100}, {150, 150}, {200, 200}, {250, 250}, {300, 300}, {350, 350}, {400, 400}, {450, 450},
	{500, 500}, {550, 550}, {600, 600}, {650, 650}, {700, 700}, {750, 750}, {800, 800}, {850, 850}
};
	

struct aks_joystick_ff_device {
	struct hid_report *report;
};

static int aks_joystick_play(struct input_dev *dev, void *data,
			 struct ff_effect *effect)
{
	u32 left, right;
	//u8 buf[4];

	struct aksys_dev *aksys_dev = input_get_drvdata(dev);

	struct aks_joystick_ff_device *ff = data;

	if(aksys_dev == NULL) {
		hid_err(aksys_dev->hdev, "Can not get hid device ......\n");
		return -1;
	}

	if(ff == NULL) {
		hid_err(aksys_dev->hdev, "Can not get hid ff data ......\n");
		return -1;
	}
	
	left = effect->u.rumble.strong_magnitude;
	right = effect->u.rumble.weak_magnitude;
	
	left = left / 256;
	right = right / 256;

	if(aksys_dev->hdev->product == AKSYS_QRD_USB) // #define AKSYS_QRD_USB 0x1000
	{
		ff->report->field[0]->value[0] = 0x64;
		ff->report->field[0]->value[1] = left;
		ff->report->field[0]->value[2] = 0x64;
		ff->report->field[0]->value[3] = right;
		hid_info(aksys_dev->hdev, "USB FF");
		hid_hw_request(aksys_dev->hdev, ff->report, HID_REQ_SET_REPORT);
	}
	
	else if(aksys_dev->hdev->product == AKSYS_ANDROID_BT) // #define AKSYS_ANDROID_BT 0x0016
	{
		buf[0] = 0x03; 			// Report ID
		buf[1] = 0x91; 			// Vibration index
		buf[2] = (u8)(left);	// Left Motor Force
		buf[3] = (u8)(right);	// Right Motor Force
		hid_info(aksys_dev->hdev, "BT FF");
		hid_hw_output_report(aksys_dev->hdev, buf, 4);
	}
	return 0;
}

static int aks_joystick_ff_init(struct hid_device *hid)
{
	struct aks_joystick_ff_device *ff;
	struct hid_report *report;
	//struct hid_input *hidinput; 
	struct list_head *report_list =
			&hid->report_enum[HID_OUTPUT_REPORT].report_list;
	struct list_head *report_ptr = report_list;
	//struct input_dev *dev;
	int error;

	struct aksys_dev *aksys_dev = hid_get_drvdata(hid);
	
	if (list_empty(report_list)) {
		hid_err(aksys_dev->hdev, "no output reports found\n");
		return -ENODEV;
	}

	report_ptr = report_ptr->next;

	if (report_ptr == report_list) {
		hid_err(aksys_dev->hdev, "required output report is missing\n");
		return -ENODEV;
	}

	report = list_entry(report_ptr, struct hid_report, list);

	ff = kzalloc(sizeof(struct aks_joystick_ff_device), GFP_KERNEL);
	if (!ff)
		return -ENOMEM;

	//dev = hidinput->input;

	input_set_capability(aksys_dev->game_input_dev, EV_FF, FF_RUMBLE);

	//if(!test_bit(FF_RUMBLE, aksys_dev->game_input_dev->ffbit)) {
	//	hid_err(aksys_dev->hdev, "aksys qrd vibrator FF_RUMBLE not set!");
	//}

	error = input_ff_create_memless(aksys_dev->game_input_dev, ff, aks_joystick_play);

	if (error) {
		printk("FF SET ERROR");
		kfree(ff);
		return 0;
	}

	ff->report = report;
	
	hid_info(aksys_dev->hdev, "Force feedback for aksys qrd USB adapter");
	
	return 0;
}

#else
static inline int aks_joystick_ff_init(struct hid_device *hid)
{
	hid_info(hid, "aks_joystick_ff_init nothing");
	return 0;
}
#endif

/*static int aks_joystick_input_configured(struct hid_device *hdev,
					struct hid_input *hidinput)
{
	int ret = 0;
	
	struct aksys_dev *aksys_dev;

	aksys_dev = devm_kzalloc(&hdev->dev, sizeof(struct aksys_dev), GFP_KERNEL);
	if(!aksys_dev)
		return -ENOMEM;
		
	aksys_dev->hdev = hdev;
	
	ret = hid_hw_open(aksys_dev->hdev);
	if (ret < 0) {
		hid_err(aksys_dev->hdev, "hw open failed\n");
		goto err_close;
	}
	
	aks_joystick_ff_init(aksys_dev->hdev);

	hid_info(aksys_dev->hdev, "aksys qrd hid vibrator configured");
		
	return 0;

err_close:
	hid_hw_close(hdev);
	return ret;
}*/

int aks_joystick_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *rd, int size)
{ 	

	struct aksys_dev *aksys_dev = hid_get_drvdata(hdev);

	if(aksys_mode_flag == GAMEPAD_MODE){

	if((rd[2]) > 7) rd[2] = 8;
	
		/*hid_info(aksys_dev->game_input_dev, "[Aksys_log] %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x",
					rd[0],rd[1],rd[2],rd[3],rd[4],rd[5],rd[6],rd[7],rd[8],rd[9],rd[10],rd[11],rd[12]);*/

		input_report_abs(aksys_dev->game_input_dev, ABS_HAT0X, aksys_hat_mapping[rd[2]].x);
		input_report_abs(aksys_dev->game_input_dev, ABS_HAT0Y, aksys_hat_mapping[rd[2]].y);
		
		input_report_key(aksys_dev->game_input_dev, BTN_GAMEPAD, rd[3] & 0x01);		// A
		input_report_key(aksys_dev->game_input_dev, BTN_EAST, !!(rd[3] & 0x02));	// B
		input_report_key(aksys_dev->game_input_dev, BTN_NORTH, !!(rd[3] & 0x08));	// X
		input_report_key(aksys_dev->game_input_dev, BTN_WEST, !!(rd[3] & 0x10));	// Y
		
		input_report_key(aksys_dev->game_input_dev, BTN_TL, !!(rd[3] & 0x40));		// LB 
		input_report_key(aksys_dev->game_input_dev, BTN_TR, !!(rd[3] & 0x80));		// RB

		input_report_key(aksys_dev->game_input_dev, BTN_THUMBL, !!(rd[4] & 0x20));	// L3
		input_report_key(aksys_dev->game_input_dev, BTN_THUMBR, !!(rd[4] & 0x40));	// R3
		
		input_report_key(aksys_dev->game_input_dev, BTN_SELECT, !!(rd[4] & 0x04));	// SELECT
		input_report_key(aksys_dev->game_input_dev, BTN_START, !!(rd[4] & 0x08));	// START
		
		input_report_key(aksys_dev->game_input_dev, BTN_TL2, rd[4] & 0x01);			// LT BTN
		input_report_key(aksys_dev->game_input_dev, BTN_TR2, !!(rd[4] & 0x02));		// RT BTN

		input_report_abs(aksys_dev->game_input_dev, ABS_X, rd[5]);					// LS (Left, Right)
		input_report_abs(aksys_dev->game_input_dev, ABS_Y, rd[6]);					// LS (Up, Down)
		input_report_abs(aksys_dev->game_input_dev, ABS_Z, rd[7]);					// RS (Left, Right)
		input_report_abs(aksys_dev->game_input_dev, ABS_RZ, rd[8]);					// RS (Up, Down)

		input_report_abs(aksys_dev->game_input_dev, ABS_BRAKE, rd[9]);				// LT
		input_report_abs(aksys_dev->game_input_dev, ABS_GAS, rd[10]);				// RT

		input_report_key(aksys_dev->game_input_dev, KEY_HOMEPAGE, rd[11] & 0x01);	// HOME
		//input_report_key(aksys_dev->game_input_dev, KEY_BACK, !!(rd[12] & 0x02));	// BACK

		input_sync(aksys_dev->game_input_dev);

	}
		
	if(aksys_mode_flag == TOUCHPAD_MODE) {
		
		input_mt_slot(aksys_dev->touch_input_dev, 0);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, rd[3] & 0x01); // A
		if(rd[3] & 0x01)
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[0].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[0].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 1);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[3] & 0x02)); // B
		if(!!(rd[3] & 0x02))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[1].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[1].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 2);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[3] & 0x08)); // X
		if(!!(rd[3] & 0x08))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[2].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[2].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 3);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[3] & 0x10)); // Y
		if(!!(rd[3] & 0x10))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[3].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[3].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 4);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[3] & 0x40)); // LB
		if(!!(rd[3] & 0x40))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[4].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[4].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 5);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[3] & 0x80)); // RB
		if(!!(rd[3] & 0x80))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[5].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[5].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 6);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, rd[4] & 0x01); // LT
		if(rd[4] & 0x01)
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[6].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[6].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 7);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[4] & 0x02)); // RT
		if(!!(rd[4] & 0x02))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[7].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[7].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 8);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, (rd[2] == 0x00) || (rd[2] == 0x01) || (rd[2] == 0x07)); // DPAD - UP
		if((rd[2] == 0x00) || (rd[2] == 0x01) || (rd[2] == 0x07))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[8].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[8].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 9);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, (rd[2] == 0x03) || (rd[2] == 0x04) || (rd[2] == 0x05)); // DPAD - DOWN
		if((rd[2] == 0x03) || (rd[2] == 0x04) || (rd[2] == 0x05))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[9].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[9].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 10);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, (rd[2] == 0x01) || (rd[2] == 0x02) || (rd[2] == 0x03)); // DPAD - RIGHT
		if((rd[2] == 0x01) || (rd[2] == 0x02) || (rd[2] == 0x03))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[10].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[10].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 11);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, (rd[2] == 0x05) || (rd[2] == 0x06) || (rd[2] == 0x07)); // DPAD - LEFT
		if((rd[2] == 0x05) || (rd[2] == 0x06) || (rd[2] == 0x07))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[11].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[11].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 12);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[4] & 0x20)); // L3
		if(!!(rd[4] & 0x20))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[12].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[12].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 13);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, !!(rd[4] & 0x40)); // R3
		if(!!(rd[4] & 0x40))
		{
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[13].x);
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[13].y);
		}

		input_mt_slot(aksys_dev->touch_input_dev, 14);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, (rd[5] != 0x80) || (rd[6] != 0x80));
		if((rd[5] != 0x80) || (rd[6] != 0x80))
		{
			//input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[14].x + (rd[5]/2 - 64));
			//input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[14].y + (rd[6] - 128));
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[14].y + ((rd[5] - 128) >> 1));
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[14].x - (rd[6] - 128));
		}

		input_mt_slot(aksys_dev->touch_input_dev, 15);
		input_mt_report_slot_state(aksys_dev->touch_input_dev, MT_TOOL_FINGER, (rd[7] != 0x80) || (rd[8] != 0x80));
		if((rd[7] != 0x80) || (rd[8] != 0x80))
		{
			//input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[15].x + (rd[7]/2 - 64));
			//input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[15].y + (rd[8] - 128));
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_Y, aksys_xy_mapping[15].y + ((rd[7] - 128) >> 1));
			input_report_abs(aksys_dev->touch_input_dev, ABS_MT_POSITION_X, aksys_xy_mapping[15].x - (rd[8] - 128));
		}

		input_mt_sync_frame(aksys_dev->touch_input_dev);
		input_sync(aksys_dev->touch_input_dev);
		
	}

	if(aksys_mode_flag ==MOUSE_MODE) {
		
		input_report_rel(aksys_dev->mouse_input_dev, REL_X, (rd[5] - 128) / 4);
		input_report_rel(aksys_dev->mouse_input_dev, REL_Y, (rd[6] - 128) / 4);

		if(rd[9] < 0x35)
		{
			input_report_rel(aksys_dev->mouse_input_dev, REL_WHEEL, -1);
		}

		if(rd[9] > 0xcd)
		{
			input_report_rel(aksys_dev->mouse_input_dev, REL_WHEEL, 1);
		}
		
		input_report_key(aksys_dev->mouse_input_dev, BTN_MOUSE, rd[3] & 0x01);
		input_report_key(aksys_dev->mouse_input_dev, BTN_RIGHT, !!(rd[3] & 0x02));
		input_report_key(aksys_dev->mouse_input_dev, BTN_MIDDLE, !!(rd[4] & 0x40));
		input_sync(aksys_dev->mouse_input_dev);
	}
	
	return 0;
}

static int aksys_open(struct input_dev *dev)
{
	struct aksys_dev *aksys_dev = input_get_drvdata(dev);

	return hid_hw_open(aksys_dev->hdev);
}

static void aksys_close(struct input_dev *dev)
{
	struct aksys_dev *aksys_dev = input_get_drvdata(dev);

	hid_hw_close(aksys_dev->hdev);
}

static struct input_dev *allocate_and_setup(struct hid_device *hdev,
		const char *name)
{
	struct input_dev *input_dev;

	input_dev = devm_input_allocate_device(&hdev->dev);
	if (!input_dev)
		return NULL;

	input_dev->name = name;
	input_dev->phys = hdev->phys;
	input_dev->dev.parent = &hdev->dev;
	input_dev->open = aksys_open;
	input_dev->close = aksys_close;
	input_dev->uniq = hdev->uniq;
	input_dev->id.bustype = hdev->bus;
	input_dev->id.vendor  = hdev->vendor;
	input_dev->id.product = hdev->product;
	input_dev->id.version = hdev->version;
	input_set_drvdata(input_dev, hid_get_drvdata(hdev));

	return input_dev;
}

static bool aksys_setup_gamepad(struct aksys_dev *aksys_dev,
		struct hid_device *hdev)
{
	struct input_dev *input_dev;
	
	input_dev = allocate_and_setup(hdev, DEVICE_NAME " Gamepad");
	if (!input_dev)
		return false;

	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);

	set_bit(EV_KEY, input_dev->evbit);
	
	set_bit(KEY_VOLUMEDOWN, input_dev->keybit);
	set_bit(KEY_VOLUMEUP, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_PLAYPAUSE, input_dev->keybit);
	set_bit(KEY_HOMEPAGE, input_dev->keybit);
	set_bit(KEY_UNKNOWN, input_dev->keybit);
	//set_bit(BTN_MOUSE, input_dev->keybit);
	//set_bit(BTN_RIGHT, input_dev->keybit);
	//set_bit(BTN_MIDDLE, input_dev->keybit);
	set_bit(BTN_GAMEPAD, input_dev->keybit);
	set_bit(BTN_EAST, input_dev->keybit);
	set_bit(BTN_C, input_dev->keybit);
	set_bit(BTN_NORTH, input_dev->keybit);
	set_bit(BTN_WEST, input_dev->keybit);
	set_bit(BTN_Z, input_dev->keybit);
	set_bit(BTN_TL, input_dev->keybit);
	set_bit(BTN_TR, input_dev->keybit);
	set_bit(BTN_TL2, input_dev->keybit);
	set_bit(BTN_TR2, input_dev->keybit);
	set_bit(BTN_SELECT, input_dev->keybit);
	set_bit(BTN_START, input_dev->keybit);
	set_bit(BTN_MODE, input_dev->keybit);
	set_bit(BTN_THUMBL, input_dev->keybit);
	set_bit(BTN_THUMBR, input_dev->keybit);
	set_bit(0x013f, input_dev->keybit);
	set_bit(KEY_TV, input_dev->keybit);

	//set_bit(EV_REL, input_dev->evbit);

	//set_bit(REL_X, input_dev->relbit);
	//set_bit(REL_Y, input_dev->relbit);
	//set_bit(REL_WHEEL, input_dev->relbit);
	//set_bit(REL_WHEEL_HI_RES, input_dev->relbit);
	
	set_bit(EV_ABS, input_dev->evbit);
	
	input_set_abs_params(input_dev, ABS_X, 0, 255, 0, 15); 		// flat 15 : 0 ~ 15 -> 0
	input_set_abs_params(input_dev, ABS_Y, 0, 255, 0, 15);
	input_set_abs_params(input_dev, ABS_Z, 0, 255, 0, 15);
	input_set_abs_params(input_dev, ABS_RZ, 0, 255, 0, 15);
	input_set_abs_params(input_dev, ABS_GAS, 0, 255, 0, 15);
	input_set_abs_params(input_dev, ABS_BRAKE, 0, 255, 0, 15);
	input_set_abs_params(input_dev, ABS_HAT0X, -1, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0Y, -1, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, 0, 255, 0, 15);

	set_bit(EV_MSC, input_dev->evbit);
	
	set_bit(MSC_SCAN, input_dev->mscbit);
		
	aksys_dev->game_input_dev = input_dev;

	return true;
}

static bool aksys_setup_touch(struct aksys_dev *aksys_dev,
		struct hid_device *hdev)
{
	struct input_dev *input_dev;
	int ret;

	input_dev = allocate_and_setup(hdev, DEVICE_NAME " Touchpad");
	if (!input_dev)
		return false;

	input_dev->evbit[0] = BIT(EV_ABS) | BIT(EV_KEY);

	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, 0, 1200, 0, 0);
	input_abs_set_res(input_dev, ABS_X, 2340);
	input_set_abs_params(input_dev, ABS_Y, 0, 1200, 0, 0);
	input_abs_set_res(input_dev, ABS_Y, 1080);
	input_set_abs_params(input_dev, ABS_MT_SLOT, 0, 15, 0, 0);
	//input_abs_set_res(input_dev, ABS_MT_SLOT, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 1200, 0, 0);
	input_abs_set_res(input_dev, ABS_MT_POSITION_X, 2340);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 1200, 0, 0);
	input_abs_set_res(input_dev, ABS_MT_POSITION_Y, 1080);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 65535, 0, 0);
	//input_abs_set_res(input_dev, ABS_MT_TRACKING_ID, 0);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	ret = input_mt_init_slots(input_dev, 16, INPUT_PROP_DIRECT);
	if (ret < 0)
		return ret;
	
	aksys_dev->touch_input_dev = input_dev;

	return true;
}

static bool aksys_setup_mouse(struct aksys_dev *aksys_dev,
		struct hid_device *hdev)
{
	struct input_dev *input_dev;

	input_dev = allocate_and_setup(hdev, DEVICE_NAME " Mouse");
	if (!input_dev)
		return false;

	input_dev->evbit[0] = BIT(EV_REL) | BIT(EV_KEY);

	set_bit(EV_REL, input_dev->evbit);

	set_bit(REL_X, input_dev->relbit);
	set_bit(REL_Y, input_dev->relbit);
	set_bit(REL_WHEEL, input_dev->relbit);

	set_bit(EV_KEY, input_dev->evbit);
	
	set_bit(BTN_MOUSE, input_dev->keybit);
	set_bit(BTN_RIGHT, input_dev->keybit);
	set_bit(BTN_MIDDLE, input_dev->keybit);

	aksys_dev->mouse_input_dev = input_dev;

	return true;
}

static int aks_joystick_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct aksys_dev *aksys_dev;
	int ret;

	aksys_dev = devm_kzalloc(&hdev->dev, sizeof(struct aksys_dev), GFP_KERNEL);
	if(!aksys_dev)
		return -ENOMEM;
		
	aksys_dev->hdev = hdev;

	hid_set_drvdata(hdev, aksys_dev);
	
	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}
	
	if (!aksys_setup_gamepad(aksys_dev, hdev)) {
		hid_err(hdev, "could not allocate interfaces\n");
		return -ENOMEM;
	}

	ret = input_register_device(aksys_dev->game_input_dev);
	if (ret) {
		hid_err(hdev, "failed to register interfaces\n");
		return ret;
	}

	if (!aksys_setup_touch(aksys_dev, hdev)) {
		hid_err(hdev, "could not allocate interfaces\n");
		return -ENOMEM;
	}

	ret = input_register_device(aksys_dev->touch_input_dev);
	if (ret) {
		hid_err(hdev, "failed to register interfaces\n");
		return ret;
	}

	if (!aksys_setup_mouse(aksys_dev, hdev)) {
		hid_err(hdev, "could not allocate interfaces\n");
		return -ENOMEM;
	}

	ret = input_register_device(aksys_dev->mouse_input_dev);
	if (ret) {
		hid_err(hdev, "failed to register interfaces\n");
		return ret;
	}
	
	ret = hid_hw_open(aksys_dev->hdev);
	if (ret < 0) {
		hid_err(aksys_dev->hdev, "hw open failed\n");
		//goto err_close;
	}
	
    aks_joystick_ff_init(aksys_dev->hdev);

	hid_info(aksys_dev->hdev, "aksys qrd hid vibrator configured");
	
	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW | HID_CONNECT_DRIVER);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	return 0;	
}

static const struct hid_device_id aks_joystick_devices[] = {
	{ HID_USB_DEVICE(USB_VENDER_ID_QUALCOMM, USB_PRODUCT_ID_AKSYS_HHG)},
	{ HID_USB_DEVICE(USB_VENDER_ID_TEMP_HHG_AKSY, USB_PRODUCT_ID_AKSYS_HHG)},
	{ HID_BLUETOOTH_DEVICE(0x0A12, 0x0016)},
	{ }
};
MODULE_DEVICE_TABLE(hid, aks_joystick_devices);

static struct hid_driver aks_joystick_driver = {
	.name = "aks_joystick_gamepad",
	.id_table = aks_joystick_devices,
	//.input_configured = aks_joystick_input_configured,
	.raw_event        = aks_joystick_raw_event,
	.probe = aks_joystick_probe,
};

module_hid_driver(aks_joystick_driver);

MODULE_LICENSE("GPL2.0");
