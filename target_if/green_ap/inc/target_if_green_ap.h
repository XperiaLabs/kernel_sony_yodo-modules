/*
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: offload lmac interface APIs for green ap
 */
#ifndef __TARGET_IF_GREEN_AP_H__
#define __TARGET_IF_GREEN_AP_H__

#include <wlan_objmgr_pdev_obj.h>

/**
 * target_if_green_ap_enable_egap() - Enable egap
 * @pdev: pdev pointer
 *
 * @Return: None
 */
void target_if_green_ap_enable_egap(struct wlan_objmgr_pdev *pdev);

/**
 * target_if_green_ap_set_ps_on_off() - PS toggle
 * @pdev: pdev pointer
 * @value: Value to send PS on/off to FW
 *
 * @Return: None
 */
void target_if_green_ap_set_ps_on_off(struct wlan_objmgr_pdev *pdev,
				     uint32_t value);
#endif
