/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "dsi_pll.h"

/* Register Offsets from PLL base address */
#define PLL_ANALOG_CONTROLS_ONE			0x0000
#define PLL_ANALOG_CONTROLS_TWO			0x0004
#define PLL_INT_LOOP_SETTINGS			0x0008
#define PLL_INT_LOOP_SETTINGS_TWO		0x000C
#define PLL_ANALOG_CONTROLS_THREE		0x0010
#define PLL_ANALOG_CONTROLS_FOUR		0x0014
#define PLL_ANALOG_CONTROLS_FIVE		0x0018
#define PLL_INT_LOOP_CONTROLS			0x001C
#define PLL_DSM_DIVIDER				0x0020
#define PLL_FEEDBACK_DIVIDER			0x0024
#define PLL_SYSTEM_MUXES			0x0028
#define PLL_FREQ_UPDATE_CONTROL_OVERRIDES	0x002C
#define PLL_CMODE				0x0030
#define PLL_PSM_CTRL				0x0034
#define PLL_RSM_CTRL				0x0038
#define PLL_VCO_TUNE_MAP			0x003C
#define PLL_PLL_CNTRL				0x0040
#define PLL_CALIBRATION_SETTINGS		0x0044
#define PLL_BAND_SEL_CAL_TIMER_LOW		0x0048
#define PLL_BAND_SEL_CAL_TIMER_HIGH		0x004C
#define PLL_BAND_SEL_CAL_SETTINGS		0x0050
#define PLL_BAND_SEL_MIN			0x0054
#define PLL_BAND_SEL_MAX			0x0058
#define PLL_BAND_SEL_PFILT			0x005C
#define PLL_BAND_SEL_IFILT			0x0060
#define PLL_BAND_SEL_CAL_SETTINGS_TWO		0x0064
#define PLL_BAND_SEL_CAL_SETTINGS_THREE		0x0068
#define PLL_BAND_SEL_CAL_SETTINGS_FOUR		0x006C
#define PLL_BAND_SEL_ICODE_HIGH			0x0070
#define PLL_BAND_SEL_ICODE_LOW			0x0074
#define PLL_FREQ_DETECT_SETTINGS_ONE		0x0078
#define PLL_FREQ_DETECT_THRESH			0x007C
#define PLL_FREQ_DET_REFCLK_HIGH		0x0080
#define PLL_FREQ_DET_REFCLK_LOW			0x0084
#define PLL_FREQ_DET_PLLCLK_HIGH		0x0088
#define PLL_FREQ_DET_PLLCLK_LOW			0x008C
#define PLL_PFILT				0x0090
#define PLL_IFILT				0x0094
#define PLL_PLL_GAIN				0x0098
#define PLL_ICODE_LOW				0x009C
#define PLL_ICODE_HIGH				0x00A0
#define PLL_LOCKDET				0x00A4
#define PLL_OUTDIV				0x00A8
#define PLL_FASTLOCK_CONTROL			0x00AC
#define PLL_PASS_OUT_OVERRIDE_ONE		0x00B0
#define PLL_PASS_OUT_OVERRIDE_TWO		0x00B4
#define PLL_CORE_OVERRIDE			0x00B8
#define PLL_CORE_INPUT_OVERRIDE			0x00BC
#define PLL_RATE_CHANGE				0x00C0
#define PLL_PLL_DIGITAL_TIMERS			0x00C4
#define PLL_PLL_DIGITAL_TIMERS_TWO		0x00C8
#define PLL_DECIMAL_DIV_START			0x00CC
#define PLL_FRAC_DIV_START_LOW			0x00D0
#define PLL_FRAC_DIV_START_MID			0x00D4
#define PLL_FRAC_DIV_START_HIGH			0x00D8
#define PLL_DEC_FRAC_MUXES			0x00DC
#define PLL_DECIMAL_DIV_START_1			0x00E0
#define PLL_FRAC_DIV_START_LOW_1		0x00E4
#define PLL_FRAC_DIV_START_MID_1		0x00E8
#define PLL_FRAC_DIV_START_HIGH_1		0x00EC
#define PLL_DECIMAL_DIV_START_2			0x00F0
#define PLL_FRAC_DIV_START_LOW_2		0x00F4
#define PLL_FRAC_DIV_START_MID_2		0x00F8
#define PLL_FRAC_DIV_START_HIGH_2		0x00FC
#define PLL_MASH_CONTROL			0x0100
#define PLL_SSC_STEPSIZE_LOW			0x0104
#define PLL_SSC_STEPSIZE_HIGH			0x0108
#define PLL_SSC_DIV_PER_LOW			0x010C
#define PLL_SSC_DIV_PER_HIGH			0x0110
#define PLL_SSC_ADJPER_LOW			0x0114
#define PLL_SSC_ADJPER_HIGH			0x0118
#define PLL_SSC_MUX_CONTROL			0x011C
#define PLL_SSC_STEPSIZE_LOW_1			0x0120
#define PLL_SSC_STEPSIZE_HIGH_1			0x0124
#define PLL_SSC_DIV_PER_LOW_1			0x0128
#define PLL_SSC_DIV_PER_HIGH_1			0x012C
#define PLL_SSC_ADJPER_LOW_1			0x0130
#define PLL_SSC_ADJPER_HIGH_1			0x0134
#define PLL_SSC_STEPSIZE_LOW_2			0x0138
#define PLL_SSC_STEPSIZE_HIGH_2			0x013C
#define PLL_SSC_DIV_PER_LOW_2			0x0140
#define PLL_SSC_DIV_PER_HIGH_2			0x0144
#define PLL_SSC_ADJPER_LOW_2			0x0148
#define PLL_SSC_ADJPER_HIGH_2			0x014C
#define PLL_SSC_CONTROL				0x0150
#define PLL_PLL_OUTDIV_RATE			0x0154
#define PLL_PLL_LOCKDET_RATE_1			0x0158
#define PLL_PLL_LOCKDET_RATE_2			0x015C
#define PLL_PLL_PROP_GAIN_RATE_1		0x0160
#define PLL_PLL_PROP_GAIN_RATE_2		0x0164
#define PLL_PLL_BAND_SEL_RATE_1			0x0168
#define PLL_PLL_BAND_SEL_RATE_2			0x016C
#define PLL_PLL_INT_GAIN_IFILT_BAND_1		0x0170
#define PLL_PLL_INT_GAIN_IFILT_BAND_2		0x0174
#define PLL_PLL_FL_INT_GAIN_PFILT_BAND_1	0x0178
#define PLL_PLL_FL_INT_GAIN_PFILT_BAND_2	0x017C
#define PLL_PLL_FASTLOCK_EN_BAND		0x0180
#define PLL_FREQ_TUNE_ACCUM_INIT_MID		0x0184
#define PLL_FREQ_TUNE_ACCUM_INIT_HIGH		0x0188
#define PLL_FREQ_TUNE_ACCUM_INIT_MUX		0x018C
#define PLL_PLL_LOCK_OVERRIDE			0x0190
#define PLL_PLL_LOCK_DELAY			0x0194
#define PLL_PLL_LOCK_MIN_DELAY			0x0198
#define PLL_CLOCK_INVERTERS			0x019C
#define PLL_SPARE_AND_JPC_OVERRIDES		0x01A0
#define PLL_BIAS_CONTROL_1			0x01A4
#define PLL_BIAS_CONTROL_2			0x01A8
#define PLL_ALOG_OBSV_BUS_CTRL_1		0x01AC
#define PLL_COMMON_STATUS_ONE			0x01B0
#define PLL_COMMON_STATUS_TWO			0x01B4
#define PLL_BAND_SEL_CAL			0x01B8
#define PLL_ICODE_ACCUM_STATUS_LOW		0x01BC
#define PLL_ICODE_ACCUM_STATUS_HIGH		0x01C0
#define PLL_FD_OUT_LOW				0x01C4
#define PLL_FD_OUT_HIGH				0x01C8
#define PLL_ALOG_OBSV_BUS_STATUS_1		0x01CC
#define PLL_PLL_MISC_CONFIG			0x01D0
#define PLL_FLL_CONFIG				0x01D4
#define PLL_FLL_FREQ_ACQ_TIME			0x01D8
#define PLL_FLL_CODE0				0x01DC
#define PLL_FLL_CODE1				0x01E0
#define PLL_FLL_GAIN0				0x01E4
#define PLL_FLL_GAIN1				0x01E8
#define PLL_SW_RESET				0x01EC
#define PLL_FAST_PWRUP				0x01F0
#define PLL_LOCKTIME0				0x01F4
#define PLL_LOCKTIME1				0x01F8
#define PLL_DEBUG_BUS_SEL			0x01FC
#define PLL_DEBUG_BUS0				0x0200
#define PLL_DEBUG_BUS1				0x0204
#define PLL_DEBUG_BUS2				0x0208
#define PLL_DEBUG_BUS3				0x020C
#define PLL_ANALOG_FLL_CONTROL_OVERRIDES	0x0210
#define PLL_VCO_CONFIG				0x0214
#define PLL_VCO_CAL_CODE1_MODE0_STATUS		0x0218
#define PLL_VCO_CAL_CODE1_MODE1_STATUS		0x021C
#define PLL_RESET_SM_STATUS			0x0220
#define PLL_TDC_OFFSET				0x0224
#define PLL_PS3_PWRDOWN_CONTROLS		0x0228
#define PLL_PS4_PWRDOWN_CONTROLS		0x022C
#define PLL_PLL_RST_CONTROLS			0x0230
#define PLL_GEAR_BAND_SELECT_CONTROLS		0x0234
#define PLL_PSM_CLK_CONTROLS			0x0238
#define PLL_SYSTEM_MUXES_2			0x023C
#define PLL_VCO_CONFIG_1			0x0240
#define PLL_VCO_CONFIG_2			0x0244
#define PLL_CLOCK_INVERTERS_1			0x0248
#define PLL_CLOCK_INVERTERS_2			0x024C
#define PLL_CMODE_1				0x0250
#define PLL_CMODE_2				0x0254
#define PLL_ANALOG_CONTROLS_FIVE_1		0x0258
#define PLL_ANALOG_CONTROLS_FIVE_2		0x025C
#define PLL_PERF_OPTIMIZE			0x0260

/* Register Offsets from PHY base address */
#define PHY_CMN_CLK_CFG0	0x010
#define PHY_CMN_CLK_CFG1	0x014
#define PHY_CMN_GLBL_CTRL	0x018
#define PHY_CMN_RBUF_CTRL	0x01C
#define PHY_CMN_CTRL_0		0x024
#define PHY_CMN_CTRL_2		0x02C
#define PHY_CMN_CTRL_3		0x030
#define PHY_CMN_PLL_CNTRL	0x03C
#define PHY_CMN_GLBL_DIGTOP_SPARE4 0x128

/* Bit definition of SSC control registers */
#define SSC_CENTER		BIT(0)
#define SSC_EN			BIT(1)
#define SSC_FREQ_UPDATE		BIT(2)
#define SSC_FREQ_UPDATE_MUX	BIT(3)
#define SSC_UPDATE_SSC		BIT(4)
#define SSC_UPDATE_SSC_MUX	BIT(5)
#define SSC_START		BIT(6)
#define SSC_START_MUX		BIT(7)

/* Dynamic Refresh Control Registers */
#define DSI_DYNAMIC_REFRESH_PLL_CTRL0		(0x014)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL1		(0x018)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL2		(0x01C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL3		(0x020)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL4		(0x024)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL5		(0x028)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL6		(0x02C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL7		(0x030)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL8		(0x034)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL9		(0x038)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL10		(0x03C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL11		(0x040)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL12		(0x044)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL13		(0x048)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL14		(0x04C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL15		(0x050)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL16		(0x054)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL17		(0x058)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL18		(0x05C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL19		(0x060)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL20		(0x064)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL21		(0x068)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL22		(0x06C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL23		(0x070)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL24		(0x074)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL25		(0x078)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL26		(0x07C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL27		(0x080)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL28		(0x084)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL29		(0x088)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL30		(0x08C)
#define DSI_DYNAMIC_REFRESH_PLL_CTRL31		(0x090)
#define DSI_DYNAMIC_REFRESH_PLL_UPPER_ADDR	(0x094)
#define DSI_DYNAMIC_REFRESH_PLL_UPPER_ADDR2	(0x098)

#define DSI_PHY_TO_PLL_OFFSET	(0x500)

enum {
	DSI_PLL_0,
	DSI_PLL_1,
	DSI_PLL_MAX
};

struct dsi_pll_div_table pll_4nm_dphy_lb[] = {
	{27270000, 30000000, 2, 11},
	{30000000, 33330000, 4, 5},
	{33330000, 37500000, 2, 9},
	{37500000, 40000000, 8, 2},
	{40000000, 42860000, 1, 15},
	{42860000, 46150000, 2, 7},
	{46150000, 50000000, 1, 13},
	{50000000, 54550000, 4, 3},
	{54550000, 60000000, 1, 11},
	{60000000, 66670000, 2, 5},
	{66670000, 75000000, 1, 9},
	{75000000, 85710000, 8, 1},
	{85710000, 100000000, 1, 7},
	{100000000, 120000000, 2, 3},
	{120000000, 150000000, 1, 5},
	{150000000, 200000000, 4, 1},
	{200000000, 300000000, 1, 3},
	{300000000, 600000000, 2, 1},
	{600000000, 1500000000, 1, 1}
};

struct dsi_pll_div_table pll_4nm_dphy_hb[] = {
	{68180000, 75000000, 2, 11},
	{75000000, 83330000, 4, 5},
	{83330000, 93750000, 2, 9},
	{93750000, 100000000, 8, 2},
	{100000000, 107140000, 1, 15},
	{107140000, 115380000, 2, 7},
	{115380000, 125000000, 1, 13},
	{125000000, 136360000, 4, 3},
	{136360000, 150000000, 1, 11},
	{150000000, 166670000, 2, 5},
	{166670000, 187500000, 1, 9},
	{187500000, 214290000, 8, 1},
	{214290000, 250000000, 1, 7},
	{250000000, 300000000, 2, 3},
	{300000000, 375000000, 1, 5},
	{375000000, 500000000, 4, 1},
	{500000000, 750000000, 1, 3},
	{750000000, 1500000000, 2, 1},
	{1500000000, 5000000000, 1, 1}
};

struct dsi_pll_div_table pll_4nm_cphy_lb[] = {
	{30000000, 37500000, 4, 5},
	{37500000, 50000000, 8, 2},
	{50000000, 60000000, 4, 3},
	{60000000, 75000000, 2, 5},
	{75000000, 100000000, 8, 1},
	{100000000, 120000000, 2, 3},
	{120000000, 150000000, 1, 5},
	{150000000, 200000000, 4, 1},
	{200000000, 300000000, 1, 3},
	{300000000, 600000000, 2, 1},
	{600000000, 1500000000, 1, 1}
};

struct dsi_pll_div_table pll_4nm_cphy_hb[] = {
	{75000000, 93750000, 4, 5},
	{93750000, 12500000, 8, 2},
	{125000000, 150000000, 4, 3},
	{150000000, 187500000, 2, 5},
	{187500000, 250000000, 8, 1},
	{250000000, 300000000, 2, 3},
	{300000000, 375000000, 1, 5},
	{375000000, 500000000, 4, 1},
	{500000000, 750000000, 1, 3},
	{750000000, 1500000000, 2, 1},
	{1500000000, 5000000000, 1, 1}
};
