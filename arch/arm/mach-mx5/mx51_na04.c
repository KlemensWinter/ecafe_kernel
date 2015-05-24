/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/i2c/siihdmi.h>
#include <linux/pwm_backlight.h>
#include <linux/gpio_keys.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/mxc_edid.h>
#include <mach/iomux-mx51.h>
#include <mach/i2c.h>
#include <linux/pci_ids.h>

#include "devices.h"
#include "crm_regs.h"
#include "usb.h"

/*!
 * @file mach-mx51/mx51_na04.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */

#define NA04_SD1_CD			(0*32 + 0)	/* GPIO_1_0 */
#define NA04_SD1_WP			(0*32 + 1)	/* GPIO_1_1 */
#define NA04_BCK_PWM			(0*32 + 2)	/* GPIO_1_2 */
#define NA04_SD2_CD_2_0			(0*32 + 4)	/* GPIO_1_4 */
#define NA04_SD2_WP			(0*32 + 5)	/* GPIO_1_5 */
//#define NA04_SD2_CD_2_5		(0*32 + 6)	/* GPIO_1_6 */
#define NA04_SD2_CD_2_5			(0*32 + 7)	/* GPIO_1_7 */
//#define NA04_USBH1_HUB_RST		(0*32 + 7)	/* GPIO_1_7 */
#define NA04_PMIC_INT			(0*32 + 8)	/* GPIO_1_8 */

//#define NA04_USB_CLK_EN_B		(1*32 + 1)	/* GPIO_2_1 */
#define NA04_OSC_EN_B			(1*32 + 2)	/* GPIO_2_2 */
#define NA04_PHY_RESET			(1*32 + 5)	/* GPIO_2_5 */
#define NA04_CAM_RESET			(1*32 + 7)	/* GPIO_2_7 */
//#define NA04_FM_PWR			(1*32 + 12)	/* GPIO_2_12 */
//#define NA04_VGA_RESET		(1*32 + 13)	/* GPIO_2_13 */
#define NA04_FEC_PHY_RESET		(1*32 + 14)	/* GPIO_2_14 */
#define NA04_FM_RESET			(1*32 + 15)	/* GPIO_2_15 */
#define NA04_AUDAMP_STBY		(1*32 + 17)	/* GPIO_2_17 */
#define NA04_POWER_KEY			(1*32 + 21)	/* GPIO_2_21 */
#define NA04_USB_CLK_EN_B		(1*32 + 27)	/* GPIO_2_27 */



#define NA04_26M_OSC_EN			(2*32 + 1)	/* GPIO_3_1 */
#define NA04_LVDS_POWER_DOWN		(2*32 + 3)	/* GPIO_3_3 */
#define NA04_DISP_BRIGHTNESS_CTL	(2*32 + 4)	/* GPIO_3_4 */
#define NA04_HDMI_RESET			(2*32 + 5)	/* GPIO_3_5 */
#define NA04_DVI_POWER			(2*32 + 6)	/* GPIO_3_6 */
#define NA04_LID_CLOSE			(2*32 + 12)	/* GPIO_3_12 */
#define NA04_EXT_MIC_DETECT		(2*32 + 13)	/* GPIO_3_13 */
#define NA04_BAT_LOW			(2*32 + 14)	/* GPIO_3_14 */
#define NA04_HEADPHONE_DET		(2*32 + 26)	/* GPIO_3_26 */
#define NA04_DVI_DET			(2*32 + 28)	/* GPIO_3_28 */


#define NA04_HDMI_INT			(3*32 + 7)	/* GPIO_4_7 */
#define NA04_LCD_3V3_ON			(3*32 + 9)	/* GPIO_4_9 */
#define NA04_LCD_5V_ON			(3*32 + 10)	/* GPIO_4_10 */
#define NA04_GS_INT			(3*32 + 11)	/* GPIO_4_11 */
#define NA04_CAM_LOW_POWER		(3*32 + 12)	/* GPIO_4_12 */
//#define NA04_DVI_I2C_EN		(3*32 + 14)	/* GPIO_4_14 */
#define NA04_CAM_PWRON			(3*32 + 14)	/* GPIO_4_14 */
#define NA04_CSP1_SS0_GPIO		(3*32 + 24)	/* GPIO_4_24 */
#define NA04_CSP1_SS1_GPIO		(3*32 + 25)	/* GPIO_4_25 */
#define NA04_AUDIO_CLK_EN		(3*32 + 26)	/* GPIO_4_26 */

extern int __init mx51_na04_init_mc13892(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;

int na04_lid_wake_enable = 0;

static struct input_dev *na04_lid_inputdev;

static struct platform_device na04_lid_dev = {
        .name = "na04_lid",
};

static struct pad_desc mx51na04_pads[] = {
	/* UART1 */
	MX51_PAD_UART1_RXD__UART1_RXD,
	MX51_PAD_UART1_TXD__UART1_TXD,
	MX51_PAD_UART1_RTS__UART1_RTS,
	MX51_PAD_UART1_CTS__UART1_CTS,
	MX51_PAD_UART2_RXD__UART2_RXD,
	MX51_PAD_UART2_TXD__UART2_TXD,

	/* USB HOST1 */
	MX51_PAD_USBH1_STP__USBH1_STP,
	MX51_PAD_USBH1_CLK__USBH1_CLK,
	MX51_PAD_USBH1_DIR__USBH1_DIR,
	MX51_PAD_USBH1_NXT__USBH1_NXT,
	MX51_PAD_USBH1_DATA0__USBH1_DATA0,
	MX51_PAD_USBH1_DATA1__USBH1_DATA1,
	MX51_PAD_USBH1_DATA2__USBH1_DATA2,
	MX51_PAD_USBH1_DATA3__USBH1_DATA3,
	MX51_PAD_USBH1_DATA4__USBH1_DATA4,
	MX51_PAD_USBH1_DATA5__USBH1_DATA5,
	MX51_PAD_USBH1_DATA6__USBH1_DATA6,
	MX51_PAD_USBH1_DATA7__USBH1_DATA7,

	MX51_PAD_GPIO_1_0__GPIO_1_0,		// SD_MMC_CD_B
	MX51_PAD_GPIO_1_1__GPIO_1_1,		// SD_MMC_WP
	MX51_PAD_GPIO_1_4__GPIO_1_4,		// WDOG_B
	MX51_PAD_GPIO_1_5__GPIO_1_5,		// SD_MMC_WP
	MX51_PAD_GPIO_1_6__GPIO_1_6,		// RST_USB_HUB_B   configure as OD output
	MX51_PAD_GPIO_1_7__GPIO_1_7,		// SD_MMC_CD_B
	MX51_PAD_GPIO_1_8__GPIO_1_8,		// INT_FROM_PMIC
	MX51_PAD_GPIO_1_9__GPIO_1_9,		// CARD_ON_OUT
	MX51_PAD_UART3_RXD__GPIO_1_22,		// Test if NA04 2.0 board

//	MX51_PAD_EIM_D17__GPIO_2_1,
//	MX51_PAD_EIM_D18__GPIO_2_2,
	MX51_PAD_EIM_D21__GPIO_2_5,		// RST1_USB_PHY
//	MX51_PAD_EIM_D23__GPIO_2_7,
	MX51_PAD_EIM_A16__GPIO_2_10,
	MX51_PAD_EIM_A17__GPIO_2_11,
	MX51_PAD_EIM_A18__GPIO_2_12,
	MX51_PAD_EIM_A19__GPIO_2_13,
	MX51_PAD_EIM_A20__GPIO_2_14,		// RST_ENET_B
	MX51_PAD_EIM_A21__GPIO_2_15,		// ENET_IRQ_B
	MX51_PAD_EIM_A22__GPIO_2_16,
	MX51_PAD_EIM_A23__GPIO_2_17,
	MX51_PAD_EIM_A27__GPIO_2_21,		// MAIN_PWR_ON
	MX51_PAD_EIM_EB3__GPIO_2_23,		// AMB_LIGHT_INT
	MX51_PAD_EIM_CS2__GPIO_2_27,		// USB_CLK_EN_B
//	MX51_PAD_EIM_DTACK__GPIO_2_31,

//	MX51_PAD_EIM_LBA__GPIO_3_1,
	MX51_PAD_DI1_D0_CS__GPIO_3_3,		// LVDS_PWR_DWN
	MX51_PAD_DISPB2_SER_DIN__GPIO_3_5,	// HDMI_RST
//	MX51_PAD_DISPB2_SER_DIO__GPIO_3_6,
	MX51_PAD_GPIO_NAND__GPIO_3_12,		// LID_CLOSE_B
	MX51_PAD_CSI1_D9__GPIO_3_13,		// EXMIC_DETECT
	MX51_PAD_CSI1_VSYNC__GPIO_3_14,		// BAT_LOW_MX51
//	MX51_PAD_NANDF_CS0__GPIO_3_16,
//	MX51_PAD_NANDF_CS1__GPIO_3_17,
//	MX51_PAD_NANDF_D14__GPIO_3_26,
//	MX51_PAD_NANDF_D12__GPIO_3_28,

	MX51_PAD_NANDF_D1__GPIO_4_7,		// HDMI_INT
	MX51_PAD_CSI2_D12__GPIO_4_9,		// LCD_3V3_ON
	MX51_PAD_CSI2_D13__GPIO_4_10,		// OC_03
	MX51_PAD_CSI2_D19__GPIO_4_12,		// BT_PWRON
	MX51_PAD_CSI2_HSYNC__GPIO_4_14,		// CAM_PWRON
	MX51_PAD_CSPI1_RDY__GPIO_4_26,		// AUDIO_CLK_EN_B

	MX51_PAD_EIM_EB2__FEC_MDIO,
	MX51_PAD_DI2_DISP_CLK__FEC_RDAT1,
	MX51_PAD_DI_GP4__FEC_RDAT2,
	MX51_PAD_EIM_CS3__FEC_RDAT3,
	MX51_PAD_EIM_CS4__FEC_RX_ER,
	MX51_PAD_EIM_CS5__FEC_CRS,
	MX51_PAD_NANDF_RB2__FEC_COL,
	MX51_PAD_NANDF_RB3__FEC_RXCLK,
	MX51_PAD_DISP2_DAT14__FEC_RDAT0,
	MX51_PAD_DISP2_DAT15__FEC_TDAT0,
	MX51_PAD_NANDF_CS2__FEC_TX_ER,
	MX51_PAD_NANDF_CS3__FEC_MDC,
	MX51_PAD_NANDF_CS4__FEC_TDAT1,
	MX51_PAD_NANDF_CS5__FEC_TDAT2,
	MX51_PAD_NANDF_CS6__FEC_TDAT3,
	MX51_PAD_DISP2_DAT9__FEC_TX_EN,
	MX51_PAD_DISP2_DAT13__FEC_TX_CLK,
	MX51_PAD_NANDF_D11__FEC_RX_DV,

	MX51_PAD_I2C1_CLK__HSI2C_CLK,
	MX51_PAD_I2C1_DAT__HSI2C_DAT,
	MX51_PAD_EIM_D16__I2C1_SDA,
	MX51_PAD_EIM_D19__I2C1_SCL,

	MX51_PAD_GPIO_1_2__PWM_PWMO,

	MX51_PAD_KEY_COL5__I2C2_SDA,
	MX51_PAD_KEY_COL4__I2C2_SCL,

	MX51_PAD_SD1_CMD__SD1_CMD,
	MX51_PAD_SD1_CLK__SD1_CLK,
	MX51_PAD_SD1_DATA0__SD1_DATA0,
	MX51_PAD_SD1_DATA1__SD1_DATA1,
	MX51_PAD_SD1_DATA2__SD1_DATA2,
	MX51_PAD_SD1_DATA3__SD1_DATA3,

	MX51_PAD_SD2_CMD__SD2_CMD,
	MX51_PAD_SD2_CLK__SD2_CLK,
	MX51_PAD_SD2_DATA0__SD2_DATA0,
	MX51_PAD_SD2_DATA1__SD2_DATA1,
	MX51_PAD_SD2_DATA2__SD2_DATA2,
	MX51_PAD_SD2_DATA3__SD2_DATA3,

	MX51_PAD_NANDF_RDY_INT__SD3_CMD,
	MX51_PAD_NANDF_CS7__SD3_CLK,
	MX51_PAD_NANDF_D8__SD3_DATA0,
	MX51_PAD_NANDF_D9__SD3_DATA1,
	MX51_PAD_NANDF_D10__SD3_DATA2,
	MX51_PAD_NANDF_RB0__SD3_DATA3,
	MX51_PAD_NANDF_D12__SD3_DATA4,
	MX51_PAD_NANDF_D13__SD3_DATA5,
	MX51_PAD_NANDF_D14__SD3_DATA6,
	MX51_PAD_NANDF_D15__SD3_DATA7,

	MX51_PAD_AUD3_BB_TXD__AUD3_BB_TXD,
	MX51_PAD_AUD3_BB_RXD__AUD3_BB_RXD,
	MX51_PAD_AUD3_BB_CK__AUD3_BB_CK,
	MX51_PAD_AUD3_BB_FS__AUD3_BB_FS,


	MX51_PAD_EIM_D27__AUD6_BB_RXC,
	MX51_PAD_EIM_D28__AUD6_BB_TXD,
	MX51_PAD_EIM_D29__AUD6_BB_RXD,
	MX51_PAD_EIM_D30__AUD6_BB_TXC,
	MX51_PAD_EIM_D31__AUD6_BB_TXFS,

//	MX51_PAD_CSPI1_SS1__CSPI1_SS1,		// SPI1_NORFLASH
	MX51_PAD_CSPI1_SS1__GPIO_4_25

};

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,},
};

struct fb_videomode na04_video_modes[] = {
	{
	 /* MITSUBISHI LVDS panel */
	 "1024x600@60", 60, 1024, 600, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 720p60 TV output */
	 "1280x720@60", 60, 1280, 720, 13468,
	 220, 110,
	 20, 5,
	 40, 5,
	 FB_SYNC_BROADCAST | FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 720p50 TV output */
	 "1280x720@50", 50, 1280, 720, 13468,
	 220, 440,
	 20, 5,
	 40, 5,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
};


struct cpu_wp *mx51_na04_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_na04_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 1000000,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.reset = mx5_vpu_reset,
};

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void mx51_na04_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{

	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			{
			struct pad_desc cspi1_ss0 = MX51_PAD_CSPI1_SS0__CSPI1_SS0;

			gpio_free(NA04_CSP1_SS0_GPIO);
			gpio_set_value(NA04_CSP1_SS1_GPIO, 1);
			mxc_iomux_v3_setup_pad(&cspi1_ss0);

			break;
			}
		case 0x2:
			{
			struct pad_desc cspi1_ss0_gpio = MX51_PAD_CSPI1_SS0__GPIO_4_24;


			mxc_iomux_v3_setup_pad(&cspi1_ss0_gpio);
			gpio_request(NA04_CSP1_SS0_GPIO, "cspi1-gpio0");
			gpio_direction_output(NA04_CSP1_SS0_GPIO, 0);
			gpio_set_value(NA04_CSP1_SS0_GPIO, 1 & (~status));
			break;
			}
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static void mx51_na04_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			{
			struct pad_desc cspi1_ss0_gpio = MX51_PAD_CSPI1_SS0__GPIO_4_24;
			mxc_iomux_v3_setup_pad(&cspi1_ss0_gpio);
			gpio_request(NA04_CSP1_SS0_GPIO, "cspi1-gpio0");
			gpio_direction_output(NA04_CSP1_SS0_GPIO, 0);
			gpio_set_value(NA04_CSP1_SS0_GPIO, 1);
			gpio_set_value(NA04_CSP1_SS1_GPIO, 0);

			break;
			}
		case 0x2:
			gpio_free(NA04_CSP1_SS0_GPIO);
			break;

		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx51_na04_gpio_spi_chipselect_active,
	.chipselect_inactive = mx51_na04_gpio_spi_chipselect_inactive,
};

static struct imxi2c_platform_data mxci2c_data = {
	.bitrate = 100000,
};

static struct mxc_srtc_platform_data srtc_data = {
	.srtc_sec_mode_addr = 0x83F98840,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
	.num_wp = 3,
};

static struct mxc_dvfsper_data dvfs_per_data = {
	.reg_id = "SW2",
	.clk_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.gpc_adu = 0x0,
	.vai_mask = MXC_DVFSPMCR0_FSVAI_MASK,
	.vai_offset = MXC_DVFSPMCR0_FSVAI_OFFSET,
	.dvfs_enable_bit = MXC_DVFSPMCR0_DVFEN,
	.irq_mask = MXC_DVFSPMCR0_FSVAIM,
	.div3_offset = 0,
	.div3_mask = 0x7,
	.div3_div = 2,
	.lp_high = 1250000,
	.lp_low = 1250000,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};


static struct mxc_fb_platform_data na04_fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_LVDS666,		// LVDS Format
	 .mode_str = "1024x600M-16@60",
	 .mode = na04_video_modes,
	 .num_modes = ARRAY_SIZE(na04_video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,		// HDMI Interface
	 .mode_str = "1280x720M-32@60",
	 .mode = na04_video_modes,
	 .num_modes = ARRAY_SIZE(na04_video_modes),
	 },
};

extern int primary_di;


void na04_power_batt_led(int power)
{
	if(power)
		gpio_set_value(NA04_BAT_LOW, 1);
	else
		gpio_set_value(NA04_BAT_LOW, 0);
}


static struct mxc_sbs11_platform_data na04_batt_data = {
	.power_led = na04_power_batt_led,
};

void na04_hdmi_display_reset(void)
{
	gpio_set_value(NA04_HDMI_RESET, 1);
	msleep(50);
	/* do reset */
	gpio_set_value(NA04_HDMI_RESET, 0);
	msleep(50);		/* tRES >= 100us */
	gpio_set_value(NA04_HDMI_RESET, 1);
	msleep(50);
}

void na04_power_lvds(int power)
{

	if(power){
		struct pad_desc gpio1_2_pwm = MX51_PAD_GPIO_1_2__PWM_PWMO;

		gpio_free(NA04_BCK_PWM);
		mxc_iomux_v3_setup_pad(&gpio1_2_pwm);
		
		gpio_set_value(NA04_LVDS_POWER_DOWN, 1);
		}
	else{
		struct pad_desc gpio1_2_gpio1_2 = MX51_PAD_GPIO_1_2__GPIO_1_2;

		mxc_iomux_v3_setup_pad(&gpio1_2_gpio1_2);
		gpio_request(NA04_BCK_PWM, "gpio1_2-gpio1_2");
		gpio_direction_output(NA04_BCK_PWM, 0);
		gpio_set_value(NA04_BCK_PWM,0);	

		gpio_set_value(NA04_LVDS_POWER_DOWN, 0);
	}
}

static struct siihdmi_platform_data mx51_na04_sii9022_data = {
	.reset       = na04_hdmi_display_reset,
	.power_lvds  = na04_power_lvds,

	.vendor      = "Guillemot",
	.description = "eCAFE",

	.framebuffer = "DISP3 BG",

	.hotplug     = {
		.start = IOMUX_TO_IRQ_V3(NA04_HDMI_INT),
		.end   = IOMUX_TO_IRQ_V3(NA04_HDMI_INT),
		.name  = "hdmi-hotplug",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
	.pixclock    = KHZ2PICOS(133000L),
};


static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	.type = "sbs11-battery",
	.addr = 0x0B,
	.platform_data = &na04_batt_data,
	},
	{
	 .type = "sii9022",
	 .addr = 0x39,
	 .platform_data = &mx51_na04_sii9022_data
	 },
};

static int __init na04_mxc_init_fb(void)
{

	if (!machine_is_mx51_na04())
		return 0;


	/* DI0-LVDS */
	gpio_set_value(NA04_LVDS_POWER_DOWN, 0);
	msleep(1);
	gpio_set_value(NA04_LVDS_POWER_DOWN, 1);
	gpio_set_value(NA04_LCD_3V3_ON, 1);


	gpio_set_value(NA04_DISP_BRIGHTNESS_CTL, 1);

	/* HDMI Settings */
	gpio_request(NA04_HDMI_RESET, "hdmi-reset");
	gpio_direction_output(NA04_HDMI_RESET, 0);
	gpio_set_value(NA04_HDMI_RESET, 1);
	msleep(50);

	/* HDMI Reset */
	/* do reset */
	gpio_set_value(NA04_HDMI_RESET, 0);
	msleep(1);		/* tRES >= 100us */
	gpio_set_value(NA04_HDMI_RESET, 1);

	printk(KERN_INFO "DI0 is main interface for LVDS and HDMI\n");

	/* DI0 -> DP-BG channel: */
	mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
	mxc_fb_devices[0].resource = mxcfb_resources;

	mxc_register_device(&mxc_fb_devices[0], &na04_fb_data[0]);

	/*  Register VPU overlay */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(na04_mxc_init_fb);

static struct mxc_lightsensor_platform_data ls_data = {
	.vdd_reg = "VIOHI",
	.rext = 100,
};

static struct mxc_mma7660_platform_data mma7660_data = {
	.irq = IOMUX_TO_IRQ_V3(NA04_GS_INT),
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	 .type = "isl29003",
	 .addr = 0x44,
	 .platform_data = &ls_data,
	 },
	{
	 .type = "mma7660",
	 .addr = 0x4c,
	 .platform_data = (void *) &mma7660_data,
	 },
};

static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = gpio_get_value(NA04_SD1_WP);

	else if (to_platform_device(dev)->id == 2)
		rc = 1;
	else
		rc = gpio_get_value(NA04_SD2_WP);

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret;
	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(NA04_SD1_CD);
		return ret;
	} else if (to_platform_device(dev)->id == 2) {
		return 0;
	} else {		/* config the det pin for SDHC2 */
		ret = gpio_get_value(NA04_SD2_CD_2_5);
		return ret;
	}
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_165_195,
	.caps = MMC_CAP_8_BIT_DATA | MMC_CAP_MMC_HIGHSPEED,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static int mxc_sgtl5000_amp_enable(int enable)
{
	return 0;
}

static int mxc_sgtl5000_clock_enable(int enable)
{
	gpio_set_value(NA04_AUDIO_CLK_EN, !enable);
	return 0;
}

static int headphone_det_status(void)
{
	return 0;
}

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.clock_enable = mxc_sgtl5000_clock_enable,
	.sysclk = 12288000,
};

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-na04-sgtl5000",
};

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_512M;
	int left_mem = 0;
	int gpu_mem = SZ_64M;
	int fb_mem = SZ_32M;

	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_na04_get_cpu_wp;
	set_num_cpu_wp = mx51_na04_set_num_cpu_wp;

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			left_mem = total_mem - gpu_mem - fb_mem;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
				if (left_mem == 0 || left_mem > total_mem)
					left_mem = total_mem - gpu_mem - fb_mem;
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
		gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}
}

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	/* We can do power down one of two ways:
	   Set the power gating
	   Set USEROFFSPI */

	/* Set the power gate bits to power down */
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
}

static struct gpio_keys_button mx51_na04_buttons[] = {
	{	/* Power button */
		.code		= KEY_POWER,
		.gpio		= NA04_POWER_KEY,
		.active_low	= 1,
		.desc		= "power",
		.type		= EV_KEY,
		.wakeup		= 1,
		.debounce_interval = 1,
	},
};

static struct gpio_keys_platform_data mxc_gpio_key_data = {
	.buttons	= mx51_na04_buttons,
	.nbuttons	= ARRAY_SIZE(mx51_na04_buttons),
};

int na04_get_lid_sw_status(void)
{
        /* ron: 0:open 1:close */
        return !gpio_get_value(NA04_LID_CLOSE);
}

static irqreturn_t lid_sw_int(int irq, void *dev_id)
{
	int lid_close;

 	lid_close = na04_get_lid_sw_status();
	if(lid_close) {
		set_irq_type(irq, IRQF_TRIGGER_RISING);

                if(na04_lid_wake_enable)
                        enable_irq_wake(irq);

		input_report_switch(na04_lid_inputdev, SW_LID, lid_close);
		input_sync(na04_lid_inputdev);

	} else {
		set_irq_type(irq, IRQF_TRIGGER_FALLING);

		input_report_switch(na04_lid_inputdev, SW_LID, lid_close);
		input_sync(na04_lid_inputdev);

                if(na04_lid_wake_enable)
                        disable_irq_wake(irq);
	}

	return IRQ_HANDLED;
}

static int na04_init_lid_sw(void)
{
	int irq, ret;

	gpio_request(NA04_LID_CLOSE, "lid_sw");
	gpio_direction_input(NA04_LID_CLOSE);

	irq = IOMUX_TO_IRQ_V3(NA04_LID_CLOSE);

	if(na04_get_lid_sw_status()) {
		set_irq_type(irq, IRQF_TRIGGER_RISING);
	} else {
		set_irq_type(irq, IRQF_TRIGGER_FALLING);
	}

	ret = request_irq(irq, lid_sw_int, 0, "lid-sw", 0);
	if(ret)
		printk("NA04 register lid switch interrupt failed\n");

	return ret;
}

static ssize_t na04_lid_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", na04_get_lid_sw_status());
}

static ssize_t na04_lid_wake_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%s\n", na04_lid_wake_enable ? "on": "off");
}

static ssize_t na04_lid_wake_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
        if(strncmp(buf, "on", 2) == 0)
                na04_lid_wake_enable = 1;
        else if (strncmp(buf, "off", 3) == 0)
                na04_lid_wake_enable = 0;
        else
                return -EINVAL;

        return count;
}

static struct kobj_attribute na04_lid_status_attribute =
        __ATTR(lid, S_IFREG | S_IRUGO, na04_lid_status_show, NULL);
static struct kobj_attribute na04_lid_wake_attribute =
        __ATTR(lid_wake, 0666, na04_lid_wake_show, na04_lid_wake_store);

static struct attribute *na04_status_attrs[] = {
        &na04_lid_status_attribute.attr,
        &na04_lid_wake_attribute.attr,
        NULL,
};

static struct attribute_group na04_status_attr_group = {
        .attrs = na04_status_attrs,
};

static int __init na04_lid_wake_setup(char *p)
{
        if(memcmp(p, "on", 2) == 0) {
                na04_lid_wake_enable = 1;
                p += 2;
        } else if(memcmp(p, "off", 3) == 0) {
                na04_lid_wake_enable = 0;
                p += 3;
        }
	return 0;
}
early_param("lid_wake=", na04_lid_wake_setup);

static int __init na04_init_lid(void)
{
        int ret ;
        struct kobject *na04_lid_kobj;

        platform_device_register(&na04_lid_dev);

        na04_lid_kobj = kobject_create_and_add("status", &na04_lid_dev.dev.kobj);
        if(!na04_lid_kobj) {
                ret = -ENOMEM;
                goto err3;
        }

        ret = sysfs_create_group(na04_lid_kobj, &na04_status_attr_group);
        if(ret) {
                goto err2;
        }

        na04_lid_inputdev = input_allocate_device();
        if(!na04_lid_inputdev) {
                printk("Failed to allocate lid input device\n");
                ret = -ENOMEM;
                goto err2;
        }

        na04_lid_inputdev->name = "NA04 Lid Switch";
        na04_lid_inputdev->phys = "na04/input1";
        na04_lid_inputdev->uniq = "na04";
        na04_lid_inputdev->id.bustype = BUS_HOST;
        na04_lid_inputdev->id.vendor = PCI_VENDOR_ID_FREESCALE;

        set_bit(EV_SW, na04_lid_inputdev->evbit);
        set_bit(SW_LID, na04_lid_inputdev->swbit);

        /* ron: 0:open 1:close */
        if(na04_get_lid_sw_status())
                set_bit(SW_LID, na04_lid_inputdev->sw);

        ret = input_register_device(na04_lid_inputdev);
        if(ret) {
                pr_err("Failed to register Na04 lid input device\n");
                ret = -ENODEV;
                goto err1;
        }

        na04_init_lid_sw();

        return ret;

 err1:
        input_free_device(na04_lid_inputdev);
 err2:
        kobject_put(na04_lid_kobj);
 err3:
        platform_device_unregister(&na04_lid_dev);

        return ret;
}
late_initcall(na04_init_lid);


static void __init mx51_na04_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx51na04_pads,ARRAY_SIZE(mx51na04_pads));

	gpio_request(NA04_PMIC_INT, "pmic-int");
	gpio_request(NA04_SD1_CD, "sdhc1-detect");
	gpio_request(NA04_SD1_WP, "sdhc1-wp");

	gpio_direction_input(NA04_PMIC_INT);
	gpio_direction_input(NA04_SD1_CD);
	gpio_direction_input(NA04_SD1_WP);

	gpio_request(NA04_SD2_CD_2_5, "sdhc2-detect");
	gpio_direction_input(NA04_SD2_CD_2_5);
	gpio_request(NA04_SD2_WP, "sdhc2-wp");
	gpio_direction_input(NA04_SD2_WP);

	/* reset FEC PHY */
	gpio_request(NA04_FEC_PHY_RESET, "fec-phy-reset");
	gpio_direction_output(NA04_FEC_PHY_RESET, 0);

	gpio_set_value(NA04_FEC_PHY_RESET, 1);
	msleep(10);
	gpio_set_value(NA04_FEC_PHY_RESET, 0);
	msleep(10);
	gpio_set_value(NA04_FEC_PHY_RESET, 1);

	/* Drive 26M_OSC_EN line high */
	gpio_request(NA04_26M_OSC_EN, "26m-osc-en");
	gpio_direction_output(NA04_26M_OSC_EN, 1);

	/* Drive USB_CLK_EN_B line low */
	gpio_request(NA04_USB_CLK_EN_B, "usb-clk_en_b");
	gpio_direction_output(NA04_USB_CLK_EN_B, 0);

	/* De-assert USB PHY RESETB */
	gpio_request(NA04_PHY_RESET, "usb-phy-reset");
	gpio_direction_output(NA04_PHY_RESET, 1);

	/* audio_clk_en_b */
	gpio_request(NA04_AUDIO_CLK_EN, "audio-clk-en");
	gpio_direction_output(NA04_AUDIO_CLK_EN, 0);

	/* LCD related gpio */
	gpio_request(NA04_DISP_BRIGHTNESS_CTL, "disp-brightness-ctl");
	gpio_request(NA04_LVDS_POWER_DOWN, "lvds-power-down");
	gpio_request(NA04_LCD_3V3_ON, "lcd-3v3-on");

	gpio_direction_output(NA04_DISP_BRIGHTNESS_CTL, 0);
	gpio_direction_output(NA04_LVDS_POWER_DOWN, 0);
	gpio_direction_output(NA04_LCD_3V3_ON, 0);

	/* DI0-LVDS */
	gpio_set_value(NA04_LVDS_POWER_DOWN, 0);
	msleep(1);
	gpio_set_value(NA04_LCD_3V3_ON, 1);

	/* DI0-HDMI */
	gpio_request(NA04_HDMI_RESET, "hdmi-reset");
	gpio_direction_output(NA04_HDMI_RESET, 0);
	gpio_set_value(NA04_HDMI_RESET, 0);

	/* Batt Low */
	gpio_request(NA04_BAT_LOW, "bat-low");
	gpio_direction_output(NA04_BAT_LOW, 0);
	gpio_set_value(NA04_BAT_LOW, 0);

	/* HDMI int */
	gpio_request(NA04_HDMI_INT, "hdmi-int");
	gpio_direction_input(NA04_HDMI_INT);

	/* Ext MIC int */
	gpio_request(NA04_EXT_MIC_DETECT, "mic-int");
	gpio_direction_input(NA04_EXT_MIC_DETECT);


	/* Camera reset */
	gpio_request(NA04_CAM_PWRON, "cam-reset");
	gpio_direction_output(NA04_CAM_PWRON, 0);
	gpio_set_value(NA04_CAM_PWRON, 1);

	/* OSC_EN */
	gpio_request(NA04_OSC_EN_B, "osc-en");
	gpio_direction_output(NA04_OSC_EN_B, 1);

	gpio_request(NA04_CSP1_SS1_GPIO, "cspi1-gpio1");
	gpio_direction_output(NA04_CSP1_SS1_GPIO, 0);
	gpio_set_value(NA04_CSP1_SS1_GPIO, 1);

}


/*!
 * Board specific initialization.
 */
static void __init mxc_na04_board_init(void)
{

	clk_set_rate(clk_get(&(mxc_fb_devices[0].dev), "axi_b_clk"), 133000000);

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "csi_mclk1");
	mxc_ipu_data.csi_clk[1] = clk_get(NULL, "csi_mclk2");

	/* SD card detect irqs */
	mxcsdhc2_device.resource[2].start = IOMUX_TO_IRQ_V3(NA04_SD2_CD_2_5);
	mxcsdhc2_device.resource[2].end = IOMUX_TO_IRQ_V3(NA04_SD2_CD_2_5);
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ_V3(NA04_SD1_CD);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ_V3(NA04_SD1_CD);

	mxc_cpu_common_init();
	mx51_na04_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);

	mxc_register_device(&mxc_rtc_device, &srtc_data);

	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);

	mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	mxc_register_device(&gpu_device, NULL);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&mx51_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, NULL);
	mxc_register_device(&sdram_autogating_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	mxc_register_device(&mxc_iim_device, NULL);
	mxc_register_device(&mxc_pwm1_device, NULL);
	mxc_register_device(&mxc_pwm1_backlight_device,
		&mxc_pwm_backlight_data);

	mxc_register_device(&mxc_gpio_key_device, &mxc_gpio_key_data);	//joechen ++

	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);

	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_ssi3_device, NULL);

	mxc_register_device(&mxc_fec_device, NULL);
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);

	mx51_na04_init_mc13892();

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));

	pm_power_off = mxc_power_off;

	if (cpu_is_mx51_rev(CHIP_REV_1_1) == 2) {
		sgtl5000_data.sysclk = 26000000;
	}
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);

	mx5_usb_dr_init();
	mx5_usbh1_init();
}

static void __init mx51_na04_timer_init(void)
{
	struct clk *uart_clk;

	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx51_na04_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_NA04 data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX51_NA04, "Hercules eCAFE 20110415")
	/* Maintainer: Guillemot Corporation, Inc. */
	.phys_io	= AIPS1_BASE_ADDR,
	.io_pg_offst	= ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_na04_board_init,
	.timer = &mxc_timer,
MACHINE_END
