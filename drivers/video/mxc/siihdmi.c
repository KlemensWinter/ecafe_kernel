/* vim: set noet ts=8 sts=8 sw=8 : */
/*
 * Copyright © 2010 Saleem Abdulrasool <compnerd@compnerd.org>.
 * Copyright © 2010 Genesi USA, Inc. <matt@genesi-usa.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/mxcfb.h>
#include <linux/uaccess.h>
#include <linux/ipu.h>

#include <linux/edid.h>
#include <linux/cea861.h>
#include <linux/cea861_modes.h>
#include <linux/i2c/siihdmi.h>

/* logging helpers */
#define CONTINUE(fmt, ...)	printk(KERN_CONT    fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)		printk(KERN_DEBUG   "SIIHDMI: " fmt, ## __VA_ARGS__)
#define ERROR(fmt, ...)		printk(KERN_ERR     "SIIHDMI: " fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...)	printk(KERN_WARNING "SIIHDMI: " fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)		printk(KERN_INFO    "SIIHDMI: " fmt, ## __VA_ARGS__)


#define ASPECT_RATIO_NONE	0x08
#define ASPECT_RATIO_4_3	0x18
#define ASPECT_RATIO_16_9	0x28


/* module parameters */
static unsigned int bus_timeout = 50;
module_param(bus_timeout, uint, 0644);
MODULE_PARM_DESC(bus_timeout, "bus timeout in milliseconds");

static unsigned int teneighty = 0;
module_param(teneighty, uint, 0644);
MODULE_PARM_DESC(teneighty, "try 1080p low field-rate modes");

static unsigned int seventwenty = 1;
module_param(seventwenty, uint, 0644);
MODULE_PARM_DESC(seventwenty, "stick to 720p instead of using EDID modes");

extern struct fb_videomode na04_video_modes[];


static bool hdmi_device_detected = false;

int na04_hdmi_video_init(const struct siihdmi_tx *tx, struct fb_var_screeninfo *var);


static int siihdmi_detect_revision(struct siihdmi_tx *tx)
{
	u8 data;
	unsigned long start;

	start = jiffies;
	do {
		data = i2c_smbus_read_byte_data(tx->client,
						SIIHDMI_TPI_REG_DEVICE_ID);
	} while (data != SIIHDMI_DEVICE_ID_902x &&
		 !time_after(jiffies, start + bus_timeout));

	if (data != SIIHDMI_DEVICE_ID_902x)
		return -ENODEV;

	INFO("Device ID: %#02x", data);

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_DEVICE_REVISION);
	if (data)
		CONTINUE(" (rev %01u.%01u)", (data >> 4) & 15, data & 15);

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_TPI_REVISION);
	CONTINUE(" (%s", data & (1 << 7) ? "Virtual " : "");
	data &= ~(1 << 7);
	data = data ? data : SIIHDMI_BASE_TPI_REVISION;
	CONTINUE("TPI revision %01u.%01u)", (data >> 4) & 15, data & 15);

	data = i2c_smbus_read_byte_data(tx->client,
					SIIHDMI_TPI_REG_HDCP_REVISION);
	if (data)
		CONTINUE(" (HDCP version %01u.%01u)",
			 (data >> 4) & 15, data & 15);

	CONTINUE("\n");

	return 0;
}




static int siihdmi_initialise(struct siihdmi_tx *tx)
{
	int ret;

	/* step 1: reset and initialise */
	if (tx->platform->reset)
		tx->platform->reset();

	ret = i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_RQB, 0x00);
	if (ret < 0) {
		WARNING("unable to initialise device to TPI mode\n");
		return ret;
	}

	/* step 2: detect revision */
	if ((ret = siihdmi_detect_revision(tx)) < 0) {
		DEBUG("unable to detect device revision\n");
		return ret;
	}

	/* step 3: power up transmitter */
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_PWR_STATE,
					SIIHDMI_POWER_STATE_D0);
	if (ret < 0) {
		ERROR("unable to power up transmitter\n");
		return ret;
	}

	/* step 4: configure input bus and pixel repetition */

	/* step 5: select YC input mode */

	/* step 6: configure sync methods */

	/* step 7: configure explicit sync DE generation */

	/* step 8: configure embedded sync extraction */

	/* step 9: setup interrupt service */

/*	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_IER,
					SIIHDMI_IER_HOT_PLUG_EVENT |
					SIIHDMI_IER_RECEIVER_SENSE_EVENT);


*/
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_IER,
					SIIHDMI_IER_HOT_PLUG_EVENT);
	
	if (ret < 0) {
		WARNING("unable to setup interrupt request\n");
		return ret;
	}

	return 0;
}

static bool siihdmi_sink_present(struct siihdmi_tx *tx)
{
	u8 isr;
	bool present;

	isr = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_ISR);

	present = isr & (SIIHDMI_ISR_HOT_PLUG_EVENT |
			 SIIHDMI_ISR_RECEIVER_SENSE_EVENT);

	DEBUG("%ssink detected%s%s%s%s\n",
	      present ? "" : "no ",
	      present ? " (" : "",
	      (isr & SIIHDMI_ISR_HOT_PLUG_EVENT) ? "hotplug, " : ", ",
	      (isr & SIIHDMI_ISR_RECEIVER_SENSE_EVENT) ? "receiver sense, " : ", ",
	      present ? "\b\b)" : "\b\b");

	hdmi_device_detected = present;
	return present;
}

static int siihdmi_read_edid(struct siihdmi_tx *tx, u8 *edid, size_t size)
{
	u8 offset, ctrl;
	int ret;
	unsigned long start;

	struct i2c_msg request[] = {
		{ .addr  = EDID_I2C_DDC_DATA_ADDRESS,
		  .len   = sizeof(offset),
		  .buf   = &offset, },
		{ .addr  = EDID_I2C_DDC_DATA_ADDRESS,
		  .flags = I2C_M_RD,
		  .len   = size,
		  .buf   = edid, },
	};

	/* step 1: (potentially) disable HDCP */

	ctrl = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_SYS_CTRL);

	/* step 2: request the DDC bus */
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl | SIIHDMI_SYS_CTRL_DDC_BUS_REQUEST);
	if (ret < 0) {
		DEBUG("unable to request DDC bus\n");
		return ret;
	}

	/* step 3: poll for bus grant */
	start = jiffies;
	do {
		ctrl = i2c_smbus_read_byte_data(tx->client,
						SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((~ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED) &&
		 !time_after(jiffies, start + bus_timeout));

	if (~ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED)
		goto relinquish;

	/* step 4: take ownership of the DDC bus */
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					SIIHDMI_SYS_CTRL_DDC_BUS_REQUEST |
					SIIHDMI_SYS_CTRL_DDC_BUS_OWNER_HOST);
	if (ret < 0) {
		DEBUG("unable to take ownership of the DDC bus\n");
		goto relinquish;
	}

	/* step 5: read edid */
	offset = 0;
	ret = i2c_transfer(tx->client->adapter, request, ARRAY_SIZE(request));
	if (ret != ARRAY_SIZE(request))
		DEBUG("unable to read EDID block\n");

relinquish:
	/* step 6: relinquish ownership of the DDC bus */
	start = jiffies;
	do {
		i2c_smbus_write_byte_data(tx->client,
					  SIIHDMI_TPI_REG_SYS_CTRL,
					  0x00);
		ctrl = i2c_smbus_read_byte_data(tx->client,
						SIIHDMI_TPI_REG_SYS_CTRL);
	} while ((ctrl & SIIHDMI_SYS_CTRL_DDC_BUS_GRANTED) &&
		 !time_after(jiffies, start + bus_timeout));

	/* step 7: (potentially) enable HDCP */

	return 0;
}

static void siihdmi_parse_cea861_timing_block(struct siihdmi_tx *tx,
					      const struct edid_extension *ext)
{
	const struct cea861_timing_block * const cea =
		(struct cea861_timing_block *) ext;
	const u8 offset = offsetof(struct cea861_timing_block, data);
	u8 index = 0;

	BUILD_BUG_ON(sizeof(*cea) != sizeof(*ext));

	tx->basic_audio = cea->basic_audio_supported;

	if (cea->underscan_supported)
		tx->pixel_mapping = PIXEL_MAPPING_UNDERSCANNED;

	if (cea->dtd_offset == CEA81_NO_DTDS_PRESENT)
		return;

	do {
		const struct cea861_data_block_header * const header =
			(struct cea861_data_block_header *) &cea->data[index];

		switch (header->tag) {
		case CEA861_DATA_BLOCK_TYPE_VENDOR_SPECIFIC: {
				const struct cea861_vendor_specific_data_block * const vsdb =
					(struct cea861_vendor_specific_data_block *) header;

				if (!memcmp(vsdb->ieee_registration,
					    CEA861_OUI_REGISTRATION_ID_HDMI_LSB,
					    sizeof(vsdb->ieee_registration))) {
					DEBUG("HDMI sink verified %s\n",
					tx->basic_audio ? "" : "but not enabled due to lack of audio support");
					if (tx->basic_audio)
						tx->connection_type = CONNECTION_TYPE_HDMI;
				}
			}
			break;
		case CEA861_DATA_BLOCK_TYPE_VIDEO: {
				const struct cea861_video_data_block * const video =
					(struct cea861_video_data_block *) header;

				int i, added = 0;

				for (i = 0; i < header->length; i++) {
					int vic = video->svd[i] & 0x7f;
					//int native = video->svd[i] & 0x80;

					if ((vic >= 1) && (vic <= 64)) {
						fb_add_videomode(&cea_modes[vic], &tx->info->modelist);
						added++;
					}

				}
				DEBUG("Added %u modes from CEA Video Data Block\n", added);
			}
			break;
		default:
			break;
		}

		index = index + header->length + sizeof(*header);
	} while (index < cea->dtd_offset - offset);
}

static void siihdmi_set_vmode_registers(struct siihdmi_tx *tx,
					struct fb_var_screeninfo *var)
{
	enum basic_video_mode_fields {
		PIXEL_CLOCK,
		REFRESH_RATE,
		X_RESOLUTION,
		Y_RESOLUTION,
		FIELDS,
	};

	u16 vmode[FIELDS];
	u32 pixclk, htotal, vtotal, refresh,refresh2;
	u8 format;
	int ret;

	BUILD_BUG_ON(sizeof(vmode) != 8);

	pixclk = var->pixclock ? PICOS2KHZ(var->pixclock) : 0;
	htotal = var->xres + var->left_margin + var->hsync_len + var->right_margin;
	vtotal = var->yres + var->upper_margin + var->vsync_len + var->lower_margin;
	refresh = (pixclk * 100000ul) / (htotal * vtotal);

	refresh2 = 1000000/(htotal*vtotal/(PICOS2KHZ(var->pixclock)/10));

	BUG_ON(pixclk == 0);

	/* basic video mode data */
	pixclk /= 10;
	DEBUG("NA04 Setting vmode: pixclk=%u\
		htotal x vtotal=%ux%u\
		refresh @%u\n\
		var->xres + var->left_margin + var->hsync_len + var->right_margin= %u+%u+%u+%u\n\
		var->yres + var->upper_margin + var->vsync_len + var->lower_margin=%u+%u+%u+%u\n"\
		, (u16) pixclk,(u16) htotal, (u16) vtotal,(u16) refresh2,\
		var->xres,var->left_margin,var->hsync_len,var->right_margin,\
		var->yres,var->upper_margin,var->vsync_len,var->lower_margin);

	vmode[PIXEL_CLOCK]  = (u16) pixclk;
	vmode[REFRESH_RATE] = (u16) refresh2;
	vmode[X_RESOLUTION] = (u16) htotal;
	vmode[Y_RESOLUTION] = (u16) vtotal;

	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_VIDEO_MODE_DATA_BASE,
					     sizeof(vmode),
					     (u8 *) vmode);
	if (ret < 0)
		DEBUG("unable to write video mode data\n");

	/* input format */
	format = SIIHDMI_INPUT_COLOR_SPACE_RGB            |
		 SIIHDMI_INPUT_VIDEO_RANGE_EXPANSION_AUTO |
		 SIIHDMI_INPUT_COLOR_DEPTH_8BIT;

	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_AVI_INPUT_FORMAT,
					format);
	if (ret < 0)
		DEBUG("unable to set input format\n");

	/* output format */
	format = SIIHDMI_OUTPUT_VIDEO_RANGE_COMPRESSION_AUTO |
		 SIIHDMI_OUTPUT_COLOR_STANDARD_BT601         |
		 SIIHDMI_OUTPUT_COLOR_DEPTH_8BIT;

	if (tx->connection_type == CONNECTION_TYPE_HDMI)
		format |= SIIHDMI_OUTPUT_FORMAT_HDMI_RGB;
	else
		format |= SIIHDMI_OUTPUT_FORMAT_DVI_RGB;

	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_AVI_OUTPUT_FORMAT,
					format);
	if (ret < 0)
		DEBUG("unable to set output format\n");

	/* we don't know why but solve compatibility issue with some TV */

	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_AVI_INFO_END_RIGHT_BAR_MSB, 0x00);

}

static int siihdmi_clear_avi_info_frame(struct siihdmi_tx *tx)
{
	const u8 buffer[SIIHDMI_TPI_REG_AVI_INFO_FRAME_LENGTH] = {0};
	int ret;

	BUG_ON(tx->connection_type != CONNECTION_TYPE_DVI);

	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_AVI_INFO_FRAME_BASE,
					     sizeof(buffer), buffer);
	if (ret < 0)
		DEBUG("unable to clear avi info frame\n");

	return ret;

}


static struct video_mode_map vmode_map[] = {
	{ 1, 1, 0, 60, PICTURE_ASPECT_RATIO_4_3,ACTIVE_FORMAT_DESCRIPTION_4_3_CENTERED },
	{ 640, 480, 1, 60, PICTURE_ASPECT_RATIO_UNSCALED,ACTIVE_FORMAT_DESCRIPTION_UNSCALED },
	{ 720, 480, 3, 50, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 720, 576, 17, 50, PICTURE_ASPECT_RATIO_4_3,ACTIVE_FORMAT_DESCRIPTION_4_3_CENTERED },
	{ 800, 600, 0, 60, PICTURE_ASPECT_RATIO_4_3,ACTIVE_FORMAT_DESCRIPTION_4_3_CENTERED },
	{ 1024, 600, 0, 60, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 1024, 768, 0, 60, PICTURE_ASPECT_RATIO_4_3,ACTIVE_FORMAT_DESCRIPTION_4_3_CENTERED },
	{ 1280, 720, 4, 60, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 1280, 720, 19, 50, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 1280, 768, 0, 60, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 1360, 768, 0, 60, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 1680, 1050, 0, 60, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 1920, 1080, 33, 25, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ 1920, 1080, 34, 30, PICTURE_ASPECT_RATIO_16_9,ACTIVE_FORMAT_DESCRIPTION_16_9_CENTERED },
	{ MAX_XRES+1, MAX_YRES+1, 0, 0, PICTURE_ASPECT_RATIO_UNSCALED,ACTIVE_FORMAT_DESCRIPTION_UNSCALED},
};

static int video_mode_map_get(__u32 x, __u32 y, u32 refresh)
{
	int i=0; 
	
	for( i=0; (vmode_map[i].xres < MAX_YRES); i++ ) {
		if ( x > vmode_map[i].xres )
			continue;
		if ( i && (x <= vmode_map[i].xres) && (y <= vmode_map[i].yres ) ) { /* so that 1124x644 will be become 720p to avoid overscan issue */
			if( DIFF(refresh, vmode_map[i].refresh) < 3 ) /* refresh rate diff < 3Hz */
				return i;
			else
				continue;
		}

		if ( y > vmode_map[i].yres ) /* no similar found */
			return 0;
	}
	return 0;
}


static int siihdmi_set_avi_info_frame(struct siihdmi_tx *tx,
				struct fb_var_screeninfo *var)
{
	int ret;
	u32 htotal = 0, vtotal=0, refresh_rate;
	int vmap_idx = 0;

	struct avi_info_frame avi = {
		.header = {
			.type    = INFO_FRAME_TYPE_AUXILIARY_VIDEO_INFO,
			.version = CEA861_AVI_INFO_FRAME_VERSION,
			.length  = sizeof(avi) - sizeof(avi.header),
		},

		.dbyte1_reserved0	        = 0,
		.pixel_format			= PIXEL_FORMAT_RGB,
		.active_format_info_valid       = true,
		.bar_info			= BAR_INFO_INVALID,
//	        .scan_information
		
		.colorimetry			= COLORIMETRY_UNKNOWN,
//		.colorimetry			= COLORIMETRY_BT709,
//		.picture_aspect_ratio		= PICTURE_ASPECT_RATIO_UNSCALED,
		.picture_aspect_ratio		= PICTURE_ASPECT_RATIO_16_9,
		.active_format_description      = ACTIVE_FORMAT_DESCRIPTION_UNSCALED,
		
		.it_content_present		= 0,
		.extended_colorimetry		= 0,
		.rgb_quantization_range		= 0,
		.non_uniform_picture_scaling    = 0,


		.dbyte4_reserved0	        = 0,
		.video_format      		= VIDEO_FORMAT_UNKNOWN,

		.ycc_quantizaton_range		= 0,
		.content_type			= 0,
		.pixel_repetition_factor	= 0,

		.end_of_top_bar			= 0,
		.start_of_bottom_bar		= 0,
		.end_of_left_bar		= 0,
		.start_of_right_bar		= 0,


	};

	htotal = (var->xres+var->left_margin+var->right_margin+var->hsync_len);
	vtotal = (var->yres+var->upper_margin+var->lower_margin+var->vsync_len);
	refresh_rate = 1000000/(htotal*vtotal/(PICOS2KHZ(var->pixclock)/10));

	vmap_idx = video_mode_map_get( var->xres, var->yres, refresh_rate/100 );


	avi.picture_aspect_ratio = vmode_map[vmap_idx].picture_aspect_ratio;
	avi.active_format_description = vmode_map[vmap_idx].active_format_description;
	avi.video_format = vmode_map[vmap_idx].video_code;

	BUG_ON(tx->connection_type != CONNECTION_TYPE_HDMI);

	switch (tx->pixel_mapping) {
	case PIXEL_MAPPING_UNDERSCANNED:
		avi.scan_information = SCAN_INFORMATION_UNDERSCANNED;
		break;
	case PIXEL_MAPPING_OVERSCANNED:
		avi.scan_information = SCAN_INFORMATION_OVERSCANNED;
		break;
	default:
		avi.scan_information = SCAN_INFORMATION_UNKNOWN;
		break;
	}
	
	cea861_checksum_hdmi_info_frame((u8 *) &avi);
	
	BUILD_BUG_ON(sizeof(avi) - SIIHDMI_AVI_INFO_FRAME_OFFSET != SIIHDMI_TPI_REG_AVI_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_AVI_INFO_FRAME_BASE,
					     sizeof(avi) - SIIHDMI_AVI_INFO_FRAME_OFFSET,
					     ((u8 *) &avi) + SIIHDMI_AVI_INFO_FRAME_OFFSET);

	if (ret < 0)
		DEBUG("unable to write avi info frame\n");


	return ret;
}

static int siihdmi_set_audio_info_frame(struct siihdmi_tx *tx)
{
	int ret;
	struct siihdmi_audio_info_frame packet = {
		.header = {
			.info_frame = SIIHDMI_INFO_FRAME_AUDIO,
			.repeat     = false,
			.enable     = tx->basic_audio,
		},

		.info_frame = {
			.header = {
				.type    = INFO_FRAME_TYPE_AUDIO,
				.version = CEA861_AUDIO_INFO_FRAME_VERSION,
				.length  = sizeof(packet.info_frame) - sizeof(packet.info_frame.header),
			},

			.channel_count         = CHANNEL_COUNT_REFER_STREAM_HEADER,
			.channel_allocation    = CHANNEL_ALLOCATION_STEREO,

			.format_code_extension = CODING_TYPE_REFER_STREAM_HEADER,

			/* required to Refer to Stream Header by CEA861-D */
			.coding_type           = CODING_TYPE_REFER_STREAM_HEADER,

			.sample_size           = SAMPLE_SIZE_REFER_STREAM_HEADER,
			.sample_frequency      = FREQUENCY_REFER_STREAM_HEADER,
		},
	};

	BUG_ON(tx->connection_type != CONNECTION_TYPE_HDMI);

	cea861_checksum_hdmi_info_frame((u8 *) &packet.info_frame);

	BUILD_BUG_ON(sizeof(packet) != SIIHDMI_TPI_REG_AUDIO_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_MISC_INFO_FRAME_BASE,
					     sizeof(packet),
					     (u8 *) &packet);
	if (ret < 0)
		DEBUG("unable to write audio info frame\n");

	return ret;
}

static void siihdmi_audio_mute(struct siihdmi_tx *tx, u8 mute)
{
	/*
	 * Very simple operation: read current data, mask off mute bit preserving existing data,
	 * and set the mute data. Mute needs to be called before setting registers 0x27-0x28.
	 */
	u8 data;

	data = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL);

	data &= 0xEF;

	data |= mute;

	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL, data);
}


static int siihdmi_audio_setup(struct siihdmi_tx *tx, u8 mode)
{
	siihdmi_audio_mute(tx, SIIHDMI_AUDIO_MUTE);

	/* enable audio interface (0x26) */
	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_PACKET_LAYOUT_CTRL, mode | SIIHDMI_AUDIO_MUTE);

	if (mode == SIIHDMI_AUDIO_SPDIF_ENABLE) {
		/* refer to stream header for everything for now (0x27) */
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_SAMPLING_HBR, 0);

		/* make a determination about whether we pass only basic audio or not (0x25)*/
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_ORIGINAL_FREQ_SAMPLE_LENGTH, 2);
	}
	else {

		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_ORIGINAL_FREQ_SAMPLE_LENGTH, 0);
		
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_SAMPLING_HBR, 0x59);

		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_AUDIO_RESERVED, 0x00);

		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_ENABLE_MAPPING, 0x80);

		//print(" rising,256Fs,ws-low=left,left justify,msb 1st,1 bit shift \n");
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_INPUT_CONFIGURATION, 0x90);

		// PCM format
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_CHANNEL_STATUS, 0x00);

		// 48K = 0x02, 44.1K = 0x00
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_ACCURACY_SAMPLING_FREQUENCY, 0x02); //0x22

		// 16bit=0x02 , 24b = 1011 (default), pass basic audio only = 0x00
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_I2S_ORIGINAL_FREQ_SAMPLE_LENGTH, 0x02); //0xd2

		//print(" back door reG0x24=16 BIT \n");
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_INTERNAL_REG_SET_PAGE, 0x02);
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_INTERNAL_REG_SET_OFFSET, 0x24);
		i2c_smbus_write_byte_data(tx->client, SIIHDMI_INTERNAL_REG_ACCESS, 0x02);
	}

	/* unmuting is left until after audioframes are sent so we do it later */
	return mode;
}

static int siihdmi_set_spd_info_frame(struct siihdmi_tx *tx)
{
	int ret;
	struct siihdmi_spd_info_frame packet = {
		.header = {
			.info_frame = SIIHDMI_INFO_FRAME_SPD_ACP,
			.repeat     = false,
			.enable     = true,
		},

		.info_frame = {
			.header = {
				.type    = INFO_FRAME_TYPE_SOURCE_PRODUCT_DESCRIPTION,
				.version = CEA861_SPD_INFO_FRAME_VERSION,
				.length  = sizeof(packet.info_frame) - sizeof(packet.info_frame.header),
			},

			.source_device_info = SPD_SOURCE_PC_GENERAL,
		},
	};

	BUG_ON(tx->connection_type != CONNECTION_TYPE_HDMI);

	strncpy(packet.info_frame.vendor, tx->platform->vendor,
		sizeof(packet.info_frame.vendor));

	strncpy(packet.info_frame.description, tx->platform->description,
		sizeof(packet.info_frame.description));

	cea861_checksum_hdmi_info_frame((u8 *) &packet.info_frame);

	BUILD_BUG_ON(sizeof(packet) != SIIHDMI_TPI_REG_MISC_INFO_FRAME_LENGTH);
	ret = i2c_smbus_write_i2c_block_data(tx->client,
					     SIIHDMI_TPI_REG_MISC_INFO_FRAME_BASE,
					     sizeof(packet),
					     (u8 *) &packet);
	if (ret < 0)
		DEBUG("unable to write SPD info frame\n");

	return ret;
}

static int siihdmi_set_resolution(struct siihdmi_tx *tx,
				  struct fb_var_screeninfo *var)
{
	u8 ctrl;
	int ret;

	u32 pixclk, htotal, vtotal, refresh,refresh2;

	pixclk = var->pixclock ? PICOS2KHZ(var->pixclock) : 0;
	htotal = var->xres + var->left_margin + var->hsync_len + var->right_margin;
	vtotal = var->yres + var->upper_margin + var->vsync_len + var->lower_margin;
	refresh = (pixclk * 100000ul) / (htotal * vtotal);

	refresh2 = (pixclk * 1000UL) / 1000000;


	INFO("Setting Resolution: %ux%u@%u.%u\n", var->xres, var->yres, (refresh / 100), (refresh % 100));

	ctrl = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_SYS_CTRL);

	/* setup the sink type */
	if (tx->connection_type == CONNECTION_TYPE_DVI)
		ctrl &= ~SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;
	else
		ctrl |= SIIHDMI_SYS_CTRL_OUTPUT_MODE_SELECT_HDMI;

	/* step 1: (potentially) disable HDCP */

	/* step 2: (optionally) blank the display */
	/*
	 * Note that if we set the AV Mute, switching to DVI could result in a
	 * permanently muted display until a hardware reset.  Thus only do this
	 * if the sink is a HDMI connection
	 */
	if (tx->connection_type == CONNECTION_TYPE_HDMI)
		ctrl |= SIIHDMI_SYS_CTRL_AV_MUTE_HDMI;
	/* optimisation: merge the write into the next one */

	/* step 3: prepare for resolution change */
	ctrl |= SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
	if (ret < 0)
		DEBUG("unable to prepare for resolution change\n");

	tx->tmds_enabled = false;

	msleep(SIIHDMI_CTRL_INFO_FRAME_DRAIN_TIME);

	/* step 4: change video resolution */

	/* step 5: set the vmode registers */
	siihdmi_set_vmode_registers(tx, var);

	/*
	 * step 6:
	 *      [DVI]  clear AVI InfoFrame
	 *      [HDMI] set AVI InfoFrame
	 */
	if (tx->connection_type == CONNECTION_TYPE_HDMI)
		siihdmi_set_avi_info_frame(tx,var);
	else
		siihdmi_clear_avi_info_frame(tx);

	/* step 7: [HDMI] set new audio information */
	if (tx->connection_type == CONNECTION_TYPE_HDMI) {
		if (tx->basic_audio) {
			siihdmi_audio_setup(tx, SIIHDMI_AUDIO_I2S_ENABLE);
			siihdmi_set_audio_info_frame(tx);
			siihdmi_audio_mute(tx, SIIHDMI_AUDIO_UNMUTE);
		}
		siihdmi_set_spd_info_frame(tx);
	}

	/* step 8: enable display */
	ctrl &= ~SIIHDMI_SYS_CTRL_TMDS_OUTPUT_POWER_DOWN;
	/* optimisation: merge the write into the next one */

	/* step 9: (optionally) un-blank the display */
	ctrl &= ~SIIHDMI_SYS_CTRL_AV_MUTE_HDMI;
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_SYS_CTRL,
					ctrl);
	if (ret < 0)
		DEBUG("unable to enable the display\n");

	/* step 10: (potentially) enable HDCP */

	/* step 11: power up transmitter */
	ret = i2c_smbus_write_byte_data(tx->client,
					SIIHDMI_TPI_REG_PWR_STATE,
					SIIHDMI_POWER_STATE_D0);
	if (ret < 0) {
		ERROR("unable to power up transmitter\n");
		return ret;
	}

	tx->tmds_enabled = true;

	return ret;
}

static void siihdmi_dump_single_modeline(const struct siihdmi_tx *tx, const struct fb_videomode *mode, char flag)
{
	const bool interlaced = (mode->vmode & FB_VMODE_INTERLACED);
	const bool double_scan = (mode->vmode & FB_VMODE_DOUBLE);
	u32 pixclk = mode->pixclock ? PICOS2KHZ(mode->pixclock) : 0;

	pixclk >>= (double_scan ? 1 : 0);

	if (!flag) flag = ' ';

	INFO("  %c \"%dx%d@%d%s\" %lu.%.2lu %u %u %u %u %u %u %u %u %chsync %cvsync\n",
	     /* list CEA or preferred status of modeline */
	     flag,
	     /* mode name */
	     mode->xres, mode->yres,
	     mode->refresh << (interlaced ? 1 : 0),
	     interlaced ? "i" : (double_scan ? "d" : ""),

	     /* dot clock frequency (MHz) */
	     pixclk / 1000ul,
	     pixclk % 1000ul,

	     /* horizontal timings */
	     mode->xres,
	     mode->xres + mode->right_margin,
	     mode->xres + mode->right_margin + mode->hsync_len,
	     mode->xres + mode->right_margin + mode->hsync_len + mode->left_margin,

	     /* vertical timings */
	     mode->yres,
	     mode->yres + mode->lower_margin,
	     mode->yres + mode->lower_margin + mode->vsync_len,
	     mode->yres + mode->lower_margin + mode->vsync_len + mode->upper_margin,

	     /* sync direction */
	     (mode->sync & FB_SYNC_HOR_HIGH_ACT) ? '+' : '-',
	     (mode->sync & FB_SYNC_VERT_HIGH_ACT) ? '+' : '-');
}

static void siihdmi_dump_modelines(struct siihdmi_tx *tx)
{
	const struct list_head * const modelines = &tx->info->modelist;
	const struct fb_modelist *entry;

	INFO("Supported modelines:\n");
	list_for_each_entry(entry, modelines, list) {
		char flag = ' ';

		if (fb_mode_is_equal(&tx->preferred, &entry->mode)) {
			flag = '*';
		}
		if (entry->mode.flag & FB_MODE_IS_CEA) {
			flag = 'C';
		}

		siihdmi_dump_single_modeline(tx, &entry->mode, flag);
	}
}

static const struct fb_videomode *
_fb_match_resolution(const struct fb_videomode * const mode,
		     struct list_head *head)
{
	const struct fb_modelist *entry, *next;

	list_for_each_entry_safe(entry, next, head, list) {
		if (fb_res_is_equal(mode, &entry->mode))
			return &entry->mode;
	}

	return NULL;
}

#define REASON_INTERLACED	1
#define REASON_DOUBLE		2
#define REASON_PIXCLOCK		3
#define REASON_MARGIN		4
#define REASON_DETAILMATCH	5
#define REASON_CEAMATCH		6

static char removal_codes[] = {
	/* lower case only please */
	0, 'i', 'd', 'p', 'v', 'm', 'c',
};

static void siihdmi_sanitize_modelist(struct siihdmi_tx * const tx)
{
	struct list_head *modelist = &tx->info->modelist;
	const struct fb_modelist *entry, *next;
	const struct fb_videomode *mode;
	int num_removed = 0;

	if ((mode = fb_find_best_display(&tx->info->monspecs, modelist)))
		memcpy(&tx->preferred, mode, sizeof(tx->preferred));

	/*
	 * Prefer detailed timings found in EDID.  Certain sinks support slight
	 * variations of VESA/CEA timings, and using those allows us to support
	 * a wider variety of devieces.
	 */
	list_for_each_entry_safe(entry, next, modelist, list) {
		int remove = 0;

		mode = &entry->mode;
		if (mode->vmode & FB_VMODE_INTERLACED) {
			remove = REASON_INTERLACED;
		} else if (mode->vmode & FB_VMODE_DOUBLE) {
			remove = REASON_DOUBLE;
		} else if (mode->pixclock < tx->platform->pixclock) {
			remove = REASON_PIXCLOCK;
		} else if (mode->lower_margin < 2) {
			/*
			 * HDMI specification requires at least 2 lines of
			 * vertical sync (sect. 5.1.2).
			 */
			remove = REASON_MARGIN;
		} else {
			const struct fb_videomode *match =
				_fb_match_resolution(mode, modelist);

			if (match && (~(mode->flag) & FB_MODE_IS_DETAILED) &&
				     (match->flag & FB_MODE_IS_DETAILED)) {
				remove = REASON_DETAILMATCH;
			} else if (match && (~(mode->flag) & FB_MODE_IS_CEA) &&
					    (match->flag & FB_MODE_IS_CEA)) {
				remove = REASON_CEAMATCH;
			}
		}

		if (remove > 0) {
			struct fb_modelist *modelist =
				container_of(mode, struct fb_modelist, mode);

			if (num_removed == 0) { // first time only
				INFO("Unsupported modelines:\n");
			}

			siihdmi_dump_single_modeline(tx, mode,
					fb_mode_is_equal(&tx->preferred, mode) ?
							removal_codes[remove] - 32 : /* hack to capitalize the code */
							removal_codes[remove] );

			list_del(&modelist->list);
			kfree(&modelist->list);
			num_removed++;
		}
	}

	if (num_removed > 0) {
		INFO("Removed %u modes due to incompatibilities\n", num_removed);
	}
}

static const struct fb_videomode *
siihdmi_select_video_mode(const struct siihdmi_tx * const tx)
{
	const struct fb_videomode *mode = NULL;
	const struct fb_videomode * const def = &cea_modes[4];

	if (teneighty) {
		int i;
		/*
		 * search the CEA modes 32, 33, 34 in the modelist, since they represent 1080p
		 * at 24, 25 and 30Hz respectively (with 74.250MHz clock rate). Do it backwards
		 * do we pick the highest field rate first if possible.
		 */
		for (i = 34; i >= 32; i--) {
			mode = fb_find_nearest_mode(&cea_modes[i], &tx->info->modelist);
			if (mode && (mode->xres == cea_modes[i].xres) && (mode->yres == cea_modes[i].yres))
				return mode;
		}
	}

	if (seventwenty) {
		/* prefer default mode if the monitor supports that mode exactly */
		mode = fb_find_nearest_mode(def, &tx->info->modelist);

		if (mode && (mode->xres == def->xres) && (mode->yres == def->yres)) {
			return mode;
		}
	}

	if (tx->preferred.xres && tx->preferred.yres) {
		/* otherwise, use the closest to the monitor preferred mode */
		mode = fb_find_nearest_mode(&tx->preferred, &tx->info->modelist);
	}

	/* if no mode was found push 1280x720 anyway */
	mode = mode ? mode : &cea_modes[4];

#if defined(CONFIG_MACH_MX51_NA04)
	/*
	 * At modes somewhat greater than 1280x720 (1280x768, 1280x800 may be
	 * fine) doing heavy video work like playing a 720p movie may cause the
	 * IPU to black the screen intermittently due to lack of IPU bandwidth.
	 */
	if ((mode->xres > 1024) && (mode->yres > 720))
		WARNING("available video bandwidth may be very low at the "
			"selected resolution (%ux%u@%u) and may cause IPU errors\n",
			mode->xres, mode->yres, mode->refresh);
#endif

	return mode;
}

static inline void na04_set_di_fmt(struct fb_info *fbi, unsigned int fmt)
{
	mm_segment_t old_fs;

	if (fbi->fbops->fb_ioctl) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fbi->fbops->fb_ioctl(fbi, MXCFB_SET_DIFMT, (unsigned long)&fmt);
		set_fs(old_fs);
	}
}

static int siihdmi_setup_display(struct siihdmi_tx *tx)
{
	struct fb_var_screeninfo var = {0};
	struct edid_block0 block0;
	int ret;

	BUILD_BUG_ON(sizeof(struct edid_block0) != EDID_BLOCK_SIZE);
	BUILD_BUG_ON(sizeof(struct edid_extension) != EDID_BLOCK_SIZE);

	/* defaults */
	tx->connection_type = CONNECTION_TYPE_DVI;
	tx->pixel_mapping = PIXEL_MAPPING_EXACT;
	tx->basic_audio = false;

	/* use EDID to detect sink characteristics */
	if ((ret = siihdmi_read_edid(tx, (u8 *) &block0, sizeof(block0))) < 0)
		return ret;

	if (!edid_verify_checksum((u8 *) &block0))
		WARNING("EDID block 0 CRC mismatch\n");

	/* need to allocate space for block 0 as well as the extensions */
	tx->edid_length = (block0.extensions + 1) * EDID_BLOCK_SIZE;

	tx->edid = kzalloc(tx->edid_length, GFP_KERNEL);
	if (!tx->edid)
		return -ENOMEM;

	if ((ret = siihdmi_read_edid(tx, tx->edid, tx->edid_length)) < 0)
		return ret;

	/* create monspecs from EDID for the basic stuff */
	fb_edid_to_monspecs(tx->edid, &tx->info->monspecs);
	fb_videomode_to_modelist(tx->info->monspecs.modedb,
				 tx->info->monspecs.modedb_len,
				 &tx->info->modelist);

	if (block0.extensions) {
		const struct edid_extension * const extensions =
			(struct edid_extension *) (tx->edid + sizeof(block0));
		u8 i;

		for (i = 0; i < block0.extensions; i++) {
			const struct edid_extension * const extension =
				&extensions[i];

			if (!edid_verify_checksum((u8 *) extension))
				WARNING("EDID block %u CRC mismatch\n", i);

			switch (extension->tag) {
			case EDID_EXTENSION_CEA:
				siihdmi_parse_cea861_timing_block(tx,
								  extension);
				break;
			default:
				break;
			}
		}
	}

	if (sysfs_create_bin_file(&tx->info->dev->kobj, &tx->edid_attr) < 0) {
		WARNING("unable to populate edid sysfs attribute\n");
	}

	if (tx->basic_audio) {
		if (sysfs_create_bin_file(&tx->info->dev->kobj, &tx->audio_attr) < 0) {
			WARNING("unable to populate audio sysfs attribute\n");
		}
	}

	siihdmi_sanitize_modelist(tx);
	siihdmi_dump_modelines(tx);

	fb_videomode_to_var(&var, siihdmi_select_video_mode(tx));

	if ((ret = siihdmi_set_resolution(tx, &var)) < 0)
		return ret;

	if (tx->platform->power_lvds)
		tx->platform->power_lvds(LVDS_OFF);

	na04_set_di_fmt(tx->info,IPU_PIX_FMT_RGB24);

	/* activate the framebuffer */
	var.activate = FB_ACTIVATE_ALL;

	var.bits_per_pixel = 32;

	acquire_console_sem();
	tx->info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(tx->info, &var);
	tx->info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();

	return 0;
}

static int siihdmi_blank(struct siihdmi_tx *tx, struct fb_var_screeninfo *var, int powerdown)
{
	u8 data;

	/* TODO Power down TMDS if the flag is set */

	data = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_RQB);

	return i2c_smbus_write_byte_data(tx->client,
					 SIIHDMI_TPI_REG_RQB,
					 data | SIIHDMI_RQB_FORCE_VIDEO_BLANK);
}

static int siihdmi_unblank(struct siihdmi_tx *tx, struct fb_var_screeninfo *var)
{
	u8 data;

	/* TODO Power up TMDS if the tx->tmds_enabled is false */

	data = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_RQB);

	return i2c_smbus_write_byte_data(tx->client,
					 SIIHDMI_TPI_REG_RQB,
					 data & ~SIIHDMI_RQB_FORCE_VIDEO_BLANK);
}

static int siihdmi_fb_event_handler(struct notifier_block *nb,
				    unsigned long val,
				    void *v)
{
	const struct fb_event * const event = v;
	struct siihdmi_tx * const tx = container_of(nb, struct siihdmi_tx, nb);
	static struct fb_var_screeninfo *var;

	var = &event->info->var;


	if (strcmp(event->info->fix.id, tx->platform->framebuffer)) {
		return 0;
	}

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
//		DEBUG("NA04 HDMI REGISTERED\n");
		if (!strcmp(event->info->fix.id, tx->platform->framebuffer)) {
			tx->info = event->info;
			return siihdmi_setup_display(tx);
		}

		break;
	case FB_EVENT_MODE_CHANGE:
		fb_videomode_to_var(var, event->info->mode);
		if(hdmi_device_detected)
			return siihdmi_set_resolution(tx, var);
		break;
	case FB_EVENT_BLANK:
		fb_videomode_to_var(var, event->info->mode);
		if(!hdmi_device_detected)
			break;

		switch (*((int *) event->data)) {

		case FB_BLANK_POWERDOWN:
			return siihdmi_blank(tx, var, 1);
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			return siihdmi_blank(tx, var, 0);
		case FB_BLANK_UNBLANK:
			return siihdmi_unblank(tx, var);
		}

		break;


	case FB_EVENT_RESUME:
		fb_videomode_to_var(var, event->info->mode);
		if(hdmi_device_detected)
			return siihdmi_set_resolution(tx, var);
		break;

	default:
		DEBUG("unhandled fb event 0x%lx", val);
		break;
	}

	return 0;
}

#if defined(CONFIG_FB_SIIHDMI_HOTPLUG)
static irqreturn_t siihdmi_hotplug_handler(int irq, void *dev_id)
{
	struct siihdmi_tx *tx = ((struct siihdmi_tx *) dev_id);

	schedule_delayed_work(&tx->hotplug,
			      msecs_to_jiffies(SIIHDMI_HOTPLUG_HANDLER_TIMEOUT));

	return IRQ_HANDLED;
}

static void siihdmi_hotplug_event(struct work_struct *work)
{
	struct siihdmi_tx *tx =
		container_of(work, struct siihdmi_tx, hotplug.work);
	u8 isr, ier;

	/* clear the interrupt */
	ier = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_IER);
	i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_IER, ier);

	DEBUG("hotplug event(s) received: %s%s%s%s%s%s\b\b\n",
	      (ier & SIIHDMI_IER_HOT_PLUG_EVENT) ? "hotplug, " : "",
	      (ier & SIIHDMI_IER_RECEIVER_SENSE_EVENT) ? "receiver, " : "",
	      (ier & SIIHDMI_IER_CTRL_BUS_EVENT) ? "control bus, " : "",
	      (ier & SIIHDMI_IER_CPI_EVENT) ? "CPI, " : "",
	      (ier & SIIHDMI_IER_AUDIO_EVENT) ? "audio error, " : "",
	      (ier & (SIIHDMI_IER_SECURITY_STATUS_CHANGE |
	              SIIHDMI_IER_HDCP_VALUE_READY       |
	              SIIHDMI_IER_HDCP_AUTHENTICATION_STATUS_CHANGE)) ? "HDCP, " : "");

	/* TODO Handle Events */

	isr = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_ISR);
	DEBUG("polled event(s) received: %s%s%s%s%s%s\b\b\n",
	      (isr & SIIHDMI_ISR_HOT_PLUG_EVENT) ? "hotplug, " : "",
	      (isr & SIIHDMI_ISR_RECEIVER_SENSE_EVENT) ? "receiver, " : "",
	      (isr & SIIHDMI_ISR_CTRL_BUS_EVENT) ? "control bus, " : "",
	      (isr & SIIHDMI_ISR_CPI_EVENT) ? "CPI, " : "",
	      (isr & SIIHDMI_ISR_AUDIO_EVENT) ? "audio error, " : "",
	      (isr & (SIIHDMI_ISR_SECURITY_STATUS_CHANGED |
	              SIIHDMI_ISR_HDCP_VALUE_READY        |
	              SIIHDMI_ISR_HDCP_AUTHENTICATION_STATUS_CHANGED)) ? "HDCP, " : "");
}


static void na04_manage_irq(struct work_struct *work){
	u8 Status;	
	struct siihdmi_tx *tx =
		container_of(work, struct siihdmi_tx, hotplug.work);

	Status = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_ISR);
	if( Status & SIIHDMI_ISR_HOT_PLUG_EVENT) {
		// Repeat this loop while cable is bouncing:
		do {
			i2c_smbus_write_byte_data(tx->client, SIIHDMI_TPI_REG_ISR, SIIHDMI_ISR_HOT_PLUG_EVENT);
			Status = i2c_smbus_read_byte_data(tx->client, SIIHDMI_TPI_REG_ISR);    // Read Interrupt status register
		} while (Status & SIIHDMI_ISR_HOT_PLUG_EVENT);              // loop as long as HP interrupts recur

		if(Status & 0x04){
			DEBUG("na04 HDMI Device detected\n");
			hdmi_device_detected = true;
			if(tx->info)
				{
				acquire_console_sem();
				fb_blank(tx->info,FB_BLANK_NORMAL);
				release_console_sem();
				if (tx->platform->power_lvds)
					tx->platform->power_lvds(LVDS_OFF);
				na04_set_di_fmt(tx->info,IPU_PIX_FMT_RGB24);
				
				acquire_console_sem();
				fb_blank(tx->info,FB_BLANK_UNBLANK);
				release_console_sem();
				}
	
		}
		else{
			DEBUG("na04 HDMI Device Removed\n");
			hdmi_device_detected = false;
			if(tx->info){
				acquire_console_sem();
				fb_blank(tx->info,FB_BLANK_NORMAL);
				release_console_sem();
				na04_set_di_fmt(tx->info,IPU_PIX_FMT_LVDS666);
				acquire_console_sem();
				fb_blank(tx->info,FB_BLANK_UNBLANK);
				release_console_sem();
				if (tx->platform->power_lvds)
					tx->platform->power_lvds(LVDS_ON);
			}
		}
	}
}

#endif

static ssize_t siihdmi_sysfs_read_edid(struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buf, loff_t off, size_t count)
{
	const struct siihdmi_tx * const tx =
		container_of(bin_attr, struct siihdmi_tx, edid_attr);

	return memory_read_from_buffer(buf, count, &off,
				       tx->edid, tx->edid_length);
}

static ssize_t siihdmi_sysfs_read_audio(struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buf, loff_t off, size_t count)
{
	const struct siihdmi_tx * const tx =
		container_of(bin_attr, struct siihdmi_tx, audio_attr);

	return memory_read_from_buffer(buf, count, &off, tx->basic_audio ? (void *) "hdmi" : (void *) "none", 4);
}

static inline unsigned long __irq_flags(const struct resource * const res)
{
	return (res->flags & IRQF_TRIGGER_MASK);
}

static int __devinit siihdmi_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct siihdmi_tx *tx;
	int i, ret;

	DEBUG("NA04 HDMI Probe\n");

	tx = kzalloc(sizeof(struct siihdmi_tx), GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx->client = client;
	tx->platform = client->dev.platform_data;

	tx->edid_attr.attr.name  = "edid";
	tx->edid_attr.attr.owner = THIS_MODULE;
	tx->edid_attr.attr.mode  = 0444;
	/* maximum size of EDID, not necessarily the size of our data */
	tx->edid_attr.size       = SZ_32K;
	tx->edid_attr.read       = siihdmi_sysfs_read_edid;

	tx->audio_attr.attr.name = "audio";
	tx->audio_attr.attr.owner = THIS_MODULE;
	tx->audio_attr.attr.mode  = 0444;
	/* we only want to return the value "hdmi" */
	tx->audio_attr.size       = 4;
	tx->audio_attr.read       = siihdmi_sysfs_read_audio;


	i2c_set_clientdata(client, tx);

	/* initialise the device */
	if ((ret = siihdmi_initialise(tx)) < 0)
		goto error2;


#if defined(CONFIG_FB_SIIHDMI_HOTPLUG)
//	INIT_DELAYED_WORK(&tx->hotplug, siihdmi_hotplug_event);
	

	set_irq_type(tx->platform->hotplug.start,IRQF_TRIGGER_FALLING);

/*	ret = request_irq(tx->platform->hotplug.start, siihdmi_hotplug_handler,
			  __irq_flags(&tx->platform->hotplug),
			  tx->platform->hotplug.name, tx);

	ret = request_irq(tx->platform->hotplug.start, siihdmi_hotplug_handler, 0, "hdmi_int", tx);
*/
	ret = request_irq(tx->platform->hotplug.start, siihdmi_hotplug_handler,
			 __irq_flags(&tx->platform->hotplug),
			 tx->platform->hotplug.name, tx);


	if (ret < 0)
		WARNING("failed to setup hotplug interrupt: %d\n", ret);

	else
		INIT_DELAYED_WORK(&tx->hotplug, na04_manage_irq);
		
#endif

	for (i = 0; i < num_registered_fb; i++) {
		struct fb_info * const info = registered_fb[i];
		if (!strcmp(info->fix.id, tx->platform->framebuffer)) {
			tx->info = info;
			break;
		}
	}


	siihdmi_sink_present(tx);

	if (hdmi_device_detected) {
		if (siihdmi_setup_display(tx) < 0)
			goto error;
	} else {
		/*
		 * A sink is not currently present.  However, the device has
		 * been initialised.  This includes setting up the crucial IER.
		 * As a result, we can power down the transmitter and be
		 * signalled when the sink state changes.
		 */
//		ret = i2c_smbus_write_byte_data(tx->client,
//						SIIHDMI_TPI_REG_PWR_STATE,
//						SIIHDMI_POWER_STATE_D3);
//		if (ret < 0)
			WARNING("unable to change to a low power-state\n");
	}

	

	/* register a notifier for future fb events */
	tx->nb.notifier_call = siihdmi_fb_event_handler;
	fb_register_client(&tx->nb);
	DEBUG("NA04 HDMI End Probe\n");

	return 0;

error:
#if defined(CONFIG_FB_SIIHDMI_HOTPLUG)
	if (tx->platform->hotplug.start)
		free_irq(tx->platform->hotplug.start, NULL);
#endif

error2:
	i2c_set_clientdata(client, NULL);
	kfree(tx);
	DEBUG("NA04 No HDMI port available \n");
	return ret;
}

static int __devexit siihdmi_remove(struct i2c_client *client)
{
	struct siihdmi_tx *tx;

	tx = i2c_get_clientdata(client);
	if (tx) {
#if defined(CONFIG_FB_SIIHDMI_HOTPLUG)
		if (tx->platform->hotplug.start)
			free_irq(tx->platform->hotplug.start, NULL);
#endif

		sysfs_remove_bin_file(&tx->info->dev->kobj, &tx->edid_attr);

		if (tx->basic_audio)
			sysfs_remove_bin_file(&tx->info->dev->kobj, &tx->audio_attr);

		if (tx->edid)
			kfree(tx->edid);

		fb_unregister_client(&tx->nb);
		i2c_set_clientdata(client, NULL);
		kfree(tx);
	}

	return 0;
}

static int siihdmi_suspend(struct i2c_client *client, pm_message_t message)
{
	DEBUG("Go to suspend \n");
	return 0;
}

static int siihdmi_resume(struct i2c_client *client)
{
	struct siihdmi_tx *tx;
	int ret;

	DEBUG("Resume from suspend mode \n");

	tx = i2c_get_clientdata(client);
	
	if (tx) {
		/* initialise the device */
		if ((ret = siihdmi_initialise(tx)) < 0)
			return 1;
	}
	return 0;
}

static const struct i2c_device_id siihdmi_device_table[] = {
	{ "sii9022", 0 },
	{ },
};

static struct i2c_driver siihdmi_driver = {
	.driver   = { .name = "sii9022" },
	.probe    = siihdmi_probe,
	.remove   = siihdmi_remove,
	.suspend  =  siihdmi_suspend,
	.resume   =  siihdmi_resume,
	.id_table = siihdmi_device_table,
};

static int __init siihdmi_init(void)
{
	return i2c_add_driver(&siihdmi_driver);
}

static void __exit siihdmi_exit(void)
{
	i2c_del_driver(&siihdmi_driver);
}

/* Module Information */
MODULE_AUTHOR("Saleem Abdulrasool <compnerd@compnerd.org>");
MODULE_LICENSE("BSD-3");
MODULE_DESCRIPTION("Silicon Image SiI9xxx TMDS Driver");
MODULE_DEVICE_TABLE(i2c, siihdmi_device_table);

module_init(siihdmi_init);
module_exit(siihdmi_exit);

