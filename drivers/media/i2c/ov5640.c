/*
 * Omnivision OV5640 CMOS Image Sensor driver
 *
 * 
 *
 * 
 *
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>
#include "ov5640.h"

#define DRIVER_NAME "ov5640"


#define REG_SOFTWARE_STANDBY		0x0100
#define REG_SC_PLL_CTRL0		0x3003
#define REG_SC_PLL_CTRL1		0x3004
#define REG_SC_PLL_CTRL2		0x3005
#define REG_SC_PLL_CTRL3		0x3006
#define REG_SC_CHIP_ID_H		0x300a
#define REG_SC_CHIP_ID_L		0x300b
#define REG_FORMAT_CTRL00		0x4300
#define REG_PRE_ISP_CTRL00		0x50a0
#define TEST_PATTERN_ENABLE		BIT(7)
#define VERTICAL_COLOR_BAR_MASK		0x53
#define REG_NULL			0x0000	/* Array end token */

#define OV564X_ID(_msb, _lsb)		((_msb) << 8 | (_lsb))
#define OV5640_ID			0x5640

struct sensor_register {
	u16 addr;
	u8 value;
};

struct ov5640_framesize {
	u16 width;
	u16 height;
	u16 max_exp_lines;
	const struct sensor_register *regs;
};

struct ov5640_pll_ctrl {
	u8 ctrl1;
	u8 ctrl2;
	u8 ctrl3;
};

struct ov5640_pixfmt {
	u32 code;
	/* Output format Register Value (REG_FORMAT_CTRL00) */
	struct sensor_register *format_ctrl_regs;
};

struct pll_ctrl_reg {
	unsigned int div;
	unsigned char reg;
};

struct ov5640 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	unsigned int xvclk_frequency;
	const struct ov5640_platform_data *pdata;
	struct mutex lock;
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_frequency;
	const struct ov5640_framesize *frame_size;
	struct sensor_register *format_ctrl_regs;
	struct ov5640_pll_ctrl pll;
	int streaming;
	/* used to control the sensor powerdownN pin */
	struct gpio_desc *pwrdn_gpio;
};

static const struct sensor_register ov5640_init_regs[] = {
	{0x3103, 0x11},
	{0x3008, 0x82},
	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x3017, 0xff},
	{0x3018, 0xff},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x46},
	{0x3037, 0x12},
	{0x3108, 0x01},
	{0x3630, 0x36},
	{0x3631, 0x0e},
	{0x3632, 0xe2},
	{0x3633, 0x12},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x12},
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x302d, 0x60},
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x40},
	{0x3622, 0x01},
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x08},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3820, 0x41},
	{0x3821, 0x07},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9b},
	{0x3808, 0x03},
	{0x3809, 0x20},
	{0x380a, 0x02},
	{0x380b, 0x58},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x300e, 0x58},
	{0x302e, 0x00},
	{0x4740, 0x20},
	{0x4300, 0x30},
	{0x501f, 0x00},
	{0x4713, 0x03},
	{0x4407, 0x04},
	{0x440e, 0x00},
	{0x460b, 0x35},
	{0x460c, 0x20},
	{0x4837, 0x22},
	{0x3824, 0x02},
	{0x5000, 0xa7},
	{0x5001, 0xa3},
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x09},
	{0x5187, 0x09},
	{0x5188, 0x09},
	{0x5189, 0x75},
	{0x518a, 0x54},
	{0x518b, 0xe0},
	{0x518c, 0xb2},
	{0x518d, 0x42},
	{0x518e, 0x3d},
	{0x518f, 0x56},
	{0x5190, 0x46},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x5384, 0x0a},
	{0x5385, 0x7e},
	{0x5386, 0x88},
	{0x5387, 0x7c},
	{0x5388, 0x6c},
	{0x5389, 0x10},
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x10},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	{0x5800, 0x23},
	{0x5801, 0x14},
	{0x5802, 0x0f},
	{0x5803, 0x0f},
	{0x5804, 0x12},
	{0x5805, 0x26},
	{0x5806, 0x0c},
	{0x5807, 0x08},
	{0x5808, 0x05},
	{0x5809, 0x05},
	{0x580a, 0x08},
	{0x580b, 0x0d},
	{0x580c, 0x08},
	{0x580d, 0x03},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x09},
	{0x5812, 0x07},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x01},
	{0x5816, 0x03},
	{0x5817, 0x08},
	{0x5818, 0x0d},
	{0x5819, 0x08},
	{0x581a, 0x05},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x0e},
	{0x581e, 0x29},
	{0x581f, 0x17},
	{0x5820, 0x11},
	{0x5821, 0x11},
	{0x5822, 0x15},
	{0x5823, 0x28},
	{0x5824, 0x46},
	{0x5825, 0x26},
	{0x5826, 0x08},
	{0x5827, 0x26},
	{0x5828, 0x64},
	{0x5829, 0x26},
	{0x582a, 0x24},
	{0x582b, 0x22},
	{0x582c, 0x24},
	{0x582d, 0x24},
	{0x582e, 0x06},
	{0x582f, 0x22},
	{0x5830, 0x40},
	{0x5831, 0x42},
	{0x5832, 0x24},
	{0x5833, 0x26},
	{0x5834, 0x24},
	{0x5835, 0x22},
	{0x5836, 0x22},
	{0x5837, 0x26},
	{0x5838, 0x44},
	{0x5839, 0x24},
	{0x583a, 0x26},
	{0x583b, 0x28},
	{0x583c, 0x42},
	{0x583d, 0xce},
	{0x5025, 0x00},
	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a1b, 0x30},
	{0x3a1e, 0x26},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},
	{0x3008, 0x02},
	{0x302c, 0xc2},
	{ REG_NULL, 0x00 },
};

/* 1280X720 720p */
static struct sensor_register ov5640_720p[] = {
	{0x3503, 0x00},
		
	{0x3c07,0x07},
	{0x3803,0xfa},
	{0x3806,0x06},////
	{0x3807,0xa9},
	{0x3808,0x05},
	{0x3809,0x00},
	{0x380a,0x02},
	{0x380b,0xd0},
	{0x380c,0x07},
	{0x380d,0x64},
	{0x380e,0x02},
	{0x380f,0xe4},
	{0x3813,0x04},
	{0x3a02,0x02},
	{0x3a03,0xe4},
	{0x3a08,0x01},///
	{0x3a09,0xbc},////
	{0x3a0a,0x01},///
	{0x3a0b,0x72},////
	{0x3a0e,0x01},
	{0x3a0d,0x02},
	{0x3a14,0x02},
	{0x3a15,0xe4},

    {0x3820, 0x41}, //ddl@rock-chips.com add start: qsxvga -> 720p isn't stream on 
	{0x3821, 0x07},
	{0x3814, 0x31},
	{0x3815, 0x31},
	
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08 ,0x01},///
	{0x3a09, 0x27},///
	{0x3a0a, 0x00},///
	{0x3a0b, 0xf6},///
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4004, 0x02},
	{0x3002, 0x1c},////
	{0x4713, 0x03},//////ddl@rock-chips.com add end
    
	{0x3002,0x00},///
	{0x4713,0x02},///
	{0x4837,0x16},
	{0x3824,0x04},///
	{0x5001,0x83},
	{0x3035,0x21},
	{0x3036,0x46},

	{0x4837, 0x22},
	{0x5001, 0xa3},
	
	{ REG_NULL, 0x00 },
};

/* 1600X1200 UXGA */
static struct sensor_register ov5640_uxga[] = {

	{ REG_NULL, 0x00 },
};

/* 1280X1024 SXGA */
static struct sensor_register ov5640_sxga[] = {

	{ REG_NULL, 0x00 },
};

/* 1024X768 SXGA */
static struct sensor_register ov5640_xga[] = {

	{ REG_NULL, 0x00 },
};

/* 800X600 SVGA */
static struct sensor_register ov5640_svga[] = {
	{0x3503, 0x00},
	{0x3c07, 0x08},
	{0x3820, 0x41},
	{0x3821, 0x07},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3803, 0x04},
	{0x3806, 0x07},///
	{0x3807, 0x9b},
	{0x3808, 0x03},
	{0x3809, 0x20},
	{0x380a, 0x02},
	{0x380b, 0x58},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08 ,0x01},///
	{0x3a09, 0x27},///
	{0x3a0a, 0x00},///
	{0x3a0b, 0xf6},///
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4004, 0x02},
	{0x3002, 0x1c},////
	{0x4713, 0x03},////
	{0x3035, 0x21},
	{0x3036, 0x46},
	{0x4837, 0x22},
	{0x3824, 0x02},////
	{0x5001, 0xa3},

	{ REG_NULL, 0x00 },
};

/* 640X480 VGA */
static struct sensor_register ov5640_vga[] = {

	{ REG_NULL, 0x00 },
};

/* 320X240 QVGA */
static  struct sensor_register ov5640_qvga[] = {

	{ REG_NULL, 0x00 },
};

static const struct pll_ctrl_reg ctrl3[] = {
	{ 1, 0x00 },
	{ 2, 0x02 },
	{ 3, 0x03 },
	{ 4, 0x06 },
	{ 6, 0x0d },
	{ 8, 0x0e },
	{ 12, 0x0f },
	{ 16, 0x12 },
	{ 24, 0x13 },
	{ 32, 0x16 },
	{ 48, 0x1b },
	{ 64, 0x1e },
	{ 96, 0x1f },
	{ 0, 0x00 },
};

static const struct pll_ctrl_reg ctrl1[] = {
	{ 2, 0x10 },
	{ 4, 0x20 },
	{ 6, 0x30 },
	{ 8, 0x40 },
	{ 10, 0x50 },
	{ 12, 0x60 },
	{ 14, 0x70 },
	{ 16, 0x80 },
	{ 18, 0x90 },
	{ 20, 0xa0 },
	{ 22, 0xb0 },
	{ 24, 0xc0 },
	{ 26, 0xd0 },
	{ 28, 0xe0 },
	{ 30, 0xf0 },
	{ 0, 0x00 },
};

static const struct ov5640_framesize ov5640_framesizes[] = {
	{ /* QVGA */
		.width		= 320,
		.height		= 240,
		.regs		= ov5640_qvga,
		.max_exp_lines	= 248,
	}, { /* VGA */
		.width		= 640,
		.height		= 480,
		.regs		= ov5640_vga,
		.max_exp_lines	= 498,
	}, { /* SVGA */
		.width		= 800,
		.height		= 600,
		.regs		= ov5640_svga,
		.max_exp_lines	= 498,
	}, { /* XGA */
		.width		= 1024,
		.height		= 768,
		.regs		= ov5640_xga,
		.max_exp_lines	= 498,
	}, { /* 720P */
		.width		= 1280,
		.height		= 720,
		.regs		= ov5640_720p,
		.max_exp_lines	= 498,
	}, { /* SXGA */
		.width		= 1280,
		.height		= 1024,
		.regs		= ov5640_sxga,
		.max_exp_lines	= 1048,
	}, { /* UXGA */
		.width		= 1600,
		.height		= 1200,
		.regs		= ov5640_uxga,
		.max_exp_lines	= 498,
	},
};

/* YUV422 YUYV*/
static struct sensor_register ov5640_format_yuyv[] = {
	{ REG_FORMAT_CTRL00, 0x30 },
	{ REG_NULL, 0x0 },
};

/* YUV422 UYVY  */
static struct sensor_register ov5640_format_uyvy[] = {
	{ REG_FORMAT_CTRL00, 0x32 },
	{ REG_NULL, 0x0 },
};

/* Raw Bayer BGGR */
static struct sensor_register ov5640_format_bggr[] = {
	{ REG_FORMAT_CTRL00, 0x00 },
	{ REG_NULL, 0x0 },
};

/* RGB565 */
static struct sensor_register ov5640_format_rgb565[] = {
	{ REG_FORMAT_CTRL00, 0x60 },
	{ REG_NULL, 0x0 },
};

static const struct ov5640_pixfmt ov5640_formats[] = {
	{
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
		.format_ctrl_regs = ov5640_format_yuyv,
	}, {
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.format_ctrl_regs = ov5640_format_uyvy,
	}, {
		.code = MEDIA_BUS_FMT_RGB565_2X8_BE,
		.format_ctrl_regs = ov5640_format_rgb565,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.format_ctrl_regs = ov5640_format_bggr,
	},
};

static inline struct ov5640 *to_ov5640(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5640, sd);
}

/* sensor register write */
static int ov5640_write(struct i2c_client *client, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_dbg(&client->dev,
		"ov5640 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

/* sensor register read */
static int ov5640_read(struct i2c_client *client, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_dbg(&client->dev,
		"ov5640 read reg(0x%x val:0x%x) failed !\n", reg, *val);

	return ret;
}

static int ov5640_write_array(struct i2c_client *client,
			      const struct sensor_register *regs)
{
	int i, ret = 0;

	for (i = 0; ret == 0 && regs[i].addr; i++)
		ret = ov5640_write(client, regs[i].addr, regs[i].value);

	return ret;
}

static void ov5640_pll_calc_params(struct ov5640 *ov5640)
{
	const struct ov5640_platform_data *pdata = ov5640->pdata;
	u8 ctrl1_reg = 0, ctrl2_reg = 0, ctrl3_reg = 0;
	struct i2c_client *client = ov5640->client;
	unsigned int desired = pdata->link_frequency;
	u32 s_prediv = 1, s_postdiv = 1, s_mult = 1;
	u32 prediv, postdiv, mult;
	u32 bestdelta = -1;
	u32 delta, actual;
	int i, j;

	for (i = 0; ctrl1[i].div != 0; i++) {
		postdiv = ctrl1[i].div;
		for (j = 0; ctrl3[j].div != 0; j++) {
			prediv = ctrl3[j].div;
			for (mult = 1; mult <= 63; mult++) {
				actual  = ov5640->xvclk_frequency;
				actual *= mult;
				actual /= prediv;
				actual /= postdiv;
				delta = actual - desired;
				delta = abs(delta);

				if ((delta < bestdelta) || (bestdelta == -1)) {
					bestdelta = delta;
					s_mult    = mult;
					s_prediv  = prediv;
					s_postdiv = postdiv;
					ctrl1_reg = ctrl1[i].reg;
					ctrl2_reg = mult;
					ctrl3_reg = ctrl3[j].reg;
				}
			}
		}
	}

	ov5640->pll.ctrl1 = ctrl1_reg;
	ov5640->pll.ctrl2 = ctrl2_reg;
	ov5640->pll.ctrl3 = ctrl3_reg;

	dev_dbg(&client->dev,
		"Actual reg config: ctrl1_reg: %02x ctrl2_reg: %02x ctrl3_reg: %02x\n",
		ctrl1_reg, ctrl2_reg, ctrl3_reg);
}

static int ov5640_set_pixel_clock(struct ov5640 *ov5640)
{
	struct i2c_client *client = ov5640->client;
	struct sensor_register pll_regs[] = {
		{REG_SC_PLL_CTRL1, ov5640->pll.ctrl1},
		{REG_SC_PLL_CTRL2, ov5640->pll.ctrl2},
		{REG_SC_PLL_CTRL3, ov5640->pll.ctrl3},
		{REG_NULL, 0x00},
	};

	dev_dbg(&client->dev, "%s\n", __func__);

	return ov5640_write_array(client, pll_regs);
};

static void ov5640_get_default_format(struct v4l2_mbus_framefmt *format)
{
	format->width = ov5640_framesizes[2].width;
	format->height = ov5640_framesizes[2].height;
	format->colorspace = V4L2_COLORSPACE_JPEG;
	format->code = ov5640_formats[0].code;
	format->field = V4L2_FIELD_NONE;
}

static void ov5640_set_streaming(struct ov5640 *ov5640, int on)
{
	struct i2c_client *client = ov5640->client;
	int ret;

	on = !!on;

	dev_dbg(&client->dev, "%s: on: %d\n", __func__, on);

	ret = ov5640_write(client, REG_SOFTWARE_STANDBY, on);
	if (ret)
		dev_dbg(&client->dev, "ov5640 soft standby failed\n");
}

static int ov5640_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ov5640_write_array(client, ov5640_init_regs);
}

/*
 * V4L2 subdev video and pad level operations
 */

static int ov5640_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (code->index >= ARRAY_SIZE(ov5640_formats))
		return -EINVAL;

	code->code = ov5640_formats[code->index].code;

	return 0;
}

static int ov5640_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i = ARRAY_SIZE(ov5640_formats);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (fse->index >= ARRAY_SIZE(ov5640_framesizes))
		return -EINVAL;

	while (--i)
		if (fse->code == ov5640_formats[i].code)
			break;

	fse->code = ov5640_formats[i].code;

	fse->min_width  = ov5640_framesizes[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->max_height = ov5640_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int ov5640_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(sd);
	struct v4l2_mbus_framefmt *mf;

	dev_dbg(&client->dev, "ov5640_get_fmt\n");

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, 0);
		mutex_lock(&ov5640->lock);
		fmt->format = *mf;
		mutex_unlock(&ov5640->lock);
		return 0;
	}

	mutex_lock(&ov5640->lock);
	fmt->format = ov5640->format;
	mutex_unlock(&ov5640->lock);

	dev_dbg(&client->dev, "ov5640_get_fmt: %x %dx%d\n",
		ov5640->format.code, ov5640->format.width,
		ov5640->format.height);

	return 0;
}

static void __ov5640_try_frame_size(struct v4l2_mbus_framefmt *mf,
				    const struct ov5640_framesize **size)
{
	const struct ov5640_framesize *fsize = &ov5640_framesizes[0];
	const struct ov5640_framesize *match = NULL;
	int i = ARRAY_SIZE(ov5640_framesizes);
	unsigned int min_err = UINT_MAX;

	while (i--) {
		int err = abs(fsize->width - mf->width)
				+ abs(fsize->height - mf->height);
		if ((err < min_err) && (fsize->regs[0].addr)) {
			min_err = err;
			match = fsize;
		}
		fsize++;
	}

	if (!match)
		match = &ov5640_framesizes[2];

	mf->width  = match->width;
	mf->height = match->height;

	if (size)
		*size = match;
}

static int ov5640_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned int index = ARRAY_SIZE(ov5640_formats);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	const struct ov5640_framesize *size = NULL;
	struct ov5640 *ov5640 = to_ov5640(sd);
	int ret = 0;

	dev_dbg(&client->dev, "ov5640_set_fmt\n");

	__ov5640_try_frame_size(mf, &size);

	while (--index >= 0)
		if (ov5640_formats[index].code == mf->code)
			break;

	if (index < 0)
		return -EINVAL;

	mf->colorspace = V4L2_COLORSPACE_JPEG;
	mf->code = ov5640_formats[index].code;
	mf->field = V4L2_FIELD_NONE;

	mutex_lock(&ov5640->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*mf = fmt->format;
	} else {
		s64 val;

		if (ov5640->streaming) {
			mutex_unlock(&ov5640->lock);
			return -EBUSY;
		}

		ov5640->frame_size = size;
		ov5640->format = fmt->format;
		ov5640->format_ctrl_regs =
			ov5640_formats[index].format_ctrl_regs;

		if (ov5640->format.code != MEDIA_BUS_FMT_SBGGR8_1X8)
			val = ov5640->pdata->link_frequency / 2;
		else
			val = ov5640->pdata->link_frequency;

		ret = v4l2_ctrl_s_ctrl_int64(ov5640->link_frequency, val);
		if (ret < 0)
			dev_warn(&client->dev,
				 "failed to set link_frequency rate (%d)\n",
				 ret);
	}

	mutex_unlock(&ov5640->lock);
	return ret;
}

static int ov5640_set_frame_size(struct ov5640 *ov5640)
{
	struct i2c_client *client = ov5640->client;

	dev_dbg(&client->dev, "%s\n", __func__);

	return ov5640_write_array(ov5640->client, ov5640->frame_size->regs);
}

static int ov5640_set_format(struct ov5640 *ov5640)
{
	struct i2c_client *client = ov5640->client;

	dev_dbg(&client->dev, "%s\n", __func__);

	return ov5640_write_array(ov5640->client, ov5640->format_ctrl_regs);
}

static int ov5640_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(sd);
	int ret = 0;

	dev_dbg(&client->dev, "%s: on: %d\n", __func__, on);

	mutex_lock(&ov5640->lock);

	on = !!on;

	if (ov5640->streaming == on)
		goto unlock;

	if (!on) {
		/* Stop Streaming Sequence */
		ov5640_set_streaming(ov5640, 0);
		ov5640->streaming = on;
		goto unlock;
	}

//	ov5640_set_pixel_clock(ov5640);
	ov5640_set_frame_size(ov5640);
	ov5640_set_format(ov5640);
	ov5640_set_streaming(ov5640, 1);
	ov5640->streaming = on;

unlock:
	mutex_unlock(&ov5640->lock);
	return ret;
}

static int ov5640_set_test_pattern(struct ov5640 *ov5640, int value)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5640->sd);
	int ret;
	u8 val;

	ret = ov5640_read(client, REG_PRE_ISP_CTRL00, &val);
	if (ret < 0)
		return ret;

	switch (value) {
	case 0:
		val &= ~TEST_PATTERN_ENABLE;
		break;
	case 1:
		val &= VERTICAL_COLOR_BAR_MASK;
		val |= TEST_PATTERN_ENABLE;
		break;
	}

	return ov5640_write(client, REG_PRE_ISP_CTRL00, val);
}

static int ov5640_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5640 *ov5640 =
			container_of(ctrl->handler, struct ov5640, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		return ov5640_set_test_pattern(ov5640, ctrl->val);
	}

	return 0;
}

static struct v4l2_ctrl_ops ov5640_ctrl_ops = {
	.s_ctrl = ov5640_s_ctrl,
};

static const char * const ov5640_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int ov5640_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);

	dev_dbg(&client->dev, "%s:\n", __func__);

	ov5640_get_default_format(format);

	return 0;
}

static const struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.s_stream = ov5640_s_stream,
};

static const struct v4l2_subdev_pad_ops ov5640_subdev_pad_ops = {
	.enum_mbus_code = ov5640_enum_mbus_code,
	.enum_frame_size = ov5640_enum_frame_sizes,
	.get_fmt = ov5640_get_fmt,
	.set_fmt = ov5640_set_fmt,
};

static const struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core  = &ov5640_subdev_core_ops,
	.video = &ov5640_subdev_video_ops,
	.pad   = &ov5640_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov5640_subdev_internal_ops = {
	.open = ov5640_open,
};

static int ov5640_detect(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 pid, ver;
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	ret = ov5640_write(client, 0x3008, 0x80);
	if (ret != 0) {
		dev_dbg(&client->dev, "Sensor soft reset failed\n");
		return -ENODEV;
	}
	usleep_range(1000, 2000);

	/* Check sensor revision */
	ret = ov5640_read(client, REG_SC_CHIP_ID_H, &pid);
	if (!ret)
		ret = ov5640_read(client, REG_SC_CHIP_ID_L, &ver);

	if (!ret) {
		unsigned short id;

		id = OV564X_ID(pid, ver);
		if (id != OV5640_ID) {
			dev_dbg(&client->dev,
				"Sensor detection failed (%04X, %d)\n",
				id, ret);
			ret = -ENODEV;
		} else {
			dev_info(&client->dev, "Found OV%04X sensor\n", id);
			ret = ov5640_init(sd, 0);
		}
	}

	return ret;
}

static struct ov5640_platform_data *
ov5640_get_pdata(struct i2c_client *client)
{
	struct ov5640_platform_data *pdata;
	struct device_node *endpoint;
	int ret;

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return client->dev.platform_data;

	endpoint = of_graph_get_next_endpoint(client->dev.of_node, NULL);
	if (!endpoint)
		return NULL;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto done;

	ret = of_property_read_u64(endpoint, "link-frequencies",
				   &pdata->link_frequency);
	if (ret) {
		dev_dbg(&client->dev, "link-frequencies property not found\n");
		pdata = NULL;
	}

done:
	of_node_put(endpoint);
	return pdata;
}

static int ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct ov5640_platform_data *pdata = ov5640_get_pdata(client);
	struct v4l2_subdev *sd;
	struct ov5640 *ov5640;
	struct clk *clk;
	struct gpio_desc *gpio;
	int ret;

	if (!pdata) {
		dev_dbg(&client->dev, "platform data not specified\n");
		return -EINVAL;
	}

	ov5640 = devm_kzalloc(&client->dev, sizeof(*ov5640), GFP_KERNEL);
	if (!ov5640)
		return -ENOMEM;

	ov5640->pdata = pdata;
	ov5640->client = client;

	clk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(clk))
	{	
		dev_dbg(&client->dev, "couldn't get clk for ov5640\n");
		return PTR_ERR(clk);
	}
	else{
		clk_prepare_enable(clk);
	}

	ov5640->xvclk_frequency = clk_get_rate(clk);
	if (ov5640->xvclk_frequency < 6000000 ||
	    ov5640->xvclk_frequency > 27000000)
		return -EINVAL;

	/* Optional gpio don't fail if not present */
	gpio = devm_gpiod_get_optional(&client->dev, "pwrdn", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	ov5640->pwrdn_gpio = gpio;

	v4l2_ctrl_handler_init(&ov5640->ctrls, 2);
	ov5640->link_frequency =
			v4l2_ctrl_new_std(&ov5640->ctrls, &ov5640_ctrl_ops,
					  V4L2_CID_PIXEL_RATE,
					  pdata->link_frequency / 2,
					  pdata->link_frequency, 1,
					  pdata->link_frequency);
	v4l2_ctrl_new_std_menu_items(&ov5640->ctrls, &ov5640_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov5640_test_pattern_menu) - 1,
				     0, 0, ov5640_test_pattern_menu);
	ov5640->sd.ctrl_handler = &ov5640->ctrls;

	if (ov5640->ctrls.error) {
		dev_dbg(&client->dev, "%s: control initialization error %d\n",
			__func__, ov5640->ctrls.error);
		return  ov5640->ctrls.error;
	}

	sd = &ov5640->sd;
	client->flags |= I2C_CLIENT_SCCB;
	v4l2_i2c_subdev_init(sd, client, &ov5640_subdev_ops);

	sd->internal_ops = &ov5640_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	ov5640->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ov5640->pad, 0);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&ov5640->ctrls);
		return ret;
	}
#endif

	mutex_init(&ov5640->lock);

	ov5640_get_default_format(&ov5640->format);
	ov5640->frame_size = &ov5640_framesizes[2];
	ov5640->format_ctrl_regs = ov5640_formats[0].format_ctrl_regs;

	ret = ov5640_detect(sd);
	if (ret < 0)
		goto error;

	/* Calculate the PLL register value needed */
//	ov5640_pll_calc_params(ov5640);

	ret = v4l2_async_register_subdev(&ov5640->sd);
	if (ret)
		goto error;

	dev_dbg(&client->dev, "%s sensor driver registered !!\n", sd->name);

	return 0;

error:
	v4l2_ctrl_handler_free(&ov5640->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&ov5640->lock);
	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640 *ov5640 = to_ov5640(sd);

	v4l2_ctrl_handler_free(&ov5640->ctrls);
	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&ov5640->lock);

	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{ "ov5640", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov5640_of_match[] = {
	{ .compatible = "ovti,5640", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov5640_of_match);
#endif

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(ov5640_of_match),
	},
	.probe		= ov5640_probe,
	.remove		= ov5640_remove,
	.id_table	= ov5640_id,
};

module_i2c_driver(ov5640_i2c_driver);

MODULE_AUTHOR("Benoit Parrot <bparrot@ti.com>");
MODULE_DESCRIPTION("OV5640 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");

