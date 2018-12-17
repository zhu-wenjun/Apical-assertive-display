/*
 * drivers/video/nusmart/tl7790_ad.c
 *
 * Copyright (C) 2016 Nufront Corporation
 *
 * This driver is designed for Apical Assertive Dispaly(AD).
 *
 * Function mode: Mode 0 or Mode 1(suggested);
 *
 * The difference between Mode 0 and Mode 1 is where the ambient light input to backlight level output mapping takes place, in Mode 0,
 * this mapping is controlled by the AD IP core itself, while in Mode 1, this mapping occurs in the implementation of the OS power profile
 * auto-brightness curve.
 *
 * "Apical shall provide a modified OS auto-brightness power profile curve that should be used by the OEM when the AD IP core is enabled", this
 * will be done by the "Apical Calibration Tool APK"
 *
 * Mode = 1; Input: ambient_light and backlight(OS backlight level) provided by software driver.
 * NOTE: "OS backlight level" in the table below refers to the Apical modified ambient light to backlight power savings auto-brightness curve.
 * NOTE: CABC algorithm later in the SoC pipeline or within the display panel.
 *
 * --------------------------------------------------------------------------------------------------------------------------------------------
 * |	Flow Type    |   Input to AD IP	     |   Output from AD IP   |	  Input to CABC	      |    Output from CABC    |    Input to Panel    |
 * |------------------------------------------------------------------------------------------------------------------------------------------|
 * |								Mode 1 - With CABC							      |
 * |------------------------------------------------------------------------------------------------------------------------------------------|
 * |                 |                       |                       |                        |                        |                      |
 * | Ambient Light / |   ambient_light &     |                       |                        |                        | CABC output backlight|
 * | backlight Flow  |   OS backlight level  |   	N/A	     |   OS backlight level   |  CABC output backlight | provided at panel's  |
 * |                 |                       |                       |                        |                        | PWM input	      |
 * |-----------------|-----------------------|-----------------------|------------------------|------------------------|----------------------|
 * |                 |                       |                       |                        |                        |                      |
 * |  Data flow	     |   Image data from     |   Image data modified |   Image data modified  |  Image data modified   | Image data modified  |
 * |		     |   display controller  |    by AD processing   |   by AD processing     |        by CABC	       | 	by CABC       |
 * |                 |                       |                       |                        |                        |                      |
 * |------------------------------------------------------------------------------------------------------------------------------------------|
 * |								Mode 1 - AD IP only							      |
 * |------------------------------------------------------------------------------------------------------------------------------------------|
 * |                 |                       |                       |                        |                        |                      |
 * | Ambient Light / |   ambient_light &     |                       |                        |                        | OS backlight level   |
 * | backlight Flow  |   OS backlight level  |   	N/A	     |		N/A	      |	          N/A	       | provided  at panel's |
 * |                 |                       |                       |                        |                        | PWM input	      |
 * |-----------------|-----------------------|-----------------------|------------------------|------------------------|----------------------|
 * |                 |                       |                       |                        |                        |                      |
 * |  Data flow	     |   Image data from     |   Image data modified |			      |			       | Image data modified  |
 * |		     |   display controller  |    by AD processing   |		N/A	      |		  N/A	       |  by AD processing    |
 * |                 |                       |                       |                        |                        |                      |
 * --------------------------------------------------------------------------------------------------------------------------------------------
 */

#include <asm/io.h>
#include <linux/of.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/reset.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/file.h>
#include <linux/major.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm-generic/uaccess.h>
#include <video/tl7790_ad.h>

#define DRIVER_NAME	"tl7790-ad"

static void __iomem *ad_mmio;

#define DBG_ERR		0x8
#define DBG_WARN	0x4
#define DBG_INFO	0x2
#define DBG_DEBUG	0x1

/*
 * Modify /sys/module/tl7790_ad/parameters/dbg_level
 * to regulate print level of this driver.
 */
static unsigned int dbg_level = DBG_ERR | DBG_WARN | DBG_INFO;// | DBG_DEBUG;
module_param_named(dbg_level, dbg_level, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define ad_err(fmt, args...)						\
	do {								\
		if(dbg_level & DBG_ERR)					\
		printk(KERN_ERR "ad err: " fmt, ##args);		\
	} while(0)

#define ad_warn(fmt, args...)						\
	do {								\
		if(dbg_level & DBG_ERR)					\
		printk(KERN_WARNING "ad warn: " fmt, ##args);		\
	} while(0)

#define ad_info(fmt, args...)						\
	do {								\
		if(dbg_level & DBG_INFO)				\
		printk(KERN_INFO "ad info: " fmt, ##args);		\
	} while(0)

#define ad_dbg(fmt, args...)						\
	do {								\
		if(dbg_level & DBG_DEBUG)				\
		printk(KERN_DEBUG fmt, ##args);				\
	} while(0)

#define set_reg_bits(reg_off, bits)					\
	do {								\
		unsigned int val;					\
		unsigned int reg_addr;					\
		reg_addr = ad_mmio + reg_off;				\
		val = readl(reg_addr);					\
		val |= bits;						\
		writel(val, reg_addr);					\
	} while (0)

#define clr_reg_bits(reg_off, bits)					\
	do {								\
		unsigned int val;					\
		unsigned int reg_addr;					\
		reg_addr = ad_mmio + reg_off;				\
		val = readl(reg_addr);					\
		val &= ~bits;						\
		writel(val, reg_addr);					\
	} while (0)

#define ad_writel(val, addr)		writel(val, ad_mmio + addr)
#define ad_readl(addr)			readl(ad_mmio + addr)

#define DECIMAL				10
#define HEXADECIMAL			16

#define AD_MESSAGE_AD_INIT		0x4629
#define AD_MESSAGE_AD_INPUT		0x462a
#define AD_MESSAGE_AD_CONFIG		0x462b
#define AD_MESSAGE_AD_CALIB_ON		0x462c
#define AD_MESSAGE_AD_CALIB_OFF		0x462d
#define AD_MESSAGE_BL_SET		0x462e

#define ad_command_ad_init		"ad:init"
#define ad_command_ad_input		"ad:input"
#define ad_command_ad_config		"ad:config"
#define ad_command_ad_calib_on		"ad:calib:on"
#define ad_command_ad_calib_off		"ad:calib:off"
#define ad_command_bl_set		"bl:set"

#define ad_param_asym_lut		"asym_lut"
#define ad_param_color_corr_lut		"color_corr_lut"
#define ad_param_i_control		"i_control"
#define ad_param_black_lvl		"black_lvl"
#define ad_param_white_lvl		"white_lvl"
#define ad_param_variance		"variance"
#define ad_param_limit_amplitude	"limit_amplitude"
#define ad_param_i_dither		"i_dither"
#define ad_param_slope_max		"slope_max"
#define ad_param_slope_min		"slope_min"
#define ad_param_dither_control		"dither_control"
#define ad_param_format			"format"
#define ad_param_autosize		"autosize"
#define ad_param_frame_width		"frame_width"
#define ad_param_frame_height		"frame_height"
#define ad_param_logo_vertical_pos	"logo_vertical_pos"
#define ad_param_logo_hor_pos		"logo_hor_pos"
#define ad_param_bl_lin_lut		"bl_lin_lut"
#define ad_param_bl_lin_inverse_lut	"bl_lin_inverse_lut"
#define ad_param_alpha			"alpha"
#define ad_param_bl_att_lut		"bl_att_lut"
#define ad_param_als_offset		"als_offset"
#define ad_param_als_thresh		"als_thresh"

#define ad_param_mode			"mode"
#define ad_param_ambient_calib_lut	"ambient_calib_lut"
#define ad_param_back_min		"back_min"
#define ad_param_back_max		"back_max"
#define ad_param_backlight_scale	"backlight_scale"
#define ad_param_ambient_light_min	"ambient_light_min"
#define ad_param_filter			"filter"
#define ad_param_calibration_abcd	"calibration_abcd"
#define ad_param_str_limit		"str_limit"
#define ad_param_temp_filter_recurs	"temp_filter_recurs"
#define ad_param_stabilization_iterat	"stabilization_iterations"

#define ad_param_power_saving_coeff	"power_saving_coeff"
#define ad_param_al_change_detect_coeff "al_change_detect_coeff"

#define ad_param_backlight		"backlight"

typedef int (*ad_param_handler_t)(char *);
struct ad_param_handler {

	char ad_param_name[32];
	ad_param_handler_t hdl;
};

static int ad_major;
static struct backlight_process bp;
static int ad_set_backlight(char *bl);
static int ad_operational_mode(void);

extern int volatile al_lux;
extern int auto_brightness;

struct class *ad_class;
struct tl7790_ad_device *ad_dev = NULL;

u32 ad_of_property_read_u32(const struct device_node *np, char *prop)
{
	u32 val = 0;

	if (of_property_read_u32(np, prop, &val))
		ad_err("Failed to read %s property in ad node.\n", prop);
	return val;
}

static int ad_open(struct inode *inode, struct file *file)
{
	ad_info("%s\n", __func__);
	return 0;
}

static int ad_release(struct inode *inode, struct file *file)
{
	ad_info("%s\n", __func__);
	return 0;
}

static long ad_calibration_mode(struct file *file, unsigned int cmd,
		unsigned long arg);

const struct file_operations ad_fops = {
	.owner = THIS_MODULE,
	.open = ad_open,
	.release = ad_release,
	.unlocked_ioctl = ad_calibration_mode,
};

static ssize_t ad_show_manual_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ad_dev->brightness);
}

static ssize_t ad_set_manual_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static struct device_attribute dev_attrs[] = {
	__ATTR(brightness, 0644, ad_show_manual_brightness, ad_set_manual_brightness),
};

static inline int ad_device_remove_files(int file_num)
{
	while (file_num >= 0)
		device_remove_file(ad_dev->dev, &dev_attrs[file_num]);

	return 0;
}

static int ad_sysfs_init(void)
{
	int i;
	int ret;
	unsigned int ad_major = 0;

	ad_class = class_create(THIS_MODULE, "assertive_display");
	if (IS_ERR(ad_class)) {
		ret = PTR_ERR(ad_class);
		ad_err("Failed to create ad class, %d.\n", ret);
		return ret;
	}

	ad_major = register_chrdev(ad_major, "ad", &ad_fops);
	if (ad_major < 0) {
		ad_err("Failed to register ad device, %d.\n", ad_major);
		goto err_register_chrdev;
	}

	ad_dev->dev = device_create(ad_class, ad_dev->device, MKDEV(ad_major, 0), NULL, "ad");
	if (IS_ERR(ad_dev->dev)) {
		ad_err("Failed to create device for AD, errno = %ld\n", PTR_ERR(ad_dev->dev));
		ad_dev->dev = NULL;
		goto err_device_create;
	}

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++) {
		ret = device_create_file(ad_dev->dev, &dev_attrs[i]);
		if (ret ) {
			ad_err("Failed to create file for AD device, %d.\n", ret);
			goto err_create_file;
		}
	}
	ad_dbg("Initialize AD device succeed, ad_major = %d\n", ad_major);
	return 0;

err_create_file:
	device_destroy(ad_class, MKDEV(ad_major, 0));
err_device_create:
	unregister_chrdev(FB_MAJOR, "ad");
err_register_chrdev:
	class_destroy(ad_class);

	return ret;
}

static int ad_sysfs_destroy(void)
{
	int i = ARRAY_SIZE(dev_attrs);

	ad_device_remove_files(i);
	device_destroy(ad_class, MKDEV(ad_major, 0));
	unregister_chrdev(FB_MAJOR, "ad");
	class_destroy(ad_class);

	return 0;
}

static int ad_notifier_callback(struct notifier_block *nb,
		unsigned long event, void *data)
{
	switch(event) {

		case FB_EVENT_SUSPEND:
			break;

		case FB_EVENT_RESUME:
			ad_operational_mode();
			break;
	}
	return 0;
}

static int ad_notifier_register(void)
{
	memset(&ad_dev->ad_notif, 0, sizeof(ad_dev->ad_notif));
	ad_dev->ad_notif.notifier_call = ad_notifier_callback;
	ad_dev->ad_notif.priority = 3;

	return fb_register_client(&ad_dev->ad_notif);
}

/*
 * In Apical AD IP core, video pipeline contains 4 modules:
 * 1. Iridix module    2. Logo module    3. Dither module    4. Gamma module.
 * Excluded the Gamma module, all the other 3 modules configuration parameters
 * should not be updated during the active frame, we can use double-buffering
 * scheme to complete the configuration, let's welcome 'config_buffer_mode'.
 *
 * Register 'config_buffer_mode' offers 4 kinds of modes to implements double-buffering
 * scheme for the above 3 modules configuration parameters of video pipeline:
 * config_buffer_mode = 0	Pass through
 * config_buffer_mode = 1	Block access
 * config_buffer_mode = 2	Update on VBI(vertical blank interval)
 * config_buffer_mode = 3	Update when config_buffer_global is assert
 * In a typical use case the external host will set config_buffer_mode to mode 1 (Block) and
 * then write to all the parameter ports. Once the write is completed the external host will change
 * the config_buffer_mode to mode 2 (update on VBI). This scheme will ensure that all the
 * video pipeline parameters are only updated during vertical blanking. In normal operation when
 * no registers are being updated the config_buffer_mode should be in mode 2 (Update on VBI).
 */
static int __maybe_unused ad_vp_writel(unsigned int addr, unsigned int val)
{
	ad_writel(1, AD_CONFIG_BUFFER_MODE);
	ad_writel(val, addr);
	ad_writel(2, AD_CONFIG_BUFFER_MODE);

	return 0;
}

static int ad_set_asym_lut(char *asym_lut)
{
	int i = 0;
	unsigned int data;
	char *ptr, *delim = ",";

	while(asym_lut) {

		ptr = strsep(&asym_lut, delim);
		data = simple_strtoul(ptr, NULL, DECIMAL);

		if(i < AD_VP_IRIDIX_ASYMMETRY_LUT_ADDR_MASK) {
			ad_writel(i, AD_VP_IRIDIX_ASYMMETRY_LUT_ADDR_I);
			ad_writel(data & 0xff, AD_VP_IRIDIX_ASYMMETRY_LUT_DATA_W_L);
			ad_writel((data >> 8) & 0xf, AD_VP_IRIDIX_ASYMMETRY_LUT_DATA_W_H);
		} else
			break;
		i++;
	}

	ad_dbg("%s  ", __func__);
	ad_dbg("AD_VP_IRIDIX_ASYMMETRY_LUT_ADDR_I:   0x%02x.\n",
			ad_readl(AD_VP_IRIDIX_ASYMMETRY_LUT_ADDR_I));
	ad_dbg("AD_VP_IRIDIX_ASYMMETRY_LUT_DATA_W_L: 0x%02x.\n",
			ad_readl(AD_VP_IRIDIX_ASYMMETRY_LUT_DATA_W_L));
	ad_dbg("AD_VP_IRIDIX_ASYMMETRY_LUT_DATA_W_H: 0x%02x.\n",
			ad_readl(AD_VP_IRIDIX_ASYMMETRY_LUT_DATA_W_H));

	return 0;
}

static int ad_set_color_corr_lut(char *color_corr_lut)
{
	int i = 0;
	unsigned int data;
	char *ptr, *delim = ",";

	while(color_corr_lut) {

		ptr = strsep(&color_corr_lut, delim);
		data = simple_strtoul(ptr, NULL, DECIMAL);

		if(i < AD_VP_IRIDIX_COLOR_CORRECTION_LUT_ADDR_I_MASK) {
			ad_writel(i, AD_VP_IRIDIX_COLOR_CORRECTION_LUT_ADDR_I);
			ad_writel(data & 0xff, AD_VP_IRIDIX_COLOR_CORRECTION_LUT_DATA_W_L);
			ad_writel((data >> 8) & 0xf, AD_VP_IRIDIX_COLOR_CORRECTION_LUT_DATA_W_H);
		} else
			break;
		i++;
	}

	ad_dbg("%s  ", __func__);
	ad_dbg("AD_VP_IRIDIX_COLOR_CORRECTION_LUT_ADDR_I:   0x%02x.\n",
			ad_readl(AD_VP_IRIDIX_COLOR_CORRECTION_LUT_ADDR_I));
	ad_dbg("AD_VP_IRIDIX_COLOR_CORRECTION_LUT_DATA_W_L: 0x%02x.\n",
			ad_readl(AD_VP_IRIDIX_COLOR_CORRECTION_LUT_DATA_W_L));
	ad_dbg("AD_VP_IRIDIX_COLOR_CORRECTION_LUT_DATA_W_H: 0x%02x.\n",
			ad_readl(AD_VP_IRIDIX_COLOR_CORRECTION_LUT_DATA_W_H));

	return 0;
}

static int ad_set_i_control(char *i_control)
{
	char *ptr, *delim = ",";
	unsigned int iridix_control_0;
	unsigned int iridix_control_1;

	ptr = strsep(&i_control, delim);
	iridix_control_0 = simple_strtoul(ptr, NULL, DECIMAL);
	ad_writel(iridix_control_0/* & AD_VP_IRIDIX_CONTROL_0_MASK*/,
			AD_VP_IRIDIX_CONTROL_0);

	ptr = strsep(&i_control, delim);
	iridix_control_1 = simple_strtoul(ptr, NULL, DECIMAL);
	ad_writel(iridix_control_1/* & AD_VP_IRIDIX_CONTROL_1_MASK*/,
			AD_VP_IRIDIX_CONTROL_1);

	ad_dbg("%s AD_VP_IRIDIX_CONTROL_0: 0x%x, AD_VP_IRIDIX_CONTROL_1: 0x%x.\n",
			__func__, ad_readl(AD_VP_IRIDIX_CONTROL_0), ad_readl(AD_VP_IRIDIX_CONTROL_1));
	return 0;
}

static int ad_set_black_lvl(char *black_lvl)
{
	unsigned int blklv;

	blklv = simple_strtoul(black_lvl, NULL, DECIMAL);
	ad_writel(blklv & AD_VP_BLACK_LEVEL_L_MASK,
			AD_VP_BLACK_LEVEL_L);
	ad_writel((blklv >> 8 ) & AD_VP_BLACK_LEVEL_H_MASK,
			AD_VP_BLACK_LEVEL_H);

	ad_dbg("%s AD_VP_BLACK_LEVEL_L: 0x%x, AD_VP_BLACK_LEVEL_H: 0x%x.\n",
			__func__, ad_readl(AD_VP_BLACK_LEVEL_L), ad_readl(AD_VP_BLACK_LEVEL_H));
	return 0;
}

static int ad_set_white_lvl(char *white_lvl)
{
	unsigned int whtlv;

	whtlv = simple_strtoul(white_lvl, NULL, DECIMAL);
	ad_writel(whtlv & AD_VP_WHITE_LEVEL_L_MASK,
			AD_VP_WHITE_LEVEL_L);
	ad_writel((whtlv >> 8 ) & AD_VP_WHITE_LEVEL_H_MASK,
			AD_VP_WHITE_LEVEL_H);

	ad_dbg("%s AD_VP_WHITE_LEVEL_L: 0x%x, AD_VP_WHITE_LEVEL_H: 0x%x.\n",
			__func__, ad_readl(AD_VP_WHITE_LEVEL_L), ad_readl(AD_VP_WHITE_LEVEL_H));
	return 0;
}

static int ad_set_variance(char *variance)
{
	unsigned int v;

	v = simple_strtoul(variance, NULL, DECIMAL);
	ad_writel(v & AD_VP_VARIANCE_MASK, AD_VP_VARIANCE);

	ad_dbg("%s AD_VP_VARIANCE: 0x%x.\n", __func__, ad_readl(AD_VP_VARIANCE));
	return 0;
}

static int ad_set_limit_amplitude(char *limit_amplitude)
{
	unsigned int limit_ampl;

	limit_ampl = simple_strtoul(limit_amplitude, NULL, DECIMAL);
	ad_writel(limit_ampl & AD_VP_LIMIT_AMPL_MASK, AD_VP_LIMIT_AMPL);

	ad_dbg("%s AD_VP_LIMIT_AMPL: 0x%x.\n", __func__, ad_readl(AD_VP_LIMIT_AMPL));
	return 0;
}

static int ad_set_i_dither(char *i_dither)
{
	unsigned int iridix_dither;

	iridix_dither = simple_strtoul(i_dither, NULL, DECIMAL);
	ad_writel(iridix_dither & AD_VP_IRIDIX_DITHER_MASK, AD_VP_IRIDIX_DITHER);

	ad_dbg("%s AD_VP_IRIDIX_DITHER: 0x%x.\n", __func__, ad_readl(AD_VP_IRIDIX_DITHER));
	return 0;
}

static int ad_set_slope_max(char *slope_max)
{
	unsigned int slp_max;

	slp_max = simple_strtoul(slope_max, NULL, DECIMAL);
	ad_writel(slp_max & AD_VP_SLOPE_MAX_MASK, AD_VP_SLOPE_MAX);

	ad_dbg("%s AD_VP_SLOPE_MAX: 0x%x.\n", __func__, ad_readl(AD_VP_SLOPE_MAX));
	return 0;
}

static int ad_set_slope_min(char *slope_min)
{
	unsigned int slp_min;

	slp_min = simple_strtoul(slope_min, NULL, DECIMAL);
	ad_writel(slp_min & AD_VP_SLOPE_MIN_MASK, AD_VP_SLOPE_MIN);

	ad_dbg("%s AD_VP_SLOPE_MIN: 0x%x.\n", __func__, ad_readl(AD_VP_SLOPE_MIN));
	return 0;
}

static int ad_set_dither_control(char *dither_control)
{
	unsigned int dither_ctrl;

	dither_ctrl = simple_strtoul(dither_control, NULL, DECIMAL);
	ad_writel(dither_ctrl & AD_VP_APICAL_DITHER_CONTROL_MASK, AD_VP_APICAL_DITHER_CONTROL);

	ad_dbg("%s AD_VP_APICAL_DITHER_CONTROL: 0x%x.\n", __func__, ad_readl(AD_VP_APICAL_DITHER_CONTROL));
	return 0;
}

static int ad_set_format(char *format)
{
	ad_dbg("%s format is not present in this IP version", __func__);
	return 0;
}

static int ad_set_autosize(char *autosize)
{
	ad_dbg("%s autosize is not present in this IP version", __func__);
	return 0;
}

static int ad_set_frame_width(char *frame_width)
{
	unsigned int frame_w;

	frame_w = simple_strtoul(frame_width, NULL, DECIMAL);
	ad_writel(frame_w & AD_VP_FRAME_WIDTH_L_MASK, AD_VP_FRAME_WIDTH_L);
	ad_writel((frame_w >> 8) & AD_VP_FRAME_WIDTH_H_MASK, AD_VP_FRAME_WIDTH_H);

	ad_dbg("%s AD_VP_FRAME_WIDTH_L: 0x%x, AD_VP_FRAME_WIDTH_H: 0x%x.\n",
			__func__,
			ad_readl(AD_VP_FRAME_WIDTH_L),
			ad_readl(AD_VP_FRAME_WIDTH_H));
	return 0;
}

static int ad_set_frame_height(char *frame_height)
{
	unsigned int frame_h;

	frame_h = simple_strtoul(frame_height, NULL, DECIMAL);
	ad_writel(frame_h & AD_VP_FRAME_HEIGHT_L_MASK, AD_VP_FRAME_HEIGHT_L);
	ad_writel((frame_h >> 8) & AD_VP_FRAME_HEIGHT_H_MASK, AD_VP_FRAME_HEIGHT_H);

	ad_dbg("%s AD_VP_FRAME_HEIGHT_L: 0x%x, AD_VP_FRAME_HEIGHT_H: 0x%x.\n",
			__func__,
			ad_readl(AD_VP_FRAME_HEIGHT_L),
			ad_readl(AD_VP_FRAME_HEIGHT_H));
	return 0;
}

static int ad_set_logo_vertical_pos(char *logo_ver_pos)
{
	unsigned int logo_v;

	logo_v = simple_strtoul(logo_ver_pos, NULL, DECIMAL);
	ad_writel(logo_v & AD_VP_LOGO_TOP_MASK, AD_VP_LOGO_TOP);

	ad_dbg("%s AD_VP_LOGO_TOP: 0x%x.\n", __func__, ad_readl(AD_VP_LOGO_TOP));
	return 0;
}

static int ad_set_logo_hor_pos(char *logo_hor_pos)
{
	unsigned int logo_h;

	logo_h = simple_strtoul(logo_hor_pos, NULL, DECIMAL);
	ad_writel(logo_h & AD_VP_LOGO_LEFT_MASK, AD_VP_LOGO_LEFT);

	ad_dbg("%s AD_VP_LOGO_LEFT: 0x%x.\n", __func__, ad_readl(AD_VP_LOGO_LEFT));
	return 0;
}

static int ad_dump_lut(const char *func, int *lut, int lut_size)
{
	int i = 0;

	if(!(dbg_level & DBG_DEBUG))
		return 0;

	printk("\n%s:", func);
	for(i = 0; i < lut_size; i++) {
		if(i % 16 == 0)
			printk("\n");
		printk("%4d, ", lut[i]);
	}
	printk("\n");

	return 0;
}

static int ad_set_bl_lin_lut(char *bl_lin_lut)
{
	int i = 0;
	char *ptr, *delim = ",";

	while(bl_lin_lut) {

		ptr = strsep(&bl_lin_lut, delim);
		bp.bl_lin_lut[i++] = simple_strtoul(ptr, NULL, DECIMAL);

		if(i >= BL_LIN_LUT_NUM)
			break;
	}
	bp.bl_lin_lut[BL_LIN_LUT_NUM - 1] = 4096;

	ad_dump_lut(__func__, bp.bl_lin_lut, ARRAY_SIZE(bp.bl_lin_lut));
	return 0;
}

static int ad_set_bl_lin_inverse_lut(char *bl_lin_inverse_lut)
{
	int i = 0;
	char *ptr, *delim = ",";

	while(bl_lin_inverse_lut) {

		ptr = strsep(&bl_lin_inverse_lut, delim);
		bp.bl_lin_inverse_lut[i++] = simple_strtoul(ptr, NULL, DECIMAL);

		if(i >= BL_LIN_LUT_NUM)
			break;
	}
	bp.bl_lin_inverse_lut[BL_LIN_LUT_NUM - 1] = 4096;

	ad_dump_lut(__func__, bp.bl_lin_inverse_lut, ARRAY_SIZE(bp.bl_lin_inverse_lut));
	return 0;
}

static int ad_set_alpha(char *alpha)
{
	bp.alpha = simple_strtoul(alpha, NULL, DECIMAL);

	ad_dbg("%s alpha = %d\n", __func__, bp.alpha);
	return 0;
}

static int ad_set_bl_att_lut(char *bl_att_lut)
{
	int i = 0;
	char *ptr, *delim = ",";

	while(bl_att_lut) {

		ptr = strsep(&bl_att_lut, delim);
		bp.bl_att_lut[i++] = simple_strtoul(ptr, NULL, DECIMAL);

		if(i >= BL_ATT_LUT_NUM)
			break;
	}
	bp.bl_att_lut[BL_ATT_LUT_NUM - 1] = 4096;

	ad_dump_lut(__func__, bp.bl_att_lut, ARRAY_SIZE(bp.bl_att_lut));
	return 0;
}

static int ad_set_als_offset(char *als_offset)
{
	/* Nufront DSI Ctrl always keep wake-up state. */
	return 0;
}

static int ad_set_als_thresh(char *als_thresh)
{
	/* Nufront DSI Ctrl always keep wake-up state. */
	return 0;
}

/*
 * An ad_message consists of an ad_command and some ad_params, two parameters are separated by semicolon (;).
 *
 * Here is an example of ad_message sent by AD Calibration Tool during the AD calibration, it is composed of
 * "ad:init" and some parameters data:
 *	   ad:init;0,211,414,609,796,975,1148,1315,1475,1630,1779,1922,2061,2195,2325,2451,2572,2690,2804,
 *	   2915,3022,3126,3227,3325,3420,3513,3603,3691,3776,3859,3940,4019,4095;255,278,302,326,350,374,
 *	   398,422,446,470,494,517,541,565,589,613,637,661,684,708,732,755,779,803,826,850,874,897,921,945,
 *	   968,992,1016;7,134;0;1023;65;240;0;60;128;5;3;0;720;1280;16;1;0,16,32,48,64,80,96,112,128,145,
 *	   161,177,193,209,225,241,257,273,289,305,321,337,353,369,385,401,418,434,450,466,482,498,514,530,
 *	   ... ...
 *	   3694,3710,3726,3742,3758,3774,3790,3806,3822,3838,3854,3870,3886,3902,3918,3934,3950,3967,3983,
 *	   3999,4015,4031,4047,4063,4079,4095;0,16,32,48,64,80,96,112,128,145,161,177,193,209,225,241,257,
 *	   273,289,305,321,337,353,369,385,401,418,434,450,466,482,498,514,530,546,562,578,594,610,626,642,
 *	   ... ...
 *	   3790,3806,3822,3838,3854,3870,3886,3902,3918,3934,3950,3967,3983,3999,4015,4031,4047,4063,4079,
 *	   4095;512;0,128,256,384,470,559,618,665,706,745,785,826,872,924,982,1047,1118,1198,1285,1382,1490,
 *	   1608,1738,1881,2039,2213,2405,2618,2853,3115,3406,3731,4095;100;0.5;
 *
 * From the ad_message, we know that the ad_command is ad:init, the following data is for different ad_params
 * which is seperated by ';'. The ad_param examples are as follows:
 *
 *	   asym_lut = 0,211,414,609,796,975,1148,1315,1475,1630,1779,1922,2061,2195,2325,2451,2572,2690,
 *	   2804,2915,3022,3126,3227,3325,3420,3513,3603,3691,3776,3859,3940,4019,4095
 *	   color_corr_lut = 255,278,302,326,350,374,398,422,446,470,494,517,541,565,589,613,637,661,
 *	   684,708,732,755,779,803,826,850,874,897,921,945,968,992,1016
 *	   i_control = 7,134
 *	   black_lvl = 0
 *	   white_lvl = 1023
 *	   varience = 65
 *	   limit_amplitude = 240
 *	   i_dither = 0
 *	   slope_max = 60
 *	   slope_min = 128
 *	   dither_control = 5
 *	   format = 3
 *	   autosize = 0
 *	   frame_width = 720
 *	   frame_height = 1280
 *	   logo_vertical_pos = 16
 *	   logo_hor_pos = 1
 *	   bl_lin_lut = 0,16,32,48,64,80,96,112,128,145,161,177,193,209,225,241,257,273,289,305,321,
 *	   337,353,369,385,401,418,434,450,466,482,498,514,530,546,562,578,594,610,626,642,658,674,691,707,
 *	   3228,3244,3260,3276,3292,3308,3324,3340,3356,3372,3388,3404,3421,3437, ... ..., 4095
 *	   bl_lin_inverse_lut = 0,16,32,48,64,80,96,112,128,145,161,177,193,209,225,241,257,273,289,
 *	   305,321,337,353,369,385,401,418,434,450,466,482,498,514,530,546,562,578,594,610,626,642,658,674,
 *	   3196,3212,3228,3244,3260,3276,3292,3308,3324,3340,3356,3372,3388,3404, ... ..., 4095
 *	   alpha = 512
 *	   bl_att_lut = 0,128,256,384,470,559,618,665,706,745,785,826,872,924,982,1047,1118,1198,1285,1382,1490,
 *	   1608,1738,1881,2039,2213,2405,2618,2853,3115,3406,3731,4095
 *	   als_offset = 100
 *	   als_thresh = 0.5
 */
struct ad_param_handler ad_init_params_handler[] = {

	{ad_param_asym_lut,		ad_set_asym_lut},
	{ad_param_color_corr_lut,	ad_set_color_corr_lut},
	{ad_param_i_control,		ad_set_i_control},
	{ad_param_black_lvl,		ad_set_black_lvl},
	{ad_param_white_lvl,		ad_set_white_lvl},
	{ad_param_variance,		ad_set_variance},
	{ad_param_limit_amplitude,	ad_set_limit_amplitude},
	{ad_param_i_dither,		ad_set_i_dither},
	{ad_param_slope_max,		ad_set_slope_max},
	{ad_param_slope_min,		ad_set_slope_min},
	{ad_param_dither_control,	ad_set_dither_control},
	{ad_param_format,		ad_set_format},
	{ad_param_autosize,		ad_set_autosize},
	{ad_param_frame_width,		ad_set_frame_width},
	{ad_param_frame_height,		ad_set_frame_height},
	{ad_param_logo_vertical_pos,	ad_set_logo_vertical_pos},
	{ad_param_logo_hor_pos,		ad_set_logo_hor_pos},
	{ad_param_bl_lin_lut,		ad_set_bl_lin_lut},
	{ad_param_bl_lin_inverse_lut,	ad_set_bl_lin_inverse_lut},
	{ad_param_alpha,		ad_set_alpha},
	{ad_param_bl_att_lut,		ad_set_bl_att_lut},
	{ad_param_als_offset,		ad_set_als_offset},
	{ad_param_als_thresh,		ad_set_als_thresh},
};

#define OPTION_SELECT_TRIGGER_START_CALC_MODE_0_MANU_TRIGGER	0 << 4
#define OPTION_SELECT_TRIGGER_START_CALC_MODE_8_AUTO_TRIGGER	8 << 4

#define OPTION_SELECT_FUNCTION_MODE_0_AUTO_BC_AUTO_ACM	0 << 0
#define OPTION_SELECT_FUNCTION_MODE_1_MANU_BC_AUTO_ACM	1 << 0
#define OPTION_SELECT_FUNCTION_MODE_3_MANU_BC_MANU_ACM	3 << 0
#define OPTION_SELECT_FUNCTION_MODE_7_MANU_BC_MANU_ACM	7 << 0

/*
 * In Apical AD IP core, non-video pipeline parameters could use start_calc to implement
 * configuration. CPU will update the parameters and then transition the start_calc input
 * from 0 to 1. For efficient use of inbuilt temporal filters it is recommended that the
 * start_calc pulse is given to the core every frame, even if the ambient light parameter
 * does not change. This can be achieved by putting the option_select[7:4] in mode 8.
 *
 * For video pipeline parameters configuration, please refer to ad_vp_writel();
 */
static int ad_set_mode(char *md)
{
	int m, mode;

	mode = simple_strtoul(md, NULL, DECIMAL);
	mode |= OPTION_SELECT_TRIGGER_START_CALC_MODE_8_AUTO_TRIGGER;

	m = (mode & 0xf) >> 4;
	if((m != 0) && (m != 8)) {
		ad_err("Trigger mode '%d' is invalid.\n", m);
		return -EINVAL;
	}

	m = mode & 0xf;
	if((m != 0) && (m != 1)) {
		ad_err("Function mode '%d' is not supported.\n", m);
		if(m == 3 || m == 7)
			ad_info("The manual Backlight Control and manual Adaptive Content Management \
					is used for non-emissive display.\n");
		return -EINVAL;
	}

	ad_writel(mode, AD_OPTION_SELECT);

	ad_dbg("%s mode:%d, AD_OPTION_SELECT: 0x%x.\n", __func__, mode, ad_readl(AD_OPTION_SELECT));
	return 0;
}

static int ad_set_ambient_calib_lut(char *al_calib_lut)
{
	int i = 0;
	unsigned int data;
	char *ptr, *delim = ",";

	while(al_calib_lut) {

		ptr = strsep(&al_calib_lut, delim);
		data = simple_strtoul(ptr, NULL, DECIMAL);

		if(i <= AD_AL_CALIB_LUT_ADDR_I_MASK) {
			ad_writel(i, AD_AL_CALIB_LUT_ADDR_I);
			ad_writel(data & 0xff, AD_AL_CALIB_LUT_DATA_W_L);
			ad_writel((data >> 8) & 0xff, AD_AL_CALIB_LUT_DATA_W_H);
		} else
			break;
		i++;
	}

	ad_dbg("%s\n", __func__);
	ad_dbg("AD_AL_CALIB_LUT_ADDR_I:   0x%x.\n", ad_readl(AD_AL_CALIB_LUT_ADDR_I));
	ad_dbg("AD_AL_CALIB_LUT_DATA_W_L: 0x%x.\n", ad_readl(AD_AL_CALIB_LUT_DATA_W_L));
	ad_dbg("AD_AL_CALIB_LUT_DATA_W_H: 0x%x.\n", ad_readl(AD_AL_CALIB_LUT_DATA_W_H));
	return 0;
}

static int ad_set_back_min(char *back_min)
{
	unsigned int data;

	data = simple_strtoul(back_min, NULL, DECIMAL);
	ad_writel(data & 0xff, AD_BACKLIGHT_MIN_L);
	ad_writel((data >> 8) & 0xff, AD_BACKLIGHT_MIN_H);

	ad_dbg("%s AD_BACKLIGHT_MIN_L: 0x%x, AD_BACKLIGHT_MIN_H: 0x%x.\n",
			__func__, ad_readl(AD_BACKLIGHT_MIN_L), ad_readl(AD_BACKLIGHT_MIN_H));
	return 0;
}

static int ad_set_back_max(char *back_max)
{
	unsigned int data;

	data = simple_strtoul(back_max, NULL, DECIMAL);
	ad_writel(data & 0xff, AD_BACKLIGHT_MAX_L);
	ad_writel((data >> 8) & 0xff, AD_BACKLIGHT_MAX_H);

	ad_dbg("%s AD_BACKLIGHT_MAX_L: 0x%x, AD_BACKLIGHT_MAX_H: 0x%x.\n",
			__func__, ad_readl(AD_BACKLIGHT_MAX_L), ad_readl(AD_BACKLIGHT_MAX_H));
	return 0;
}

static int ad_set_backlight_scale(char *back_scale)
{
	unsigned int data;

	data = simple_strtoul(back_scale, NULL, DECIMAL);
	ad_writel(data & 0xff, AD_BACKLIGHT_SCALE_L);
	ad_writel((data >> 8) & 0xff, AD_BACKLIGHT_SCALE_H);

	ad_dbg("%s AD_BACKLIGHT_SCALE_L: 0x%x, AD_BACKLIGHT_SCALE_H: 0x%x.\n",
			__func__, ad_readl(AD_BACKLIGHT_SCALE_L), ad_readl(AD_BACKLIGHT_SCALE_H));
	return 0;
}

static int ad_set_ambient_light_min(char *ambient_light_min)
{
	unsigned int data;

	data = simple_strtoul(ambient_light_min, NULL, DECIMAL);
	ad_writel(data & 0xff, AD_AMBIENT_LIGHT_MIN_L);
	ad_writel((data >> 8) & 0xff, AD_AMBIENT_LIGHT_MIN_H);


	ad_dbg("%s AD_AMBIENT_LIGHT_MIN_L: 0x%x, AD_AMBIENT_LIGHT_MIN_H: 0x%x.\n",
			__func__, ad_readl(AD_AMBIENT_LIGHT_MIN_L), ad_readl(AD_AMBIENT_LIGHT_MIN_H));
	return 0;

}

static int ad_set_filter(char *filter)
{
	char *ptr, *delim = ",";
	unsigned int filter_a, filter_b;

	ptr = strsep(&filter, delim);
	filter_a = simple_strtoul(ptr, NULL, DECIMAL);
	ad_writel(filter_a & 0xff, AD_FILTER_A_L);
	ad_writel((filter_a >> 8) & 0xff, AD_FILTER_A_H);

	ptr = strsep(&filter, delim);
	filter_b = simple_strtoul(ptr, NULL, DECIMAL);
	ad_writel(filter_b & 0xff, AD_FILTER_B);

	ad_dbg("%s AD_FILTER_A_L: 0x%x, AD_FILTER_A_H: 0x%x, AD_FILTER_B: 0x%x.\n",
			__func__, ad_readl(AD_FILTER_A_L), ad_readl(AD_FILTER_A_H), ad_readl(AD_FILTER_B));

	return 0;
}

static int ad_set_calibration_abcd(char *calib_abcd)
{
	char *ptr, *delim = ",";
	unsigned int ca, cb, cc, cd;

	ptr = strsep(&calib_abcd, delim);
	ca = simple_strtoul(ptr, NULL, DECIMAL);
	ca *= ad_dev->assertiveness;
	ad_writel(ca & 0xff, AD_CALIBRATION_A_L);
	ad_writel((ca >> 8) & 0xff, AD_CALIBRATION_A_H);

	ptr = strsep(&calib_abcd, delim);
	cb = simple_strtoul(ptr, NULL, DECIMAL);
	cb *= ad_dev->assertiveness;
	ad_writel(cb & 0xff, AD_CALIBRATION_B_L);
	ad_writel((cb >> 8) & 0xff, AD_CALIBRATION_B_H);

	ptr = strsep(&calib_abcd, delim);
	cc = simple_strtoul(ptr, NULL, DECIMAL);
	cc *= ad_dev->assertiveness;
	ad_writel(cc & 0xff, AD_CALIBRATION_C_L);
	ad_writel((cc >> 8) & 0xff, AD_CALIBRATION_C_H);

	ptr = strsep(&calib_abcd, delim);
	cd = simple_strtoul(ptr, NULL, DECIMAL);
	cd *= ad_dev->assertiveness;
	ad_writel(cd & 0xff, AD_CALIBRATION_D_L);
	ad_writel((cd >> 8) & 0xff, AD_CALIBRATION_D_H);

	ad_dbg("%s\n", __func__);
	ad_dbg("AD_CALIBRATION_A_L: 0x%02x, AD_CALIBRATION_A_H: 0x%02x.\n",
		ad_readl(AD_CALIBRATION_A_L), ad_readl(AD_CALIBRATION_A_H));
	ad_dbg("AD_CALIBRATION_B_L: 0x%02x, AD_CALIBRATION_B_H: 0x%02x.\n",
		ad_readl(AD_CALIBRATION_B_L), ad_readl(AD_CALIBRATION_B_H));
	ad_dbg("AD_CALIBRATION_C_L: 0x%02x, AD_CALIBRATION_C_H: 0x%02x.\n",
		ad_readl(AD_CALIBRATION_C_L), ad_readl(AD_CALIBRATION_C_H));
	ad_dbg("AD_CALIBRATION_D_L: 0x%02x, AD_CALIBRATION_D_H: 0x%02x.\n",
		ad_readl(AD_CALIBRATION_D_L), ad_readl(AD_CALIBRATION_D_H));

	return 0;
}

static int ad_set_str_limit(char *str_limit)
{
	unsigned int data;

	data = simple_strtoul(str_limit, NULL, DECIMAL);
	ad_writel(data, AD_STRENGTH_LIMIT);

	ad_dbg("%s AD_STRENGTH_LIMIT: 0x%x.\n", __func__, ad_readl(AD_STRENGTH_LIMIT));
	return 0;
}

static int ad_set_temp_filter_recurs(char *temp_filter_recurs)
{
	unsigned int data;

	data = simple_strtoul(temp_filter_recurs, NULL, DECIMAL);
	ad_writel(data, AD_T_FILTER_CONTROL);

	ad_dbg("%s AD_T_FILTER_CONTROL: 0x%x.\n", __func__, ad_readl(AD_T_FILTER_CONTROL));
	return 0;
}

static int ad_set_stabilization_iterat(char *si)
{
	unsigned int data;

	data = simple_strtoul(si, NULL, DECIMAL);
	ad_writel(data & AD_START_CALC_MASK, AD_START_CALC);

	ad_dbg("%s AD_START_CALC: 0x%x.\n", __func__, ad_readl(AD_START_CALC));
	return 0;
}

/*
 * An ad_message consists of an ad_command and some ad_params, two parameters are separated by semicolon (;).
 *
 * Here is an example of ad_message sent by AD Calibration Tool during the AD calibration, it is composed of
 * "ad:config" and some parameters data:
 *	   ad:config;1;0,2048,4096,6144,8192,10240,12288,14336,16384,18432,20480,22528,24576,26624,28672,30720,32768,
 *	   34815,36863,38911,40959,43007,45055,47103,49151,51199,53247,55295,57343,59391,61439,63487,65535;480;4095;
 *	   4095;14;1738,6;110,255,0,0;0;5;60
 *
 * From the ad_message, we know that the ad_command is ad:config, the following data is for different ad_params
 * which is seperated by ';'. The ad_param examples are as follows:
 *
 *	   mode = 1
 *	   ambient_calib_lut = 0,2048,4096,6144,8192,10240,12288,14336,16384,18432,20480,22528,24576,26624,28672,30720,
 *	   32768,34815,36863,38911,40959,43007,45055,47103,49151,51199,53247,55295,57343,59391,61439,63487,65535
 *	   back_min = 480
 *	   back_max = 4095
 *	   backlight_scale = 4095
 *	   ambient_light_min = 14
 *	   filter = 1738,6
 *	   calibration_abcd = 110,255,0,0
 *	   str_limit = 0
 *	   temp_filter_recurs = 5
 *	   stabilization_iterations = 60
 */
struct ad_param_handler ad_config_params_handler[] = {

	{ad_param_mode,			ad_set_mode},
	{ad_param_ambient_calib_lut,	ad_set_ambient_calib_lut},
	{ad_param_back_min,		ad_set_back_min},
	{ad_param_back_max,		ad_set_back_max},
	{ad_param_backlight_scale,	ad_set_backlight_scale},
	{ad_param_ambient_light_min,	ad_set_ambient_light_min},
	{ad_param_filter,		ad_set_filter},
	{ad_param_calibration_abcd,	ad_set_calibration_abcd},
	{ad_param_str_limit,		ad_set_str_limit},
	{ad_param_temp_filter_recurs,	ad_set_temp_filter_recurs},
	{ad_param_stabilization_iterat, ad_set_stabilization_iterat},
};

static int ad_pow(int x, int y)
{
	int i = 1, result = 1;

	if(y < 0) {
		ad_err("ad_pow invalid y:%d.\n", y);
		return -EINVAL;
	}

	if(y == 0)
		return 1;

	for(i = 1; i <= y; i++)
		result *= x;

	return result;
}

/*
 * Function to output interpolated values from the lookup table
 * dat_w - number of bits
 * lut_a - log2 of number of nodes;
 * lut_nodes - array of values of nodes;
 * x - input
 * y - output
 */
int ad_apical_lut(int dat_w, int lut_a, int x, int *lut_nodes)
{
	int result1;
	int node0, node1;
	int sign, diff, y;
	int node_mul, diff_mul;

	result1 = x >> (dat_w - lut_a);
	node0 = lut_nodes[result1];
	node1 = lut_nodes[result1+1];
	diff = node1 - node0;
	if (diff < 0)
		sign = 1; // negative slope
	else
		sign = 0; // positive slope
	diff = abs(diff);
	node_mul = x & ((int)ad_pow(2, (dat_w-lut_a)) -1);
	diff_mul = node_mul * diff;
	if(sign == 0)
		y= node0 + (diff_mul >> (dat_w - lut_a));
	else
		y = node0 - (diff_mul >> (dat_w - lut_a));
	return y;
}

/*
 * Function calculates the attenuated and blended backlight values from the input - (Note
 * this is not going through inverse linearization)
 *
 * all three lookup tables BLlin256 (256 elements), BLatt33 (33 elements) are read from
 * the ad_calib.cfg file
 * alpha - Tuning parameter read from the ad_calib.cfg file
 * BLscale - Maximum value of backlight read from the ad_calib.cfg file
 * inpt_BL - input value of backlight
 *
 * returns the calculated output backlight
 */
int ad_calculate_attenuated_blended_bl(int* BLlin256, int* BLatt33, int alpha, int num_bl_bits, int inpt_BL)
{
	int backlight_scaled = ad_pow(2, (12-num_bl_bits))*inpt_BL;
	int backlight_lin = ad_apical_lut(12, 8, backlight_scaled, BLlin256);
	int backlight_atten = ad_apical_lut(12, 5, backlight_lin, BLatt33);
	int bl_atten_minus_bl_lin = backlight_atten - backlight_lin;
	int mult_o = (abs(bl_atten_minus_bl_lin) * alpha) >> 10;
	int my_sign;

	if (bl_atten_minus_bl_lin < 0)
		my_sign = -1;
	else
		my_sign = 1;

	return (backlight_lin + my_sign*(mult_o));
}

static int ad_set_ambient_light(int ambient_light)
{
	static int last_ambient_light = 0;

	if(last_ambient_light == ambient_light)
		return 0;
	else
		last_ambient_light = ambient_light;

	mutex_lock(&ad_dev->al_lock);

	ad_writel(ambient_light & 0xff, AD_AMBIENT_LIGHT_L);
	ad_writel((ambient_light >> 8) & 0xff, AD_AMBIENT_LIGHT_H);

	ad_dbg("\n%s ambient_light = %d, AD_AMBIENT_LIGHT_L: 0x%x, AD_AMBIENT_LIGHT_H: 0x%x.\n",
			__func__, ambient_light, ad_readl(AD_AMBIENT_LIGHT_L), ad_readl(AD_AMBIENT_LIGHT_H));
	mutex_unlock(&ad_dev->al_lock);

	return 0;
}

/*
 * Deliver the current backlight value as set by the OS auto or manual brightness function to
 * the register backlight. However, if the backlight has not changed, then an update is not necessary.
 */
static int ad_set_backlight(char *bl)
{
	int ret = 0, num_bl_bits = 12;
	int backlight_scaleup, backlight_blended, backlight_out_linear, backlight_out;
	static int backlight, last_backlight = 0;

	if(bl == NULL)
		return backlight;

	backlight = simple_strtoul(bl, NULL, DECIMAL);
	if(backlight < 0 || backlight > 255) {
		ad_err("AD driver can't process bl:%d, update the AD driver.\n", backlight);
		return backlight;
	}
	ad_dev->brightness = backlight;

	if(bp.bl_lin_lut[BL_LIN_LUT_NUM - 1] == 0 || bp.bl_lin_inverse_lut[BL_LIN_LUT_NUM - 1] == 0 || \
			bp.bl_att_lut[BL_ATT_LUT_NUM - 1] == 0) {

		ad_warn("bl_lin_lut / bl_lin_inverse_lut / bl_att_lut should be filled.\n");
		return -EPERM;
	}

	if(last_backlight == backlight)
		return 0;
	else
		last_backlight = backlight;

	mutex_lock(&ad_dev->bl_lock);

	/* 8bit backlight 0~255 scacle to 12bit */
	backlight_scaleup = backlight << 4;
	/* Attenuated and blended backlight value going into the AD core */
	backlight_blended = ad_calculate_attenuated_blended_bl(bp.bl_lin_lut, bp.bl_att_lut,
			bp.alpha, num_bl_bits, backlight_scaleup);
	/* output backlight, LiMing: 12bit range of backlight_blended */
	backlight_out_linear = ad_apical_lut(12, 8, backlight_blended, bp.bl_lin_inverse_lut);
	/* rescaled output backlight going into the panel, shift right to rescale back to 8 bits */
	backlight_out = backlight_out_linear >> 7;

	backlight_out = backlight_out <= 0? 1 :backlight_out;
	backlight_out = backlight_out > 20? 20:backlight_out;

	ad_dbg("\n%s backlight: %4d, backlight_blended: %4d, backlight_out_scaledn: %4d\n",
			__func__, backlight, backlight_blended,	backlight_out);

	ad_writel(backlight_blended & AD_BACKLIGHT_L_MASK, AD_BACKLIGHT_L);
	ad_writel((backlight_blended >> 8 ) & AD_BACKLIGHT_H_MASK, AD_BACKLIGHT_H);
	ad_dbg("%s AD_BACKLIGHT_L: 0x%x, AD_BACKLIGHT_H: 0x%x\n",
			__func__, ad_readl(AD_BACKLIGHT_L), ad_readl(AD_BACKLIGHT_H));

	nu7tlbl_set_brightness(backlight_out);
	mutex_unlock(&ad_dev->bl_lock);

	return ret;
}

/*
 * An ad_message consists of an ad_command and some ad_params, two parameters are separated by semicolon (;).
 *
 * Here is an example of ad_message sent by AD Calibration Tool during the AD calibration, it is composed of
 * "bl:set" and the backlight value "255":
 * Here is an example of ad_message sent by AD Calibration Tool during the AD calibration:
 *	   bl:set;255
 *
 * From the ad_message, we know that the ad_command is bl:set, the following data is for ad_param
 * backlight, the backlight value is 255.
 *
 */
struct ad_param_handler ad_bl_set_params_handler[] = {

	{ad_param_backlight,		ad_set_backlight},
};

static int ad_message_handler(char *msg)
{
	int i = 0, ret = 0;
	char *delim = ";";
	char *ad_command, *param;
	struct ad_param_handler *param_handler;

	if(msg == NULL)
		return -EIO;

	ad_dbg("---------------------------------------------------\n");
	ad_dbg("ad_message: %s\n", msg);

	ad_command = strsep(&msg, delim);
	if(!strcmp(ad_command, ad_command_ad_init))
		param_handler = ad_init_params_handler;

	else if(!strcmp(ad_command, ad_command_ad_input))
		param_handler = ad_config_params_handler;

	else if(!strcmp(ad_command, ad_command_ad_config))
		param_handler = ad_config_params_handler;

	else if(!strcmp(ad_command, ad_command_bl_set))
		param_handler = ad_bl_set_params_handler;
	else {
		ad_err("Unsupported AD command %s\n", ad_command);
		return -EIO;
	}
	ad_dbg("ad_command: %s\n", ad_command);

	while(msg) {
		param = strsep(&msg, delim);
		if(param) {
			ad_dbg("\nad_param ");
			ad_dbg("%s: %s\n",  param_handler[i].ad_param_name, param);
			ret = param_handler[i].hdl(param);
			if(ret)
				return ret;
			i++;
		}
	}
	return 0;
}

static int ad_parse_display_interface(struct device_node *bl_node)
{
	struct device_node *display_interface;

	display_interface = of_parse_phandle(bl_node, "display_interface", 0);
	if (!display_interface) {
		pr_err("Failed to get display_interface for %s.\n", bl_node->full_name);
		return -ENODEV;
	}

	if(!strcmp(display_interface->name, "VGA") || !strcmp(display_interface->name, "vga")) {
		pr_warn("VGA display interface doesn't need assertive display driver.\n");
		return -EINVAL;
	}

	return 0;
}

static int ad_enable(void)
{
	ad_writel(0x07, AD_VP_IRIDIX_CONTROL_0);
	ad_dbg("%s AD_VP_IRIDIX_CONTROL_0: 0x%08x\n",
			__func__, ad_readl(AD_VP_IRIDIX_CONTROL_0));
	return 0;
}

static int ad_disable(void)
{
	ad_writel(0x06, AD_VP_IRIDIX_CONTROL_0);
	ad_dbg("%s AD_VP_IRIDIX_CONTROL_0: 0x%08x\n",
			__func__, ad_readl(AD_VP_IRIDIX_CONTROL_0));
	return 0;
}

/*
 * The AD IP software drivers support two modes of operation:
 * 1. Calibration Mode, refer to ad_calibration_mode()
 * 2. Operational Mode, refer to ad_operational_mode()
 *
 * The operation of the two modes are actually the same, they
 * all load the message param to the corresponding registers.
 *
 * How to enable the calibration mode or operational mode??
 * Take the nufront-tl7790-phone-test.dts for example, the
 * "calibration_mode" property in "ad" node:
 *	calibration_mode = <0>; selects operational mode.
 *	calibration_mode = <1>; selects calibration mode.
 */

/*
 * When developing a new panel or changing the ALS on a product,
 * R&D should calibrate the panel with the AD Calibration Tool.apk,
 * to generate a set of configuration in /data/misc/display/calib.cfg.
 * A brief calibration process is as follows:
 *
 *		AD Calibration Tool
 *			|
 *			|  send
 *			| message
 *			|   to
 *			| service
 *			|
 *		Socket "pps" service
 *			|
 *			|  ioctl
 *			| message
 *			|   to
 *			|  driver
 *			|
 *		ad_calibration_mode()
 *		(handle the message params
 *		to corresponding registers)
 */
static long ad_calibration_mode(struct file *file, unsigned int cmd, unsigned long arg)
{
	char args[4096] = {0};

	switch(cmd) {
		case AD_MESSAGE_AD_INIT:
			if (!copy_from_user(args, (void*)arg, sizeof(args))) {
				printk("\nAD_MESSAGE_AD_INIT\n");
				ad_message_handler(args);
			}
			break;

		case AD_MESSAGE_AD_INPUT:
			if (!copy_from_user(args, (void*)arg, 12)) {
				//printk("\nAD_MESSAGE_AD_INPUT\n");
				//ad_message_handler(args);
			}
			break;

		case AD_MESSAGE_AD_CONFIG:
			if (!copy_from_user(args, (void*)arg, sizeof(args))) {
				printk("\nAD_MESSAGE_AD_CONFIG\n");
				ad_message_handler(args);
			}
			break;

		case AD_MESSAGE_AD_CALIB_ON:
			printk("\nAD_MESSAGE_AD_CALIB_ON\n");
			ad_enable();
			break;

		case AD_MESSAGE_AD_CALIB_OFF:
			printk("\nAD_MESSAGE_AD_CALIB_OFF\n");
			ad_disable();
			break;

		case AD_MESSAGE_BL_SET:
			if (!copy_from_user(args, (void*)arg, 10)) {
				printk("\nAD_MESSAGE_BL_SET\n");
				ad_message_handler(args);
			}
			break;
		default:
			ad_err("Unsupported AD message 0x%x, update the AD driver.\n", cmd);
			return -EIO;
	};

	return 0;
}

/*
 * When the calibration is completed, there generate the /data/misc/display/calib.cfg,
 * R&D should cut out the available(exclusive the line with #) string after "=init"
 * and "=config", a cutted out string please refer to "ad_calibrated_settings_ad_init"
 * and "ad_calibrated_settings_ad_config", ad_operational_mode() will load these params
 * to the corresponding registers.
 * ad_operational_mode() is normal mode of operation for the AD IP core.
 */
static int ad_operational_mode(void)
{
	char args[4096] = {0};

	if(ad_dev->calibration_mode)
		return -EPERM;

	memcpy(args, ad_calibrated_settings_ad_init,
			strlen(ad_calibrated_settings_ad_init));
	ad_message_handler(args);

	memcpy(args, ad_calibrated_settings_ad_config,
			strlen(ad_calibrated_settings_ad_config));
	ad_message_handler(args);

	return 0;
}

/*
 * ----------------------------------------------------------------------------------------------
 * |      Backlight Type	|	Ambient light		|	    Backlight		|
 * |---------------------------------------------------------------------------------------------
 * |				|				|				|
 * |		Auto		|				| Update according to ALS value	|
 * |	     Brightness		|	Update at 10Hz		|	(al_bl_mapping)		|
 * |	(auto_brightness = 1)	|  in ad_set_auto_brightness()	| in ad_set_auto_brightness()	|
 * |				|				|				|
 * |----------------------------|-------------------------------|-------------------------------|
 * |				|				|				|
 * |		Manual		|				|				|
 * |	     Brightness		|	Update at 10Hz		|   Update by Android slider	|
 * |	(auto_brightness = 0)	|  in ad_set_auto_brightness()	| in ad_set_manual_brightness()	|
 * |				|				|				|
 * |----------------------------|-------------------------------|-------------------------------|
 */
static ssize_t ad_set_manual_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	char bl[5] = {0};
	unsigned long brightness;

	if(auto_brightness) {
		//ad_warn("auto_brightness:%d, manual brightness is not permitted.\n",
		//		auto_brightness);
		return count;
	}

	ret = kstrtoul(buf, 0, &brightness);
	if (ret)
		return ret;

	if(brightness > ad_dev->abm[ad_dev->abm_size - 1].bl)
		brightness =  ad_dev->abm[ad_dev->abm_size - 1].bl;

	sprintf(bl, "%ld\n", brightness);
	ret = ad_set_backlight(bl);
	if(ret) {
		ad_err("%s set ad backlight error.\n", __func__);
		return ret;
	}

	ret = count;
	return ret;
}

static void ad_set_auto_brightness(struct work_struct *work)
{
	int i;
	char bl[5] = {0};
	int backlight_from, cur_backlight, to_new_backlight = 0;

	ad_set_ambient_light(al_lux);

	if(auto_brightness && (ad_dev->calibration_mode == 0)) {

		cur_backlight = ad_set_backlight(NULL);

		for(i = 0; i < ad_dev->abm_size; i++) {
			if(al_lux <= ad_dev->abm[i].al) {
				to_new_backlight = ad_dev->abm[i].bl;
				break;
			}
		}
		if(cur_backlight < to_new_backlight) {
			for(backlight_from = cur_backlight; backlight_from <=
					to_new_backlight; backlight_from += 5) {
				msleep(5);
				sprintf(bl, "%d\n", backlight_from);
				ad_set_backlight(bl);
			}
		}
		else if(cur_backlight > to_new_backlight) {
			for(backlight_from = cur_backlight; backlight_from >=
					to_new_backlight; backlight_from -= 5) {
				msleep(5);
				sprintf(bl, "%d\n", backlight_from);
				ad_set_backlight(bl);
			}
		}

		backlight_from = to_new_backlight;
		sprintf(bl, "%d\n", backlight_from);
		ad_set_backlight(bl);

	}
	schedule_delayed_work(&ad_dev->auto_brightness_work, msecs_to_jiffies(100));
}

static int ad_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;

	ret = ad_parse_display_interface(pdev->dev.of_node);
	if(ret) {
		ad_warn("Assertive Display driver is not loaded.\n");
		return 0;
	}

	ad_dev = kzalloc(sizeof(struct tl7790_ad_device), GFP_KERNEL);
	if(!ad_dev) {
		ad_err("Failed to allocate configuration data.\n");
		return -ENXIO;
	}

	ret = ad_sysfs_init();
	if(ret) {
		ad_err("Initialize AD device failed.\n");
		goto err_sysfs_init;
	}

	ad_dev->assertiveness =  ad_of_property_read_u32(np,
			"assertiveness");
	ad_dev->calibration_mode =  ad_of_property_read_u32(np,
			"calibration_mode");

	if(ad_dev->calibration_mode)
		ad_info("####### AD calibration mode #######\n");
	else
		ad_info("####### AD operational mode #######\n");

	ad_dev->abm = sample_abm;
	ad_dev->abm_size = ARRAY_SIZE(sample_abm);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ad_err("Failed to get AD mem resource.\n");
		goto err_iosrc_remap;
	}

	ad_mmio = ioremap_nocache(res->start, resource_size(res));
	if (!ad_mmio) {
		ad_err("AD controller address ioremap error.\n");
		goto err_iosrc_remap;
	}

	ret = ad_notifier_register();
	if(ret) {
		ad_err("Failed to register AD notifier client.\n");
		goto err_regist_notif;
		return -EPERM;
	}

	mutex_init(&ad_dev->al_lock);
	mutex_init(&ad_dev->bl_lock);

	INIT_DELAYED_WORK(&ad_dev->auto_brightness_work, ad_set_auto_brightness);
	schedule_delayed_work(&ad_dev->auto_brightness_work, HZ);

	return 0;

err_regist_notif:
	iounmap(ad_mmio);
err_iosrc_remap:
	ad_sysfs_destroy();
err_sysfs_init:
	kfree(ad_dev);

	return ret;
}

static int ad_remove(struct platform_device *pdev)
{
	iounmap(ad_mmio);
	ad_sysfs_destroy();
	kfree(ad_dev);

	return 0;
}

static int ad_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int ad_resume(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ad_of_match[] = {
	{ .compatible = "nufront, assertive display", },
	{},
};
MODULE_DEVICE_TABLE(of, ad_of_match);

static struct platform_driver ad_driver =
{
	.probe = ad_probe,
	.remove = ad_remove,
	.resume = ad_resume,
	.suspend = ad_suspend,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(ad_of_match),
	},
};

static int __init ad_init(void)
{
	return platform_driver_register(&ad_driver);
}

static void __exit ad_exit(void)
{
	platform_driver_unregister(&ad_driver);
}

module_init(ad_init);
module_exit(ad_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NU7TL(TL7790) Assertive Display Driver");
