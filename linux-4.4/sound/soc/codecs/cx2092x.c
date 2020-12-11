/*
* ALSA SoC CX2092X codec driver
*
* Copyright:   (C) 2014 Conexant Systems, Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*************************************************************************
*  Modified Date:  07/03/14
*  File Version:   3.8.0.21
*************************************************************************
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/gpio.h>
#include <sound/jack.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <sound/cx2092x_pdata.h>
#include "cx2092x.h"
#include "cxflash.h"
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>

#define I2S 0

struct cx2092x_priv *g_cx2092x;
struct snd_soc_codec *g_codec;

#define CAPE_ID(a,b,c,d)  ((((a)-0x20)<<8)|(((b)-0x20)<<14)|(((c)-0x20)<<20)|(((d)-0x20)<<26))
#define CHAR_FROM_CAPE_ID_A(id)  (((((unsigned int)(id))>>8) & 0x3f) + 0x20)
#define CHAR_FROM_CAPE_ID_B(id)  (((((unsigned int)(id))>>14) & 0x3f) + 0x20)
#define CHAR_FROM_CAPE_ID_C(id)  (((((unsigned int)(id))>>20) & 0x3f) + 0x20)
#define CHAR_FROM_CAPE_ID_D(id)  (((((unsigned int)(id))>>26) & 0x3f) + 0x20)

// Jeter 20170109 Add this definition to fix byte-order issue.
//#define FIX_BYTE_ORDER

#define CX2092X_CONTROL(xname, xinfo, xget, xput, xaccess) {\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = xaccess,\
	.info = xinfo, \
	.get = xget, .put = xput, \
	}

#define CMD_GET(item)   ((item) |  0x0100)
#define MAX_COMMAND_SIZE 13
struct cx_command{
  int           num_32b_words:16;
  unsigned int  command_id:15;
  unsigned int  reply:1;
  unsigned int  app_module_id;
  unsigned int  data[MAX_COMMAND_SIZE] ;
};


#define CX2092X_DRIVER_VERSION AUDDRV_VERSION( 3, 8, 0 ,0x21)

#define CX1070X_MAX_REGISTER 0X1300
#define AUDIO_NAME	"cx2092x"
#define MAX_REGISTER_NUMBER 0x1250
#define CX2092X_RATES	( \
       SNDRV_PCM_RATE_8000  \
    | SNDRV_PCM_RATE_11025 \
    | SNDRV_PCM_RATE_16000 \
    | SNDRV_PCM_RATE_22050 \
    | SNDRV_PCM_RATE_32000 \
    | SNDRV_PCM_RATE_44100 \
    | SNDRV_PCM_RATE_48000 \
    | SNDRV_PCM_RATE_88200 \
    | SNDRV_PCM_RATE_96000 )

#define CX2092X_FORMATS ( SNDRV_PCM_FMTBIT_S16_LE \
    | SNDRV_PCM_FMTBIT_S16_BE \
    | SNDRV_PCM_FMTBIT_S32_LE \
    | SNDRV_PCM_FMTBIT_MU_LAW \
    | SNDRV_PCM_FMTBIT_A_LAW )

/* codec private data*/
struct cx2092x_priv {
	struct regmap *regmap;
	int gpio_reset;
	unsigned int sysclk;
	struct device *dev;
	struct snd_soc_dai_driver *dai_drv;
	struct cx_command cmd;
	int cmd_res;
	bool is_updating_fw;
};

#define get_cx2092x_priv(_codec_) ((struct cx2092x_priv *)snd_soc_codec_get_drvdata(codec))

// Jeter 20170109 Used to get the bootloader of CX2092x named "iflash.bin"
const struct firmware *iflash_fw;

#ifdef FIX_BYTE_ORDER
/*
* Convert a 4-byte number from a ByteOrder into another ByteOrder.
*/
uint32_t  byte_order_swap_ulong(uint32_t i)
{
    return((i&0xff)<<24)+((i&0xff00)<<8)+((i&0xff0000)>>8)+((i>>24)&0xff);
}
#endif

static int mem_write_imp(void * i2c_dev, unsigned char slave_addr, unsigned long sub_addr,
	unsigned long write_len,unsigned char* write_buf)
{
#ifdef FIX_BYTE_ORDER
	unsigned long *byte_order_buf = NULL;
	unsigned long  byte_order_index = 0;
	unsigned long *source_buf = (unsigned long *)write_buf;
	int            byte_order_ret = 0;
#endif
	//struct snd_soc_codec *codec = (struct snd_soc_codec *) i2c_dev;
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround

#ifdef FIX_BYTE_ORDER
	byte_order_buf = (unsigned long*) kmalloc(write_len*4, GFP_KERNEL);
	if (!byte_order_buf)
	{
		pr_err("%s: allocate memory for byte_order_buf failed.\n", __func__);
		return 0;
	}

	for (byte_order_index = 0; byte_order_index < write_len; byte_order_index ++)
	{
		*(byte_order_buf + byte_order_index) = byte_order_swap_ulong(*(source_buf + byte_order_index));
	}

	byte_order_ret = regmap_bulk_write(cx2092x->regmap,sub_addr,byte_order_buf,write_len);

	if (byte_order_buf)
	{
		kfree(byte_order_buf);
	}

	return byte_order_ret;
#else
	return regmap_bulk_write(cx2092x->regmap,sub_addr,write_buf,write_len);
#endif
}

static int mem_read_imp(void* i2c_dev, unsigned char slave_addr,
	unsigned long sub_addr, unsigned long rd_len, unsigned char*rd_buf)
{
#ifdef FIX_BYTE_ORDER
	unsigned long *byte_order_buf = NULL;
	unsigned long  byte_order_index = 0;
	unsigned long *source_buf = (unsigned long *)rd_buf;
	int            byte_order_ret = 0;
#endif
	//struct snd_soc_codec *codec = (struct snd_soc_codec *) i2c_dev;
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround

#ifdef FIX_BYTE_ORDER
	byte_order_buf = (unsigned long*) kmalloc(rd_len*4, GFP_KERNEL);
	if (!byte_order_buf)
	{
		pr_err("%s: allocate memory for byte_order_buf failed.\n", __func__);
		return 0;
	}

	byte_order_ret = regmap_bulk_read(cx2092x->regmap,sub_addr,byte_order_buf,rd_len);

	for (byte_order_index = 0; byte_order_index < rd_len; byte_order_index ++)
	{
		*(source_buf + byte_order_index) = byte_order_swap_ulong(*(byte_order_buf + byte_order_index));
	}

	if (byte_order_buf)
	{
		kfree(byte_order_buf);
	}

	return byte_order_ret;
#else
	return regmap_bulk_read(cx2092x->regmap,sub_addr,rd_buf,rd_len);
#endif
}

static int resetpin_imp (void* gpio_dev, int set)
{
	//struct snd_soc_codec *codec = (struct snd_soc_codec *) gpio_dev;
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround

	if (gpio_is_valid(cx2092x->gpio_reset)) {
		gpio_set_value(cx2092x->gpio_reset, set?1:0);
	} else
		return -ENODEV;

	return 0;
}


void cx_mdelay (unsigned int intreval_ms)
{
	mdelay(intreval_ms);
}

#define REPLY_POLL_INTERVAL_MSEC     1
#define REPLY_POLL_TIMEOUT_MSEC   2000


static int cx2092x_sendcmd (struct snd_soc_codec *codec, struct cx_command *cmd)
{
	int ret;
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);
	int num_32b_words = cmd->num_32b_words;
	unsigned long time_out;
	unsigned int *i2c_data = (unsigned int *)cmd;
	int size = num_32b_words + 2;

	cx2092x = g_cx2092x; // workaround

	cmd->num_32b_words = (cmd->command_id&CMD_GET(0)) ? MAX_COMMAND_SIZE : num_32b_words;


	time_out = msecs_to_jiffies(REPLY_POLL_TIMEOUT_MSEC);
	/* avoid to access device during fw update*/
	mutex_lock(&codec->mutex);
	if (cx2092x->is_updating_fw) {
		mutex_unlock(&codec->mutex);
		return -EBUSY;
	}
	mutex_unlock(&codec->mutex);

	/* write words 1 to N-1 , to addresses 4 to 4+4*N-1*/
	ret = mem_write_imp(codec,0,4,(size-1),
		(u8*)&i2c_data[1]);

	if (ret < 0)
		goto LEAVE;

	// write word 0 to address 0
	ret = mem_write_imp(codec,0,0,(1),
		(u8*)&i2c_data[0]);

	if (ret < 0)
		goto LEAVE;

	time_out += jiffies;
	ret = -EBUSY;
	do {
		// only read the first word and check the reply bit
		mem_read_imp(codec, 0, 0, 1,
			(u8*)&i2c_data[0]);

		if (cmd->reply==1)
			break;
		cx_mdelay(REPLY_POLL_INTERVAL_MSEC);

	} while (!time_after(jiffies, time_out));

	if (cmd->reply==1)
	{
		if( cmd->num_32b_words >0)
		{
			mem_read_imp(codec, 0, 8, cmd->num_32b_words,
				(u8*)&i2c_data[2]);
		}
		return(cmd->num_32b_words);
	}

LEAVE:
	return ret;
}




static int cmd_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct cx_command);
	return 0;
}

static int cmd_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround
	codec = g_codec;

	memcpy( ucontrol->value.bytes.data, &cx2092x->cmd,
			sizeof(cx2092x->cmd));

	return cx2092x->cmd_res < 0? cx2092x->cmd_res: 0;
}

static int cmd_put(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround
	codec = g_codec;

	memcpy(&cx2092x->cmd, ucontrol->value.bytes.data,
			sizeof(cx2092x->cmd));
	cx2092x->cmd_res = cx2092x_sendcmd(codec,&cx2092x->cmd);
	if( cx2092x->cmd_res < 0 ) {
		dev_err(codec->dev, "Failed to send cmd, sendcmd = %d\n",cx2092x->cmd_res);
	}

	return cx2092x->cmd_res < 0? cx2092x->cmd_res: 0;
}


static int mode_info(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = 4;
	return 0;
}

static int mode_get(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cx_command cmd;
	int ret;

	codec = g_codec; // workaround

	cmd.command_id = 0x12f; /*CMD_GET(SOS_RESOURCE);*/
	cmd.reply = 0;
	cmd.app_module_id = CAPE_ID ( 'S', 'O', 'S', ' ');
	cmd.num_32b_words = 1;
	cmd.data[0] = CAPE_ID ( 'C', 'T', 'R', 'L');
	ret = cx2092x_sendcmd(codec,&cmd);
	if( ret <= 0 || ret > MAX_COMMAND_SIZE )
		dev_err(codec->dev, "Failed to get tv mode, sendcmd = %d\n",ret);
	else {
		ucontrol->value.bytes.data[0]= CHAR_FROM_CAPE_ID_A(cmd.data[0]);
		ucontrol->value.bytes.data[1] = CHAR_FROM_CAPE_ID_B(cmd.data[0]);
		ucontrol->value.bytes.data[2] = CHAR_FROM_CAPE_ID_C(cmd.data[0]);
		ucontrol->value.bytes.data[3] = CHAR_FROM_CAPE_ID_D(cmd.data[0]);
		dev_info(codec->dev, "Current tv mode = %c%c%c%c\n",
				ucontrol->value.bytes.data[0],
				ucontrol->value.bytes.data[1],
				ucontrol->value.bytes.data[2],
				ucontrol->value.bytes.data[3]);
		ret = 0;
	}

	return ret;
}

static int mode_put(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	struct cx_command cmd;
	int ret;

	codec = g_codec; // workaround

	cmd.command_id =  4;/*CMD_SET(CONTROL_APP_EXEC_FILE)*/
	cmd.reply = 0;
	cmd.app_module_id = CAPE_ID ( 'C', 'T', 'R', 'L');
	cmd.num_32b_words = 1;
	cmd.data[0] = CAPE_ID( ucontrol->value.bytes.data[0],
				ucontrol->value.bytes.data[1],
				ucontrol->value.bytes.data[2],
				ucontrol->value.bytes.data[3]);

	ret = cx2092x_sendcmd(codec,&cmd);
	if( ret <= 0 || ret > MAX_COMMAND_SIZE ) {
		dev_err(codec->dev, "Failed to set tv mode, sendcmd =%d\n",ret);
		ret = -EIO;
	} else
		ret = 0 ;
	return ret;
}

static void cx2092x_firmware_cont(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = (struct snd_soc_codec *)context;
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);
	int ret;
	u8 *buf;

	cx2092x = g_cx2092x; // workaround
	codec = g_codec;

	if (fw == NULL) {
		dev_err(cx2092x->dev, "Firmware is not available!\n");
		return;
	}

#define MAX_BURST_WRITE_LEN 0x10
#define MAX_BURST_READ_LEN 0x10

	regcache_cache_bypass(cx2092x->regmap,true);
	/*Set up the callback functions*/
	SetupI2cWriteMemCallback( codec, mem_write_imp, MAX_BURST_WRITE_LEN);
	SetupI2cReadMemCallback( codec, mem_read_imp, MAX_BURST_READ_LEN);
	SetupSetResetPin(codec, resetpin_imp);
	buf = (u8*) kmalloc(GetSizeOfBuffer(), GFP_KERNEL);

	if (!buf) {
		dev_err(cx2092x->dev,
			"kmalloc() failed\n");
		ret = -ENOMEM;
		goto LEAVE;
	}

	if (!fw->data || !fw->size) {
		dev_err(cx2092x->dev, "Invalid firmware data\n");
		ret = -EINVAL;
		goto LEAVE;
	}
#if 0
	ret = cx_download_fw(buf,NULL,0, fw->data,fw->size,0,SFS_UPDATE_AUTO);
#else
	// Jeter 20170109 The steps of Updating firmware of CX2092x:
	// 1. adb root
	// 2. adb remount
	// 3. adb push cx2092x.sfs /vendor/firmware/cnxt/cx2092x.sfs
	// 4. adb push iflash.bin /vendor/firmware/cnxt/iflash.bin
	// 5. adb shell
	// 6. adb > tinymix -D 2 2 1 (based on the card number) (for Allwinner)
	// 7. adb > tinymix -D 0 'Update Firmware' 1 (for DB2.x)
	if (iflash_fw->data && iflash_fw->size)
	{
		// Add the bootloader of CX2092x named "iflash.bin" as the parameter.
		mutex_lock(&codec->mutex);
		cx2092x->is_updating_fw = 1;
		mutex_unlock(&codec->mutex);
		ret = cx_download_fw(buf, iflash_fw->data, iflash_fw->size, fw->data,fw->size,0x41,SFS_UPDATE_AUTO);
	}
	else
	{
		// Origin
		mutex_lock(&codec->mutex);
		cx2092x->is_updating_fw = 1;
		mutex_unlock(&codec->mutex);
		ret = cx_download_fw(buf,NULL,0, fw->data,fw->size,0x41,SFS_UPDATE_AUTO);
	}
#endif

	if (ret < 0) {
		dev_err(cx2092x->dev,
			"Failed to update firmware, error code = %d\n",ret );
	} else {
		dev_info(cx2092x->dev,  "download firmware patch successfully.\n");
	}

LEAVE:
	if (buf)
		kfree(buf);
	mutex_lock(&codec->mutex);
	cx2092x->is_updating_fw = 0;
	mutex_unlock(&codec->mutex);
	release_firmware(fw);
#if 1
	release_firmware(iflash_fw);
#endif
	regcache_cache_bypass(cx2092x->regmap,false);
}

static int cx2092x_update_fw( struct snd_soc_codec *codec)
{
	int ret = 0;
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround

	mutex_lock(&codec->mutex);
	if (cx2092x->is_updating_fw) {
		mutex_unlock(&codec->mutex);
		ret =  -EBUSY;
		goto LEAVE;
	}
	mutex_unlock(&codec->mutex);
#if 1
	// Jeter 20170109 Request the bootloader of CX2092x
	ret = request_firmware(&iflash_fw, CONFIG_SND_SOC_CX2092X_BOOTLOADER_FILE, cx2092x->dev);
#endif
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		CONFIG_SND_SOC_CX2092X_FIRMWARE_FILE,
		cx2092x->dev, GFP_KERNEL,
		codec, cx2092x_firmware_cont);
	if (ret) {
		dev_err(cx2092x->dev, "Failed to load firmware %s\n",
				CONFIG_SND_SOC_CX2092X_FIRMWARE_FILE);
		mutex_lock(&codec->mutex);
		cx2092x->is_updating_fw = 0;
		mutex_unlock(&codec->mutex);
		goto LEAVE;
	}

LEAVE:
	return ret;
}

static int fw_get(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround
	codec = g_codec;

	mutex_lock(&codec->mutex);
	ucontrol->value.enumerated.item[0] =
			cx2092x->is_updating_fw;
	mutex_unlock(&codec->mutex);
         return 0;
}

static int fw_put(struct snd_kcontrol *kcontrol,
                                 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	codec = g_codec; // workaround

	if (ucontrol->value.enumerated.item[0])
	{
		return cx2092x_update_fw(codec);
	}
	else
	{
		return 0;
	}
}


static int ver_info(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 4;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xffffffff;
	/*Int32.MaxValue*/
	return 0;
}

static int ver_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cx_command cmd;
	int ret;

	codec = g_codec; // workaround

	cmd.num_32b_words = 0;
	cmd.command_id =  0x0103;
	cmd.reply = 0;
	cmd.app_module_id = 0xb32d2300;

	ret = cx2092x_sendcmd(codec,&cmd);
	if( ret <= 0 )
		dev_err(codec->dev, "Failed to get firmware version, ret =%d\n",ret);
	else {
		ucontrol->value.integer.value[0] = cmd.data[0];
		ucontrol->value.integer.value[1] = cmd.data[1];
		ucontrol->value.integer.value[2] = cmd.data[2];
		ucontrol->value.integer.value[3] = cmd.data[3];
		ret = 0;
	}
	return ret;

}

static int ver_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	/*this function should never be called.*/
	return 0;
}


static const struct snd_kcontrol_new cx2092x_snd_controls[] = {
	CX2092X_CONTROL("SendCmd", cmd_info, cmd_get, cmd_put,
		SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_WRITE|
		SNDRV_CTL_ELEM_ACCESS_VOLATILE),
	CX2092X_CONTROL("Mode", mode_info, mode_get, mode_put,
		SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_WRITE|
		SNDRV_CTL_ELEM_ACCESS_VOLATILE),
	SOC_SINGLE_BOOL_EXT("Update Firmware", 0, fw_get, fw_put),
	CX2092X_CONTROL("Firmware Version", ver_info, ver_get, ver_put,
		SNDRV_CTL_ELEM_ACCESS_READ ),
};



static int cx2092x_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	/*FW controled*/
	return 0;
}

static int cx2092x_mute(struct snd_soc_dai *dai, int mute)
{
	/*FW controled*/
	return 0;
}

static int cx2092x_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	/*FW controled*/
	return 0;
}

static const struct snd_soc_dapm_widget cx2092x_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("MIC AIF", "Capture", 0,
			     SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("ANA IN1"),
	SND_SOC_DAPM_INPUT("ANA IN2"),
};

static const struct snd_soc_dapm_route cx2092x_intercon[] = {
	{"DMIC AIF", NULL, "ANA IN1"},
	{"DMIC AIF", NULL, "ANA IN2"},
};


struct snd_soc_dai_ops cx2092x_dai_ops;

struct snd_soc_dai_driver soc_codec_cx2092x_dai[NUM_OF_DAI];

EXPORT_SYMBOL_GPL(soc_codec_cx2092x_dai);

static int cx2092x_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	/*FW controled*/
	codec->dapm.bias_level = level;
	return 0;
}


static int cx2092x_probe(struct snd_soc_codec *codec)
{
	struct cx2092x_priv *cx2092x = get_cx2092x_priv(codec);
//	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;

#if 1
	if (gpio_is_valid(cx2092x->gpio_reset)) {
		ret = gpio_request(cx2092x->gpio_reset, "cx2092x reset");
		if (ret != 0) {
			dev_err(codec->dev, "gpio_request failed, gpio = %d\n",
				cx2092x->gpio_reset);
			goto LEAVE;
		}
		gpio_direction_output(cx2092x->gpio_reset, 1);
		mdelay(100);
		gpio_direction_output(cx2092x->gpio_reset, 0);
		mdelay(200);
		gpio_direction_output(cx2092x->gpio_reset, 1);

	}
#endif
	mutex_init(&codec->mutex);

	codec->control_data = cx2092x->regmap;

	//ret = snd_soc_codec_set_cache_io(codec, 16, 32, SND_SOC_REGMAP);



	dev_info(codec->dev, "codec driver version: %02x,%02x,%02x,%02x\n",
		(u8)((CX2092X_DRIVER_VERSION)>>24),
		(u8)((CX2092X_DRIVER_VERSION)>>16),
		(u8)((CX2092X_DRIVER_VERSION)>>8),
		(u8)((CX2092X_DRIVER_VERSION)));

	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache i/o: %d\n", ret);
		return ret;
	}

	snd_soc_add_codec_controls(codec, cx2092x_snd_controls,
			ARRAY_SIZE(cx2092x_snd_controls));

#if 0 // do below in  cx2092x_register_codec_driver()
	snd_soc_dapm_new_controls(dapm, cx2092x_dapm_widgets,
				  ARRAY_SIZE(cx2092x_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, cx2092x_intercon,
				ARRAY_SIZE(cx2092x_intercon));
	snd_soc_dapm_new_widgets(dapm);
#endif
	g_codec = codec;
LEAVE:
	return ret;
}

static int cx2092x_remove(struct snd_soc_codec *codec)
{
	struct cx2092x_priv *cx2092x; // = get_cx2092x_priv(codec);

	cx2092x = g_cx2092x; // workaround

	cx2092x_set_bias_level(g_codec, SND_SOC_BIAS_OFF);
	if (gpio_is_valid(cx2092x->gpio_reset)) {
		gpio_set_value(cx2092x->gpio_reset, 0);
		gpio_free(cx2092x->gpio_reset);
	}
    return 0;
}

#ifdef CONFIG_PM
static int cx2092x_suspend(struct snd_soc_codec *codec)
{
    cx2092x_set_bias_level(g_codec, SND_SOC_BIAS_OFF);
    return 0;
}

static int cx2092x_resume(struct snd_soc_codec *codec)
{
    cx2092x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
    return 0;
}
#else
#define cx2092x_suspend NULL
#define cx2092x_resume NULL
#endif

struct snd_soc_codec_driver soc_codec_dev_cx2092x;
static  struct regmap_config cx2092x_regmap;


static bool cx20901_volatile_register(struct device *dev, unsigned int reg)
{
	return 1; /*all register are volatile*/
}

static int cx2092x_register_codec_driver(struct cx2092x_priv * cx2092x)
{
	int ret;

	/*this is nesscarry for non gcc compilier*/
	memset(&cx2092x_dai_ops, 0, sizeof(cx2092x_dai_ops));

	cx2092x_dai_ops.set_sysclk = cx2092x_set_dai_sysclk;
	cx2092x_dai_ops.digital_mute = cx2092x_mute;
	cx2092x_dai_ops.hw_params = cx2092x_hw_params;

	/*this is nesscarry for non gcc compilier*/
	memset(soc_codec_cx2092x_dai, 0, sizeof(soc_codec_cx2092x_dai));

	soc_codec_cx2092x_dai[0].name = "qca-i2s-conexant-dai";
	soc_codec_cx2092x_dai[0].ops = &cx2092x_dai_ops;
	soc_codec_cx2092x_dai[0].id = I2S;
	soc_codec_cx2092x_dai[0].capture.stream_name= "Capture" ;
	soc_codec_cx2092x_dai[0].capture.formats =
			CX2092X_FORMATS;
	soc_codec_cx2092x_dai[0].capture.rates =
			SNDRV_PCM_RATE_8000_96000;
	soc_codec_cx2092x_dai[0].capture.channels_min = 1;
	soc_codec_cx2092x_dai[0].capture.channels_max = 2;

	/*this is nesscarry for non gcc compilier*/
	memset(&soc_codec_dev_cx2092x, 0, sizeof(soc_codec_dev_cx2092x));
	soc_codec_dev_cx2092x.probe = cx2092x_probe;
	soc_codec_dev_cx2092x.remove = cx2092x_remove;
	soc_codec_dev_cx2092x.suspend = cx2092x_suspend;
	soc_codec_dev_cx2092x.resume = cx2092x_resume;
	soc_codec_dev_cx2092x.set_bias_level = cx2092x_set_bias_level;
	//soc_codec_dev_cx2092x.dapm_widgets = cx2092x_dapm_widgets;
	//soc_codec_dev_cx2092x.num_dapm_widgets = ARRAY_SIZE(cx2092x_dapm_widgets);
	//soc_codec_dev_cx2092x.dapm_routes = cx2092x_intercon;
	//soc_codec_dev_cx2092x.num_dapm_routes = ARRAY_SIZE(cx2092x_intercon);

	ret = snd_soc_register_codec(cx2092x->dev,
			&soc_codec_dev_cx2092x, soc_codec_cx2092x_dai,
			NUM_OF_DAI);
	if (ret < 0)
		dev_err(cx2092x->dev,
			"Failed to register codec: %d\n", ret);
	else
		dev_dbg(cx2092x->dev,
			"%s: Register codec.\n", __func__);
	return ret;
}
#if defined(CONFIG_SPI_MASTER)
static int cx2092x_spi_probe(struct spi_device *spi)
{
	struct cx2092x_platform_data *pdata =
		(struct cx2092x_platform_data *) spi->dev.platform_data;
	struct cx2092x_priv *cx2092x;
	int ret;

	cx2092x = (struct cx2092x_priv *)devm_kzalloc(&spi->dev,
			sizeof(struct cx2092x_priv), GFP_KERNEL);
	if (cx2092x == NULL) {
		dev_err(&spi->dev, "Out of memory!\n");
		return -ENOMEM;
	}
	spi_set_drvdata(spi, cx2092x);

	cx2092x_regmap.reg_bits = 16;
	cx2092x_regmap.val_bits = 32;
	cx2092x_regmap.reg_stride = 4;
	cx2092x_regmap.max_register = CX2092X_REG_MAX;
	cx2092x_regmap.write_flag_mask = CX2092X_SPI_WRITE_FLAG;
	cx2092x_regmap.cache_type = REGCACHE_RBTREE;
	cx2092x_regmap.volatile_reg = cx20901_volatile_register;
	cx2092x_regmap.val_format_endian = REGMAP_ENDIAN_NATIVE;

	cx2092x->regmap = devm_regmap_init_spi(spi, &cx2092x_regmap);
	if (IS_ERR(&cx2092x->regmap)) {
		ret = PTR_ERR(cx2092x->regmap);
		dev_err(&spi->dev, "Failed to init regmap: %d\n", ret);
		return ret ;
	}

	cx2092x->dev = &spi->dev;

	if (pdata)
		cx2092x->gpio_reset = pdata->gpio_reset;
	else
		cx2092x->gpio_reset = -1;


	return cx2092x_register_codec_driver(cx2092x);
}

static int cx2092x_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	return 0;
}
static struct spi_driver cx2092x_spi_driver;
#endif
#if 1
static ssize_t show_send_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cx2092x_priv *cx2092x;
	char data[60];

	cx2092x = g_cx2092x;
	memcpy(data, &cx2092x->cmd, sizeof(data));
	return sprintf(buf, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
				        "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
				        "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
				        "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
				        "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
				        "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
				        "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
				        "0x%02x 0x%02x 0x%02x 0x%02x\r\n",
				data[0],  data[1],  data[2],  data[3],  data[4],  data[5],  data[6],  data[7],
				data[8],  data[9],  data[10], data[11], data[12], data[13], data[14], data[15],
				data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23],
				data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31],
				data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39],
				data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47],
				data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55],
				data[56], data[57], data[59], data[59]);
}

static ssize_t store_send_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct snd_soc_codec *codec;
	struct cx2092x_priv *cx2092x;
	char data[60];

	memset(data, 0x00, sizeof(data));

	cx2092x = g_cx2092x;

	sscanf(buf, "0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
				"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
				"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
				"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
				"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
				"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
				"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
				"0x%x 0x%x 0x%x 0x%x",
				(unsigned int *)&data[0], (unsigned int *)&data[1], (unsigned int *)&data[2], (unsigned int *)&data[3],
				(unsigned int *)&data[4], (unsigned int *)&data[5], (unsigned int *)&data[6], (unsigned int *)&data[7],
				(unsigned int *)&data[8], (unsigned int *)&data[9], (unsigned int *)&data[10], (unsigned int *)&data[11],
				(unsigned int *)&data[12], (unsigned int *)&data[13], (unsigned int *)&data[14], (unsigned int *)&data[15],
				(unsigned int *)&data[16], (unsigned int *)&data[17], (unsigned int *)&data[18], (unsigned int *)&data[19],
				(unsigned int *)&data[20], (unsigned int *)&data[21], (unsigned int *)&data[22], (unsigned int *)&data[23],
				(unsigned int *)&data[24], (unsigned int *)&data[25], (unsigned int *)&data[26], (unsigned int *)&data[27],
				(unsigned int *)&data[28], (unsigned int *)&data[29], (unsigned int *)&data[30], (unsigned int *)&data[31],
				(unsigned int *)&data[32], (unsigned int *)&data[33], (unsigned int *)&data[34], (unsigned int *)&data[35],
				(unsigned int *)&data[36], (unsigned int *)&data[37], (unsigned int *)&data[38], (unsigned int *)&data[39],
				(unsigned int *)&data[40], (unsigned int *)&data[41], (unsigned int *)&data[42], (unsigned int *)&data[43],
				(unsigned int *)&data[44], (unsigned int *)&data[45], (unsigned int *)&data[46], (unsigned int *)&data[47],
				(unsigned int *)&data[48], (unsigned int *)&data[49], (unsigned int *)&data[50], (unsigned int *)&data[51],
				(unsigned int *)&data[52], (unsigned int *)&data[53], (unsigned int *)&data[54], (unsigned int *)&data[55],
				(unsigned int *)&data[56], (unsigned int *)&data[57], (unsigned int *)&data[58], (unsigned int *)&data[59]);

	memcpy(&cx2092x->cmd, data, sizeof(data));

	codec = g_codec;

	cx2092x->cmd_res = cx2092x_sendcmd(codec,&cx2092x->cmd);
	if( cx2092x->cmd_res < 0 ) {
		dev_err(codec->dev, "Failed to send cmd, sendcmd = %d\n",cx2092x->cmd_res);
	}

	return count;
}

static DEVICE_ATTR(SendCmd , S_IWUSR | S_IRUGO, show_send_cmd, store_send_cmd);

static ssize_t show_DoA(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_codec *codec;
	struct cx2092x_priv *cx2092x;
	struct cx_command cmd;
	unsigned int mag, max_mag = 0;
	unsigned int angle_of_arrival;
	int index;
	int ret;
	unsigned int temp1, temp2;
	unsigned int data0, data1;

	cx2092x = g_cx2092x;
	codec = g_codec;

	cmd.num_32b_words = 0x02;
	cmd.command_id =  0x0140;
	cmd.reply = 0;
	cmd.app_module_id = 0xD3086339;

	for (index=0; index<4; index++)
	{
		cmd.data[0] = index;

		ret = cx2092x_sendcmd(codec,&cmd);
		if( ret < 0 ) {
			dev_err(codec->dev, "Failed to send cmd, sendcmd = %d\n",ret);
			return sprintf(buf, "-1\r\n");
		}

		// Use bit shift to instead of division
		if (cmd.data[0] & 0x80000000) // Check negative value
		{
			temp1 = cmd.data[0] & 0x7FFFFFFF;
			temp2 = temp1 >> 23;
			temp1 = temp2 | 0x00000100;
		}
		else // positive value
		{
			temp1 = cmd.data[0] & 0x7FFFFFFF;
			temp2 = temp1 >> 23;
			temp1 = temp2;
			data0 = temp2;
		}

		if (cmd.data[1] & 0x80000000)
		{
			data1 = 0;
		}
		else
		{
			data1 = cmd.data[1];
		}

		mag = data1;

		max_mag = (max_mag < mag) ? mag : max_mag;
		if (max_mag == mag) {
			angle_of_arrival = 2 * data0;
		}
	}

	return sprintf(buf, "%d\r\n", angle_of_arrival);
}

static DEVICE_ATTR(DoA , S_IRUGO, show_DoA, NULL);
#endif

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int cx2092x_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	// Not used
	//struct cx2092x_platform_data *pdata =
	//	(struct cx2092x_platform_data *) i2c->dev.platform_data;
	struct cx2092x_priv *cx2092x;
	int    ret = 0;
	struct dev_pin_info *pins;
	struct pinctrl_state *pin_state;

	cx2092x = (struct cx2092x_priv *)devm_kzalloc(
			&i2c->dev,sizeof(struct cx2092x_priv), GFP_KERNEL);
	if (cx2092x == NULL) {
		dev_err(&i2c->dev, "Out of memory!\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(i2c, cx2092x);

	cx2092x_regmap.reg_bits = 16;
	cx2092x_regmap.val_bits = 32;
	cx2092x_regmap.reg_stride = 4;

	cx2092x_regmap.max_register = CX2092X_REG_MAX;
	cx2092x_regmap.cache_type = REGCACHE_RBTREE;
	cx2092x_regmap.volatile_reg = cx20901_volatile_register;
	cx2092x_regmap.val_format_endian = REGMAP_ENDIAN_NATIVE;

	cx2092x->regmap = devm_regmap_init_i2c(i2c, &cx2092x_regmap);
	if (IS_ERR(&cx2092x->regmap)) {
		ret = PTR_ERR(cx2092x->regmap);
		dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
		return ret ;
	}

	cx2092x->dev = &i2c->dev;
	pins = cx2092x->dev->pins;

	pin_state = pinctrl_lookup_state(pins->p, "default");
	if (IS_ERR(pin_state)) {
		dev_err(&i2c->dev, "cx2092x pinctrl state not available\n");
		return PTR_ERR(pin_state);
	}

	pinctrl_select_state(pins->p, pin_state);

	mdelay(50);
#if 0
	if (pdata)
		cx2092x->gpio_reset = pdata->gpio_reset;
	else
		cx2092x->gpio_reset = -1;
#else
	cx2092x->gpio_reset = of_get_named_gpio(i2c->dev.of_node, "reset-gpio", 0);
	pr_err("gpio-reset=%d\r\n", cx2092x->gpio_reset);
#endif
	/* TODO : add support for device tree here */

	g_cx2092x = cx2092x;

	device_create_file(&i2c->dev, &dev_attr_SendCmd);
	device_create_file(&i2c->dev, &dev_attr_DoA);

	return cx2092x_register_codec_driver(cx2092x);
}

static int cx2092x_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id cx2092x_i2c_id[] =
{
	{ "cx2092x", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, cx2092x_i2c_id);
#endif

static struct i2c_driver cx2092x_i2c_driver;

int cx2092x_modinit(void)
{
	int ret = 0;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	memset(&cx2092x_i2c_driver,0,sizeof(struct i2c_driver));
	cx2092x_i2c_driver.probe = cx2092x_i2c_probe;
	cx2092x_i2c_driver.remove = cx2092x_i2c_remove;
	cx2092x_i2c_driver.id_table = cx2092x_i2c_id;
	cx2092x_i2c_driver.driver.name = "cx2092x";
	cx2092x_i2c_driver.driver.owner = THIS_MODULE;

	ret = i2c_add_driver(&cx2092x_i2c_driver);
	if (ret != 0)
		printk(KERN_ERR "Failed to register cx2092x I2C driver: %d\n",
			ret);
#endif

#if defined(CONFIG_SPI_MASTER)
	cx2092x_spi_driver.driver.name = "cx2092x";
	cx2092x_spi_driver.driver.owner = THIS_MODULE;
	cx2092x_spi_driver.probe = cx2092x_spi_probe,
	cx2092x_spi_driver.remove = cx2092x_spi_remove;

	ret = spi_register_driver(&cx2092x_spi_driver);
	if (ret)
		printk(KERN_ERR "Failed to register cx2092x SPI driver %d\n",
			ret);
#endif
	return ret;
}
module_init(cx2092x_modinit);


static void __exit cx2092x_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&cx2092x_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	spi_unregister_driver(&cx2092x_spi_driver);
#endif
}
module_exit(cx2092x_exit);

EXPORT_SYMBOL_GPL(soc_codec_dev_cx2092x);
MODULE_DESCRIPTION("ASoC cx2092x Codec Driver");
MODULE_AUTHOR("Simon Ho <simon.ho@conexant.com>");
MODULE_LICENSE("GPL");




