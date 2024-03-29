/*
 * Driver for the MCU (I2C bus)
 *
 * Copyright (C) 2018 Wistron Corporation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/firmware.h>
#include <linux/string.h>

#define MCU_APROM_ADDR           0x15
#define MCU_LDROM_ADDR           0x36

#define MCU_REG_LED              0x00
#define MCU_REG_LED_SPEED        0x01
#define MCU_REG_FAN              0x02
#define MCU_REG_READ_FAN         0x03
#define MCU_REG_READ_TEMPER1     0x04
#define MCU_REG_READ_TEMPER2     0x05
#define MCU_REG_PWM_LED          0x06
#define MCU_REG_READ_FW_VER      0x09
#define MCU_REG_LED_F_RED        0x10
#define MCU_REG_LED_F_GREEN      0x11
#define MCU_REG_LED_F_BLUE       0x12
#define MCU_REG_LED_RED          0x0A
#define MCU_REG_LED_GREEN        0x0B
#define MCU_REG_LED_BLUE         0x0C
#define MCU_REG_PWM_LED_DUTY     0x0D
#define MCU_REG_BOOTLOADER_MODE  0x0F
#define MCU_DATA_BOOTLOADER_MODE 0x1E
#define CMD_UPDATE_APROM         0xA0
#define CMD_READ_CONFIG          0xA2
#define CMD_SYNC_PACKNO          0xA4
#define CMD_GET_FWVER            0xA6
#define CMD_GET_DEVICEID         0xB1

#define MCU_BOOT_GPIO 38
#define MCU_RESET_GPIO 5

#define PACKET_SIZE 64

//#define FW_DEBUG

struct mcu_bus_data {
	struct i2c_client *client;
	unsigned int addr;
	struct miscdevice	mcu_led_fan_device;
	u8	reset_gpio;
	u8	boot_gpio;
};
struct mcu_bus_data *data;

#define BOOT_UPDATE_FIRMWARE_NAME "mcu_025.bin"
const struct firmware *fw_entry = NULL;
unsigned int g_packno = 1;
unsigned short gcksum;
volatile unsigned int AP_file_checksum;
uint8_t sendbuf[PACKET_SIZE];
uint8_t rcvbuf[PACKET_SIZE];
uint8_t buf[512];
uint32_t temp_count;



static int MCU_I2C_READ(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	break;
		retries++;
	}
	return ret;
}

static int MCU_I2C_READ_DATA(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = len;
	msgs[0].buf   = &buf[0];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 1);
		if(ret == 1)	break;
		retries++;
	}
	return ret;
}


static int  MCU_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = data;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return ret;
}

void WordsCpy(void *dest, void *src, int32_t size)
{
	uint8_t *pu8Src, *pu8Dest;
	int32_t i;

	pu8Dest = (uint8_t *)dest;
	pu8Src = (uint8_t *)src;

	for (i = 0; i < size; i++)
		pu8Dest[i] = pu8Src[i];
}

uint16_t Checksum(unsigned char *buf, int len)
{
	int i;
	uint16_t c;

	for (c = 0, i = 0; i < len; i++) {
		c += buf[i];
	}
	return c;
}

unsigned char SendData(void)
{


	gcksum = Checksum(sendbuf, PACKET_SIZE);

#ifdef FW_DEBUG
	for ( temp_count = 0; temp_count < 64; temp_count++ )
	{
		dev_info(&data->client->dev,"send data=%x\n", sendbuf[temp_count]);
	}
#endif

	MCU_I2C_WRITE(data->client, MCU_LDROM_ADDR, sendbuf,PACKET_SIZE);

	return 0;
}

static int RcvData(void)
{
	int Result=0;
	unsigned short lcksum,rcv_packno;
	uint8_t *pBuf;

	MCU_I2C_READ_DATA(data->client, MCU_LDROM_ADDR, rcvbuf,PACKET_SIZE);

#ifdef FW_DEBUG
	for ( temp_count = 0; temp_count < 64; temp_count++ )
	{
		dev_info(&data->client->dev,"receive data=%x\n", rcvbuf[temp_count]);
	}
#endif

	pBuf = rcvbuf;
	WordsCpy(&lcksum, pBuf, 2);
	WordsCpy(&rcv_packno, pBuf+4, 4);

#ifdef FW_DEBUG
	dev_info(&data->client->dev,"gcksum=0x%x lcksum=0x%x\n", gcksum, lcksum);
	dev_info(&data->client->dev,"g_packno=%d rcv=%d\n", g_packno, rcv_packno);
#endif

	if (rcv_packno != g_packno)
	{
		dev_info(&data->client->dev,"g_packno=%d rcv=%d\n", g_packno, rcv_packno);
		Result = -1;
	}
	else
	{
		if (lcksum != gcksum)
		{
			dev_info(&data->client->dev,"gcksum=%x lcksum=%x\n", gcksum, lcksum);
			Result = -1;
		}
		g_packno++;

	}
	return Result;
}

unsigned char CmdSyncPackno(void)
{
	int Result;
	unsigned long cmdData;

	//sync send&recv packno
	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_SYNC_PACKNO;
	WordsCpy(sendbuf + 0, &cmdData, 4);
	WordsCpy(sendbuf + 4, &g_packno, 4);
	WordsCpy(sendbuf + 8, &g_packno, 4);
	g_packno++;

	Result = SendData();
	if (Result)
		return Result;
	msleep(100);

	Result = RcvData();

	return Result;
}

unsigned char CmdGetDeviceID(unsigned int *devid)
{
	int Result;
	unsigned long cmdData;
	unsigned int ldevid;

	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_GET_DEVICEID;
	WordsCpy(sendbuf + 0, &cmdData, 4);
	WordsCpy(sendbuf + 4, &g_packno, 4);
	g_packno++;

	Result = SendData();
	if (Result)
		return Result;
	msleep(100);

	Result = RcvData();
	if (Result)
		return Result;

	WordsCpy(&ldevid, rcvbuf + 8, 4);
	*devid = ldevid;

	return Result;
}

unsigned char CmdGetConfig(unsigned int *config)
{
	int Result;
	unsigned long cmdData;
	unsigned int lconfig[2];

	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_READ_CONFIG;
	WordsCpy(sendbuf + 0, &cmdData, 4);
	WordsCpy(sendbuf + 4, &g_packno, 4);
	g_packno++;

	Result = SendData();
	if (Result)
		return Result;
	msleep(100);

	Result = RcvData();
	if (Result)
		return Result;

	WordsCpy(&lconfig[0], rcvbuf + 8, 4);
	WordsCpy(&lconfig[1], rcvbuf + 12, 4);
	config[0] = lconfig[0];
	config[1] = lconfig[1];


	return Result;
}

int CmdFWVersion(unsigned int *fwver)
{
	int Result;
	unsigned long cmdData;
	unsigned int lfwver;

	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_GET_FWVER;
	WordsCpy(sendbuf + 0, &cmdData, 4);
	WordsCpy(sendbuf + 4, &g_packno, 4);
	g_packno++;

	Result = SendData();
	if (Result)
		return Result;
	msleep(100);

	Result = RcvData();
	if (Result)
		return Result;

	WordsCpy(&lfwver, rcvbuf + 8, 4);
	*fwver = lfwver;

	return Result;
}

unsigned int Get_File_CheckSum(void)
{
	unsigned int binfile_checksum = 0;
	unsigned int temp_ct;

	for(temp_ct = 0; temp_ct < fw_entry->size;temp_ct++)
	{
		binfile_checksum+= fw_entry->data[temp_ct];
	}

	return binfile_checksum;
}

static int CmdUpdateAprom(void)
{
	int Result;
	unsigned int devid, config[2], fwver, i, j,k, ct, l;
	unsigned long cmdData, startaddr;
	unsigned short get_cksum;
	g_packno = 1;

	gpio_direction_output(data->boot_gpio,1);
	msleep(100);

	sendbuf[0]=MCU_REG_BOOTLOADER_MODE;
	sendbuf[1]=MCU_DATA_BOOTLOADER_MODE;
	Result = MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, sendbuf, 2);
	if(Result < 0)
	{
		dev_info(&data->client->dev,"APROM is crashed, reset MCU to LDROM\n");
		gpio_direction_output(data->reset_gpio,1);
		msleep(10);
		gpio_direction_output(data->reset_gpio,0);
		msleep(10);
		gpio_direction_output(data->reset_gpio,1);
	}
	msleep(200);
	Result = CmdSyncPackno();
	if (Result)
	{
		dev_info(&data->client->dev,"Send Sync Packno cmd fail\n");
		goto out;
	}

	CmdGetDeviceID(&devid);
	dev_info(&data->client->dev,"DeviceID: 0x%x\n", devid);

	CmdGetConfig(config);
	dev_info(&data->client->dev,"config0: 0x%x\n", config[0]);
	dev_info(&data->client->dev,"config1: 0x%x\n", config[1]);

	CmdFWVersion(&fwver);
	dev_info(&data->client->dev,"FW version: 0x%x\n", fwver & 0xff);

	memset(sendbuf, 0, PACKET_SIZE);
	cmdData = CMD_UPDATE_APROM;
	WordsCpy(sendbuf + 0, &cmdData, 4);
	WordsCpy(sendbuf + 4, &g_packno, 4);
	g_packno++;
	startaddr = 0;
	WordsCpy(sendbuf + 8, &startaddr, 4);
	WordsCpy(sendbuf + 12, (void *)&fw_entry->size, 4);
	ct=48;
	for(i=0;i<ct;i++)
	{
		buf[i] = fw_entry->data[i];
	}

	WordsCpy(sendbuf + 16, buf, 48);

	Result = SendData();
	if (Result)
		goto out;

	for (i = 0; i < (fw_entry->size/512 + 1); i++)
	msleep(200);

	Result = RcvData();
	if (Result)
		goto out;

	for (i = 48; i < fw_entry->size; i = i + 56)
	{
		if (((i - 48) % 448) == 0)
		{
			if((fw_entry->size - i) < 448)
			{
				l=fw_entry->size - i;
			}
			else
			{
				l=448;
			}

			for(k=ct;k<ct+l;k++)
			{
				buf[k-ct] = fw_entry->data[k];
			}
			ct+=448;
		}
#ifdef FW_DEBUG
		dev_info(&data->client->dev, "i=%d\n", i);
		dev_info(&data->client->dev, "ct=%d\n", ct);
#endif
		dev_info(&data->client->dev, "Programm: %d %%", (i*100) / fw_entry->size);

		for (j = 0; j < 64; j++)
		{
			sendbuf[j] = 0;
		}

		WordsCpy(sendbuf + 4, &g_packno, 4);
		g_packno++;
		if ((fw_entry->size - i) > 56)
		{
			WordsCpy(sendbuf + 8, &buf[((i - 48) % 448)], 56);

			Result = SendData();
			if (Result)
				goto out;

			msleep(100);

			Result = RcvData();
			if (Result)
				goto out;
		}
		else
		{
			WordsCpy(sendbuf + 8, &buf[((i - 48) % 448)], fw_entry->size - i);

			Result = SendData();
			if (Result)
				goto out;

			msleep(100);

			Result = RcvData();
			if (Result)
				goto out;

			WordsCpy(&get_cksum, rcvbuf + 8, 2);
			dev_info(&data->client->dev, "get_cksum=%x \n", get_cksum);

			AP_file_checksum = Get_File_CheckSum();
			dev_info(&data->client->dev, "AP_file_checksum=%x \n", AP_file_checksum);
			if ((AP_file_checksum & 0xffff) != get_cksum)
			{
				dev_info(&data->client->dev, "Check sum is incorrect!\n");
				Result = -1;
				goto out;
			}
		}
	}

out:
	if (Result)
	{
		dev_info(&data->client->dev, "Programmer Failed!!\n");
	} else {
		dev_info(&data->client->dev, "Programm: 100.0 %%");
		dev_info(&data->client->dev, "Firmware update Success!\n");
	}
	dev_info(&data->client->dev, "Reset MCU to APROM....\n");
	gpio_direction_output(data->boot_gpio,0);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);
	msleep(10);
	gpio_direction_output(data->reset_gpio,0);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);
	msleep(100);

	return Result;
}

void update_firmware_release(void)
{
	if(fw_entry)
	{
		release_firmware(fw_entry);
	}
	fw_entry=NULL;
}

int update_firmware_request(char *filename)
{
	int ret=0;

	if(NULL==filename)
	{
		return -1;
	}

	dev_info(&data->client->dev, "%s : filename=%s\n", __func__, filename);

	ret=request_firmware(&fw_entry, filename, &data->client->dev);
	if(ret)
	{
		dev_err(&data->client->dev, "#######request firmware failed\n");
		return  ret;
	}

	dev_info(&data->client->dev, "%s : firmware bin size=%d\n", __func__, fw_entry->size);

	ret = CmdUpdateAprom();
	if(ret)
	{
		dev_err(&data->client->dev, "#######request firmware failed\n");
		update_firmware_release();
		return  ret;
	}

	update_firmware_release();

	return 0;
}

/*temperature 1*/
static ssize_t mcu_show_temper1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_READ_TEMPER1;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);
	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

/*temperature2*/
static ssize_t mcu_show_temper2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_READ_TEMPER2;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);
	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

/*fan mode*/
static ssize_t mcu_show_fan(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_READ_FAN;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);
	return sprintf(buf, "%d\n", I2C_BUF[1]);

}

/*APROM firmware version*/
static ssize_t mcu_show_fw_ver(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_READ_FW_VER;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);
	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

/*LDROM firmware version*/
static ssize_t mcu_show_bootloader_fw_ver(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int fwver;

	gpio_direction_output(data->boot_gpio,1);
	msleep(100);
	gpio_direction_output(data->reset_gpio,1);
	msleep(10);
	gpio_direction_output(data->reset_gpio,0);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);

	msleep(200);
	CmdSyncPackno();

	CmdFWVersion(&fwver);
	fwver = fwver & 0xff;

	gpio_direction_output(data->boot_gpio,0);
	msleep(100);
	gpio_direction_output(data->reset_gpio,1);
	msleep(10);
	gpio_direction_output(data->reset_gpio,0);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);

	return sprintf(buf, "%d\n", fwver);
}

/*fan speed*/
static ssize_t mcu_show_fan_speed(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_FAN;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);
	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

static ssize_t mcu_store_fan_speed(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_FAN;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led*/
static ssize_t mcu_show_led(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);
	return sprintf(buf, "0x%x\n", I2C_BUF[1]);
}


static ssize_t mcu_store_led(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*pwm led*/
static ssize_t mcu_show_pwm_led(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_PWM_LED;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}


static ssize_t mcu_store_pwm_led(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_PWM_LED;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*pwm led duty cycle*/
static ssize_t mcu_show_pwm_led_duty(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_PWM_LED_DUTY;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}


static ssize_t mcu_store_pwm_led_duty(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_PWM_LED_DUTY;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led blinking speed*/
static ssize_t mcu_show_led_speed(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED_SPEED;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}


static ssize_t mcu_store_led_speed(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED_SPEED;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led red brightness*/
static ssize_t mcu_show_led_red(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED_RED;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

static ssize_t mcu_store_led_red(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED_RED;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led green brightness*/
static ssize_t mcu_show_led_green(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED_GREEN;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}


static ssize_t mcu_store_led_green(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED_GREEN;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led blue brightness*/
static ssize_t mcu_show_led_blue(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED_BLUE;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

static ssize_t mcu_store_led_blue(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED_BLUE;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led foreground red brightness*/
static ssize_t mcu_show_led_f_red(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED_F_RED;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

static ssize_t mcu_store_led_f_red(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED_F_RED;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led foreground green brightness*/
static ssize_t mcu_show_led_f_green(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED_F_GREEN;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}


static ssize_t mcu_store_led_f_green(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED_F_GREEN;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}

/*led foreground blue brightness*/
static ssize_t mcu_show_led_f_blue(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t I2C_BUF[16] = {0};

	I2C_BUF[0] = MCU_REG_LED_F_BLUE;
	MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return sprintf(buf, "%d\n", I2C_BUF[1]);
}

static ssize_t mcu_store_led_f_blue(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;
	uint8_t I2C_BUF[2]={0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	I2C_BUF[0]=MCU_REG_LED_F_BLUE;
	I2C_BUF[1]=val;
	MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

	return count;
}


/*Enter Bootloader mode*/
static ssize_t mcu_store_bootloader_mode(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	unsigned long val;

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	gpio_direction_output(data->boot_gpio,1);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);
	msleep(10);
	gpio_direction_output(data->reset_gpio,0);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);

	return count;
}

/*Firmware update*/
static ssize_t mcu_store_fw_update(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret_val;
	int ver;
	unsigned long val;
	uint8_t I2C_BUF[16] = {0};

	ret_val = kstrtoul(buf, 0, &val);
	if (ret_val)
		return ret_val;

	if (val == 1){
		char firmware_name[256]="";
		sprintf(firmware_name, BOOT_UPDATE_FIRMWARE_NAME);

		I2C_BUF[0] = MCU_REG_READ_FW_VER;
		MCU_I2C_READ(data->client, MCU_APROM_ADDR, I2C_BUF, 2);

		dev_info(&data->client->dev, "Firmware current version =%d\n", I2C_BUF[1]);

		sscanf(firmware_name,"mcu_%3d",&ver);

		dev_info(&data->client->dev, "Firmware bin file version =%d\n", ver);

		if(ver > I2C_BUF[1]){
			dev_info(&data->client->dev, "###Request firmware update\n");

			ret_val = update_firmware_request(firmware_name);
			if(ret_val)
			{
				dev_info(&data->client->dev, "%s : update_firmware_request failed.\n", __func__);
				return ret_val;
			}

			I2C_BUF[0]=MCU_REG_LED;
			I2C_BUF[1]=0xB0;
			MCU_I2C_WRITE(data->client, MCU_APROM_ADDR, I2C_BUF, 2);
		}
	}

	return count;
}

static DEVICE_ATTR(temper1, S_IRUGO, mcu_show_temper1, NULL);
static DEVICE_ATTR(temper2, S_IRUGO, mcu_show_temper2, NULL);
static DEVICE_ATTR(fan, S_IRUGO, mcu_show_fan, NULL);
static DEVICE_ATTR(fw_ver, S_IRUGO, mcu_show_fw_ver, NULL);
static DEVICE_ATTR(bootloader_fw_ver, S_IRUGO, mcu_show_bootloader_fw_ver, NULL);
static DEVICE_ATTR(fan_speed, S_IWUSR | S_IRUGO, mcu_show_fan_speed, mcu_store_fan_speed);
static DEVICE_ATTR(led, S_IWUSR | S_IRUGO, mcu_show_led, mcu_store_led);
static DEVICE_ATTR(pwm_led, S_IWUSR | S_IRUGO, mcu_show_pwm_led, mcu_store_pwm_led);
static DEVICE_ATTR(pwm_led_duty, S_IWUSR | S_IRUGO, mcu_show_pwm_led_duty, mcu_store_pwm_led_duty);
static DEVICE_ATTR(led_speed, S_IWUSR | S_IRUGO, mcu_show_led_speed, mcu_store_led_speed);
static DEVICE_ATTR(led_red, S_IWUSR | S_IRUGO, mcu_show_led_red, mcu_store_led_red);
static DEVICE_ATTR(led_green, S_IWUSR | S_IRUGO, mcu_show_led_green, mcu_store_led_green);
static DEVICE_ATTR(led_blue, S_IWUSR | S_IRUGO, mcu_show_led_blue, mcu_store_led_blue);
static DEVICE_ATTR(led_f_red, S_IWUSR | S_IRUGO, mcu_show_led_f_red, mcu_store_led_f_red);
static DEVICE_ATTR(led_f_green, S_IWUSR | S_IRUGO, mcu_show_led_f_green, mcu_store_led_f_green);
static DEVICE_ATTR(led_f_blue, S_IWUSR | S_IRUGO, mcu_show_led_f_blue, mcu_store_led_f_blue);
static DEVICE_ATTR(bootloader_mode, S_IWUSR, NULL, mcu_store_bootloader_mode);
static DEVICE_ATTR(fw_update, S_IWUSR, NULL, mcu_store_fw_update);

static const struct file_operations mcu_led_fan_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
};

static struct attribute *mcu_attributes[] = {
	&dev_attr_temper1.attr,
	&dev_attr_temper2.attr,
	&dev_attr_fan.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_bootloader_fw_ver.attr,
	&dev_attr_fan_speed.attr,
	&dev_attr_led.attr,
	&dev_attr_pwm_led.attr,
	&dev_attr_pwm_led_duty.attr,
	&dev_attr_led_speed.attr,
	&dev_attr_led_red.attr,
	&dev_attr_led_green.attr,
	&dev_attr_led_blue.attr,
	&dev_attr_led_f_red.attr,
	&dev_attr_led_f_green.attr,
	&dev_attr_led_f_blue.attr,
	&dev_attr_bootloader_mode.attr,
	&dev_attr_fw_update.attr,
	NULL
};

static const struct attribute_group mcu_attr_group = {
	.attrs = mcu_attributes,
};

static int mcu_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int err = 0;

	dev_info(&client->dev, "#####MCU driver probe start!\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "#######I2C_FUNC_SMBUS not Supported\n");
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct mcu_bus_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,"mcu_led_fan probe: failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	data->reset_gpio = MCU_RESET_GPIO;
	data->boot_gpio = MCU_BOOT_GPIO;
	err= devm_gpio_request(&client->dev, data->reset_gpio, "MCU_reset_gpio");
	if (err) {
		dev_err(&client->dev,"##Unable to request reset_gpio\n");
		return err;
	}
	err= devm_gpio_request(&client->dev, data->boot_gpio, "MCU_boot_gpio");
	if (err) {
		dev_err(&client->dev,"##Unable to request boot_gpio\n");
		return err;
	}
	gpio_direction_output(data->boot_gpio,0);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);
	msleep(10);
	gpio_direction_output(data->reset_gpio,0);
	msleep(10);
	gpio_direction_output(data->reset_gpio,1);
	msleep(100);
	data->client = client;
	i2c_set_clientdata(client, data);

	err = i2c_smbus_write_byte_data(client, MCU_REG_LED, 0xC0);
	if (err) {
		dev_err(&client->dev, "####Init LED patern failed %d\n",err);
	}

	data->mcu_led_fan_device.minor = MISC_DYNAMIC_MINOR;
	data->mcu_led_fan_device.name = "mcu";
	data->mcu_led_fan_device.fops = &mcu_led_fan_dev_fops;

	err = misc_register(&data->mcu_led_fan_device);
	if (err) {
		dev_err(&client->dev, "misc_register failed, %d\n",err);
		goto registermisc_failed;
	}

	err = sysfs_create_group(&client->dev.kobj, &mcu_attr_group);
	if (err){
		dev_err(&client->dev, "#####MCU: device create file failed\n");
		goto createfile_failed;
	}

	err = sysfs_create_link(&data->mcu_led_fan_device.this_device->kobj,&client->dev.kobj, "i2c");
	if (err){
		dev_err(&client->dev, "#####MCU: device link file failed\n");
		goto createfile_failed;
	}
	dev_info(&client->dev, "#####MCU driver probed\n");
	return 0;

createfile_failed:
	sysfs_remove_link(&data->mcu_led_fan_device.this_device->kobj, "i2c");
	kfree(data);

registermisc_failed:
	misc_deregister(&data->mcu_led_fan_device);
	return err;

}

static int mcu_i2c_remove(struct i2c_client *client)
{
	struct mcu_bus_data *data;
	data = i2c_get_clientdata(client);

	sysfs_remove_link(&data->mcu_led_fan_device.this_device->kobj, "i2c");
	sysfs_remove_group(&client->dev.kobj, &mcu_attr_group);
	kfree(data);
	return 0;
}

static const struct i2c_device_id mcu_id[] = {
	{"mcu", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcu_id);

static const struct of_device_id mcu_of_match[] = {
	{ .compatible = "wistron,mcu", },
	{ },
};
MODULE_DEVICE_TABLE(of, mcu_of_match);

static struct i2c_driver mcu_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mcu",
		.of_match_table = mcu_of_match,
	},
	.probe		= mcu_i2c_probe,
	.remove		= mcu_i2c_remove,
	.id_table	= mcu_id,
};

module_i2c_driver(mcu_i2c_driver);


MODULE_DESCRIPTION("Wistron MCU driver");
MODULE_LICENSE("GPL");
