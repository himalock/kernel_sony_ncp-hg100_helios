/*
 * device_misc.c - driver for Device Misc control
 *
 * Copyright (C) 2018-2020 Wistron Corporation. www.wistron.com
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/thermal.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#define LOG_HEADER_TXT "[--------- HELIOS MISC ---------]"

#ifdef GET_BOOT_REASON
unsigned int resetInd = 0;
#endif

struct helios_misc_dev {
	struct miscdevice helios_misc_device;
	u32 usb_en_gpio;
	u32 usb_ocp_det_gpio;
	u32 bt_rst_gpio;
	u32 bt_vreg_en_gpio;
	u32 mic_mute_state_gpio;
	u32 uart_zwave_switch_gpio;
	u32 uart_zwave_switch;
	u32 lte_power_gpio;
	u32 lte_reset_gpio;
	u32 lineout_switch_gpio;
	u32 lineout_detect_gpio;
	int lineout_detect_irq;
	u32 usb_enabled;
	u32 bluetooth_enabled;
	u32 lte_enabled;

	struct delayed_work work;
};

static ssize_t usb_pwr_show(struct device *dev, struct device *attr, char *buf)
{
	struct helios_misc_dev *helios_misc_dev = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", helios_misc_dev->usb_enabled);
}

static ssize_t usb_pwr_store(struct device *dev, struct device *attr, const char *buf, size_t count)
{
	struct helios_misc_dev *helios_misc_dev = dev_get_drvdata(dev);
	unsigned int input;

	if (!sscanf(buf, "%u", &input))
		return -EINVAL;

	if (input > 0)
		input = 1;

	gpio_direction_output(helios_misc_dev->usb_en_gpio, input);
	helios_misc_dev->usb_enabled = input;

	return count;
}

static ssize_t usb_flt_show(struct device *dev, struct device *attr, char *buf)
{
	struct helios_misc_dev *helios_misc_dev = dev_get_drvdata(dev);
	
	if (gpio_get_value(helios_misc_dev->usb_ocp_det_gpio))
		return sprintf(buf, "%u\n", 0);	//high = no ocp
	else
		return sprintf(buf, "%u\n", 1);	//low = ocp
}

static ssize_t lte_pwr_show(struct device *dev, struct device *attr, char *buf)
{
	struct helios_misc_dev *helios_misc_dev = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", helios_misc_dev->lte_enabled);
}

static ssize_t lte_pwr_store(struct device *dev, struct device *attr, const char *buf, size_t count)
{
	struct helios_misc_dev *helios_misc_dev = dev_get_drvdata(dev);
	unsigned int input;

	if (!sscanf(buf, "%u", &input))
		return -EINVAL;

	if (input > 0)
		input = 1;

	if ((input == 1) && (helios_misc_dev->lte_enabled == 0)) {
		gpio_direction_input(helios_misc_dev->lte_power_gpio);//ext pull up
		msleep(20);
		gpio_direction_input(helios_misc_dev->lte_reset_gpio);//ext pull up
	} else if ((input == 0) && (helios_misc_dev->lte_enabled == 1)) {
		gpio_direction_output(helios_misc_dev->lte_reset_gpio, 0);
		gpio_direction_output(helios_misc_dev->lte_power_gpio, 0);
	}

	helios_misc_dev->lte_enabled = input;

	return count;
}

static const struct file_operations helios_misc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
};

static DEVICE_ATTR(usb_pwr, 0644, usb_pwr_show, usb_pwr_store);
static DEVICE_ATTR(usb_flt, 0444, usb_flt_show, NULL);
static DEVICE_ATTR(lte_pwr, 0644, lte_pwr_show, lte_pwr_store);

static struct attribute *helios_misc_attrs[] = {
	&dev_attr_usb_pwr.attr,
	&dev_attr_usb_flt.attr,
	&dev_attr_lte_pwr.attr,
	NULL,
};

static struct attribute_group helios_misc_attribute_group = {
	.attrs = helios_misc_attrs,
};


static int helios_misc_probe(struct platform_device *pdev)
{
	struct helios_misc_dev *helios_misc_dev;
	struct device_node *node = pdev->dev.of_node;
	struct dev_pin_info *pins;
	struct pinctrl_state *pin_state;
	int irq;

	int r = 0;

	pr_err(LOG_HEADER_TXT"HELIOS_MISC driver probe\n");
	if (!node) {
		pr_err(LOG_HEADER_TXT"device node invalid\n");
		return -ENODEV;
	}

	helios_misc_dev = devm_kzalloc(&pdev->dev, sizeof(*helios_misc_dev), GFP_KERNEL);
	if (helios_misc_dev == NULL) {
		pr_err(LOG_HEADER_TXT"failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, helios_misc_dev);
	
	//USB_EN GPIO
	helios_misc_dev->usb_en_gpio = of_get_named_gpio(node, "usb-en-gpio", 0);
	if (!gpio_is_valid(helios_misc_dev->usb_en_gpio)) {
		pr_err(LOG_HEADER_TXT"Missing dt property: usb-en-gpio\n");
		r = -EINVAL;
		goto dt_node_error;
	}
	//USB_OCP_DETECT GPIO
	helios_misc_dev->usb_ocp_det_gpio = of_get_named_gpio(node, "usb-ocp-det-gpio", 0);
	if (!gpio_is_valid(helios_misc_dev->usb_ocp_det_gpio)) {
		pr_err(LOG_HEADER_TXT"Missing dt property: usb-ocp-det-gpio\n");
		r = -EINVAL;
		goto dt_node_error;
	}

	//LTE_PWR GPIO
	helios_misc_dev->lte_power_gpio = of_get_named_gpio(node, "lte-pwr-gpio", 0);
	if (!gpio_is_valid(helios_misc_dev->lte_power_gpio)) {
		pr_err(LOG_HEADER_TXT"Missing dt property: lte-pwr-gpio\n");
		r = -EINVAL;
		goto dt_node_error;
	}
	//LTE_RST_GPIO
	helios_misc_dev->lte_reset_gpio = of_get_named_gpio(node, "lte-rst-gpio", 0);
	if (!gpio_is_valid(helios_misc_dev->lte_reset_gpio)) {
		pr_err(LOG_HEADER_TXT"Missing dt property: lte-rst-gpio\n");
		r = -EINVAL;
		goto dt_node_error;
	}

	//get USB power default setting
	if (of_property_read_u32(node, "usb-default-on", &helios_misc_dev->usb_enabled) < 0) {
		helios_misc_dev->usb_enabled = 1;//default enable if cannot get setting from dtsi
	}

	//get LTE power default setting
	if (of_property_read_u32(node, "lte-default-on", &helios_misc_dev->lte_enabled) < 0) {
		helios_misc_dev->lte_enabled = 1;//default enable if cannot get setting from dtsi
	}

	//USB_EN USB_OCP_DETECT GPIO
	r = gpio_request(helios_misc_dev->usb_en_gpio, "usb_en");
	if (r) {
		pr_info(LOG_HEADER_TXT"request usb_en_gpio failed\n");
		goto err_gpios;
	}
	r = gpio_request(helios_misc_dev->usb_ocp_det_gpio, "usb_ocp_det");
	if (r) {
		pr_info(LOG_HEADER_TXT"request usb_ocp_det_gpio failed\n");
		goto err_gpios;
	}
	r = gpio_direction_output(helios_misc_dev->usb_en_gpio, helios_misc_dev->usb_enabled);
	if (r) {
		pr_info(LOG_HEADER_TXT"config usb_en_gpio direction failed\n");
		goto err_gpios;
	}
	r = gpio_direction_input(helios_misc_dev->usb_ocp_det_gpio);
	if (r) {
		pr_info(LOG_HEADER_TXT"config usb_ocp_det_gpio direction failed\n");
		goto err_gpios;
	}

	//LTE_PWR LTE_RST GPIO
	r = gpio_request(helios_misc_dev->lte_power_gpio, "lte_pwr");
	if (r) {
		pr_info(LOG_HEADER_TXT"request lte_power_gpio failed\n");
		goto err_gpios;
	}
	r = gpio_request(helios_misc_dev->lte_reset_gpio, "lte_reset");
	if (r) {
		pr_info(LOG_HEADER_TXT"request lte_reset_gpio failed\n");
		goto err_gpios;
	}
	r = gpio_direction_output(helios_misc_dev->lte_reset_gpio, 0);
	if (r) {
		pr_info(LOG_HEADER_TXT"config lte_reset_gpio direction failed\n");
		goto err_gpios;
	}
	r = gpio_direction_output(helios_misc_dev->lte_power_gpio, 0);
	if (r) {
		pr_info(LOG_HEADER_TXT"config lte_power_gpio direction failed\n");
		goto err_gpios;
	}
	if (helios_misc_dev->lte_enabled) {
		msleep(250);
		r = gpio_direction_input(helios_misc_dev->lte_power_gpio);//external pull high
		if (r) {
			pr_info(LOG_HEADER_TXT"config lte_power_gpio direction failed\n");
			goto err_gpios;
		}
		msleep(20);
		r = gpio_direction_input(helios_misc_dev->lte_reset_gpio);//external pull high
		if (r) {
			pr_info(LOG_HEADER_TXT"config lte_reset_gpio direction failed\n");
			goto err_gpios;
		}
	}

	//Create sysfs nodes to misc device
	helios_misc_dev->helios_misc_device.minor = MISC_DYNAMIC_MINOR;
	helios_misc_dev->helios_misc_device.name = "helios_misc";
	helios_misc_dev->helios_misc_device.fops = &helios_misc_dev_fops;
	r = misc_register(&helios_misc_dev->helios_misc_device);
	if (r) {
		pr_err(LOG_HEADER_TXT"misc_register failed, %d\n",r);
		goto registermisc_failed;
	}

	r = sysfs_create_group(&helios_misc_dev->helios_misc_device.this_device->kobj, &helios_misc_attribute_group);
	if (r) {
		pr_err(LOG_HEADER_TXT"sysfs_create_group failed, %d\n",r);
		goto createfile_failed;
	}

	return 0;


err_request_irq:
	sysfs_remove_group(&helios_misc_dev->helios_misc_device, &helios_misc_attribute_group);
createfile_failed:
	misc_deregister(&helios_misc_dev->helios_misc_device);
registermisc_failed:
err_gpios:
	if (gpio_is_valid(helios_misc_dev->usb_en_gpio))
		gpio_free(helios_misc_dev->usb_en_gpio);
	if (gpio_is_valid(helios_misc_dev->usb_ocp_det_gpio))
		gpio_free(helios_misc_dev->usb_ocp_det_gpio);
dt_node_error:
	kfree(helios_misc_dev);
	helios_misc_dev = NULL;

	return r;
}

static int helios_misc_remove(struct platform_device *pdev)
{
	struct helios_misc_dev *helios_misc_dev;
	helios_misc_dev = platform_get_drvdata(pdev);

	sysfs_remove_group(&helios_misc_dev->helios_misc_device, &helios_misc_attribute_group);

	gpio_free(helios_misc_dev->usb_en_gpio);
	gpio_free(helios_misc_dev->usb_ocp_det_gpio);

	kfree(helios_misc_dev);
	helios_misc_dev = NULL;

	return 0;
}

/**********************************************************************/
static struct of_device_id helios_misc_match_table[] = {
	{ .compatible = "wistron,helios-misc"},
	{}
};
MODULE_DEVICE_TABLE(of, helios_misc_match_table);

static struct platform_driver helios_misc_driver = {
	.probe = helios_misc_probe,
	.remove = helios_misc_remove,
	.driver = {
		.name = "helios_misc",
		.of_match_table = helios_misc_match_table,
	},
};

module_platform_driver(helios_misc_driver);


MODULE_DESCRIPTION("Helios MISC control driver");
MODULE_LICENSE("GPL v2");
