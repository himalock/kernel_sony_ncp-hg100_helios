/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/nvmem-consumer.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include "tsens.h"

#define CAL_MDEGC		30000

#define CONFIG_ADDR		0x3640
/* CONFIG_ADDR bitmasks */
#define CONFIG			0x9b
#define CONFIG_MASK		0xf
#define CONFIG_SHIFT		0

#define STATUS_CNTL_8064	0x3660
#define CNTL_ADDR		0x3620
/* CNTL_ADDR bitmasks */
#define EN			BIT(0)
#define SW_RST			BIT(1)
#define SENSOR0_EN		BIT(3)
#define SLP_CLK_ENA		BIT(26)
#define MEASURE_PERIOD		1
#define SENSOR0_SHIFT		3

/* INT_STATUS_ADDR bitmasks */
#define MIN_STATUS_MASK		BIT(0)
#define LOWER_STATUS_CLR	BIT(1)
#define UPPER_STATUS_CLR	BIT(2)
#define MAX_STATUS_MASK		BIT(3)

#define THRESHOLD_ADDR		0x3624
/* THRESHOLD_ADDR bitmasks */
#define THRESHOLD_MAX_CODE		0xff
#define THRESHOLD_MIN_CODE		0
#define THRESHOLD_MAX_LIMIT_SHIFT	24
#define THRESHOLD_MIN_LIMIT_SHIFT	16
#define THRESHOLD_UPPER_LIMIT_SHIFT	8
#define THRESHOLD_LOWER_LIMIT_SHIFT	0
#define THRESHOLD_MAX_LIMIT_MASK	(THRESHOLD_MAX_CODE << \
						THRESHOLD_MAX_LIMIT_SHIFT)
#define THRESHOLD_MIN_LIMIT_MASK	(THRESHOLD_MAX_CODE << \
						THRESHOLD_MIN_LIMIT_SHIFT)
#define THRESHOLD_UPPER_LIMIT_MASK	(THRESHOLD_MAX_CODE << \
						THRESHOLD_UPPER_LIMIT_SHIFT)
#define THRESHOLD_LOWER_LIMIT_MASK	(THRESHOLD_MAX_CODE << \
						THRESHOLD_LOWER_LIMIT_SHIFT)

/* Initial temperature threshold values */
#define LOWER_LIMIT_TH		0x9d /* 95C */
#define UPPER_LIMIT_TH		0xa6 /* 105C */
#define MIN_LIMIT_TH		0x0
#define MAX_LIMIT_TH		0xff

#define S0_STATUS_ADDR		0x3628
#define STATUS_ADDR_OFFSET	2
#define SENSOR_STATUS_SIZE	4
#define INT_STATUS_ADDR		0x363c
#define TRDY_MASK		BIT(7)
#define TIMEOUT_US		100

#define TSENS_EN		BIT(0)
#define TSENS_SW_RST		BIT(1)
#define TSENS_ADC_CLK_SEL	BIT(2)
#define SENSOR0_EN		BIT(3)
#define SENSOR1_EN		BIT(4)
#define SENSOR2_EN		BIT(5)
#define SENSOR3_EN		BIT(6)
#define SENSOR4_EN		BIT(7)
#define SENSORS_EN		(SENSOR0_EN | SENSOR1_EN | \
				SENSOR2_EN | SENSOR3_EN | SENSOR4_EN)
#define TSENS_8064_SENSOR5_EN				BIT(8)
#define TSENS_8064_SENSOR6_EN				BIT(9)
#define TSENS_8064_SENSOR7_EN				BIT(10)
#define TSENS_8064_SENSOR8_EN				BIT(11)
#define TSENS_8064_SENSOR9_EN				BIT(12)
#define TSENS_8064_SENSOR10_EN				BIT(13)
#define TSENS_8064_SENSORS_EN				(SENSORS_EN | \
						TSENS_8064_SENSOR5_EN | \
						TSENS_8064_SENSOR6_EN | \
						TSENS_8064_SENSOR7_EN | \
						TSENS_8064_SENSOR8_EN | \
						TSENS_8064_SENSOR9_EN | \
						TSENS_8064_SENSOR10_EN)

#define TSENS_8064_SEQ_SENSORS	5
#define TSENS_8064_S4_S5_OFFSET	40
#define TSENS_FACTOR		1000

/* Trips: from very hot to very cold */
enum tsens_trip_type {
	TSENS_TRIP_STAGE3 = 0,
	TSENS_TRIP_STAGE2,
	TSENS_TRIP_STAGE1,
	TSENS_TRIP_STAGE0,
	TSENS_TRIP_NUM,
};

u32 tsens_8064_slope[] = {
			1176, 1176, 1154, 1176,
			1111, 1132, 1132, 1199,
			1132, 1199, 1132
			};

/* Temperature on y axis and ADC-code on x-axis */
static inline int code_to_degC(u32 adc_code, const struct tsens_sensor *s)
{
	int degcbeforefactor, degc;

	degcbeforefactor = (adc_code * s->slope) + s->offset;

	if (degcbeforefactor == 0)
		degc = degcbeforefactor;
	else if (degcbeforefactor > 0)
		degc = (degcbeforefactor + TSENS_FACTOR/2)
			/ TSENS_FACTOR;
	else
		degc = (degcbeforefactor - TSENS_FACTOR/2)
			/ TSENS_FACTOR;

	return degc;
}

static int degC_to_code(int degC, const struct tsens_sensor *s)
{
	int code = ((degC * TSENS_FACTOR - s->offset) + (s->slope/2))
			/ s->slope;

	if (code > THRESHOLD_MAX_CODE)
		code = THRESHOLD_MAX_CODE;
	else if (code < THRESHOLD_MIN_CODE)
		code = THRESHOLD_MIN_CODE;
	return code;
}

static int suspend_ipq8064(struct tsens_device *tmdev)
{
	int ret;
	unsigned int mask;
	struct regmap *map = tmdev->map;

	ret = regmap_read(map, THRESHOLD_ADDR, &tmdev->ctx.threshold);
	if (ret)
		return ret;

	ret = regmap_read(map, CNTL_ADDR, &tmdev->ctx.control);
	if (ret)
		return ret;

	mask = SLP_CLK_ENA | EN;

	ret = regmap_update_bits(map, CNTL_ADDR, mask, 0);
	if (ret)
		return ret;

	return 0;
}

static int resume_ipq8064(struct tsens_device *tmdev)
{
	int ret;
	struct regmap *map = tmdev->map;

	ret = regmap_update_bits(map, CNTL_ADDR, SW_RST, SW_RST);
	if (ret)
		return ret;

	ret = regmap_update_bits(map, CONFIG_ADDR, CONFIG_MASK, CONFIG);
	if (ret)
		return ret;

	ret = regmap_write(map, THRESHOLD_ADDR, tmdev->ctx.threshold);
	if (ret)
		return ret;

	ret = regmap_write(map, CNTL_ADDR, tmdev->ctx.control);
	if (ret)
		return ret;

	return 0;
}

static void notify_uspace_tsens_fn(struct work_struct *work)
{
	struct tsens_sensor *s = container_of(work, struct tsens_sensor,
								notify_work);

	sysfs_notify(&s->tzd->device.kobj, NULL, "type");
}

static void tsens_scheduler_fn(struct work_struct *work)
{
	struct tsens_device *tmdev = container_of(work, struct tsens_device,
					tsens_work);
	unsigned int threshold, threshold_low, code, reg, sensor, mask;
	bool upper_th_x, lower_th_x;
	int ret;

	ret = regmap_read(tmdev->map, STATUS_CNTL_8064, &reg);
	if (ret)
		return;
	reg = reg | LOWER_STATUS_CLR | UPPER_STATUS_CLR;
	ret = regmap_write(tmdev->map, STATUS_CNTL_8064, reg);
	if (ret)
		return;

	mask = ~(LOWER_STATUS_CLR | UPPER_STATUS_CLR);
	ret = regmap_read(tmdev->map, THRESHOLD_ADDR, &threshold);
	if (ret)
		return;
	threshold_low = (threshold & THRESHOLD_LOWER_LIMIT_MASK)
				>> THRESHOLD_LOWER_LIMIT_SHIFT;
	threshold = (threshold & THRESHOLD_UPPER_LIMIT_MASK)
				>> THRESHOLD_UPPER_LIMIT_SHIFT;

	ret = regmap_read(tmdev->map, STATUS_CNTL_8064, &reg);
	if (ret)
		return;

	ret = regmap_read(tmdev->map, CNTL_ADDR, &sensor);
	if (ret)
		return;
	sensor &= (uint32_t) TSENS_8064_SENSORS_EN;
	sensor >>= SENSOR0_SHIFT;

	/* Constraint: There is only 1 interrupt control register for all
	 * 11 temperature sensor. So monitoring more than 1 sensor based
	 * on interrupts will yield inconsistent result. To overcome this
	 * issue we will monitor only sensor 0 which is the master sensor.
	 */

	/* Skip if the sensor is disabled */
	if (sensor & 1) {
		ret = regmap_read(tmdev->map, tmdev->sensor[0].status, &code);
		if (ret)
			return;
		upper_th_x = code >= threshold;
		lower_th_x = code <= threshold_low;
		if (upper_th_x)
			mask |= UPPER_STATUS_CLR;
		if (lower_th_x)
			mask |= LOWER_STATUS_CLR;
		if (upper_th_x || lower_th_x) {
			/* Notify user space */
			schedule_work(&tmdev->sensor[0].notify_work);
			pr_debug("Trigger (%d degrees) for sensor %d\n",
				code_to_degC(code, &tmdev->sensor[0]), 0);
		}
	}
	regmap_write(tmdev->map, STATUS_CNTL_8064, reg & mask);

	/* force memory to sync */
	mb();
}

static irqreturn_t tsens_isr(int irq, void *data)
{
	struct tsens_device *tmdev = data;

	schedule_work(&tmdev->tsens_work);
	return IRQ_HANDLED;
}

static void hw_init(struct tsens_device *tmdev)
{
	int ret;
	unsigned int reg_cntl = 0, reg_cfg = 0, reg_thr = 0;
	unsigned int reg_status_cntl = 0;

	regmap_read(tmdev->map, CNTL_ADDR, &reg_cntl);
	regmap_write(tmdev->map, CNTL_ADDR, reg_cntl | TSENS_SW_RST);

	reg_cntl |= SLP_CLK_ENA | (MEASURE_PERIOD << 18)
		| (((1 << tmdev->num_sensors) - 1) << SENSOR0_SHIFT);
	regmap_write(tmdev->map, CNTL_ADDR, reg_cntl);
	regmap_read(tmdev->map, STATUS_CNTL_8064, &reg_status_cntl);
	reg_status_cntl |= LOWER_STATUS_CLR | UPPER_STATUS_CLR
			| MIN_STATUS_MASK | MAX_STATUS_MASK;
	regmap_write(tmdev->map, STATUS_CNTL_8064, reg_status_cntl);
	reg_cntl |= TSENS_EN;
	regmap_write(tmdev->map, CNTL_ADDR, reg_cntl);

	regmap_read(tmdev->map, CONFIG_ADDR, &reg_cfg);
	reg_cfg = (reg_cfg & ~CONFIG_MASK) | (CONFIG << CONFIG_SHIFT);
	regmap_write(tmdev->map, CONFIG_ADDR, reg_cfg);

	reg_thr |= (LOWER_LIMIT_TH << THRESHOLD_LOWER_LIMIT_SHIFT)
		| (UPPER_LIMIT_TH << THRESHOLD_UPPER_LIMIT_SHIFT)
		| (MIN_LIMIT_TH << THRESHOLD_MIN_LIMIT_SHIFT)
		| (MAX_LIMIT_TH << THRESHOLD_MAX_LIMIT_SHIFT);

	regmap_write(tmdev->map, THRESHOLD_ADDR, reg_thr);

	ret = devm_request_irq(tmdev->dev, tmdev->tsens_irq, tsens_isr,
			IRQF_TRIGGER_RISING, "tsens_interrupt", tmdev);
	if (ret < 0) {
		pr_err("%s: request_irq FAIL: %d\n", __func__, ret);
		return;
	}

	INIT_WORK(&tmdev->tsens_work, tsens_scheduler_fn);
}

static int init_ipq8064(struct tsens_device *tmdev)
{
	int ret, i;
	u32 reg_cntl, offset = 0;

	init_common(tmdev);
	if (!tmdev->map)
		return -ENODEV;

	/*
	 * The status registers for each sensor are discontiguous
	 * because some SoCs have 5 sensors while others have more
	 * but the control registers stay in the same place, i.e
	 * directly after the first 5 status registers.
	 */
	for (i = 0; i < tmdev->num_sensors; i++) {
		if (i >= TSENS_8064_SEQ_SENSORS)
			offset = TSENS_8064_S4_S5_OFFSET;

		tmdev->sensor[i].status = S0_STATUS_ADDR + offset
					+ (i << STATUS_ADDR_OFFSET);
		tmdev->sensor[i].slope = tsens_8064_slope[i];
		INIT_WORK(&tmdev->sensor[i].notify_work,
						notify_uspace_tsens_fn);
	}

	reg_cntl = SW_RST;
	ret = regmap_update_bits(tmdev->map, CNTL_ADDR, SW_RST, reg_cntl);
	if (ret)
		return ret;

	reg_cntl |= SLP_CLK_ENA | (MEASURE_PERIOD << 18);
	reg_cntl &= ~SW_RST;
	ret = regmap_update_bits(tmdev->map, CONFIG_ADDR,
					 CONFIG_MASK, CONFIG);

	reg_cntl |= GENMASK(tmdev->num_sensors - 1, 0) << SENSOR0_SHIFT;
	ret = regmap_write(tmdev->map, CNTL_ADDR, reg_cntl);
	if (ret)
		return ret;

	reg_cntl |= EN;
	ret = regmap_write(tmdev->map, CNTL_ADDR, reg_cntl);
	if (ret)
		return ret;

	return 0;
}

static int calibrate_ipq8064(struct tsens_device *tmdev)
{
	int i;
	char *data, *data_backup;

	ssize_t num_read = tmdev->num_sensors;
	struct tsens_sensor *s = tmdev->sensor;

	data = qfprom_read(tmdev->dev, "calib");
	if (IS_ERR(data)) {
		pr_err("Calibration not found.\n");
		return PTR_ERR(data);
	}

	data_backup = qfprom_read(tmdev->dev, "calib_backup");
	if (IS_ERR(data_backup)) {
		pr_err("Backup calibration not found.\n");
		return PTR_ERR(data_backup);
	}

	for (i = 0; i < num_read; i++) {
		s[i].calib_data = readb_relaxed(data + i);
		s[i].calib_data_backup = readb_relaxed(data_backup + i);

		if (s[i].calib_data_backup)
			s[i].calib_data = s[i].calib_data_backup;
		if (!s[i].calib_data) {
			pr_err("QFPROM TSENS calibration data not present\n");
			return -ENODEV;
		}
		s[i].slope = tsens_8064_slope[i];
		s[i].offset = CAL_MDEGC - (s[i].calib_data * s[i].slope);
	}

	hw_init(tmdev);

	return 0;
}

static int get_temp_ipq8064(struct tsens_device *tmdev, int id, int *temp)
{
	int ret;
	u32 code, trdy;
	const struct tsens_sensor *s = &tmdev->sensor[id];
	unsigned long timeout;

	timeout = jiffies + usecs_to_jiffies(TIMEOUT_US);
	do {
		ret = regmap_read(tmdev->map, INT_STATUS_ADDR, &trdy);
		if (ret)
			return ret;
		if (!(trdy & TRDY_MASK))
			continue;
		ret = regmap_read(tmdev->map, s->status, &code);
		if (ret)
			return ret;
		*temp = code_to_degC(code, s);
		return 0;
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int set_trip_temp_ipq8064(void *data, int trip, int temp)
{
	unsigned int reg_th, reg_cntl;
	int ret, code, code_chk, hi_code, lo_code;
	const struct tsens_sensor *s = data;
	struct tsens_device *tmdev = s->tmdev;

	code_chk = code = degC_to_code(temp, s);

	if (code < THRESHOLD_MIN_CODE || code > THRESHOLD_MAX_CODE)
		return -EINVAL;

	ret = regmap_read(tmdev->map, STATUS_CNTL_8064, &reg_cntl);
	if (ret)
		return ret;

	ret = regmap_read(tmdev->map, THRESHOLD_ADDR, &reg_th);
	if (ret)
		return ret;

	hi_code = (reg_th & THRESHOLD_UPPER_LIMIT_MASK)
			>> THRESHOLD_UPPER_LIMIT_SHIFT;
	lo_code = (reg_th & THRESHOLD_LOWER_LIMIT_MASK)
			>> THRESHOLD_LOWER_LIMIT_SHIFT;

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		code <<= THRESHOLD_MAX_LIMIT_SHIFT;
		reg_th &= ~THRESHOLD_MAX_LIMIT_MASK;
		break;
	case TSENS_TRIP_STAGE2:
		if (code_chk <= lo_code)
			return -EINVAL;
		code <<= THRESHOLD_UPPER_LIMIT_SHIFT;
		reg_th &= ~THRESHOLD_UPPER_LIMIT_MASK;
		break;
	case TSENS_TRIP_STAGE1:
		if (code_chk >= hi_code)
			return -EINVAL;
		code <<= THRESHOLD_LOWER_LIMIT_SHIFT;
		reg_th &= ~THRESHOLD_LOWER_LIMIT_MASK;
		break;
	case TSENS_TRIP_STAGE0:
		code <<= THRESHOLD_MIN_LIMIT_SHIFT;
		reg_th &= ~THRESHOLD_MIN_LIMIT_MASK;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(tmdev->map, THRESHOLD_ADDR, reg_th | code);
	if (ret)
		return ret;

	return 0;
}

static int set_trip_activate_ipq8064(void *data, int trip,
					enum thermal_trip_activation_mode mode)
{
	unsigned int reg_cntl, mask, val;
	const struct tsens_sensor *s = data;
	struct tsens_device *tmdev = s->tmdev;
	int ret;

	if (!tmdev || trip < 0)
		return -EINVAL;

	ret = regmap_read(tmdev->map, STATUS_CNTL_8064, &reg_cntl);
	if (ret)
		return ret;

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		mask = MAX_STATUS_MASK;
		break;
	case TSENS_TRIP_STAGE2:
		mask = UPPER_STATUS_CLR;
		break;
	case TSENS_TRIP_STAGE1:
		mask = LOWER_STATUS_CLR;
		break;
	case TSENS_TRIP_STAGE0:
		mask = MIN_STATUS_MASK;
		break;
	default:
		return -EINVAL;
	}

	if (mode == THERMAL_TRIP_ACTIVATION_DISABLED)
		val = reg_cntl | mask;
	else
		val = reg_cntl & ~mask;

	ret = regmap_write(tmdev->map, STATUS_CNTL_8064, val);
	if (ret)
		return ret;

	/* force memory to sync */
	mb();
	return 0;
}

const struct tsens_ops ops_ipq8064 = {
	.init		= init_ipq8064,
	.calibrate	= calibrate_ipq8064,
	.get_temp	= get_temp_ipq8064,
	.suspend	= suspend_ipq8064,
	.resume		= resume_ipq8064,
	.set_trip_temp	= set_trip_temp_ipq8064,
	.set_trip_activate = set_trip_activate_ipq8064,
};

const struct tsens_data data_ipq8064 = {
	.num_sensors	= 11,
	.ops		= &ops_ipq8064,
};
