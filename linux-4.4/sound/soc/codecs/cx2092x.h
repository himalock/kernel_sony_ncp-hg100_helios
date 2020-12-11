/*
 * ALSA SoC CX2092X Channel codec driver
 *
 * Copyright:   (C) 2009/2010 Conexant Systems
 *
 * Based on sound/soc/codecs/tlv320aic2x.c by Vladimir Barinov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef CX2092X_H
#define CX2092X_H

#define CX2092X_NAME "cx2092x"

#define CX2092X_SPI_WRITE_FLAG 0x80

#define NUM_OF_DAI 1

#ifndef CONFIG_SND_SOC_CX2092X_FIRMWARE_FILE
#define CONFIG_SND_SOC_CX2092X_FIRMWARE_FILE "cnxt/cx2092x.sfs"
#endif
#ifndef CONFIG_SND_SOC_CX2092X_BOOTLOADER_FILE
#define CONFIG_SND_SOC_CX2092X_BOOTLOADER_FILE "cnxt/iflash.bin"
#endif
#define CX2092X_REG_MAX 0x2000
#define AUDDRV_VERSION(major0,major1, minor, build ) ((major0)<<24|(major1)<<16| (minor)<<8 |(build))

#define CX2092X_LOADER_TIMEOUT 50 /*50 ms*/
#define CX2092X_SW_RESET_TIMEOUT 50 /*50 ms*/
#define CX2092X_MEMORY_UPDATE_TIMEOUT  30 /*5 times*/
#define CX2092X_MAX_MEM_BUF 0x100

#endif // CX2092X_H

