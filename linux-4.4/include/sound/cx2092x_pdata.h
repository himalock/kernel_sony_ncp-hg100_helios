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
struct cx2092x_platform_data {
	int gpio_reset; /* < 0 if not used */
};
