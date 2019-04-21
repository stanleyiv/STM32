
#ifndef __MAIN_H
#define __MAIN_H


#include "stm32f7xx_hal.h"
#include "delay.h"
#include "rk043fn48h.h"
#include "ltdc.h"

#define SDRAM_BASE 0xC0000000

	/*
					THIS IS BPP
 	 if(pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
	{
	tmp = 4;
	}
	else if (pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
	{
	tmp = 3;
	}
	else if((pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
	(pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_RGB565)   || \
	(pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_ARGB1555) || \
	(pLayerCfg->PixelFormat == LTDC_PIXEL_FORMAT_AL88))
	{
	tmp = 2;
	}
	else
	{
	tmp = 1;
	} */

#endif /* __MAIN_H */
